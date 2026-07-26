"""select a detected object as the manipulation target and broker the handoff.

exposes a SelectObject action. on select the manager reads the bridge's
`zed_object_<id>` collision object from the planning scene, freezes that id in
the bridge (so it stops updating/reaping it), and promotes it to a stable
`grasp_target` collision object for MTC to pick and attach. a release service
reverses this. state is minimal (the current selection); geometry is read from
the planning scene, so this node never touches zed_msgs.
"""

import time

import rclpy
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger

from pmp_a0509_msgs.action import SelectObject

FROZEN_TOPIC = "zed_object_bridge/frozen_objects"
TARGET_ID = "grasp_target"


class ObjectSelectionManager(Node):
    def __init__(self):
        super().__init__("object_selection_manager")

        self.prefix = self.declare_parameter("object_id_prefix", "zed_object_").value
        # guard so the bridge sees the freeze before we remove the source object
        self.freeze_settle_sec = self.declare_parameter("freeze_settle_sec", 0.2).value

        cb = ReentrantCallbackGroup()
        self.scene_client = self.create_client(
            GetPlanningScene, "get_planning_scene", callback_group=cb
        )
        self.scene_pub = self.create_publisher(PlanningScene, "planning_scene", 10)

        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.frozen_pub = self.create_publisher(Int32MultiArray, FROZEN_TOPIC, latched)
        self.frozen_pub.publish(Int32MultiArray(data=[]))  # start unfrozen

        self.action_server = ActionServer(
            self,
            SelectObject,
            "select_object",
            execute_callback=self.on_select,
            callback_group=cb,
        )
        self.create_service(
            Trigger, "~/release_object", self.on_release, callback_group=cb
        )

        # (object_id, source collision id) of the current selection, or None
        self.selected = None
        self.get_logger().info("object selection manager ready")

    def on_select(self, goal_handle):
        object_id = goal_handle.request.object_id
        source_id = f"{self.prefix}{object_id}"
        result = SelectObject.Result()

        if self.selected is not None:
            self._release()  # only one target at a time

        source = self.lookup_object(source_id)
        if source is None:
            goal_handle.abort()
            result.success = False
            result.message = f"{source_id} not in planning scene"
            return result

        # freeze first so the bridge stops touching the source before we move it
        self.frozen_pub.publish(Int32MultiArray(data=[object_id]))
        time.sleep(self.freeze_settle_sec)

        self.scene_pub.publish(self.promote_diff(source, source_id))
        self.selected = (object_id, source_id)

        self.get_logger().info(f"selected {source_id} -> {TARGET_ID}")
        goal_handle.succeed()
        result.success = True
        result.collision_object_id = TARGET_ID
        result.message = f"promoted {source_id} to {TARGET_ID}"
        return result

    def on_release(self, request, response):
        if self.selected is None:
            response.success = False
            response.message = "no active selection"
            return response
        released = self.selected[1]
        self._release()
        response.success = True
        response.message = f"released {released}"
        return response

    def _release(self):
        # unfreeze and drop the promoted target; the bridge resumes managing the
        # source object on its next detection
        self.frozen_pub.publish(Int32MultiArray(data=[]))
        remove = CollisionObject()
        remove.id = TARGET_ID
        remove.operation = CollisionObject.REMOVE
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(remove)
        self.scene_pub.publish(scene)
        self.selected = None

    def lookup_object(self, object_id: str):
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        if not self.scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("get_planning_scene unavailable")
            return None
        future = self.scene_client.call_async(req)
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done():
            self.get_logger().error("get_planning_scene timed out")
            return None
        scene = future.result().scene
        for obj in scene.world.collision_objects:
            if obj.id == object_id:
                return obj
        return None

    def promote_diff(self, source: CollisionObject, source_id: str) -> PlanningScene:
        remove = CollisionObject()
        remove.id = source_id
        remove.operation = CollisionObject.REMOVE

        # preserve the object exactly as detected (ZED is source of truth); only
        # the id changes. move_group canonicalises stored objects: the origin
        # lives in .pose and primitive_poses are relative to it, so copy .pose
        # too or the target lands at the frame origin
        target = CollisionObject()
        target.header.frame_id = source.header.frame_id
        target.id = TARGET_ID
        target.pose = source.pose
        target.primitives = source.primitives
        target.primitive_poses = source.primitive_poses
        target.meshes = source.meshes
        target.mesh_poses = source.mesh_poses
        target.operation = CollisionObject.ADD

        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [remove, target]
        return scene


def main(args=None):
    rclpy.init(args=args)
    node = ObjectSelectionManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
