"""bridge zed object detections into moveit planning scene collision objects

subscribes zed_msgs/ObjectsStamped, transforms each detection into the planning
frame, and publishes a planning scene diff adding a box per object. owns only
the `<prefix><label_id>` id namespace and only ever writes diffs, so it coexists
with planning_scene_loader (static safety objects) and MTC (grasp targets)

zed tracking ids are ephemeral and TERMINATE is rarely seen; objects are
removed via staleness
"""

import math

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Int32MultiArray
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener
from zed_msgs.msg import Object, ObjectsStamped

FROZEN_TOPIC = "zed_object_bridge/frozen_objects"

# zed_msgs/Object tracking_state
TRACK_OFF = 0
TRACK_OK = 1
TRACK_SEARCHING = 2
TRACK_TERMINATE = 3


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _norm(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _unit(a):
    n = _norm(a)
    return (a[0] / n, a[1] / n, a[2] / n) if n > 1e-9 else None


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _quat_from_columns(ex, ey, ez):
    """quaternion (x,y,z,w) from an orthonormal right-handed basis as columns."""
    m00, m01, m02 = ex[0], ey[0], ez[0]
    m10, m11, m12 = ex[1], ey[1], ez[1]
    m20, m21, m22 = ex[2], ey[2], ez[2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w, x, y, z = (m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w, x, y, z = (m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w, x, y, z = (m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s
    return (x, y, z, w)


def pose_and_dims_from_bbox(obj: Object):
    """oriented box pose (source frame) and per-axis dims from the 8 corners.

    corner layout (zed_msgs/BoundingBox3D): edges out of corner 0 are 0->3,
    0->1, 0->4; opposite corner is 6. returns (Pose, [dx,dy,dz]) or None if the
    corners are missing or degenerate, so the caller can fall back.
    """
    corners = obj.bounding_box_3d.corners
    if len(corners) != 8:
        return None
    c = [tuple(float(v) for v in kp.kp) for kp in corners]
    edge_x, edge_y, edge_z = _sub(c[3], c[0]), _sub(c[1], c[0]), _sub(c[4], c[0])
    ex, ey = _unit(edge_x), _unit(edge_y)
    if ex is None or ey is None:
        return None
    ez = _cross(ex, ey)  # guarantees a right-handed orthonormal basis
    dims = [_norm(edge_x), _norm(edge_y), _norm(edge_z)]
    if min(dims) <= 0.0:
        return None

    pose = Pose()
    pose.position.x = (c[0][0] + c[6][0]) / 2.0
    pose.position.y = (c[0][1] + c[6][1]) / 2.0
    pose.position.z = (c[0][2] + c[6][2]) / 2.0
    qx, qy, qz, qw = _quat_from_columns(ex, ey, ez)
    pose.orientation.x, pose.orientation.y = qx, qy
    pose.orientation.z, pose.orientation.w = qz, qw
    return pose, dims


class ZedObjectBridge(Node):
    def __init__(self):
        super().__init__("zed_object_bridge")

        self.planning_frame = self.declare_parameter(
            "planning_frame", "base_link"
        ).value
        self.objects_topic = self.declare_parameter(
            "objects_topic", "/zed/zed_node/obj_det/objects"
        ).value
        self.prefix = self.declare_parameter("object_id_prefix", "zed_object_").value
        self.min_confidence = self.declare_parameter("min_confidence", 30.0).value
        self.stale_timeout = self.declare_parameter("stale_timeout_sec", 0.5).value
        reaper_period = self.declare_parameter("reaper_period_sec", 0.25).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scene_pub = self.create_publisher(PlanningScene, "planning_scene", 10)
        self.create_subscription(
            ObjectsStamped, self.objects_topic, self.on_objects, 10
        )
        self.create_timer(reaper_period, self.reap_stale)

        # ids the selection manager has taken ownership of; the bridge stops
        # adding/reaping them so MTC can attach without a fight. latched so the
        # current frozen set survives start-order and reconnects
        self.frozen_ids: set[str] = set()
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Int32MultiArray, FROZEN_TOPIC, self.on_frozen, latched)

        # object id -> last-seen ros time (seconds, float)
        self.last_seen: dict[str, float] = {}

        self.get_logger().info(
            f"bridging {self.objects_topic} -> planning_scene "
            f"(frame {self.planning_frame}, prefix {self.prefix})"
        )

    def on_objects(self, msg: ObjectsStamped) -> None:
        tf = self.lookup(msg.header.frame_id, msg.header.stamp)
        if tf is None:
            return

        objects = []
        now = self.now_sec()
        for obj in msg.objects:
            if obj.tracking_state in (TRACK_OFF, TRACK_TERMINATE):
                continue
            if obj.confidence < self.min_confidence:
                continue
            if f"{self.prefix}{obj.label_id}" in self.frozen_ids:
                continue  # owned by the selection manager / MTC
            collision_obj = self.to_collision_object(obj, tf)
            objects.append(collision_obj)
            self.last_seen[collision_obj.id] = now

        if objects:
            self.publish_diff(objects)

    def lookup(self, source_frame: str, stamp):
        # prefer the detection stamp; fall back to the latest available transform
        # (detections can arrive slightly ahead of the arm tf -> extrapolation)
        for when in (stamp, rclpy.time.Time()):
            try:
                return self.tf_buffer.lookup_transform(
                    self.planning_frame,
                    source_frame,
                    when,
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
            except TransformException as ex:
                last = ex
        self.get_logger().warn(
            f"no transform {source_frame} -> {self.planning_frame}: {last}",
            throttle_duration_sec=2.0,
        )
        return None

    def on_frozen(self, msg: Int32MultiArray) -> None:
        self.frozen_ids = {f"{self.prefix}{i}" for i in msg.data}
        # stop tracking frozen ids so the reaper never removes them out from
        # under the manager
        for oid in self.frozen_ids:
            self.last_seen.pop(oid, None)
        self.get_logger().info(f"frozen ids: {sorted(self.frozen_ids)}")

    def to_collision_object(self, obj: Object, tf) -> CollisionObject:
        # prefer the oriented bounding box (correct orientation + per-axis dims);
        # fall back to the centroid with identity orientation and dimensions_3d
        # if corners are missing/degenerate
        result = pose_and_dims_from_bbox(obj)
        if result is not None:
            src_pose, dims = result
        else:
            src_pose = Pose()
            src_pose.position.x = float(obj.position[0])
            src_pose.position.y = float(obj.position[1])
            src_pose.position.z = float(obj.position[2])
            src_pose.orientation.w = 1.0
            dims = [float(d) for d in obj.dimensions_3d]

        # src_pose is in the detection (source) frame; carry it into planning
        pose = do_transform_pose(src_pose, tf)

        collision_obj = CollisionObject()
        collision_obj.header.frame_id = self.planning_frame
        collision_obj.id = f"{self.prefix}{obj.label_id}"
        # object origin goes in .pose (canonical form); leaving it default gives
        # a zero quaternion that move_group mishandles, landing the object at the
        # frame origin. primitive sits at the object origin
        collision_obj.pose = pose
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = dims
        identity = Pose()
        identity.orientation.w = 1.0
        collision_obj.primitives.append(primitive)
        collision_obj.primitive_poses.append(identity)
        collision_obj.operation = CollisionObject.ADD
        return collision_obj

    def reap_stale(self) -> None:
        now = self.now_sec()
        stale = [
            oid
            for oid, seen in self.last_seen.items()
            if now - seen > self.stale_timeout
        ]
        if not stale:
            return
        removals = []
        for oid in stale:
            obj = CollisionObject()
            obj.header.frame_id = self.planning_frame
            obj.id = oid
            obj.operation = CollisionObject.REMOVE
            removals.append(obj)
            del self.last_seen[oid]
        self.publish_diff(removals)
        self.get_logger().info(f"reaped stale objects: {stale}")

    def publish_diff(self, collision_objects: list) -> None:
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = collision_objects
        self.scene_pub.publish(scene)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = ZedObjectBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
