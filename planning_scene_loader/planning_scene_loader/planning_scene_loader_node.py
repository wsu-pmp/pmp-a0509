import traceback
from abc import ABC, abstractmethod
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

NODE_NAME: str = "planning_scene_loader"

# Fallback colour (r, g, b, a) used when a shape doesn't specify its own
# "color" field in the yaml config.
DEFAULT_COLOR: tuple = (0.8, 0.0, 0.75, 0.8)


class ShapeDefinition(ABC):
    def __init__(
        self,
        name: str,
        frame_id: str,
        position: list,
        orientation: list,
        color: list = None,
        attached_link: str = None,
    ):
        self.name = name
        self.frame_id = frame_id
        self.position = position
        self.orientation = orientation  # quaternion [x, y, z, w]
        self.color = list(color) if color is not None else list(DEFAULT_COLOR)
        self.attached_link = attached_link

    def create_pose(self) -> Pose:
        pose = Pose()
        pose.position = Point(
            x=self.position[0], y=self.position[1], z=self.position[2]
        )
        pose.orientation = Quaternion(
            x=self.orientation[0],
            y=self.orientation[1],
            z=self.orientation[2],
            w=self.orientation[3],
        )
        return pose

    @abstractmethod
    def create_collision_object(self) -> CollisionObject:
        pass


class BoxShape(ShapeDefinition):
    def __init__(
        self,
        name: str,
        frame_id: str,
        position: list,
        orientation: list,
        dimensions: list,
        color: list = None,
        attached_link: str = None,
    ):
        super().__init__(name, frame_id, position, orientation, color, attached_link)
        self.dimensions = dimensions  # [x, y, z]

    def create_collision_object(self) -> CollisionObject:
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = self.frame_id
        collision_obj.id = self.name

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX

        dims = [0.0, 0.0, 0.0]
        dims[SolidPrimitive.BOX_X] = self.dimensions[0]
        dims[SolidPrimitive.BOX_Y] = self.dimensions[1]
        dims[SolidPrimitive.BOX_Z] = self.dimensions[2]
        primitive.dimensions = dims

        collision_obj.primitives.append(primitive)
        collision_obj.primitive_poses.append(self.create_pose())
        collision_obj.operation = CollisionObject.ADD

        return collision_obj


class SphereShape(ShapeDefinition):
    def __init__(
        self,
        name: str,
        frame_id: str,
        position: list,
        orientation: list,
        radius: float,
        color: list = None,
        attached_link: str = None,
    ):
        super().__init__(name, frame_id, position, orientation, color, attached_link)
        self.radius = radius

    def create_collision_object(self) -> CollisionObject:
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = self.frame_id
        collision_obj.id = self.name

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE

        dims = [0.0]
        dims[SolidPrimitive.SPHERE_RADIUS] = self.radius
        primitive.dimensions = dims

        collision_obj.primitives.append(primitive)
        collision_obj.primitive_poses.append(self.create_pose())
        collision_obj.operation = CollisionObject.ADD

        return collision_obj


class CylinderShape(ShapeDefinition):
    def __init__(
        self,
        name: str,
        frame_id: str,
        position: list,
        orientation: list,
        height: float,
        radius: float,
        color: list = None,
        attached_link: str = None,
    ):
        super().__init__(name, frame_id, position, orientation, color, attached_link)
        self.height = height
        self.radius = radius

    def create_collision_object(self) -> CollisionObject:
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = self.frame_id
        collision_obj.id = self.name

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER

        dims = [0.0, 0.0]
        dims[SolidPrimitive.CYLINDER_HEIGHT] = self.height
        dims[SolidPrimitive.CYLINDER_RADIUS] = self.radius
        primitive.dimensions = dims

        collision_obj.primitives.append(primitive)
        collision_obj.primitive_poses.append(self.create_pose())
        collision_obj.operation = CollisionObject.ADD

        return collision_obj


# cones are not supported by geometric_shapes' bodies::Body; move_group's
# self-filter/occupancy-map perception pipeline will log 'Unknown shape type' for
# this object. the object itself is added and collision-checked correctly
class ConeShape(ShapeDefinition):
    def __init__(
        self,
        name: str,
        frame_id: str,
        position: list,
        orientation: list,
        height: float,
        radius: float,
        color: list = None,
        attached_link: str = None,
    ):
        super().__init__(name, frame_id, position, orientation, color, attached_link)
        self.height = height
        self.radius = radius

    def create_collision_object(self) -> CollisionObject:
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = self.frame_id
        collision_obj.id = self.name

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CONE

        dims = [0.0, 0.0]
        dims[SolidPrimitive.CONE_HEIGHT] = self.height
        dims[SolidPrimitive.CONE_RADIUS] = self.radius
        primitive.dimensions = dims

        collision_obj.primitives.append(primitive)
        collision_obj.primitive_poses.append(self.create_pose())
        collision_obj.operation = CollisionObject.ADD

        return collision_obj


class ShapeFactory:
    @staticmethod
    def _require(shape_config: dict, key: str, name: str, shape_type: str):
        """Fetch a required field, raising a clear error identifying the
        offending shape if it's missing."""
        if key not in shape_config or shape_config[key] is None:
            raise ValueError(
                f"Shape '{name}' (type: {shape_type}) is missing required field '{key}'"
            )
        return shape_config[key]

    @staticmethod
    def _validate_vector(value, expected_len: int, field_name: str, name: str):
        if not isinstance(value, (list, tuple)) or len(value) != expected_len:
            raise ValueError(
                f"Shape '{name}': field '{field_name}' must be a list of "
                f"{expected_len} numbers, got {value!r}"
            )
        return list(value)

    @staticmethod
    def create_shape(shape_config: dict) -> ShapeDefinition:
        name = shape_config.get("name")
        if not name:
            raise ValueError("Shape definition is missing required field 'name'")

        shape_type = shape_config.get("type")
        if not shape_type:
            raise ValueError(f"Shape '{name}' is missing required field 'type'")
        shape_type = shape_type.lower()

        frame_id = shape_config.get("frame_id", "world")

        position = ShapeFactory._validate_vector(
            shape_config.get("position", [0.0, 0.0, 0.0]), 3, "position", name
        )
        orientation = ShapeFactory._validate_vector(
            shape_config.get("orientation", [0.0, 0.0, 0.0, 1.0]),
            4,
            "orientation",
            name,
        )

        color = shape_config.get("color")
        if color is not None:
            color = ShapeFactory._validate_vector(color, 4, "color", name)

        attached_link = shape_config.get("attached_link", None)

        if shape_type == "box":
            dimensions = ShapeFactory._require(
                shape_config, "dimensions", name, shape_type
            )
            dimensions = ShapeFactory._validate_vector(
                dimensions, 3, "dimensions", name
            )
            return BoxShape(
                name,
                frame_id,
                position,
                orientation,
                dimensions,
                color,
                attached_link,
            )

        elif shape_type == "sphere":
            radius = ShapeFactory._require(shape_config, "radius", name, shape_type)
            return SphereShape(
                name, frame_id, position, orientation, radius, color, attached_link
            )

        elif shape_type == "cylinder":
            height = ShapeFactory._require(shape_config, "height", name, shape_type)
            radius = ShapeFactory._require(shape_config, "radius", name, shape_type)
            return CylinderShape(
                name,
                frame_id,
                position,
                orientation,
                height,
                radius,
                color,
                attached_link,
            )

        elif shape_type == "cone":
            height = ShapeFactory._require(shape_config, "height", name, shape_type)
            radius = ShapeFactory._require(shape_config, "radius", name, shape_type)
            return ConeShape(
                name,
                frame_id,
                position,
                orientation,
                height,
                radius,
                color,
                attached_link,
            )

        else:
            raise ValueError(
                f"Shape '{name}': unknown shape type '{shape_type}' "
                f"(supported: box, sphere, cylinder, cone)"
            )


class PlanningSceneLoader(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.declare_parameter("config_file", "")

        config_file = (
            self.get_parameter("config_file").get_parameter_value().string_value
        )

        if not config_file:
            self.get_logger().error(
                "No config file specified. Use --ros-args -p config_file:=<path>"
            )
            raise RuntimeError("No config file specified")

        # load config
        self.shapes = self.load_config(config_file)

        if not self.shapes:
            self.get_logger().warn("No shapes loaded from configuration")
            return

        # service client for applying planning scene. relative (not "/apply_
        # planning_scene") so this node's own namespace - which must match
        # move_group's - determines which instance it talks to
        self.planning_scene_client = self.create_client(
            ApplyPlanningScene, "apply_planning_scene"
        )

        self.get_logger().info("Waiting for apply_planning_scene service...")
        if not self.planning_scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service apply_planning_scene not available")
            raise RuntimeError("Planning scene service not available")

        self.get_logger().info("Service available, adding shapes to planning scene...")
        self.add_shapes_to_scene()

    def load_config(self, config_file: str) -> list:
        try:
            config_path = Path(config_file)
            if not config_path.exists():
                self.get_logger().error(f"Config file not found: {config_file}")
                raise FileNotFoundError(f"Config file not found: {config_file}")

            with open(config_path, "r") as f:
                config = yaml.safe_load(f)

            shapes = []
            seen_names = set()

            if config and "shapes" in config:
                for i, shape_config in enumerate(config["shapes"]):
                    name = shape_config.get("name", f"<shape #{i}>")

                    if name in seen_names:
                        self.get_logger().warn(
                            f"Duplicate shape name '{name}' in config; "
                            f"ignoring repeated entry"
                        )
                        continue

                    try:
                        shape = ShapeFactory.create_shape(shape_config)
                        shapes.append(shape)
                        seen_names.add(name)
                        self.get_logger().info(
                            f"Loaded shape: {shape.name} (type: {shape_config['type']})"
                        )
                    except Exception as e:
                        self.get_logger().error(f"Failed to create shape: {e}")

            return shapes

        except Exception as e:
            self.get_logger().error(f"Failed to load config file: {e}")
            raise

    def add_shapes_to_scene(self):
        planning_scene_msg = PlanningScene()
        planning_scene_msg.is_diff = True

        # separate attached and world objects
        attached_objects = []
        world_objects = []

        # create collision objects for all shapes
        for shape in self.shapes:
            collision_obj: CollisionObject = shape.create_collision_object()

            # handle attached objects
            if shape.attached_link:
                from moveit_msgs.msg import AttachedCollisionObject

                attached_obj = AttachedCollisionObject()
                attached_obj.link_name = shape.attached_link
                attached_obj.object = collision_obj
                attached_objects.append(attached_obj)
                self.get_logger().info(
                    f"Adding attached shape: {shape.name} to link {shape.attached_link}"
                )
            else:
                world_objects.append(collision_obj)
                self.get_logger().info(f"Adding shape: {shape.name}")

            obj_color = ObjectColor()
            obj_color.id = collision_obj.id
            obj_color.color = ColorRGBA(
                r=shape.color[0],
                g=shape.color[1],
                b=shape.color[2],
                a=shape.color[3],
            )
            planning_scene_msg.object_colors.append(obj_color)

        # only populate robot_state if we have attached objects
        if attached_objects:
            # create proper robot_state with a valid (even if empty) joint_state
            planning_scene_msg.robot_state.joint_state.header.stamp = (
                self.get_clock().now().to_msg()
            )
            planning_scene_msg.robot_state.attached_collision_objects = attached_objects
            planning_scene_msg.robot_state.is_diff = True

        # add world objects
        planning_scene_msg.world.collision_objects = world_objects

        request = ApplyPlanningScene.Request()
        request.scene = planning_scene_msg

        future = self.planning_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(
                    f"Successfully added {len(self.shapes)} shapes to planning scene"
                )
            else:
                self.get_logger().error("Failed to apply planning scene")
                raise RuntimeError("Failed to apply planning scene")
        else:
            self.get_logger().error("Service call failed")
            raise RuntimeError("Service call to apply_planning_scene failed")


def main(args=None):
    rclpy.init(args=args)

    logger = rclpy.logging.get_logger(NODE_NAME)

    node = None
    try:
        node = PlanningSceneLoader()
    except KeyboardInterrupt:
        logger.info("Interrupt received. Shutting down.")
    except BaseException as ex:
        logger.error(str(ex))
        logger.debug(traceback.format_exc())
    finally:
        if node:
            node.destroy_node()

        if rclpy.ok():
            rclpy.try_shutdown()
