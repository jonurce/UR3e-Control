from launch import LaunchDescription
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.srv import GetPositionIK, ApplyPlanningScene, GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import Plane
from onrobot_msgs.srv import GripExternal
import math

# This launch file performs IK with collision avoidance for floor, gripper and camera, as well as motion planning
# It gives the user pose options to select, as well as to open or close the gripper
def generate_launch_description():
    class JointTrajectoryPublisher(Node):
        def __init__(self):
            super().__init__('joint_trajectory_publisher')
            self.publisher_ = self.create_publisher(
                JointTrajectory,
                '/scaled_joint_trajectory_controller/joint_trajectory',
                10
            )
            self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
            self.planning_scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
            self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
            self.gripper_client = self.create_client(GripExternal, '/grip_external')
            for client, name in [(self.ik_client, '/compute_ik'),
                                 (self.planning_scene_client, '/apply_planning_scene'),
                                 (self.plan_client, '/plan_kinematic_path'),
                                 (self.gripper_client, '/grip_external')]:
                if not client.wait_for_service(timeout_sec=10.0):
                    self.get_logger().error(f'Service {name} not available after 10s!')
                    return
                self.get_logger().info(f'Service {name} connected.')
            self.joint_names = [
                'shoulder_pan_joint',
                'shoulder_lift_joint',
                'elbow_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint'
            ]
            self.poses = {
                '1': [0.155, -0.095, 0.575, 2.6, 0.96, 0.848],
                '2': [0.0, -0.4, 0.4, -1.57, 0.0, -3.14],
                '3': [-0.2, -0.2, 0.3, 2.6, 0.96, 0.848]
            }
            self.apply_floor_collision()
            self.get_logger().info('JointTrajectoryPublisher initialized.')
            self.get_logger().info('Enter 1, 2, 3 for poses, o to open gripper, c to close, q to quit.')

        def apply_floor_collision(self):
            try:
                collision_object = CollisionObject()
                collision_object.header.frame_id = 'base_link'
                collision_object.id = 'floor'
                plane = Plane()
                plane.coef = [0.0, 0.0, 1.0, -0.05]
                collision_object.planes = [plane]
                collision_object.operation = CollisionObject.ADD
                planning_scene = PlanningScene()
                planning_scene.world.collision_objects = [collision_object]
                planning_scene.is_diff = True
                scene_request = ApplyPlanningScene.Request()
                scene_request.scene = planning_scene
                future = self.planning_scene_client.call_async(scene_request)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                if response and response.success:
                    self.get_logger().info('Applied floor collision object at z=0.05.')
                else:
                    self.get_logger().error('Failed to apply planning scene.')
                    return False
                return True
            except Exception as e:
                self.get_logger().error(f'Failed to apply planning scene: {str(e)}')
                return False

        def grip_external(self, width, force=20, speed=10, is_wait=True):
            try:
                request = GripExternal.Request()
                request.index = 0
                request.width = width
                request.force = force
                request.speed = speed
                request.is_wait = is_wait
                future = self.gripper_client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                if response:
                    self.get_logger().info(f'Gripper set to width={width}mm, force={force}N')
                    return True
                else:
                    self.get_logger().error('Failed to call gripper service')
                    return False
            except Exception as e:
                self.get_logger().error(f'Gripper service call failed: {str(e)}')
                return False

        def publish_trajectory(self, trajectory):
            try:
                self.get_logger().info(f'Publishing trajectory with {len(trajectory.points)} points: {trajectory.points[-1].positions}')
                trajectory.joint_names = self.joint_names
                self.publisher_.publish(trajectory)
                self.get_logger().info('Published trajectory.')
            except Exception as e:
                self.get_logger().error(f'Failed to publish trajectory: {str(e)}')

        def compute_ik_and_plan(self, pose):
            x, y, z, roll, pitch, yaw = pose
            x_new = -x
            y_new = -y
            z_new = z
            yaw_new = yaw + math.pi if yaw <= 0 else yaw - math.pi
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'base_link'
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.pose.position.x = x_new
            target_pose.pose.position.y = y_new
            target_pose.pose.position.z = z_new
            quaternion = self.rpy_to_quaternion(roll, pitch, yaw_new)
            target_pose.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
            self.get_logger().info(f'Target: pos=({x_new:.3f}, {y_new:.3f}, {z_new:.3f}), quat=({quaternion[0]:.3f}, {quaternion[1]:.3f}, {quaternion[2]:.3f}, {quaternion[3]:.3f})')
            motion_plan_request = MotionPlanRequest()
            motion_plan_request.group_name = 'ur_manipulator'
            motion_plan_request.num_planning_attempts = 20
            motion_plan_request.allowed_planning_time = 5.0
            motion_plan_request.max_velocity_scaling_factor = 0.3
            motion_plan_request.max_acceleration_scaling_factor = 0.3
            constraints = Constraints()
            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = 'base_link'
            position_constraint.link_name = 'tool0'
            position_constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
            position_constraint.constraint_region = BoundingVolume()
            position_constraint.constraint_region.primitive_poses = [target_pose.pose]
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [0.001]
            position_constraint.constraint_region.primitives = [primitive]
            constraints.position_constraints = [position_constraint]
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = 'base_link'
            orientation_constraint.link_name = 'tool0'
            orientation_constraint.orientation = target_pose.pose.orientation
            orientation_constraint.absolute_x_axis_tolerance = 0.01
            orientation_constraint.absolute_y_axis_tolerance = 0.01
            orientation_constraint.absolute_z_axis_tolerance = 0.01
            orientation_constraint.weight = 1.0
            constraints.orientation_constraints = [orientation_constraint]
            motion_plan_request.goal_constraints = [constraints]
            plan_request = GetMotionPlan.Request()
            plan_request.motion_plan_request = motion_plan_request
            try:
                self.get_logger().info('Calling /plan_kinematic_path...')
                future = self.plan_client.call_async(plan_request)
                rclpy.spin_until_future_complete(self, future)
                if not future.result():
                    self.get_logger().error('Planning service call returned None!')
                    return None
                response = future.result()
                if response and response.motion_plan_response.error_code.val == 1:
                    trajectory = response.motion_plan_response.trajectory.joint_trajectory
                    if trajectory.points:
                        final_point = trajectory.points[-1]
                        self.get_logger().info(f'Final joints: {final_point.positions}')
                    else:
                        self.get_logger().error('Trajectory has no points!')
                        return None
                    self.get_logger().info('Planning succeeded.')
                    return trajectory
                else:
                    self.get_logger().error(f'Planning failed: {response.motion_plan_response.error_code}')
                    return None
            except Exception as e:
                self.get_logger().error(f'Planning service call failed: {str(e)}')
                return None

        def rpy_to_quaternion(self, roll, pitch, yaw):
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)
            w = cr * cp * cy + sr * sp * sy
            x = sr * cp * cy - cr * sp * sy
            y = cr * sp * cy + sr * cp * sy
            z = cr * cp * sy - sr * sp * cy
            return [x, y, z, w]

    rclpy.init()
    node = JointTrajectoryPublisher()
    try:
        while rclpy.ok():
            user_input = input("Enter 1, 2, 3 for poses, o to open gripper, c to close, q to quit: ")
            if user_input in node.poses:
                trajectory = node.compute_ik_and_plan(node.poses[user_input])
                if trajectory:
                    node.publish_trajectory(trajectory)
                else:
                    node.get_logger().error(f'Failed to plan trajectory for pose {user_input}')
            elif user_input == 'o':
                node.grip_external(width=37.0)
            elif user_input == 'c':
                node.grip_external(width=0.0)
            elif user_input == 'q':
                break
            else:
                node.get_logger().warn(f'Invalid input: {user_input}. Enter 1, 2, 3, o, c, or q.')
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    except Exception as e:
        node.get_logger().error(f'Error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return LaunchDescription([])