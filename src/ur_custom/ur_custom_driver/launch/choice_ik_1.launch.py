from launch import LaunchDescription
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from moveit_msgs.srv import GetPositionIK, ApplyPlanningScene, GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import Plane
import math

#This launch file makes the IK with collision avoidance of the floor and trajectory planning
def generate_launch_description():
    # Define the node class inline
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
            while not self.ik_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().info('Waiting for /compute_ik service...')
            while not self.planning_scene_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().info('Waiting for /apply_planning_scene service...')
            while not self.plan_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().info('Waiting for /plan_kinematic_path service...')
            self.joint_names = [
                'shoulder_pan_joint',
                'shoulder_lift_joint',
                'elbow_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint'
            ]
            # Define Cartesian poses (x, y, z in meters; roll, pitch, yaw in radians)
            self.poses = {
                '1': [0.155, -0.095, 0.575, 2.6, 0.96, 0.848],
                '2': [0.0, -0.4, 0.4, -1.57, 0.0, -3.14],
                '3': [-0.2, -0.2, 0.3, 2.6, 0.96, 0.848]
            }
            # Apply floor collision object during initialization
            self.apply_floor_collision()
            self.get_logger().info('JointTrajectoryPublisher initialized.')
            self.get_logger().info('Enter 1, 2, 3 to publish poses. Enter q to quit.')

        def apply_floor_collision(self):
            """
            Apply a floor plane at z=0.02 as a collision object to the planning scene.
            """
            try:
                # Create collision object for the floor
                collision_object = CollisionObject()
                collision_object.header.frame_id = 'base_link'
                collision_object.id = 'floor'
                plane = Plane()
                plane.coef = [0.0, 0.0, 1.0, -0.02]  # Plane equation: z = 0.02
                collision_object.planes = [plane]
                collision_object.operation = CollisionObject.ADD

                # Create PlanningScene
                planning_scene = PlanningScene()
                planning_scene.world.collision_objects = [collision_object]
                planning_scene.is_diff = True

                # Call ApplyPlanningScene service
                scene_request = ApplyPlanningScene.Request()
                scene_request.scene = planning_scene
                future = self.planning_scene_client.call_async(scene_request)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                if response and response.success:
                    self.get_logger().info('Successfully applied floor collision object at z=0.02.')
                else:
                    self.get_logger().error('Failed to apply planning scene.')
                    return False
                return True
            except Exception as e:
                self.get_logger().error(f'Failed to apply planning scene: {str(e)}')
                return False

        def publish_trajectory(self, trajectory):
            """
            Publish a planned JointTrajectory.
            """
            try:
                self.publisher_.publish(trajectory)
                self.get_logger().info('Published planned trajectory.')
            except Exception as e:
                self.get_logger().error(f'Failed to publish trajectory: {str(e)}')

        def compute_ik_and_plan(self, pose):
            """
            Compute IK and plan a collision-free trajectory to the target pose.
            Input: pose = [x, y, z, roll, pitch, yaw]
            Output: JointTrajectory or None if failed
            """
            x, y, z, roll, pitch, yaw = pose
            # Apply 180-degree rotation around z-axis
            x_new = -x  # Invert x
            y_new = -y  # Invert y
            z_new = z   # z unchanged
            yaw_new = yaw + math.pi if yaw <= 0 else yaw - math.pi  # Adjust yaw by π

            # Create PoseStamped message
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'base_link'  # Adjust if your base frame is different
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.pose.position.x = x_new
            target_pose.pose.position.y = y_new
            target_pose.pose.position.z = z_new
            # Convert roll, pitch, yaw to quaternion
            quaternion = self.rpy_to_quaternion(roll, pitch, yaw_new)
            target_pose.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

            # Log target pose for debugging
            self.get_logger().info(f'Target pose: position=({x_new:.3f}, {y_new:.3f}, {z_new:.3f}), '
                                   f'orientation(RPY)=({roll:.3f}, {pitch:.3f}, {yaw_new:.3f})'
                                   f'orientation(Quaternion)=({quaternion[0]:.3f}, {quaternion[1]:.3f}, {quaternion[2]:.3f}, {quaternion[3]:.3f})')

            # Create MotionPlanRequest
            motion_plan_request = MotionPlanRequest()
            motion_plan_request.group_name = 'ur_manipulator'  # Adjust if your planning group is different
            motion_plan_request.num_planning_attempts = 10
            motion_plan_request.allowed_planning_time = 2.0
            motion_plan_request.max_velocity_scaling_factor = 0.5
            motion_plan_request.max_acceleration_scaling_factor = 0.5

            # Add pose constraints
            constraints = Constraints()

            # Position constraint
            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = 'base_link'
            position_constraint.link_name = 'tool0'  # Adjust to your end-effector link
            position_constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
            position_constraint.constraint_region = BoundingVolume()
            position_constraint.constraint_region.primitive_poses = [target_pose.pose]
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [0.02]  # Increased tolerance for position
            position_constraint.constraint_region.primitives = [primitive]
            constraints.position_constraints = [position_constraint]

            # Orientation constraint
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = 'base_link'
            orientation_constraint.link_name = 'tool0'  # Adjust to your end-effector link
            orientation_constraint.orientation = target_pose.pose.orientation
            orientation_constraint.absolute_x_axis_tolerance = 0.1  # 0.1 radians tolerance
            orientation_constraint.absolute_y_axis_tolerance = 0.1
            orientation_constraint.absolute_z_axis_tolerance = 0.1
            orientation_constraint.weight = 1.0
            constraints.orientation_constraints = [orientation_constraint]

            motion_plan_request.goal_constraints = [constraints]

            # Create GetMotionPlan request
            plan_request = GetMotionPlan.Request()
            plan_request.motion_plan_request = motion_plan_request

            # Call planning service
            try:
                future = self.plan_client.call_async(plan_request)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                if response and response.motion_plan_response.error_code.val == 1:  # SUCCESS
                    trajectory = response.motion_plan_response.trajectory.joint_trajectory
                    # Log final joint positions for debugging
                    if trajectory.points:
                        final_point = trajectory.points[-1]
                        self.get_logger().info(f'Final joint positions: {final_point.positions}')
                    self.get_logger().info('Planned collision-free trajectory.')
                    return trajectory
                else:
                    self.get_logger().error(f'Planning failed: {response.motion_plan_response.error_code}')
                    return None
            except Exception as e:
                self.get_logger().error(f'Planning service call failed: {str(e)}')
                return None

        def rpy_to_quaternion(self, roll, pitch, yaw):
            """
            Convert roll, pitch, yaw to quaternion [x, y, z, w].
            """
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

    # Initialize rclpy and run the node inline
    rclpy.init()
    node = JointTrajectoryPublisher()
    try:
        while rclpy.ok():
            user_input = input("Enter 1, 2, 3 to publish poses. Enter q to quit: ")
            if user_input in node.poses:
                trajectory = node.compute_ik_and_plan(node.poses[user_input])
                if trajectory:
                    node.publish_trajectory(trajectory)
                else:
                    node.get_logger().error(f'Failed to plan trajectory for pose {user_input}')
            elif user_input == 'q':
                break
            else:
                node.get_logger().warn(f'Invalid input: {user_input}. Enter 1, 2, 3, or q.')
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    except Exception as e:
        node.get_logger().error(f'Error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # Return an empty LaunchDescription since the node runs inline
    return LaunchDescription([])