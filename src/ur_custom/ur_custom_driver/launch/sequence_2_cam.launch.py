from launch import LaunchDescription
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.srv import GetPositionIK, ApplyPlanningScene, GetMotionPlan, GetCartesianPath
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, JointConstraint
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion, Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import Plane
from onrobot_msgs.srv import GripExternal
import math
import time

# This launch file uses the camera and YOLO model to detect the pose to pick up, and performs a hardcoded pick and place task
# Each position in the sequence is defined in the 6 coordinates of space. IK is used to solve them
# Generated trajectories are linear in the joint space or in the coordinate space, depending the pose
# Collision avoidance for floor, gripper, camera and shelf
# Motion planning (OMPL) with path optimization (RRTConnectkConfigDefault)
# Opens and closes the gripper when necessary
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
            self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
            self.gripper_client = self.create_client(GripExternal, '/grip_external')
            for client, name in [(self.ik_client, '/compute_ik'),
                                 (self.planning_scene_client, '/apply_planning_scene'),
                                 (self.plan_client, '/plan_kinematic_path'),
                                 (self.cartesian_client, '/compute_cartesian_path'),
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
            #14cm offset from tool0 to tool in polyscope - In tools z axis (x base)
            #0.6cm offset from tool0 to tool in polyscope - In tools x axis (z base)
            self.poses = {
                '1': [-0.261+0.14, -0.322, 0.352+0.006, 2.813, -1.525, -2.79],  # Pose 1: Joint trajectory
                '2': [-0.315+0.14, -0.322, 0.352+0.006, 2.813, -1.525, -2.79],  # Pose 2: Linear trajectory
                '3': [-0.200+0.14, -0.322, 0.352+0.006, 2.813, -1.525, -2.79],  # Pose 3: Linear trajectory
                '4': [-0.200+0.14, -0.274, 0.352+0.006, 2.056, -1.552, -2.036],  # Pose 4: Joint trajectory
                '5': [-0.323+0.14, -0.274, 0.352+0.006, 2.056, -1.552, -2.036],  # Pose 5: Linear trajectory
                '6': [-0.200+0.14, -0.274, 0.352+0.006, 2.056, -1.552, -2.036]  # Pose 6: Linear trajectory
            }
            self.apply_collision_objects()
            self.get_logger().info('JointTrajectoryPublisher initialized.')
            self.get_logger().info('Enter "start" to begin sequence, "q" to quit.')

        def joint_state_callback(self, msg):
            try:
                for i, name in enumerate(self.joint_names):
                    idx = msg.name.index(name)
                    self.current_joint_positions[i] = msg.position[idx]
            except ValueError:
                self.get_logger().warn('Joint state names do not match expected joints.')

        def apply_collision_objects(self):
            try:
                planning_scene = PlanningScene()
                planning_scene.is_diff = True

                # Floor collision object
                floor_collision = CollisionObject()
                floor_collision.header.frame_id = 'base_link'
                floor_collision.id = 'floor'
                plane = Plane()
                plane.coef = [0.0, 0.0, 1.0, -0.06]
                floor_collision.planes = [plane]
                floor_collision.operation = CollisionObject.ADD

                # Shelf collision object
                shelf_collision = CollisionObject()
                shelf_collision.header.frame_id = 'world'
                shelf_collision.id = 'shelf'
                shelf_pose = Pose()
                shelf_pose.position.x = -0.4 * -1
                shelf_pose.position.y = -0.3 * -1
                shelf_pose.position.z = 0.21
                shelf_pose.orientation.w = 1.0
                shelf_primitive = SolidPrimitive()
                shelf_primitive.type = SolidPrimitive.BOX
                shelf_primitive.dimensions = [0.1, 0.12, 0.42]
                shelf_collision.primitives = [shelf_primitive]
                shelf_collision.primitive_poses = [shelf_pose]
                shelf_collision.operation = CollisionObject.ADD

                planning_scene.world.collision_objects = [floor_collision, shelf_collision]
                scene_request = ApplyPlanningScene.Request()
                scene_request.scene = planning_scene
                future = self.planning_scene_client.call_async(scene_request)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                if response and response.success:
                    self.get_logger().info('Applied floor and shelf collision objects.')
                else:
                    self.get_logger().error('Failed to apply planning scene.')
                    return False
                return True
            except Exception as e:
                self.get_logger().error(f'Failed to apply planning scene: {str(e)}')
                return False

        def grip_external(self, width, force=50, speed=10, is_wait=True):
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
                if trajectory.points:
                    duration = trajectory.points[-1].time_from_start.sec + trajectory.points[-1].time_from_start.nanosec / 1e9
                    time.sleep(duration + 1.0)
                self.get_logger().info('Published trajectory.')
            except Exception as e:
                self.get_logger().error(f'Failed to publish trajectory: {str(e)}')

        def compute_pose(self, pose):
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
            return target_pose

        def execute_sequence(self):
            # Pose 1: Joint trajectory
            pose1 = self.compute_pose(self.poses['1'])
            self.get_logger().info(f'Planning joint trajectory to pose 1: pos=({pose1.pose.position.x:.3f}, {pose1.pose.position.y:.3f}, {pose1.pose.position.z:.3f})')
            traj1 = self.plan_to_pose(pose1)
            if not traj1:
                self.get_logger().error('Failed to plan to pose 1.')
                return False
            self.publish_trajectory(traj1)

            # Pose 2: Linear trajectory
            pose2 = self.compute_pose(self.poses['2'])
            self.get_logger().info(f'Planning linear trajectory to pose 2: pos=({pose2.pose.position.x:.3f}, {pose2.pose.position.y:.3f}, {pose2.pose.position.z:.3f})')
            traj2 = self.plan_linear_to_pose(pose1, pose2)
            if not traj2:
                self.get_logger().error('Failed to plan linear trajectory to pose 2.')
                return False
            self.publish_trajectory(traj2)

            # Close gripper
            self.get_logger().info('Closing gripper.')
            if not self.grip_external(width=0.0):
                self.get_logger().error('Failed to close gripper.')
                return False

            # Pose 3: Linear trajectory
            pose3 = self.compute_pose(self.poses['3'])
            self.get_logger().info(f'Planning linear trajectory to pose 3: pos=({pose3.pose.position.x:.3f}, {pose3.pose.position.y:.3f}, {pose3.pose.position.z:.3f})')
            traj3 = self.plan_linear_to_pose(pose2, pose3)
            if not traj3:
                self.get_logger().error('Failed to plan linear trajectory to pose 3.')
                return False
            self.publish_trajectory(traj3)

            # Pose 4: Joint trajectory
            pose4 = self.compute_pose(self.poses['4'])
            self.get_logger().info(f'Planning joint trajectory to pose 4: pos=({pose4.pose.position.x:.3f}, {pose4.pose.position.y:.3f}, {pose4.pose.position.z:.3f})')
            traj4 = self.plan_to_pose(pose4)
            if not traj4:
                self.get_logger().error('Failed to plan to pose 4.')
                return False
            self.publish_trajectory(traj4)

            # Pose 5: Linear trajectory
            pose5 = self.compute_pose(self.poses['5'])
            self.get_logger().info(f'Planning linear trajectory to pose 5: pos=({pose5.pose.position.x:.3f}, {pose5.pose.position.y:.3f}, {pose5.pose.position.z:.3f})')
            traj5 = self.plan_linear_to_pose(pose4, pose5)
            if not traj5:
                self.get_logger().error('Failed to plan linear trajectory to pose 5.')
                return False
            self.publish_trajectory(traj5)

            # Open gripper
            self.get_logger().info('Opening gripper.')
            if not self.grip_external(width=37.0):
                self.get_logger().error('Failed to open gripper.')
                return False

            # Pose 6: Linear trajectory
            pose6 = self.compute_pose(self.poses['6'])
            self.get_logger().info(f'Planning linear trajectory to pose 6: pos=({pose6.pose.position.x:.3f}, {pose6.pose.position.y:.3f}, {pose6.pose.position.z:.3f})')
            traj6 = self.plan_linear_to_pose(pose5, pose6)
            if not traj6:
                self.get_logger().error('Failed to plan linear trajectory to pose 6.')
                return False
            self.publish_trajectory(traj6)

            return True

        def plan_to_pose(self, target_pose):
            motion_plan_request = MotionPlanRequest()
            motion_plan_request.group_name = 'ur_manipulator'
            motion_plan_request.num_planning_attempts = 1000
            motion_plan_request.allowed_planning_time = 7.0
            motion_plan_request.max_velocity_scaling_factor = 0.2
            motion_plan_request.max_acceleration_scaling_factor = 0.2
            motion_plan_request.planner_id = "RRTConnectkConfigDefault"
            #motion_plan_request.path_optimization.optimize_path_length = True

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
            orientation_constraint.absolute_x_axis_tolerance = 0.005
            orientation_constraint.absolute_y_axis_tolerance = 0.005
            orientation_constraint.absolute_z_axis_tolerance = 0.005
            orientation_constraint.weight = 1.0
            constraints.orientation_constraints = [orientation_constraint]

            joint_constraint = JointConstraint()
            joint_constraint.joint_name = 'shoulder_pan_joint'
            joint_constraint.position = 0.0
            joint_constraint.tolerance_above = math.radians(180)
            joint_constraint.tolerance_below = math.radians(180)
            joint_constraint.weight = 1.0
            constraints.joint_constraints = [joint_constraint]

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
                    return trajectory
                else:
                    self.get_logger().error(f'Planning failed: {response.motion_plan_response.error_code}')
                    return None
            except Exception as e:
                self.get_logger().error(f'Planning service call failed: {str(e)}')
                return None

        def plan_linear_to_pose(self, start_pose, goal_pose):
            cartesian_request = GetCartesianPath.Request()
            cartesian_request.header.frame_id = 'base_link'
            cartesian_request.header.stamp = self.get_clock().now().to_msg()
            cartesian_request.group_name = 'ur_manipulator'
            cartesian_request.link_name = 'tool0'
            cartesian_request.max_step = 0.0005
            cartesian_request.waypoints = [start_pose.pose, goal_pose.pose]
            cartesian_request.max_velocity_scaling_factor = 0.2
            cartesian_request.max_acceleration_scaling_factor = 0.2
            cartesian_request.jump_threshold = 0.0
            try:
                self.get_logger().info('Calling /compute_cartesian_path...')
                future = self.cartesian_client.call_async(cartesian_request)
                rclpy.spin_until_future_complete(self, future)
                if not future.result():
                    self.get_logger().error('Cartesian planning service call returned None!')
                    return None
                response = future.result()
                if response and response.fraction >= 0.8:
                    trajectory = response.solution.joint_trajectory
                    if trajectory.points:
                        final_point = trajectory.points[-1]
                        self.get_logger().info(f'Final joints (cartesian): {final_point.positions}')
                    else:
                        self.get_logger().error('Cartesian trajectory has no points!')
                        return None
                    return trajectory
                else:
                    self.get_logger().error(f'Cartesian planning failed or incomplete: fraction={response.fraction}')
                    return None
            except Exception as e:
                self.get_logger().error(f'Cartesian planning failed: {str(e)}')
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
            user_input = input('Enter "start" to begin sequence, "q" to quit: ')
            if user_input == 'start':
                if not node.execute_sequence():
                    node.get_logger().error('Sequence execution failed.')
            elif user_input == 'q':
                break
            else:
                node.get_logger().warn(f'Invalid input: {user_input}. Enter "start" or "q".')
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    except Exception as e:
        node.get_logger().error(f'Error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return LaunchDescription([])