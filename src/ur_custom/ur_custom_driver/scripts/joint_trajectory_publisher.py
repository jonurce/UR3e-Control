#!/usr/bin/env python3

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
from std_msgs.msg import Int32MultiArray
from ur_custom_driver.srv import CameraProcess
import math
import time

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
        self.camera_client = self.create_client(CameraProcess, '/camera_process')

        for client, name in [
            (self.ik_client, '/compute_ik'),
            (self.planning_scene_client, '/apply_planning_scene'),
            (self.plan_client, '/plan_kinematic_path'),
            (self.cartesian_client, '/compute_cartesian_path'),
            (self.gripper_client, '/grip_external'),
            (self.camera_client, '/camera_process')
        ]:
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
        self.current_joint_positions = [0.0] * 6

        # Define poses: camera pose + 8 sequences (3 poses each, 24 total)
        self.poses = {
            'camera_pose': [-0.250+0.14, -0.300, 0.400+0.006, 2.813, -1.525, -2.79],  # Hardcoded camera pose
            # Sequence for value 2 (even)
            '2_1': [-0.261+0.14, -0.322, 0.352+0.006, 2.813, -1.525, -2.79],
            '2_2': [-0.315+0.14, -0.322, 0.352+0.006, 2.813, -1.525, -2.79],
            '2_3': [-0.200+0.14, -0.322, 0.352+0.006, 2.813, -1.525, -2.79],
            # Sequence for value 3 (odd)
            '3_1': [-0.200+0.14, -0.274, 0.352+0.006, 2.056, -1.552, -2.036],
            '3_2': [-0.323+0.14, -0.274, 0.352+0.006, 2.056, -1.552, -2.036],
            '3_3': [-0.200+0.14, -0.274, 0.352+0.006, 2.056, -1.552, -2.036],
            # Sequence for value 4 (even)
            '4_1': [-0.270+0.14, -0.310, 0.360+0.006, 2.813, -1.525, -2.79],
            '4_2': [-0.325+0.14, -0.310, 0.360+0.006, 2.813, -1.525, -2.79],
            '4_3': [-0.210+0.14, -0.310, 0.360+0.006, 2.813, -1.525, -2.79],
            # Sequence for value 5 (odd)
            '5_1': [-0.190+0.14, -0.264, 0.340+0.006, 2.056, -1.552, -2.036],
            '5_2': [-0.313+0.14, -0.264, 0.340+0.006, 2.056, -1.552, -2.036],
            '5_3': [-0.190+0.14, -0.264, 0.340+0.006, 2.056, -1.552, -2.036],
            # Sequence for value 6 (even)
            '6_1': [-0.280+0.14, -0.320, 0.370+0.006, 2.813, -1.525, -2.79],
            '6_2': [-0.335+0.14, -0.320, 0.370+0.006, 2.813, -1.525, -2.79],
            '6_3': [-0.220+0.14, -0.320, 0.370+0.006, 2.813, -1.525, -2.79],
            # Sequence for value 7 (odd)
            '7_1': [-0.180+0.14, -0.254, 0.330+0.006, 2.056, -1.552, -2.036],
            '7_2': [-0.303+0.14, -0.254, 0.330+0.006, 2.056, -1.552, -2.036],
            '7_3': [-0.180+0.14, -0.254, 0.330+0.006, 2.056, -1.552, -2.036],
            # Sequence for value 8 (even)
            '8_1': [-0.290+0.14, -0.330, 0.380+0.006, 2.813, -1.525, -2.79],
            '8_2': [-0.345+0.14, -0.330, 0.380+0.006, 2.813, -1.525, -2.79],
            '8_3': [-0.230+0.14, -0.330, 0.380+0.006, 2.813, -1.525, -2.79],
            # Sequence for value 9 (odd)
            '9_1': [-0.170+0.14, -0.244, 0.320+0.006, 2.056, -1.552, -2.036],
            '9_2': [-0.293+0.14, -0.244, 0.320+0.006, 2.056, -1.552, -2.036],
            '9_3': [-0.170+0.14, -0.244, 0.320+0.006, 2.056, -1.552, -2.036]
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

    def call_camera_service(self):
        try:
            request = CameraProcess.Request()
            future = self.camera_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if response:
                value1, value2 = response.value1, response.value2
                if value1 in range(2, 10) and value2 in range(2, 10) and (value1 % 2 == 0) != (value2 % 2 == 0):
                    self.get_logger().info(f'Camera service returned values: {value1}, {value2}')
                    return value1, value2
                else:
                    self.get_logger().error(f'Invalid camera service response: values {value1}, {value2} not in [2-9] or not one even, one odd')
                    return None
            else:
                self.get_logger().error('Camera service call failed')
                return None
        except Exception as e:
            self.get_logger().error(f'Camera service call failed: {str(e)}')
            return None

    def execute_sequence(self):
        # Move to camera pose
        camera_pose = self.compute_pose(self.poses['camera_pose'])
        self.get_logger().info(f'Planning to camera pose: pos=({camera_pose.pose.position.x:.3f}, {camera_pose.pose.position.y:.3f}, {camera_pose.pose.position.z:.3f})')
        traj_camera = self.plan_to_pose(camera_pose)
        if not traj_camera:
            self.get_logger().error('Failed to plan to camera pose.')
            return False
        self.publish_trajectory(traj_camera)

        # Call camera service
        result = self.call_camera_service()
        if not result:
            self.get_logger().error('Failed to get camera service response.')
            return False
        value1, value2 = result
        odd_value = value1 if value1 % 2 == 1 else value2
        even_value = value2 if value2 % 2 == 0 else value1

        # Execute odd value sequence first
        if not self.execute_value_sequence(odd_value, camera_pose):
            self.get_logger().error(f'Failed to execute sequence for value {odd_value}.')
            return False

        # Execute even value sequence
        if not self.execute_value_sequence(even_value, self.compute_pose(self.poses[f'{odd_value}_3'])):
            self.get_logger().error(f'Failed to execute sequence for value {even_value}.')
            return False

        return True

    def execute_value_sequence(self, value, start_pose):
        # Execute three poses for the given value
        for i in range(1, 4):
            pose_key = f'{value}_{i}'
            target_pose = self.compute_pose(self.poses[pose_key])
            self.get_logger().info(f'Planning to pose {pose_key}: pos=({target_pose.pose.position.x:.3f}, {target_pose.pose.position.y:.3f}, {target_pose.pose.position.z:.3f})')
            traj = self.plan_linear_to_pose(start_pose, target_pose) if i > 1 else self.plan_to_pose(target_pose)
            if not traj:
                self.get_logger().error(f'Failed to plan to pose {pose_key}.')
                return False
            self.publish_trajectory(traj)
            start_pose = target_pose

            # After second pose, perform gripper action based on parity
            if i == 2:
                action = 'Opening' if value % 2 == 0 else 'Closing'
                width = 37.0 if value % 2 == 0 else 0.0
                self.get_logger().info(f'{action} gripper for value {value}.')
                if not self.grip_external(width=width):
                    self.get_logger().error(f'Failed to {action.lower()} gripper.')
                    return False
        return True

    def plan_to_pose(self, target_pose):
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = 'ur_manipulator'
        motion_plan_request.num_planning_attempts = 1000
        motion_plan_request.allowed_planning_time = 7.0
        motion_plan_request.max_velocity_scaling_factor = 0.2
        motion_plan_request.max_acceleration_scaling_factor = 0.2
        motion_plan_request.planner_id = "RRTConnectkConfigDefault"

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

def main():
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

if __name__ == '__main__':
    main()