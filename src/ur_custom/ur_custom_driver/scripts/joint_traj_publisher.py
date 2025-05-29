#!/usr/bin/env python3

import rclpy
import math
import time
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.srv import GetPositionIK, ApplyPlanningScene, GetMotionPlan, GetCartesianPath
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, JointConstraint
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion, Pose
from shape_msgs.msg import SolidPrimitive, Plane
from moveit_msgs.msg import PlanningScene, CollisionObject
from onrobot_msgs.srv import GripExternal
from ur_custom_driver.srv import CameraProcess, ArucoPose
from scipy.spatial.transform import Rotation as R
import numpy as np

class JointTrajectoryPublisher(Node):
    # Initialize node
    def __init__(self):
        super().__init__('joint_traj_publisher')
        # Define node as publisher to other topics, services or actions
        self.publisher_ = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        # Define node as client to other topics, services or actions
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.planning_scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.gripper_client = self.create_client(GripExternal, '/grip_external')
        self.camera_client = self.create_client(CameraProcess, '/camera_process')
        self.aruco_client = self.create_client(ArucoPose, '/aruco_pose')

        for client, name in [
            (self.ik_client, '/compute_ik'), (self.planning_scene_client, '/apply_planning_scene'),
            (self.plan_client, '/plan_kinematic_path'), (self.cartesian_client, '/compute_cartesian_path'),
            (self.gripper_client, '/grip_external'), (self.camera_client, '/camera_process'),
            (self.aruco_client, '/aruco_pose')
                ]:
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f'Service {name} not available after 10s!')
                return
            self.get_logger().info(f'Service {name} connected.')

        # Define joint names and initialize joint positions
        # self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint','wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint','wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        # Hardcoded poses for camera to take pictures (x, y, z, roll, pitch, yaw)
        # self.station_pose = [0.025, -0.203, 0.366, -2.03, 0.014, 1.788] # for tool0
        # self.start_pose = [-0.192, 0.0217, 0.337, -2.052, -0.043, -2.632] # for tool0
        # self.start_pose = [0.125, -0.091, 0.263, -2.0541109, -0.0377662, 0.5078882] #Quat: -0.8257, -0.2244, 0.1143, 0.504
        # self.station_pose = [0.115, 0.233, 0.295, -2.0356742, 0.0105608, -1.3606299] #Quat: 0.658, -0.536, 0.3267, -0.4153
        # self.start_pose = [0.125, -0.091, 0.263, 0.507, -0.896, 2.418] #Quat: -0.8257, -0.2244, 0.1143, 0.504
        self.station_pose = [0.115, 0.233, 0.295, 1.209, 0.749, -1.759] #Quat: 0.658, -0.536, 0.3267, -0.4153
        self.start_pose = [-0.201, -0.322, 0.352, 2.813, -1.525, -2.79]

        # Points A, B, C RELATIVE to marker pose (x, y, z, roll, pitch, yaw) in marker frame
        # Marker frame: x down - y left - z out
        # Each number 0-9 is a possible output from the YOLO model
        # 0: Drone Empty - 1: Drone Full
        # 2: Station Top Left Empty - 3: Station Top Left Full
        # 4: Station Top Right Empty - 5: Station Top Right Full
        # 6: Station Bottom Left Empty - 7: Station Bottom Left Full
        # 8: Station Bottom Right Empty - 9: Station Bottom Right Full
        self.points = {
            0: {'A': [-0.102, 0.0, 0.1, math.pi/2, 0.0, 0.0], 'B': [-0.102, 0.0, 0.0017, math.pi/2, 0.0, 0.0], 'C': [-0.102, 0.0, 0.1, math.pi/2, 0.0, 0.0]},
            1: {'A': [-0.102, 0.0, 0.07, math.pi/2, 0.0, 0.0], 'B': [-0.102, 0.0, 0.02, math.pi/2, 0.0, 0.0], 'C': [-0.102, 0.0, 0.07, math.pi/2, 0.0, 0.0]},
            2: {'A': [-0.195, -0.022, 0.1, math.pi/2, 0.0, 0.0], 'B': [-0.195, -0.022, 0.0017, math.pi/2, 0.0, 0.0], 'C': [-0.195, -0.022, 0.1, math.pi/2, 0.0, 0.0]},
            3: {'A': [-0.195, -0.022, 0.07, math.pi/2, 0.0, 0.0], 'B': [-0.195, -0.022, 0.02, math.pi/2, 0.0, 0.0], 'C': [-0.195, -0.022, 0.07, math.pi/2, 0.0, 0.0]},
            4: {'A': [-0.195, 0.022, 0.1, math.pi/2, 0.0, 0.0], 'B': [-0.195, 0.022, 0.0017, math.pi/2, 0.0, 0.0], 'C': [-0.195, 0.022, 0.1, math.pi/2, 0.0, 0.0]},
            5: {'A': [-0.195, 0.022, 0.07, math.pi/2, 0.0, 0.0], 'B': [-0.195, 0.022, 0.02, math.pi/2, 0.0, 0.0], 'C': [-0.195, 0.022, 0.07, math.pi/2, 0.0, 0.0]},
            6: {'A': [-0.14, -0.022, 0.1, math.pi/2, 0.0, 0.0], 'B': [-0.14, -0.022, 0.0017, math.pi/2, 0.0, 0.0], 'C': [-0.14, -0.022, 0.1, math.pi/2, 0.0, 0.0]},
            7: {'A': [-0.14, -0.022, 0.07, math.pi/2, 0.0, 0.0], 'B': [-0.14, -0.022, 0.02, math.pi/2, 0.0, 0.0], 'C': [-0.14, -0.022, 0.07, math.pi/2, 0.0, 0.0]},
            8: {'A': [-0.14, 0.022, 0.1, math.pi/2, 0.0, 0.0], 'B': [-0.14, 0.022, 0.0017, math.pi/2, 0.0, 0.0], 'C': [-0.14, 0.022, 0.1, math.pi/2, 0.0, 0.0]},
            9: {'A': [-0.14, 0.022, 0.07, math.pi/2, 0.0, 0.0], 'B': [-0.14, 0.022, 0.02, math.pi/2, 0.0, 0.0], 'C': [-0.14, 0.022, 0.07, math.pi/2, 0.0, 0.0]}
        }

        # Apply collision objects: floor, station
        self.apply_collision_objects()

    # Function for applying collision area in the motion planning
    def apply_collision_objects(self):
        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        # Collision object floor
        floor_collision = CollisionObject()
        floor_collision.header.frame_id = 'base_link'
        floor_collision.id = 'floor'
        plane = Plane()
        plane.coef = [0.0, 0.0, 1.0, -0.06] # 6 cm safety distance from floor
        floor_collision.planes = [plane]
        floor_collision.operation = CollisionObject.ADD

        # COllision object: station
        station_collision = CollisionObject()
        station_collision.header.frame_id = 'world'
        station_collision.id = 'station'
        station_pose = Pose()
        station_pose.position.x = -0.4 * -1
        station_pose.position.y = -0.3 * -1
        station_pose.position.z = 0.21
        station_pose.orientation.w = 1.0
        station_primitive = SolidPrimitive()
        station_primitive.type = SolidPrimitive.BOX
        station_primitive.dimensions = [0.1, 0.12, 0.42]
        station_collision.primitives = [station_primitive]
        station_collision.primitive_poses = [station_pose]
        station_collision.operation = CollisionObject.ADD

        planning_scene.world.collision_objects = [floor_collision, station_collision]
        scene_request = ApplyPlanningScene.Request()
        scene_request.scene = planning_scene
        future = self.planning_scene_client.call_async(scene_request)
        rclpy.spin_until_future_complete(self, future)
        if not future.result() or not future.result().success:
            self.get_logger().error('Failed to apply planning scene.')
            return False
        self.get_logger().info('Applied collision objects.')
        return True

    # Function for moving gripper to desired width
    def grip_external(self, width, force=50, speed=10, is_wait=True):
        request = GripExternal.Request()
        request.index = 0
        request.width = width
        request.force = force
        request.speed = speed
        request.is_wait = is_wait
        future = self.gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if not future.result():
            self.get_logger().error('Gripper service failed')
            return False
        self.get_logger().info(f'Gripper set to width={width}mm, force={force}N')
        return True

    #Function for publishing trajectory
    def publish_trajectory(self, trajectory):
        if not trajectory or not trajectory.points:
            self.get_logger().error('Invalid or empty trajectory in publish_trajectory')
            return False
        trajectory.joint_names = self.joint_names
        self.publisher_.publish(trajectory)
        # Wait until the motion is completed to continue
        duration = trajectory.points[-1].time_from_start.sec + trajectory.points[-1].time_from_start.nanosec / 1e9
        time.sleep(duration + 1.0)
        return True

    # Function for rotating the base_link coordinate system 180 degreed around the z axis
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

    # Function to obtain quaternion from roll, pitch, yaw angles
    def rpy_to_quaternion(self, roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return [x, y, z, w]

    # Define call for the camera service that takes a picture and processes the YOLO model
    def call_camera_service(self):
        request = CameraProcess.Request()
        future = self.camera_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if not future.result():
            self.get_logger().error('Camera service failed')
            return -1, -1
        value1, value2 = future.result().value1, future.result().value2
        if value1 == -1 and value2 == -1:
            self.get_logger().error('Invalid camera response: both values are 0')
            return -1, -1
        return value1, value2

    # Define call for aruco service, using the robot pose as an input
    def call_aruco_service(self, robot_pose):
        request = ArucoPose.Request()
        request.x, request.y, request.z = robot_pose[0], robot_pose[1], robot_pose[2]
        request.roll, request.pitch, request.yaw = robot_pose[3], robot_pose[4], robot_pose[5]
        future = self.aruco_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res and res.success:
            return [res.x, res.y, res.z, res.roll, res.pitch, res.yaw]
        self.get_logger().error('ArUco service failed')
        return None

    # Define function to obtain target pose from marker pose (obtained by aruco service)
    # and offset pose (hardcoded relative to aruco for each point A, B, C for each 0-9 value)
    def transform_pose(self, marker_pose, offset_pose):
        marker_rot = R.from_euler('xyz', [marker_pose[3], marker_pose[4], marker_pose[5]])
        marker_mat = np.eye(4)
        marker_mat[0:3, 0:3] = marker_rot.as_matrix()
        marker_mat[0:3, 3] = marker_pose[0:3]
        offset_rot = R.from_euler('xyz', [offset_pose[3], offset_pose[4], offset_pose[5]])
        offset_mat = np.eye(4)
        offset_mat[0:3, 0:3] = offset_rot.as_matrix()
        offset_mat[0:3, 3] = offset_pose[0:3]
        target_mat = marker_mat @ offset_mat
        target_pos = target_mat[0:3, 3]
        target_rot = R.from_matrix(target_mat[0:3, 0:3])
        target_euler = target_rot.as_euler('xyz')
        return [target_pos[0], target_pos[1], target_pos[2], target_euler[0], target_euler[1], target_euler[2]]

    # Define function to go to a target pose using joint trajectory
    def plan_to_pose(self, target_pose):
        # Define parameters for motion planning
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = 'ur_manipulator'
        motion_plan_request.num_planning_attempts = 1000
        motion_plan_request.allowed_planning_time = 7.0
        motion_plan_request.max_velocity_scaling_factor = 0.2
        motion_plan_request.max_acceleration_scaling_factor = 0.2
        motion_plan_request.planner_id = "RRTConnectkConfigDefault"

        # Position constrains for precision
        constraints = Constraints()
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'base_link'
        position_constraint.link_name = 'tool_tip'
        position_constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
        position_constraint.constraint_region = BoundingVolume()
        position_constraint.constraint_region.primitive_poses = [target_pose.pose]
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.001]
        position_constraint.constraint_region.primitives = [primitive]
        constraints.position_constraints = [position_constraint]

        # Orientation constrains for precision
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'base_link'
        orientation_constraint.link_name = 'tool_tip'
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.005
        orientation_constraint.absolute_y_axis_tolerance = 0.005
        orientation_constraint.absolute_z_axis_tolerance = 0.005
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints = [orientation_constraint]

        # Apply position and orientation constrains
        motion_plan_request.goal_constraints = [constraints]

        # Call for motion plan
        plan_request = GetMotionPlan.Request()
        plan_request.motion_plan_request = motion_plan_request
        future = self.plan_client.call_async(plan_request)
        rclpy.spin_until_future_complete(self, future)
        if not future.result() or future.result().motion_plan_response.error_code.val != 1:
            self.get_logger().error('Planning failed in plan to pose')
            return None

        #Return joint trajectory from motion plan
        return future.result().motion_plan_response.trajectory.joint_trajectory

    # Define function to go to a target pose using linear trajectory
    def plan_linear_to_pose(self, start_pose, goal_pose):
        cartesian_request = GetCartesianPath.Request()
        cartesian_request.header.frame_id = 'base_link'
        cartesian_request.header.stamp = self.get_clock().now().to_msg()
        cartesian_request.group_name = 'ur_manipulator'
        cartesian_request.link_name = 'tool_tip'
        cartesian_request.max_step = 0.0005
        cartesian_request.waypoints = [start_pose.pose, goal_pose.pose]
        cartesian_request.max_velocity_scaling_factor = 0.2
        cartesian_request.max_acceleration_scaling_factor = 0.2
        cartesian_request.jump_threshold = 0.0
        future = self.cartesian_client.call_async(cartesian_request)
        rclpy.spin_until_future_complete(self, future)
        if not future.result() or future.result().fraction < 0.8:
            self.get_logger().error(f'Cartesian planning failed, fraction={future.result().fraction}')
            return None
        # Return linear trajectory from motion plan
        return future.result().solution.joint_trajectory

    # Function for executing sequence, depending on:
    # 1) Yolo models value, which will define the relative poses to the aruco marker and also open or close gripper
    # 2) Marker's pose, to calculate the target poses from the hardcoded relative poses to the marker (A, B, C for each yolo value 0-9)
    def execute_sequence(self, value, marker_pose):
        # Check if value is between 0 and 9
        if value not in self.points:
            self.get_logger().error(f"Invalid value: {value}")
            return False
        # Assign A, B, C point from value (hardcoded)
        points = self.points[value]

        # Compute target pose for A point, from relative pose to marker (hardcoded), marker pose and 180 degree rotation in base frame
        target_a = self.compute_pose(self.transform_pose(marker_pose, points['A']))
        # Calculate joint trajectory to get to target pose A from current
        traj_a = self.plan_to_pose(target_a)
        if not self.publish_trajectory(traj_a):
            return False

        # Compute target pose for B point, from relative pose to marker (hardcoded), marker pose and 180 degree rotation in base frame
        target_b = self.compute_pose(self.transform_pose(marker_pose, points['B']))
        # Calculate linear trajectory to get to target pose B from A
        traj_b = self.plan_linear_to_pose(target_a, target_b)
        if not self.publish_trajectory(traj_b):
            return False

        # Check if value is even, for the gripper.
        # If is even, then the hole is empty and the gripper will deploy a battery. So gripper opens
        # If is odd, then the hole is full and the gripper will pick the battery. So gripper closes
        is_even = value % 2 == 0
        width = 37.0 if is_even else 0.0
        if not self.grip_external(width=width):
            return False

        # Compute target pose for C point, from relative pose to marker (hardcoded), marker pose and 180 degree rotation in base frame
        target_c = self.compute_pose(self.transform_pose(marker_pose, points['C']))
        # Calculate linear trajectory to get to target pose C from B
        traj_c = self.plan_linear_to_pose(target_b, target_c)
        if not self.publish_trajectory(traj_c):
            return False

        return True

    # Define main sequence
    def main_sequence(self):

        # Step 0: open gripper
        if not self.grip_external(width=37.0):
            return False

        # Step 1: Move to start_pose, call services: see where the drone is, confirm it has a battery
        start_pose = self.compute_pose(self.start_pose)
        if not self.publish_trajectory(self.plan_to_pose(start_pose)):
            return False
        value1, value2 = self.call_camera_service()
        if value1 < 0 and value2 < 0:
            self.get_logger().error("Camera service failed in step 1")
            return False
        odd_value = value1 if value1 >= 0 and value1 % 2 == 1 else -1
        marker_pose = self.call_aruco_service(self.start_pose)
        if marker_pose is None:
            return False

        # Step 2: Sequence for odd value. Close gripper: pick up the battery
        if odd_value >= 0 and odd_value in range(1, 10, 2):
            if not self.execute_sequence(odd_value, marker_pose):
                return False
        else:
            self.get_logger().error("No valid odd value in step 2")
            return False

        # Step 3: Move to station_pose, call services: see with hole is empty (to deploy) and which full (to pick new battery)
        station_pose = self.compute_pose(self.station_pose)
        if not self.publish_trajectory(self.plan_to_pose(station_pose)):
            return False
        value1, value2 = self.call_camera_service()
        if value1 < 0 or value2 < 0:
            self.get_logger().error("Camera service failed in step 3")
            return False
        odd_value = value1 if value1 % 2 == 1 else -1
        even_value = value2 if value2 % 2 == 0 else -1
        marker_pose = self.call_aruco_service(self.station_pose)
        if marker_pose is None:
            return False

        # Step 4: Sequences for even then odd value
        if even_value in range(0, 10, 2) and odd_value in range(1, 10, 2):
            # Sequence for even value. Open gripper: deploy the battery
            if not self.execute_sequence(even_value, marker_pose):
                return False
            # Sequence for odd value. Close gripper: pick up new battery
            if not self.execute_sequence(odd_value, marker_pose):
                return False
        else:
            self.get_logger().error("Invalid even/odd values in step 4")
            return False

        # Step 5: Repeat step 1. Check drone position, confirm its empty to deploy new battery
        if not self.publish_trajectory(self.plan_to_pose(start_pose)):
            return False
        value1, value2 = self.call_camera_service()
        if value1 < 0 and value2 < 0:
            self.get_logger().error("Camera service failed in step 5")
            return False
        even_value = value2 if value2 >= 0 and value2 % 2 == 0 else -1
        marker_pose = self.call_aruco_service(self.start_pose)
        if marker_pose is None:
            return False

        # Step 6: Sequence for even value. Open gripper: deploy new battery
        if even_value >= 0 and even_value in range(0, 10, 2):
            if not self.execute_sequence(even_value, marker_pose):
                return False
        else:
            self.get_logger().error("No valid even value in step 6")
            return False

        # Step 7: Return to start_pose
        return self.publish_trajectory(self.plan_to_pose(start_pose))

def main():
    rclpy.init()
    node = JointTrajectoryPublisher()
    try:
        time.sleep(8)
        if not node.main_sequence():
            node.get_logger().error('Sequence execution failed.')

    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()