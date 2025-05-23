from launch import LaunchDescription
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
import math

#This launch file makes the IK without collision avoidance and trajectory planning
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
            while not self.ik_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().info('Waiting for /compute_ik service...')
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
                '3': [-0.1, -0.2, 0.3, 2.6, 0.96, 0.848]
            }
            self.get_logger().info('JointTrajectoryPublisher initialized.')
            self.get_logger().info('Enter 1, 2, or 3 to publish poses. Enter q to quit.')

        def publish_trajectory(self, joint_angles):
            try:
                msg = JointTrajectory()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = ''
                msg.joint_names = self.joint_names

                point = JointTrajectoryPoint()
                point.positions = joint_angles
                point.velocities = []
                point.accelerations = []
                point.effort = []
                point.time_from_start.sec = 4
                point.time_from_start.nanosec = 0

                msg.points = [point]

                self.publisher_.publish(msg)
                self.get_logger().info(f'Published joint angles: {joint_angles}')
            except Exception as e:
                self.get_logger().error(f'Failed to publish trajectory: {str(e)}')

        def compute_ik(self, pose):
            """
            Call MoveIt's /compute_ik service to compute joint angles.
            Input: pose = [x, y, z, roll, pitch, yaw]
            Output: joint angles [theta1, theta2, ..., theta6] or None if failed
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
            target_pose.pose.orientation.x = quaternion[0]
            target_pose.pose.orientation.y = quaternion[1]
            target_pose.pose.orientation.z = quaternion[2]
            target_pose.pose.orientation.w = quaternion[3]

            # Create IK request
            ik_request = GetPositionIK.Request()
            ik_request.ik_request.group_name = 'ur_manipulator'  # Adjust if your planning group is different
            ik_request.ik_request.robot_state.joint_state.name = self.joint_names
            ik_request.ik_request.robot_state.joint_state.position = [0.0] * len(self.joint_names)  # Initial guess
            ik_request.ik_request.pose_stamped = target_pose
            ik_request.ik_request.timeout.sec = 1
            ik_request.ik_request.timeout.nanosec = 0

            # Call IK service
            try:
                future = self.ik_client.call_async(ik_request)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                if response and response.error_code.val == 1:  # SUCCESS
                    return list(response.solution.joint_state.position)
                else:
                    self.get_logger().error(f'IK failed: {response.error_code}')
                    return None
            except Exception as e:
                self.get_logger().error(f'IK service call failed: {str(e)}')
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
                joint_angles = node.compute_ik(node.poses[user_input])
                if joint_angles:
                    node.publish_trajectory(joint_angles)
                else:
                    node.get_logger().error(f'Failed to compute IK for pose {user_input}')
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