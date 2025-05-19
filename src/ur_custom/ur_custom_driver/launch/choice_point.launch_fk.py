from launch import LaunchDescription
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

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
            self.joint_names = [
                'shoulder_pan_joint',
                'shoulder_lift_joint',
                'elbow_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint'
            ]
            self.poses = {
                '0': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0], #Top home position
                '1': [0.785, -1.57, 0.785, 0.785, 0.785, 0.785],  # Pose from example
                '2': [0.0, -1.0, 0.5, 0.0, 0.0, 0.0],           # Neutral pose
                '3': [1.57, -2.0, 1.0, 1.0, 1.0, 1.0]           # Another pose
            }
            self.get_logger().info('JointTrajectoryPublisher initialized.')
            self.get_logger().info('Enter 1, 2, or 3 to publish poses. Enter q to quit.')

        def publish_trajectory(self, positions):
            try:
                msg = JointTrajectory()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = ''
                msg.joint_names = self.joint_names

                point = JointTrajectoryPoint()
                point.positions = positions
                point.velocities = []
                point.accelerations = []
                point.effort = []
                point.time_from_start.sec = 4
                point.time_from_start.nanosec = 0

                msg.points = [point]

                self.publisher_.publish(msg)
                self.get_logger().info(f'Published trajectory: {positions}')
            except Exception as e:
                self.get_logger().error(f'Failed to publish trajectory: {str(e)}')

    # Initialize rclpy and run the node inline
    rclpy.init()
    node = JointTrajectoryPublisher()
    try:
        while rclpy.ok():
            user_input = input("Enter 0, 1, 2, 3, or q: ")
            if user_input in node.poses:
                node.publish_trajectory(node.poses[user_input])
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