import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(Pose, '/target_pose', 10)
        self.pose = Pose()
        self.pose.position.x = 0.4
        self.pose.position.y = 0.0
        self.pose.position.z = 0.45
        self.pose.orientation.x = 0.0
        self.pose.orientation.y = 0.0
        self.pose.orientation.z = 0.0
        self.pose.orientation.w = 1.0
        self.jenga_width = 0.025
        self.jenga_length = 0.075
        self.jenga_thickness = 0.015
        self.list_positions = [
            [0, 0], 
            [self.jenga_width * 2, 0], 
            [self.jenga_width,  - self.jenga_length/2 + self.jenga_width/2], 
            [self.jenga_width, self.jenga_length/2 - self.jenga_width/2]
        ]

        # Initialize pose list and current index
        self.pose_list = self.create_pose_list()
        self.current_pose_index = 0

    def create_pose_list(self):
        """Create and return a list of target poses"""
        poses = []

        # Pose 1
        pose1 = Pose()
        pose1.position.x = 0.5
        pose1.position.y = 0.0
        pose1.position.z = 0.5
        pose1.orientation.x = 0.0
        pose1.orientation.y = 0.0
        pose1.orientation.z = 0.0
        pose1.orientation.w = 1.0
        poses.append(pose1)

        # Pose 2
        pose2 = Pose()
        pose2.position.x = 1.0
        pose2.position.y = 0.5
        pose2.position.z = 0.5
        pose2.orientation.x = 0.0
        pose2.orientation.y = 0.0
        pose2.orientation.z = 0.707
        pose2.orientation.w = 0.707
        poses.append(pose2)

        # Pose 3
        pose3 = Pose()
        pose3.position.x = 0.0
        pose3.position.y = 1.0
        pose3.position.z = 0.3
        pose3.orientation.x = 0.0
        pose3.orientation.y = 0.0
        pose3.orientation.z = 1.0
        pose3.orientation.w = 0.0
        poses.append(pose3)

        return poses

    def timer_callback(self):
        # Get the current pose from the list
        current_pose = self.pose_list[self.current_pose_index]

        # Publish the current pose
        self.publisher_.publish(current_pose)
        self.get_logger().info(f'Publishing pose {self.current_pose_index + 1}/{len(self.pose_list)}: '
                              f'x={current_pose.position.x:.2f}, y={current_pose.position.y:.2f}, z={current_pose.position.z:.2f}')

        # Move to the next pose (cycle back to beginning when we reach the end)
        self.current_pose_index = (self.current_pose_index + 1) % len(self.pose_list)


    def basic_layer_logic(self):
        for i, pos in enumerate(self.list_positions):
            if i%2 == 0 and i != 0:
                self.pose.position.z += self.jenga_thickness
            self.pose.position.x = pos[0]
            self.pose.position.y = pos[1]

            self.publisher_.publish(self.pose)
            self.get_logger().info(f'Publishing pose at position: {pos}')
            rclpy.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()

    # Create timer in main function
    timer_period = 5.0  # seconds
    timer = node.create_timer(timer_period, node.timer_callback)

    rclpy.spin(node)
    node.basic_layer_logic()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
