
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(Pose, '/target_pose', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pose = Pose()
        self.pose.position.x = 0.0
        self.pose.position.y = 0.0
        self.pose.position.z = 0.1
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

    def timer_callback(self):
        self.publisher_.publish(self.pose)
        self.get_logger().info('Publishing constant target pose')

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
    rclpy.spin(node)
    node.basic_layer_logic()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
