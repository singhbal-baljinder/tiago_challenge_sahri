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
        self.pose.position.z = 1.2
        self.pose.orientation.x = 0.0
        self.pose.orientation.y = -np.pi / 2
        self.pose.orientation.z = 0.0
        self.pose.orientation.w = 1.0
        self.action_counter = 0
        self.jenga_width = 0.025
        self.jenga_length = 0.075
        self.jenga_thickness = 0.015
        self.table_height = 0.45
        self.home = [0.4, 0.0, 1.]
        self.jenga_tower_z_displacement = -self.jenga_thickness
        self.list_positions = [
            self.home,
            [0.4, 0.],
            self.home,
            [0.4 + self.jenga_width * 2, 0.],
            self.home,
            [0.4 + self.jenga_width,  - self.jenga_length/2 + self.jenga_width/2],
            self.home,
            [0.4 + self.jenga_width, self.jenga_length/2 - self.jenga_width/2],
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
        pose2.position.x = 0.40
        pose2.position.y = 0.1
        pose2.position.z = 0.5
        pose2.orientation.x = 0.0
        pose2.orientation.y = 0.0
        pose2.orientation.z = 0.0
        pose2.orientation.w = 1.0
        poses.append(pose2)

        # Pose 3
        pose3 = Pose()
        pose3.position.x = 0.45
        pose3.position.y = 0.1
        pose3.position.z = 0.45
        pose3.orientation.x = 0.0
        pose3.orientation.y = 0.0
        pose3.orientation.z = 0.0
        pose3.orientation.w = 1.0
        poses.append(pose3)

        return poses

    def timer_callback(self):
        # Get the current pose from the list
        current_cmd_index = self.action_counter % len(self.list_positions)
        pos = self.list_positions[current_cmd_index]
        if current_cmd_index % 4 == 1:
            self.jenga_tower_z_displacement += self.jenga_thickness
        if current_cmd_index % 2 == 0:
            self.pose.position.z = pos[2]
        else:
            self.pose.position.z = self.table_height + self.jenga_tower_z_displacement
        self.pose.position.x = pos[0]
        self.pose.position.y = pos[1]
        self.pose.position.y = pos[1]

        # Publish the current pose
        self.publisher_.publish(self.pose)
        self.get_logger().info(
            f'Publishing pose {self.current_pose_index + 1}/{len(self.list_positions)}: '
            f'x={self.pose.position.x:.3f}, y={self.pose.position.y:.3f}, z={self.pose.position.z:.3f}: '
            f'ox={self.pose.orientation.x:.3f}, oy={self.pose.orientation.y:.3f}, oz={self.pose.orientation.z:.3f}'
        )

        # Move to the next pose (cycle back to beginning when we reach the end)
        self.current_pose_index = (self.current_pose_index + 1) % len(self.list_positions)
        self.action_counter += 1

        # Stop at some point
        # if self.pose.position.z >= self.home[2]:
        #     stop

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()

    # Create timer in main function
    timer_period = 5.0  # seconds
    timer = node.create_timer(timer_period, node.timer_callback)

    rclpy.spin(node)
    # node.basic_layer_logic()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
