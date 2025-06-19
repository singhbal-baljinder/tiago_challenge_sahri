
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(Pose, '/target_pose', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pose = Pose()
        self.pose.position.x = 0.5
        self.pose.position.y = 0.0
        self.pose.position.z = 0.5
        self.pose.orientation.x = 0.0
        self.pose.orientation.y = 0.0
        self.pose.orientation.z = 0.0
        self.pose.orientation.w = 1.0

    def timer_callback(self):
        self.publisher_.publish(self.pose)
        self.get_logger().info('Publishing constant target pose')

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
