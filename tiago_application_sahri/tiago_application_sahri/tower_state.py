#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, String


class tower_state:
    """
    Class to represent the state of a Jenga tower.
    """

    def __init__(self):
        """Initialize the tower state with default values."""
        self._center = Point()
        self._center.x = 0.0
        self._center.y = 0.0
        self._center.z = 0.0
        self._num_blocks = 0

    def set_center(self, x, y, z):
        """
        Update the center position of the Jenga tower.

        Args:
            x (float): X coordinate of the center
            y (float): Y coordinate of the center
            z (float): Z coordinate of the center
        """
        self._center.x = float(x)
        self._center.y = float(y)
        self._center.z = float(z)

    def set_center_from_point(self, point):
        """
        Update the center position from a Point message.

        Args:
            point (geometry_msgs.msg.Point): Point message with center coordinates
        """
        self._center.x = point.x
        self._center.y = point.y
        self._center.z = point.z

    def get_center(self):
        """
        Get the current center position of the tower.

        Returns:
            geometry_msgs.msg.Point: Current center position
        """
        return self._center

    def set_num_blocks(self, num_blocks):
        """
        Update the number of Jenga blocks in the tower.

        Args:
            num_blocks (int): Number of blocks constituting the tower
        """
        self._num_blocks = max(0, int(num_blocks))  # Ensure non-negative

    def get_num_blocks(self):
        """
        Get the current number of blocks in the tower.

        Returns:
            int: Number of blocks in the tower
        """
        return self._num_blocks

    def add_block(self):
        """Add one block to the tower."""
        self._num_blocks += 1

    def remove_block(self):
        """Remove one block from the tower (minimum 0)."""
        self._num_blocks = max(0, self._num_blocks - 1)

    def get_state_info(self):
        """
        Get a formatted string with current tower state information.

        Returns:
            str: Formatted string with tower state
        """
        return (f"Tower State - Center: ({self._center.x:.2f}, {self._center.y:.2f}, "
                f"{self._center.z:.2f}), Blocks: {self._num_blocks}")


class TowerStateNode(Node):
    """ROS2 Node for managing Jenga tower state."""

    def __init__(self):
        super().__init__('tower_state_node')

        # Initialize tower state
        self.tower = tower_state()

        # Publishers
        self.center_publisher = self.create_publisher(Point, 'tower_center', 1)
        self.blocks_publisher = self.create_publisher(Int32, 'tower_blocks', 1)
        self.state_publisher = self.create_publisher(String, 'tower_state_info', 1)

        # Subscribers
        self.center_subscriber = self.create_subscription(
            Point, 'set_tower_center', self.center_callback, 1)
        self.blocks_subscriber = self.create_subscription(
            Int32, 'set_tower_blocks', self.blocks_callback, 1)

        # Timer for periodic state publishing
        self.timer = self.create_timer(1.0, self.publish_state)

        self.get_logger().info('Tower State Node initialized')

    def center_callback(self, msg):
        """Callback for updating tower center."""
        self.tower.set_center_from_point(msg)
        self.get_logger().info(f'Tower center updated: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})')

    def blocks_callback(self, msg):
        """Callback for updating number of blocks."""
        if msg.data == 1:
            self.tower.add_block()
            self.get_logger().info('One block added.')
        elif msg.data == -1:
            self.tower.remove_block()
            self.get_logger().info('One block removed.')
        else:
            self.tower.set_num_blocks(msg.data)
            self.get_logger().info(f'Tower blocks set to: {msg.data}')

    def publish_state(self):
        """Publish current tower state."""
        # Publish center
        center_msg = self.tower.get_center()
        self.center_publisher.publish(center_msg)

        # Publish block count
        blocks_msg = Int32()
        blocks_msg.data = self.tower.get_num_blocks()
        self.blocks_publisher.publish(blocks_msg)

        # Publish state info
        state_msg = String()
        state_msg.data = self.tower.get_state_info()
        self.state_publisher.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)

    tower_state_node = TowerStateNode()

    try:
        rclpy.spin(tower_state_node)
    except KeyboardInterrupt:
        pass
    finally:
        tower_state_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
