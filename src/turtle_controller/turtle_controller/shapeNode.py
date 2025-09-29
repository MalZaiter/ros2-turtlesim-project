#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class ShapeNode(Node):
    def __init__(self):
        super().__init__('shape_node')
        self.publisher_ = self.create_publisher(Int32, '/selected_shape', 10)
        self.timer = self.create_timer(1.0, self.get_user_input)

    def get_user_input(self):
        print("\nChoose a command:")
        print("1. Lissajous")
        print("2. Epicycloid")
        print("3. Torus Knot")
        print("4. Clear Screen")
        print("5. Stop Drawing\n")

        try:
            choice = int(input("Enter number: "))
            msg = Int32()
            msg.data = choice
            self.publisher_.publish(msg)
            self.get_logger().info(f"Sent command {choice}")
        except ValueError:
            self.get_logger().warn("Invalid input, please enter a number 1-5.")

def main(args=None):
    rclpy.init(args=args)
    node = ShapeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()