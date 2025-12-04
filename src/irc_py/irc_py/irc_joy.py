import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class IrcJoy(Node):
    def __init__(self):
        super().__init__('irc_joy')
        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.axes_list = []
        self.buttons_list = []

    def joy_callback(self, msg):
        # Store axes and buttons in lists
        self.axes_list = list(msg.axes)
        self.buttons_list = list(msg.buttons)

        # Optional: print for debug
        self.get_logger().info(f"Axes: {self.axes_list}")
        self.get_logger().info(f"Buttons: {self.buttons_list}")

def main(args=None):
    rclpy.init(args=args)
    node = IrcJoy()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
