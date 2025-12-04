import rclpy
from rclpy.node import Node
from irc_interfaces.msg import Ps4

class IrcArm(Node):
    def __init__(self):
        super().__init__("irc_arm_node") #Modify name
        # self.declare_parameter("timer_period", 0.05) 
        # self.timer_period_ = self.get_parameter("timer_period").value 
        self.subscriber = self.create_subscription(
            Ps4, "ps4_data_arm", self.callback_arm_data, 10)
        self.get_logger().info("IrcArm node has been started.")
        # self.number_timer_ = self.create_timer(self.timer_period_, self.timer_period)

    def callback_arm_data(self, msg: Ps4):
        axes = msg.ps4_data_analog      # List of 7 floats
        buttons = msg.ps4_data_buttons  # List of 16 bools
        self.get_logger().info(f"Analog axes: {axes}")
        self.get_logger().info(f"Buttons: {buttons}")



def main(args=None):
    rclpy.init(args=args)
    node = IrcArm()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()