import rclpy 
import pygame
from rclpy.node import Node 
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from irc_interfaces.msg import Ps4 

class IrcPs4(Node): 
    def __init__(self):
        super().__init__("irc_ps4_node")
        self.get_logger().info("IrcPs4 node has been started.")

        # --------------------------
        # QoS profile for Wi-Fi data transmission
        # (Best Effort → faster and reliable over wireless)
        # # --------------------------
        # qos = QoSProfile(depth=10)
        # qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # --------------------------
        # Declare publishers with Best Effort QoS
        # --------------------------
        self.chassis = self.create_publisher(Ps4, "ps4_data_chassis", 10)
        self.arm = self.create_publisher(Ps4, "ps4_data_arm", 10)
        self.astro = self.create_publisher(Ps4, "ps4_data_astro", 10)

        # --------------------------
        # Timer for publishing data
        # --------------------------
        self.declare_parameter("timer_period", 0.05)
        self.timer_period_ = self.get_parameter("timer_period").value
        self.number_timer_ = self.create_timer(self.timer_period_, self.publish_ps_data)

        # --------------------------
        # Initialize joystick
        # --------------------------
        pygame.init()
        pygame.joystick.init()

        count = pygame.joystick.get_count()
        while count < 1:
            self.get_logger().warn("No joystick found! Please connect your PS4 controller.")
            pygame.time.wait(1000)
            count = pygame.joystick.get_count()

        self.joysticks = [pygame.joystick.Joystick(i) for i in range(count)]
        for js in self.joysticks:
            js.init()
            self.get_logger().info(f"Using joystick: {js.get_name()}")


    def ps_data(self, joystick_id=0):
        pygame.event.pump()

        js = self.joysticks[joystick_id]

        # Scale each axis from [-1, 1] → [-127, 127]
        axes = [int(round(axis * 127)) for axis in [js.get_axis(i) for i in range(js.get_numaxes())]]

        # Convert button values to boolean
        buttons = [bool(js.get_button(i)) for i in range(js.get_numbuttons())]

        return axes, buttons
        
        
    def publish_ps_data(self): 
        # axes_chassis, buttons_chassis = self.ps_data(0)
        axes_arm, buttons_arm = self.ps_data(0)

        # msg1 = Ps4()
        msg2 = Ps4()
        # Ensure correct sizes for your message fields
        # msg1.ps4_data_analog = axes_chassis[:7] + [0.0] * (7 - len(axes_chassis))   # pad if fewer than 7
        # msg1.ps4_data_buttons = buttons_chassis[:16] + [False] * (16 - len(buttons_chassis))  # pad if fewer than 16

        # self.chassis.publish(msg1) 
        # self.astro.publish(msg1) 
        # self.get_logger().info(f'Published Ps4 message: {msg1}')

        msg2.ps4_data_analog = axes_arm[:7] + [0.0] * (7 - len(axes_arm))   # pad if fewer than 7
        msg2.ps4_data_buttons = buttons_arm[:16] + [False] * (16 - len(buttons_arm))  # pad if fewer than 16

        self.arm.publish(msg2) 
        self.get_logger().info(f'Published Ps4 message: {msg2}')


                
def main(args=None): 
    rclpy.init(args=args) 
    node = IrcPs4() 
    rclpy.spin(node) 
    rclpy.shutdown() 
        
if __name__ == "__main__": 
    main()