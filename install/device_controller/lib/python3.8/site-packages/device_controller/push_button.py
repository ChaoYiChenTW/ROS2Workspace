# Modules for ROS 2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool

# Modules for hardwares
from gpiozero import Button

class PushButtonNode(Node):
    def __init__(self):
        super().__init__("push_button_node")

        self.declare_parameter("gpio_pin", 12)
        gpio_pin = self.get_parameter("gpio_pin").get_parameter_value().integer_value
        self.button = Button(gpio_pin)
        self.button.when_pressed = self.btn_pressed
        self.button.when_released = self.btn_released

        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(Bool, "button/press_status", qos_profile)

        self.get_logger().info(f"PushButtonNode has been started on GPIO{gpio_pin}.")

    def btn_pressed(self):
        self.publish_btn_status()

    def btn_released(self):
        self.publish_btn_status()

    def publish_btn_status(self):
        msg = Bool()
        msg.data = self.button.is_pressed
        self.publisher_.publish(msg)
        self.get_logger().info(f"Button status: {'Pressed' if msg.data else 'Released'}")


def main(args=None):
    rclpy.init(args=args)
    node = PushButtonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
