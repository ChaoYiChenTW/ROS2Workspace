# Modules for ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# Modules for hardwares
from gpiozero import LED


class LEDNode(Node):
    def __init__(self):
        super().__init__("led_node")

        self.declare_parameter("gpio_pin", 13)
        gpio_pin = self.get_parameter("gpio_pin").get_parameter_value().integer_value
        self.led = LED(gpio_pin)

        self.subscription_ = self.create_subscription(
            Bool, "button/press_status", self.listener_callback, 10
        )

        self.get_logger().info(f"LEDNode has been started on GPIO{gpio_pin}.")

    def listener_callback(self, msg):
        if msg.data:
            self.led.on()
            self.get_logger().info("Turn on the LED.")
        else:
            self.led.off()
            self.get_logger().info("Turn off the LED.")


def main(args=None):
    rclpy.init(args=args)
    node = LEDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
