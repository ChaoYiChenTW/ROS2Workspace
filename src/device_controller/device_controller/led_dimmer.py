"""
ROS 2 Node: LEDDimmerNode

This script implements a ROS 2 node that controls the brightness of an LED using PWM (Pulse Width Modulation).
The brightness level is determined by messages received on the "potentiometer/value" topic.

Key Features:
- Subscribes to the "potentiometer/value" topic to receive Float32 messages representing a normalized value (0.0 to 1.0).
- Uses the gpiozero library to control an LED's brightness via PWM.
- Allows configuring the GPIO pin for the LED via a ROS 2 parameter ("gpio_pin").
- Adjusts the LED brightness dynamically based on the received values.

Wiring Instructions:
- Connect the LED's positive leg to a GPIO pin on the Raspberry Pi capable of PWM (default: GPIO 13).
- Connect the LED's negative leg to a resistor and then to the Raspberry Pi's ground.

Dependencies:
1. ROS 2 Python client library (`rclpy`)
2. gpiozero for PWM LED control
3. std_msgs for Float32 message type
4. Proper PWM-capable GPIO pin on the Raspberry Pi (e.g., GPIO 13)
5. SPI enabled on the Raspberry Pi:
   - Enable SPI using `raspi-config` or appropriate configuration commands.
   - Ensure the current user has the necessary permissions to access SPI.

Usage:
- Ensure the LED is connected to a PWM-capable GPIO pin and ground as described above.
- Run the script as a ROS 2 node.
- Publish Float32 messages (values between 0.0 and 1.0) to the "potentiometer/value" topic to control the LED brightness.

Example Command:
$ ros2 run device_controller led_dimmer_node --ros-args -p gpio_pin:=13

Example Topic Publishing:
$ ros2 topic pub /potentiometer/value std_msgs/Float32 "data: 0.5"  # Sets LED to 50% brightness.

Author: Chao-Yi Chen
Date: 2024Dec07
"""

# Modules for ROS 2
import rclpy  # ROS 2 Python client library.
from rclpy.node import Node  # Base class for creating ROS 2 nodes.
from std_msgs.msg import Float32  # Message type for subscription.
from rclpy.qos import QoSProfile  # Custom QoS profile for publishers and subscribers.

# Modules for hardware control
from gpiozero import PWMLED  # Class for controlling GPIO-connected LEDs.


class LEDDimmerNode(Node):

    def __init__(self):
        super().__init__("led_dimmer_node")  # Initialize the node with a name.
        self.declare_parameter(
            "gpio_pin", 13
        )  # Declare a parameter for the GPIO pin connected to the LED, with a default value of 13.
        gpio_pin = self.get_parameter("gpio_pin").get_parameter_value().integer_value

        self.led = PWMLED(gpio_pin)
        qos_profile = QoSProfile(depth=10)

        self.subscription_ = self.create_subscription(
            Float32, "potentiometer/value", self.listener_callback, qos_profile
        )
        self.get_logger().info(f"LEDDimmerNode has been started on GPIO{gpio_pin}.")

    def listener_callback(self, msg):
        msg_value = msg.data
        if msg_value < 0.001:
            self.led.value = 0
        else:
            self.led.value = msg.data
        self.get_logger().info(f"Set LED brightness to {msg.data}.")


def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 client library
    node = LEDDimmerNode()  # Create an instance of the LEDDimmerNode class
    try:
        rclpy.spin(node)  # Spin the node so the callback function is called
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
