"""
ROS 2 LED Control Node

This script defines a ROS 2 node (`LEDNode`) that subscribes to a topic
and controls an LED connected to a GPIO pin on a Raspberry Pi. The node
listens to messages of type `Bool` published on the "button/press_status"
topic. Based on the received message, the LED is turned on or off.

Key Features:
- Configurable GPIO pin for the LED via a ROS 2 parameter.
- Subscribes to a topic to control the LED's state (on/off).
- Logs LED state changes for monitoring purposes.

Prerequisites:
- A Raspberry Pi with ROS 2 installed.
- An LED connected to a GPIO pin (with a current-limiting resistor).
- Another ROS 2 node publishing `Bool` messages on the "button/press_status" topic.

How to Use:
1. Connect the LED to the Raspberry Pi GPIO pin as specified in the parameter.
2. Ensure another ROS 2 node publishes `Bool` messages on "button/press_status".
3. Run this script using `ros2 run` and observe the LED's behavior.

Author: Chao-Yi Chen
Date: 2024Nov27
"""

# Modules for ROS 2
import rclpy  # ROS 2 Python client library.
from rclpy.node import Node  # Base class for creating ROS 2 nodes.
from std_msgs.msg import Bool  # Message type for subscription.

# Modules for hardware control
from gpiozero import LED  # Class for controlling GPIO-connected LEDs.


class LEDNode(Node):
    """
    A ROS 2 node that controls an LED based on the state of messages
    received on the "button/press_status" topic.
    """

    def __init__(self):
        super().__init__("led_node")  # Initialize the node with the name "led_node".

        # Declare a parameter for the GPIO pin connected to the LED, with a default value of 13.
        self.declare_parameter("gpio_pin", 13)

        # Retrieve the GPIO pin parameter's value.
        gpio_pin = self.get_parameter("gpio_pin").get_parameter_value().integer_value

        # Initialize the LED object with the specified GPIO pin.
        self.led = LED(gpio_pin)

        # Create a subscription to the "button/press_status" topic.
        # This subscription listens for Bool messages and calls `listener_callback` when a message is received.
        self.subscription_ = self.create_subscription(
            Bool, "button/press_status", self.listener_callback, 10
        )

        # Log a message indicating the node has started and which GPIO pin is being used.
        self.get_logger().info(f"LEDNode has been started on GPIO{gpio_pin}.")

    def listener_callback(self, msg):
        """
        Callback function triggered when a message is received on "button/press_status".
        Controls the LED based on the message's data field.
        """
        if msg.data:
            self.led.on()  # Turn the LED on if the message data is True.
            self.get_logger().info("Turn on the LED.")  # Log the LED state change.
        else:
            self.led.off()  # Turn the LED off if the message data is False.
            self.get_logger().info("Turn off the LED.")  # Log the LED state change.


def main(args=None):
    """
    Entry point for the script. Initializes the ROS 2 environment,
    creates the LEDNode, and spins it to keep it alive.
    """
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library.
    node = LEDNode()  # Create an instance of the LEDNode.
    try:
        rclpy.spin(node)  # Keep the node running, listening for messages.
    except KeyboardInterrupt:
        pass  # Gracefully handle keyboard interruption (Ctrl+C).
    finally:
        node.destroy_node()  # Destroy the node instance before shutdown.
        rclpy.shutdown()  # Shutdown the ROS 2 client library.


# Check if the script is being run directly (not imported as a module).
if __name__ == "__main__":
    main()
