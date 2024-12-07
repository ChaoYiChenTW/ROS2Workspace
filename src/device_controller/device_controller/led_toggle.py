"""
ROS 2 Node: LEDToggleNode

This script implements a ROS 2 node that controls an LED connected to a GPIO pin on a Raspberry Pi.
The LED's state is toggled based on messages received on the "button/press_status" topic.

Key Features:
- Subscribes to the "button/press_status" topic to receive Bool messages.
- Turns the LED on if the message data is `True` and off if it is `False`.
- Uses the gpiozero library to control the LED via the specified GPIO pin.
- Allows configuring the GPIO pin via a ROS 2 parameter ("gpio_pin").

Wiring Instructions:
- Connect the LED's positive leg to the specified GPIO pin on the Raspberry Pi (default: GPIO 13).
- Connect the LED's negative leg to a resistor and then to the Raspberry Pi's ground.

Usage:
- Ensure the LED is connected to the correct GPIO pin and ground as described above.
- Run the script as a ROS 2 node.
- Publish Bool messages to the "button/press_status" topic to control the LED's state.

Example Command:
$ ros2 run device_controller led_toggle_node --ros-args -p gpio_pin:=13

Dependencies:
- ROS 2 Python client library (`rclpy`)
- gpiozero for LED control
- std_msgs for Bool message type

Author: Chao-Yi Chen
Date: 2024Nov27
"""

# Modules for ROS 2
import rclpy  # ROS 2 Python client library.
from rclpy.node import Node  # Base class for creating ROS 2 nodes.
from std_msgs.msg import Bool  # Message type for subscription.

# Modules for hardware control
from gpiozero import LED  # Class for controlling GPIO-connected LEDs.


class LEDToggleNode(Node):
    """
    A ROS 2 node that controls an LED based on the state of messages
    received on the "button/press_status" topic.
    """

    def __init__(self):
        super().__init__("led_toggle_node")  # Initialize the node with a name.

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
    rclpy.init(args=args)  # Initialize the ROS 2 client library.
    node = LEDToggleNode()  # Create an instance of the LEDToggleNode.
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
