"""
ROS 2 Push Button Node

This script defines a ROS 2 node for monitoring the state of a push button 
connected to a Raspberry Pi GPIO pin. It publishes the button's state 
(pressed or released) to a ROS 2 topic, enabling other nodes to react to 
button interactions. The script uses the gpiozero library for hardware 
control and the rclpy library for ROS 2 communication.

Key Features:
- Monitors the push button state (pressed/released).
- Publishes the button's state to the "button/press_status" topic as a Bool message.
- Implements event-driven behavior using gpiozero's button event handlers.

Prerequisites:
- Raspberry Pi with gpiozero and rclpy installed.
- A push button connected to the specified GPIO pin.

How to Use:
1. Connect the button to the GPIO pin specified in the "gpio_pin" parameter (default is GPIO 26).
2. Run the script to start publishing button state messages to the ROS 2 topic.
3. Subscribe to the "button/press_status" topic to observe the button's behavior.

Author: Chao-Yi Chen
Date: 2024Nov27
"""

# Modules for ROS 2
import rclpy  # ROS 2 Python client library.
from rclpy.node import Node  # Base class for creating ROS 2 nodes.
from rclpy.qos import (
    QoSProfile,
)  # Quality of Service profile for publishers and subscribers.
from std_msgs.msg import Bool  # Standard ROS 2 message type for boolean data.

# Modules for hardware control
from gpiozero import Button  # GPIO control library for Raspberry Pi.


class PushButtonNode(Node):
    """
    A ROS 2 node for monitoring a push button's state and publishing it to a topic.
    """

    def __init__(self):
        # Initialize the node with the name "push_button_node".
        super().__init__("push_button_node")

        # Declare a parameter for the GPIO pin number, with a default value of 26.
        self.declare_parameter("gpio_pin", 26)

        # Retrieve the GPIO pin number from the parameter.
        gpio_pin = self.get_parameter("gpio_pin").get_parameter_value().integer_value

        # Create a Button object for the specified GPIO pin.
        self.button = Button(gpio_pin)

        # Assign event handlers for button press and release events.
        self.button.when_pressed = self.btn_pressed
        self.button.when_released = self.btn_released

        # Define a QoS profile for the publisher.
        qos_profile = QoSProfile(depth=10)

        # Create a publisher to publish Bool messages on the "button/press_status" topic.
        self.publisher_ = self.create_publisher(
            Bool, "button/press_status", qos_profile
        )

        # Log a message indicating that the node has started.
        self.get_logger().info(f"PushButtonNode has been started on GPIO{gpio_pin}.")

    def btn_pressed(self):
        """
        Callback function triggered when the button is pressed.
        Publishes the current button state.
        """
        self.publish_btn_status()

    def btn_released(self):
        """
        Callback function triggered when the button is released.
        Publishes the current button state.
        """
        self.publish_btn_status()

    def publish_btn_status(self):
        """
        Publishes the current state of the button (pressed or released) as a Bool message.
        Logs the button state to the console.
        """
        msg = Bool()
        msg.data = self.button.is_pressed  # Get the button's current state.
        self.publisher_.publish(msg)  # Publish the button state.
        # Log the button state for debugging purposes.
        self.get_logger().info(
            f"Button status: {'Pressed' if msg.data else 'Released'}"
        )


def main(args=None):
    """
    Main function to initialize the ROS 2 system, start the node,
    and keep it running until interrupted.
    """
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library.
    node = PushButtonNode()  # Create an instance of the PushButtonNode.

    try:
        rclpy.spin(node)  # Keep the node running and processing callbacks.
    except KeyboardInterrupt:
        pass  # Allow graceful shutdown on Ctrl+C.
    finally:
        node.destroy_node()  # Destroy the node to release resources.
        rclpy.shutdown()  # Shut down the ROS 2 system.


# Run the script only if it is executed directly.
if __name__ == "__main__":
    main()
