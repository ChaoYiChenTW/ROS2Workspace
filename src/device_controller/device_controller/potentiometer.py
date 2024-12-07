"""
ROS 2 Node: PotentiometerNode

This script implements a ROS 2 node to read analog input from a potentiometer 
connected to an MCP3008 Analog-to-Digital Converter (ADC) and publish the 
readings as ROS 2 messages. The node publishes two types of data:
1. Potentiometer value (normalized between 0 and 1).
2. Potentiometer voltage (calculated based on the ADC's reference voltage).

MCP3008 Wiring for Raspberry Pi:
1. MCP3008 Pins:
   - Pin 1-8 (CH0-CH7): Analog inputs for sensors (e.g., potentiometer).
   - Pin 9 (DGND): Connect to Raspberry Pi ground.
   - Pin 10 (CS/SHDN): Chip select, connect to GPIO 8 (CE0).
   - Pin 11 (DIN): SPI MOSI, connect to GPIO 10 (MOSI).
   - Pin 12 (DOUT): SPI MISO, connect to GPIO 9 (MISO).
   - Pin 13 (CLK): SPI clock, connect to GPIO 11 (SCLK).
   - Pin 14 (AGND): Connect to Raspberry Pi ground.
   - Pin 15 (VREF): Reference voltage, usually connected to 3.3V.
   - Pin 16 (VDD): Power supply, connect to 3.3V.

2. Raspberry Pi SPI Pins:
   - Ensure SPI is enabled in the Raspberry Pi configuration.
   - Connect the MCP3008 pins to the corresponding SPI GPIO pins as described above.

3. Notes:
   - The MCP3008 requires a stable 3.3V power supply for both VDD and VREF.
   - Ground connections (DGND and AGND) must be securely connected to the Raspberry Pi ground.
   - Ensure proper SPI configuration in your Raspberry Pi operating system before running the node.


Key Features:
- Uses the gpiozero library to interface with the MCP3008 ADC.
- Publishes potentiometer data on two separate ROS 2 topics:
  - "potentiometer/value": The normalized potentiometer value.
  - "potentiometer/voltage": The calculated potentiometer voltage.
- Allows configuring the MCP3008 channel via a ROS 2 parameter ("channel").
- Utilizes ROS 2 Quality of Service (QoS) profiles to manage communication reliability.

Usage:
- Ensure that the MCP3008 is connected to the Raspberry Pi and wired correctly.
- Run the script as a ROS 2 node.
- Subscribe to the "potentiometer/value" and "potentiometer/voltage" topics 
  to receive the published data.

Dependencies:
- ROS 2 Python client library (`rclpy`)
- gpiozero for hardware control
- std_msgs for Float32 message type

Example Command:
$ ros2 run <package_name> potentiometer_node --ros-args -p channel:=0

Author: Chao-Yi Chen
Date: 2024Nov27
"""
import rclpy  # ROS 2 Python client library.
from rclpy.node import Node  # Base class for creating ROS 2 nodes.
from rclpy.qos import (
    QoSProfile,
)  # Quality of Service profile for publishers and subscribers.
from std_msgs.msg import Float32  # Message type for publishing potentiometer readings.

# Modules for hardware control
from gpiozero import MCP3008  # Class for controlling MCP3008 ADC.

class PotentiometerNode(Node):
    def __init__(self);
        super().__init__("potentiometer_node")  # Initialize the node with the name "potentiometer_node".

        # Declare a parameter for the channel number of the MCP3008 ADC connected to the potentiometer.
        self.declare_parameter("channel", 0)

        # Retrieve the channel parameter's value.
        channel = self.get_parameter("channel").get_parameter_value().integer_value

        # Initialize the MCP3008 object with the specified channel.
        self.adc = MCP3008(channel=0, clock_pin=11, mosi_pin=10, miso_pin=9, select_pin=8)

        # Define a QoS profile for the publisher.
        qos_profile = QoSProfile(depth=10)

        # Create a publisher to publish Float32 messages on the "potentiometer/reading" topic.
        self.publisher_value = self.create_publisher(
            Float32, "potentiometer/value", qos_profile
        )
        self.publisher_voltage = self.create_publisher(
            Float32, "potentiometer/voltage", qos_profile
        )

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Log a message indicating the node has started and which channel is being used.
        self.get_logger().info(f"PotentiometerNode has been started on channel {channel}.")

    def timer_callback(self):
        self.publish_potentiometer_value()
        self.publish_potentiometer_voltage()
        self.get_logger().info("Published potentiometer readings.")

    def publish_potentiometer_value(self):
        """
        Publishes the current potentiometer value as a Float32 message.
        """
        potentiometer_value = self.adc.value
        msg = Float32()
        msg.data = potentiometer_value
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published potentiometer value: {potentiometer_value}")

    def publish_potentiometer_voltage(self):
        """
        Publishes the current potentiometer voltage as a Float32 message.
        """
        potentiometer_voltage = self.adc.voltage
        msg = Float32()
        msg.data = potentiometer_voltage
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published potentiometer voltage: {potentiometer_voltage}")


def main(args=None):
    """
    Main function to initialize the ROS 2 system, start the node,
    and keep it running until interrupted.
    """
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library.
    node = PotentiometerNode()  # Create an instance of the PotentiometerNode.

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