import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import serial

class BoomEncodersNode(Node):

    def __init__(self):
        super().__init__('boom_encoders_node')

        self.get_logger().info("Starting boom encoders node.")

        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)

        self.declare_parameter('boom_length', 5.0) # in meters
        self.boom_length = self.get_parameter('boom_length').get_parameter_value().double_value

        self.orientation_pub = self.create_publisher(Float32, '/boom/orientation', 10)
        self.height_pub = self.create_publisher(Float32, '/boom/height', 10)

        self.declare_parameter('publish_rate', 100) # in Hz
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.publish_timer = self.create_timer(1.0 / publish_rate, self.publish)

        self.get_logger().info("Boom encoders node initialized.")

    def publish(self):
        
        # Ask Teensy for encoder info
        self.serial_port.write(("0\n").encode())

        # Read from serial
        orientation = float(self.serial_port.readline().decode())
        height = float(self.serial_port.readline().decode())

        # Convert to ROS type
        orientation_msg = Float32()
        orientation_msg.data = orientation
        height_msg = Float32()
        height_msg.data = height

        # Publish messages
        self.orientation_pub.publish(orientation_msg)
        self.height_pub.publish(height_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = BoomEncodersNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    except KeyboardInterrupt:
        pass
    node.get_logger().info("Boom encoder node closed.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
