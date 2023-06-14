import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import serial

class BoomEncodersNode(Node):

    def __init__(self):
        super().__init__('boom_encoders_node')

        self.get_logger().info("Starting boom encoders node.")

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        sport = self.get_parameter('serial_port').get_parameter_value().string_value
        brate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.serial_port = serial.Serial(sport, brate)

        self.orientation_pub = self.create_publisher(Float32, '/boom/orientation', 10)
        self.tilt_pub = self.create_publisher(Float32, '/boom/tilt', 10)

        self.declare_parameter('publish_rate', 100) # in Hz
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().integer_value
        self.publish_timer = self.create_timer(1.0 / publish_rate, self.publish)

        self.declare_parameter('base_cycles_per_revolution', int(40000*128/16))
        self.declare_parameter('arm_cycles_per_revolution', int(4500*4*72/24))
        self.base_cpr = self.get_parameter('base_cycles_per_revolution').get_parameter_value().integer_value
        self.arm_cpr = self.get_parameter('arm_cycles_per_revolution').get_parameter_value().integer_value

        self.get_logger().info("Boom encoders node initialized.")

    def publish(self):

        # Ask Teensy for encoder info
        self.serial_port.write(('r').encode())

        # Read from serial
        orientation = float(self.serial_port.readline().decode())
        tilt = float(self.serial_port.readline().decode())

        # Convert to ROS type
        orientation_msg = Float32()
        orientation_msg.data = orientation / self.base_cpr
        tilt_msg = Float32()
        tilt_msg.data = tilt / self.arm_cpr

        # Publish messages
        self.orientation_pub.publish(orientation_msg)
        self.tilt_pub.publish(tilt_msg)

        #self.get_logger().info("Published encoder data.")
        

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
