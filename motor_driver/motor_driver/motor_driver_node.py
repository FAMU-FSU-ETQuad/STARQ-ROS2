import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

from motor_driver.moteus_driver import MoteusDriver

PUBLISH_INFO_RATE = 20 # Hz

class MotorDriverNode(Node):
    
    def __init__(self):
        super().__init__('motor_driver_node')

        # New moteus driver object
        # Eventually convert this to read motor setup from config file
        self.moteus_driver = MoteusDriver([9, 10])
        #self.moteus_driver.set_zero_positions([0.0 0.5])

        # Start moteus update loop (New thread)
        self.moteus_driver.start()

        # Subscribers
        self.cmd_sub = self.create_subscription(JointTrajectoryPoint, '/motors/cmd', self.motors_cmd_callback, 10)

        # Publishers
        self.info_position_pub = self.create_publisher(Float32MultiArray, '/motors/info/position', 10)
        self.info_velocity_pub = self.create_publisher(Float32MultiArray, '/motors/info/velocity', 10)
        self.info_torque_pub = self.create_publisher(Float32MultiArray, '/motors/info/torque', 10)
        self.info_temperature_pub = self.create_publisher(Float32MultiArray, '/motors/info/temperature', 10)
        self.info_mode_pub = self.create_publisher(Int32MultiArray, '/motors/info/mode', 10)
        self.info_fault_pub = self.create_publisher(Int32MultiArray, '/motors/info/fault', 10)

        # Timers
        self.info_publish_timer = self.create_timer(1/PUBLISH_INFO_RATE, self.publish_info)

        self.get_logger().info("Motor driver node initialized.")

    # Set motor state
    def motors_cmd_callback(self, msg : JointTrajectoryPoint):
        if len(msg.positions) != 0:
            self.moteus_driver.set_positions(msg.positions)
        if len(msg.velocities) != 0:
            self.moteus_driver.set_velocities(msg.velocities)
        self.get_logger().info("Motor state updated.")

    # Publish motor info
    def publish_info(self):
        self.info_position_pub.publish(Float32MultiArray(data=self.moteus_driver.get_motor_positions()))
        self.info_velocity_pub.publish(Float32MultiArray(data=self.moteus_driver.get_motor_velocities()))
        self.info_torque_pub.publish(Float32MultiArray(data=self.moteus_driver.get_motor_torques()))
        self.info_temperature_pub.publish(Float32MultiArray(data=self.moteus_driver.get_motor_temperatures()))
        self.info_mode_pub.publish(Int32MultiArray(data=self.moteus_driver.get_motor_modes()))
        self.info_fault_pub.publish(Int32MultiArray(data=self.moteus_driver.get_motor_faults()))
        self.get_logger().info("Motor info published.")

    def close(self):
        self.moteus_driver.close()
        self.get_logger().info("Motor driver node closed.")

# ROS Entry
def main(args=None):
    try:
        rclpy.init(args=args)
        motor_driver_node = MotorDriverNode()
        rclpy.spin(motor_driver_node)
    except KeyboardInterrupt:
        motor_driver_node.close()
    finally:
        motor_driver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()