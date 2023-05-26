import math
import time
from typing import Dict
from typing import Any
from dataclasses import dataclass

# ODrive imports
import odrive
from odrive.enums import ControlMode, AxisState

# ROS imports
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

@dataclass
class ODriveMotor():
    name : str
    serial_number : str
    controller : Any 

class MotorDriverNode(Node):
    
    def __init__(self):
        super().__init__('motor_driver_node')

        # Initialize motors from config
        self.declare_parameter('motors')
        self.motors : Dict[int, ODriveMotor]
        self.motor_count = 0
        for motor_name, details in self.get_parameter('motors').value.items():

            motor_id = int(details['id'])
            motor_sn = str(details['serial_number'])
            motor_ctrl = None
            while motor_ctrl is None:
                motor_ctrl = odrive.find_any(serial_number=motor_sn, timeout=1.0)
                self.get_logger().warn(f"Couldn't connect to motor {motor_name}. Trying again...")
                time.sleep(1.0)

            self.motors[motor_id] = ODriveMotor(
                name=motor_name,
                serial_number=motor_sn,
                controller=motor_ctrl
            )
            self.get_logger().info(f"Connected to {motor_name} [SN: {motor_sn}] (ID: {motor_id}) ")
            self.motor_count += 1

        # Set control mode
        self.declare_parameter('control_mode')
        self.control_mode = ControlMode(self.get_parameter('control_mode').value)
        for motor in self.motors.values():
            motor.controller.axis0.controller.config.control_mode = self.control_mode

        # Start the motors
        for motor in self.motors.values():
            motor.controller.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL

        # ROS topics
        self.cmd_sub = self.create_subscription(JointTrajectoryPoint, '/motors/cmd', self.motors_cmd_callback, 10)
        self.info_pub = self.create_publisher(JointTrajectoryPoint, "/motors/info", 10)

        # Publish info timer
        publish_info_rate = 20 # Hz
        self.info_publish_timer = self.create_timer(1.0/publish_info_rate, self.publish_info)

        self.get_logger().info("Motor driver node initialized.")

    # Set motor state
    def motors_cmd_callback(self, msg : JointTrajectoryPoint):

        # Check for no motors
        if not self.motors:
            self.get_logger().warn("No motors attached.")
            return

        # Send position command to ODrive
        for idx, motor in self.motors.items():
            if len(msg.positions) == self.motor_count:
                motor.axis0.controller.input_pos = msg.positions[idx]
            if len(msg.velocities) == self.motor_count:
                motor.axis0.controller.input_vel = msg.velocities[idx]
            if len(msg.effort) == self.motor_count:
                motor.axis0.controller.input_torque = msg.effort[idx]

        self.get_logger().info(f"Sent motor command to {motor.name}")


    # Publish motor info
    def publish_info(self):
        info_msg = JointTrajectoryPoint()
        info_msg.positions = [float(motor.controller.axis0.pos_vel_mapper.pos_abs) for motor in self.motors.values()]
        info_msg.velocities = [float(motor.controller.axis0.pos_vel_mapper.vel) for motor in self.motors.values()]
        self.info_pub.publish(info_msg)
        self.get_logger().info("Motor info published.")

    def close(self):
        # Shut off the motors
        for motor in self.motors.values():
            motor.controller.axis0.requested_state = AxisState.IDLE
        self.get_logger().info("Motor driver node closed.")

# ROS Entry
def main(args=None):
    motor_driver_node = None
    try:
        rclpy.init(args=args)
        motor_driver_node = MotorDriverNode()
        rclpy.spin(motor_driver_node)
    except Exception:
        if motor_driver_node is not None:
            motor_driver_node.close()
            motor_driver_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()