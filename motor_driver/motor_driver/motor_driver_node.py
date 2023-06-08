import os
import yaml
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
from ament_index_python.packages import get_package_share_directory

@dataclass
class ODriveMotor():
    name : str
    serial_number : str
    controller : Any 
    control_mode : int
    gear_ratio : float

class MotorDriverNode(Node):
    
    def __init__(self):
        super().__init__('motor_driver')

        self.get_logger().info("Starting motor driver node.")

        # Load config
        self.declare_parameter('config', 'motors.yaml')
        config_name = self.get_parameter('config').get_parameter_value().string_value
        config_yaml = os.path.join(get_package_share_directory('motor_driver'), 'config', config_name)
        with open(config_yaml, 'r') as f:
            motors_dict = yaml.safe_load(f)
        if motors_dict is None:
            self.get_logger().error("Could not read parameters.")
            raise Exception
        
        # Read motors from config
        self.get_logger().info("Connecting motors...")
        self.motors : Dict[int, ODriveMotor] = {}
        self.motor_count = 0
        for motor_name, details in motors_dict.items():

            motor_id = int(details['id'])
            motor_sn = str(details['serial_number'])
            motor_mode = int(details['control_mode'])
            motor_gr = float(details['gear_ratio'])

            # Connect to motor
            self.get_logger().info(f"Searching for {motor_name} [SN: {motor_sn}] (ID: {motor_id}) ")
            motor_ctrl = odrive.find_any(serial_number=motor_sn)
            self.get_logger().info(f"Connected.")

            # Add to index map
            self.motors[motor_id] = ODriveMotor(
                name=motor_name,
                serial_number=motor_sn,
                controller=motor_ctrl,
                control_mode=motor_mode,
                gear_ratio=motor_gr
            )
            self.motor_count += 1

        # Initialize motors
        self.get_logger().info("Initializing motors...")
        for motor in self.motors.values():
            motor.controller.axis0.controller.input_pos = 0 # = zero position
            motor.controller.axis0.controller.input_vel = 0
            motor.controller.axis0.controller.input_torque = 0
            motor.controller.axis0.controller.config.control_mode = ControlMode(motor.control_mode)
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
        if len(self.motors) == 0:
            self.get_logger().warn("No motors attached.")
            return

        # Send position command to ODrive
        for idx, motor in self.motors.items():
            if len(msg.positions) == self.motor_count:
                motor.controller.axis0.controller.input_pos = msg.positions[idx] * motor.gear_ratio
            if len(msg.velocities) == self.motor_count:
                motor.controller.axis0.controller.input_vel = msg.velocities[idx] * motor.gear_ratio
            if len(msg.effort) == self.motor_count:
                motor.controller.axis0.controller.input_torque = msg.effort[idx] * motor.gear_ratio

        self.get_logger().info(f"Sent motor command to {motor.name}")


    # Publish motor info
    def publish_info(self):
        info_msg = JointTrajectoryPoint()
        info_msg.positions = [float(motor.controller.axis0.pos_vel_mapper.pos_abs) / motor.gear_ratio for motor in self.motors.values()]
        info_msg.velocities = [float(motor.controller.axis0.pos_vel_mapper.vel) / motor.gear_ratio  for motor in self.motors.values()]
        self.info_pub.publish(info_msg)

    # Put motors in idle state
    def idle(self):
        for motor in self.motors.values():
            motor.controller.axis0.requested_state = AxisState.IDLE

# ROS Entry
def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    except KeyboardInterrupt:
        pass
    node.idle()
    node.get_logger().info("Motor driver node closed.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()