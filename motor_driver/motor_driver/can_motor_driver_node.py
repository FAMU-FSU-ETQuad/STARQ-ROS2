import os
import yaml
from typing import Dict
from typing import Any
from dataclasses import dataclass

# ODrive imports
from odrive.enums import ControlMode, AxisState

# ROS imports
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory

import motor_driver.can_functions as canfunc

@dataclass
class ODriveMotor():
    name : str
    id : int
    serial_number : str
    control_mode : int

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
        self.get_logger().info("Creating motors...")
        self.motors : list[ODriveMotor] = []
        self.motor_count = 0
        for motor_name, details in motors_dict.items():

            motor_id = int(details['id'])
            motor_sn = str(details['serial_number'])
            motor_mode = int(details['control_mode'])

            # Add to index map
            self.motors.append(ODriveMotor(
                name=motor_name,
                id=motor_id,
                serial_number=motor_sn,
                control_mode=motor_mode
            ))
            self.motor_count += 1

        # Initialize motors
        self.get_logger().info("Initializing motors...")
        for motor in self.motors:
            canfunc.set_position(motor.id, 0.0, 0.0, 0.0)
            canfunc.set_control_mode(motor.id, ControlMode(motor.control_mode))
            canfunc.set_state(motor.id, AxisState.CLOSED_LOOP_CONTROL)
            
        # ROS topics
        self.cmd_sub = self.create_subscription(JointTrajectoryPoint, '/motors/cmd', self.motors_cmd_callback, 10)
        self.info_pub = self.create_publisher(JointTrajectoryPoint, "/motors/info", 10)

        # Publish info timer
        publish_info_rate = 20 # Hz
        self.info_publish_timer = self.create_timer(1.0/publish_info_rate, self.publish_info)

        self.get_logger().info("Motor driver node initialized.")

    # Set motor state
    def motors_cmd_callback(self, msg : JointTrajectoryPoint):
        if not self.motors:
            self.get_logger().warn("No motors attached.")
            return

        def get_command_value(values, id):
            try:
                return values[id]
            except IndexError:
                return 0.0

        # Send position command to ODrive
        for motor in self.motors:
            id = motor.id
            control_mode = motor.control_mode
            position = get_command_value(msg.positions, id)
            velocity = get_command_value(msg.velocities, id)
            torque = get_command_value(msg.effort, id)

            if control_mode == ControlMode.POSITION_CONTROL:
                canfunc.set_position(id, position=position, velocity_ff=velocity, torque_ff=torque)
            elif control_mode == ControlMode.VELOCITY_CONTROL:
                canfunc.set_velocity(id, velocity=velocity, torque_ff=torque)
            elif control_mode == ControlMode.TORQUE_CONTROL:
                canfunc.set_torque(id, torque=torque)

            self.get_logger().info(f"Sent motor command to {motor.name}")



    # Publish motor info
    def publish_info(self):
        info_msg = JointTrajectoryPoint()
        for motor in self.motors:
            encoder_data = canfunc.get_encoder(motor.id)
            if encoder_data is None:
                self.get_logger().error("Could not read encoder data!")
                return
            info_msg.positions.insert(motor.id, float(encoder_data['Position']))
            info_msg.velocities.insert(motor.id, float(encoder_data['Velocity']))
        self.info_pub.publish(info_msg)

    # Put motors in idle state
    def idle(self):
        for motor in self.motors:
            canfunc.set_state(motor.id, AxisState.IDLE)

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