import os
import yaml
from dataclasses import dataclass

# ODrive imports
from odrive.enums import AxisState

# ROS imports
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory

import motor_driver.can_functions as canfunc

@dataclass
class ODriveMotor():
    name : str
    id : int
    control_mode : int
    gear_ratio : float
    can_id : int
    gains : list[float]
    max_velocity : float
    max_current : float

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
            motor_mode = int(details['control_mode'])
            motor_gr = float(details['gear_ratio'])
            motor_can_id = int(details['can_id'])
            motor_pgain = float(details['pos_gain'])
            motor_vgain = float(details['vel_gain'])
            motor_again = float(details['acc_gain'])
            motor_vlim = float(details['velocity_limit'])
            motor_clim = float(details['current_limit'])

            # Add to index map
            self.motors.append(ODriveMotor(
                name=motor_name,
                id=motor_id,
                control_mode=motor_mode,
                gear_ratio=motor_gr,
                can_id=motor_can_id,
                gains=[motor_pgain, motor_vgain, motor_again],
                max_velocity=motor_vlim,
                max_current=motor_clim
            ))
            self.motor_count += 1

        # Initialize motors
        self.get_logger().info("Initializing motors...")
        for motor in self.motors:
            canfunc.clear_errors(motor.can_id)
            canfunc.set_control_mode(motor.can_id, motor.control_mode)
            canfunc.set_state(motor.can_id, int(AxisState.CLOSED_LOOP_CONTROL))
            canfunc.set_gains(motor.can_id, motor.gains[0], motor.gains[1], motor.gains[2])
            
        # ROS topics
        self.cmd_sub = self.create_subscription(JointTrajectoryPoint, '/motors/cmd', self.motors_cmd_callback, 10)
        self.encoder_pub = self.create_publisher(JointTrajectoryPoint, "/motors/info/encoders", 10)
        self.error_pub = self.create_publisher(Float32MultiArray, "/motors/info/errors", 10)
        self.qcurrent_pub = self.create_publisher(Float32MultiArray, "/motors/info/q_current", 10)
        self.bus_voltage_pub = self.create_publisher(Float32MultiArray, "/motors/info/bus_voltage", 10)
        self.bus_current_pub = self.create_publisher(Float32MultiArray, "/motors/info/bus_current", 10)
        #self.temp_pub = self.create_publisher(Float32MultiArray, "/motors/info/temperature", 10)

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
            can_id = motor.can_id
            control_mode = motor.control_mode
            position = get_command_value(msg.positions, id) * motor.gear_ratio
            velocity = get_command_value(msg.velocities, id) * motor.gear_ratio
            torque = get_command_value(msg.effort, id) * motor.gear_ratio

            if control_mode == 3:
                canfunc.set_position(can_id, position=position, velocity_ff=velocity, torque_ff=torque)
            elif control_mode == 2:
                canfunc.set_velocity(can_id, velocity=velocity, torque_ff=torque)
            elif control_mode == 1:
                canfunc.set_torque(can_id, torque=torque)

            self.get_logger().info(f"Sent motor command [P: {position}, V:{velocity}, T:{torque}] to {motor.name}")



    # Publish motor info
    def publish_info(self):
        encoder_msg = JointTrajectoryPoint()
        bus_volt_msg = Float32MultiArray()
        bus_curr_msg = Float32MultiArray()
        iq_data_msg = Float32MultiArray()
        error_msg = Float32MultiArray()
        #temp_msg = Float32MultiArray()
        for motor in self.motors:
            encoder_data = canfunc.get_encoder(motor.can_id)
            bus_vc_data = canfunc.get_voltage(motor.can_id)
            iq_data = canfunc.get_current(motor.can_id)
            error_data = canfunc.get_error(motor.can_id)
            #temp_data = canfunc.get_temperature(motor.can_id)
            encoder_msg.positions.insert(motor.id, float(encoder_data['Pos_Estimate']) / motor.gear_ratio)
            encoder_msg.velocities.insert(motor.id, float(encoder_data['Vel_Estimate']) / motor.gear_ratio)
            bus_volt_msg.data.insert(motor.id, float(bus_vc_data['Bus_Voltage']))
            bus_curr_msg.data.insert(motor.id, float(bus_vc_data['Bus_Current']))
            iq_data_msg.data.insert(motor.id, float(iq_data['Iq_Measured']))
            error_msg.data.insert(motor.id, float(error_data['Axis_Error']))
            #temp_msg.data.insert(motor.id, float(temp_data['Motor_Temperature']))
        self.encoder_pub.publish(encoder_msg)
        self.bus_voltage_pub.publish(bus_volt_msg)
        self.bus_current_pub.publish(bus_curr_msg)
        self.qcurrent_pub.publish(iq_data_msg)
        self.error_pub.publish(error_msg)
        #self.temp_pub.publish(temp_msg)

    # Put motors in idle state
    def idle(self):
        for motor in self.motors:
            canfunc.set_state(motor.can_id, int(AxisState.IDLE))

# ROS Entry
def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(str(e))
    except KeyboardInterrupt:
        pass
    node.idle()
    canfunc.close()
    node.get_logger().info("Motor driver node closed.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()