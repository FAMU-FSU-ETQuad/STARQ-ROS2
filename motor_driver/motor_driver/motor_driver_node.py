import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Empty

import asyncio
import moteus
import math

# Boom motors
MOTOR_IDS = [9, 10]

# Dina's values
MOTOR_KP = 127.8
MOTOR_KI = 0.0
MOTOR_KD = 2.25
MOTOR_FLUX_BRAKE = 35.5
ZERO_POSITIONS = [0.0, 0.5, 0.0, 0.5, 0.0, 0.5, 0.0, 0.5]

REZERO_ON_START = True

class MotorDriverNode(Node):

    def __init__(self):
        super().__init__('motor_driver_node')

        # Declare ROS parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('motor_ids', MOTOR_IDS),
                ('motor_kp', MOTOR_KP),
                ('motor_ki', MOTOR_KI),
                ('motor_kd', MOTOR_KD),
                ('motor_flux_brake', MOTOR_FLUX_BRAKE),
                ('zero_positions', ZERO_POSITIONS),
                ('rezero_on_start', REZERO_ON_START)
            ]
        )

        # Set parameters from launch file
        (p_motor_ids, p_motor_kp, p_motor_ki, p_motor_kd, p_motor_flux_brake, p_zero_positions, p_rezero_on_start) = self.get_parameters(
            ['motor_ids', 'motor_kp', 'motor_ki', 'motor_kd', 'motor_flux_brake', 'zero_positions', 'rezero_on_start'])
        self.motor_ids = p_motor_ids.get_parameter_value().integer_array_value
        self.motor_kp = p_motor_kp.get_parameter_value().double_value
        self.motor_ki = p_motor_ki.get_parameter_value().double_value
        self.motor_kd = p_motor_kd.get_parameter_value().double_value
        self.motor_flux_brake = p_motor_flux_brake.get_parameter_value().double_value
        self.zero_positions = p_zero_positions.get_parameter_value().double_array_value
        self.rezero_on_start = p_rezero_on_start.get_parameter_value().bool_value

        self.get_logger().info('Parameters Saved.')

        # Initialize controllers
        asyncio.run(self.init_controllers())

        # --- SUBSCRIBERS ---

        # Position command subscriber
        self.cmd_pos_sub = self.create_subscription(
            Float32MultiArray,
            '/starq/motors/cmd/position',
            self.set_position,
            10
        )

        # Velocity command subscriber
        self.cmd_vel_sub = self.create_subscription(
            Float32MultiArray,
            '/starq/motors/cmd/velocity',
            self.set_velocity,
            10
        )

        self.get_logger().info('Subscribers created.')

        # --- PUBLISHERS ---

        self.info_position_pub = self.create_publisher(
            Float32MultiArray,
            '/starq/motors/info/position',
            10
        )

        self.info_velocity_pub = self.create_publisher(
            Float32MultiArray,
            '/starq/motors/info/velocity',
            10
        )

        self.info_torque_pub = self.create_publisher(
            Float32MultiArray,
            '/starq/motors/info/torque',
            10
        )

        self.info_qcurrent_pub = self.create_publisher(
            Float32MultiArray,
            '/starq/motors/info/qcurrent',
            10
        )

        self.info_temp_pub = self.create_publisher(
            Float32MultiArray,
            '/starq/motors/info/temperature',
            10
        )

        self.info_fault_pub = self.create_publisher(
            Int32MultiArray,
            '/starq/motors/info/fault',
            10
        )

        self.get_logger().info('Publishers created.')

        # --- SERVICES ---

        self.set_as_zero_srv = self.create_service(
            Empty,
            '/starq/motors/set_as_zero',
            self.set_as_zero
        )

        self.get_logger().info('Services created.')

        # -------------------

        # Sample motor information
        info_sample_rate = 0.1 # seconds
        self.info_timer = self.create_timer(info_sample_rate, self.publish_info)

        # Done initializing
        self.get_logger().info('Motor Driver Initialized.')

    # Initialize motor controllers
    async def init_controllers(self):

        # Use FDCANUSB (Power distribution board) for comm
        self.transport = moteus.Fdcanusb()

        # Create motors with IDs
        self.servos = {
            idx : moteus.Controller(id=self.motor_ids[idx])
            for idx in range(len(self.motor_ids))
        }

        self.get_logger().info('Controllers Initialized.')

        # Create command stream for each motor
        self.streams = {
            idx : moteus.Stream(controller)
            for idx, controller in self.servos.items()
        }

        self.get_logger().info('Streams Initialized.')
        
        # Reset motor faults
        self.reset_faults()
        self.set_gains(self.motor_kp, self.motor_ki, self.motor_kd)
        self.set_flux_brake(self.motor_flux_brake)

        if self.rezero_on_start:
            self.set_as_zero(None, None)

        self.get_logger().info('Motors Initialized.')

    # Reset faults function
    async def reset_faults(self):
        await self.transport.cycle([servo.make_stop() for servo in self.servos.values()])

    # Set position command callback
    async def set_position(self, msg):
        commands = {
            servo.make_position( 
                position=msg.data[idx],
                velocity=math.nan
                )
            for idx, servo in self.servos.items()
        }
        await self.transport.cycle(commands)

    # Set velocity command callback
    async def set_velocity(self, msg):
        commands = {
            servo.make_position( 
                position=math.nan,
                velocity=msg.data[idx]
                )
            for idx, servo in self.servos.items()
        }
        await self.transport.cycle(commands)

    # Sample motor information callback
    async def publish_info(self):

        query_commands = {
            servo.make_query()
            for servo in self.servos.values()
        }

        positions = Float32MultiArray()
        velocities = Float32MultiArray()
        torques = Float32MultiArray()
        qcurrents = Float32MultiArray()
        temps = Float32MultiArray()
        faults = Int32MultiArray()

        results = await self.transport.cycle(query_commands)
        for idx in range(len(results)):
            result = results[idx]
            positions.data[idx] = result.values[moteus.Register.POSITION]
            velocities.data[idx] = result.values[moteus.Register.VELOCITY]
            torques.data[idx] = result.values[moteus.Register.TORQUE]
            qcurrents.data[idx] = result.values[moteus.Register.Q_CURRENT]
            temps.data[idx] = result.values[moteus.Register.TEMPERATURE]
            faults.data[idx] = result.values[moteus.Register.FAULT]
            
        self.info_position_pub.publish(positions)
        self.info_velocity_pub.publish(velocities)
        self.info_torque_pub.publish(torques)
        self.info_qcurrent_pub.publish(qcurrents)
        self.info_temp_pub.publish(temps)
        self.info_fault_pub.publish(faults)

    # Set motor gains
    async def set_gains(self, KP, KI, KD):
        for stream in self.streams.values():
            await stream.command(bytes(str("conf set servo.pid_position.kp " + str(KP)), 'utf-8'))
            await stream.command(bytes(str("conf set servo.pid_position.ki " + str(KI)), 'utf-8'))
            await stream.command(bytes(str("conf set servo.pid_position.kd " + str(KD)), 'utf-8'))
        
    # Set flux brake
    async def set_flux_brake(self, V):
        for stream in self.streams.values():
            await stream.command(bytes("conf set servo.flux_brake_min_voltage " + str(V), 'utf-8'))

    # Set motor positions to zero
    async def set_as_zero(self, request, response):
        for idx, servo in self.servos.items():
            await servo.set_output_exact(position = self.zero_positions[idx])
        return response

# ROS Entry
def main(args=None):
    rclpy.init(args=args)
    motor_driver = MotorDriverNode()
    rclpy.spin(motor_driver)
    motor_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()