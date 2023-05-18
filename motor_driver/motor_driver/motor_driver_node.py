import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Empty

import asyncio
import moteus
import math
import threading

# Boom motors
MOTOR_IDS = [9, 10]

# Dina's values
MOTOR_KP = [127.8, 25.0]
MOTOR_KI = [0.0, 0.0]
MOTOR_KD = [2.25, 0.12] 
MOTOR_FLUX_BRAKE = [35.5, 35.5]
ZERO_POSITIONS = [0.0, 0.5]

REZERO_ON_START = True

class MotorDriverNode(Node):

    def __init__(self):
        super().__init__('motor_driver_node')

        self.get_logger().info('Initializing motor driver node.')

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
        self.motor_kp = p_motor_kp.get_parameter_value().double_array_value
        self.motor_ki = p_motor_ki.get_parameter_value().double_array_value
        self.motor_kd = p_motor_kd.get_parameter_value().double_array_value
        self.motor_flux_brake = p_motor_flux_brake.get_parameter_value().double_array_value
        self.zero_positions = p_zero_positions.get_parameter_value().double_array_value
        self.rezero_on_start = p_rezero_on_start.get_parameter_value().bool_value

        self.get_logger().info('Parameters saved.')

        # --- SUBSCRIBERS ---

        # Position command subscriber
        self.cmd_pos_sub = self.create_subscription(
            Float32MultiArray,
            '/starq/motors/cmd/position',
            self.set_position_callback,
            10
        )

        # Velocity command subscriber
        self.cmd_vel_sub = self.create_subscription(
            Float32MultiArray,
            '/starq/motors/cmd/velocity',
            self.set_velocity_callback,
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

        # --- TIMERS ---

        # Sample motor information
        info_sample_rate = 0.1 # seconds
        self.info_timer = self.create_timer(info_sample_rate, self.publish_info_callback)

        # --- MOTORS ---

        # Use FDCANUSB (Power distribution board) for comm
        self.transport = moteus.Fdcanusb()

        # Create motors with IDs
        self.servos = {
            idx : moteus.Controller(id=self.motor_ids[idx], transport=self.transport)
            for idx in range(len(self.motor_ids))
        }

        self.get_logger().info('Controllers initialized.')

        # Create command stream for each motor
        self.streams = {
            idx : moteus.Stream(controller)
            for idx, controller in self.servos.items()
        }

        self.get_logger().info('Streams initialized.')

        # Done initializing
        self.get_logger().info('Motor Driver initialized.')

    @classmethod
    async def create(cls):
        self = MotorDriverNode()

        # asynchronous initialization
        await self.main()

        return self

    async def main(self):
        self.get_logger().info('Initializing motors.')
        # Reset motor faults
        await self.reset_faults()
        self.get_logger().info('Reset faults.')
        await self.set_gains(self.motor_kp, self.motor_ki, self.motor_kd)
        self.get_logger().info('Set motor gains.')
        await self.set_flux_brake(self.motor_flux_brake)
        self.get_logger().info('Set motor flux brake.')

        if self.rezero_on_start:
            await self.set_as_zero(None, None)

        self.get_logger().info('Motors initialized.')

    # Set position command callback
    def set_position_callback(self, msg):
        asyncio.create_task(self.set_position(msg))

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
    def set_velocity_callback(self, msg):
        asyncio.create_task(self.set_velocity(msg))

    async def set_velocity(self, msg):
        commands = {
            servo.make_position( 
                position=math.nan,
                velocity=msg.data[idx]
                )
            for idx, servo in self.servos.items()
        }
        await self.transport.cycle(commands)

    # Publish motor information callback
    def publish_info_callback(self):
        asyncio.create_task(self.publish_info())

    async def publish_info(self):
        # Create a mapping of publisher to data array
        pub_data_map = {
            self.info_position_pub: Float32MultiArray(),
            self.info_velocity_pub: Float32MultiArray(),
            self.info_torque_pub: Float32MultiArray(),
            self.info_qcurrent_pub: Float32MultiArray(),
            self.info_temp_pub: Float32MultiArray(),
            self.info_fault_pub: Int32MultiArray(),
        }

        # Create a mapping of register to publisher
        reg_pub_map = {
            moteus.Register.POSITION: self.info_position_pub,
            moteus.Register.VELOCITY: self.info_velocity_pub,
            moteus.Register.TORQUE: self.info_torque_pub,
            moteus.Register.Q_CURRENT: self.info_qcurrent_pub,
            moteus.Register.TEMPERATURE: self.info_temp_pub,
            moteus.Register.FAULT: self.info_fault_pub,
        }

        # Query all servo states
        results = await self.transport.cycle(servo.make_query() for servo in self.servos.values())
        
        # Process each result
        for result in results:
            for reg, pub in reg_pub_map.items():
                pub_data_map[pub].data.append(result.values[reg])

        # Publish all messages
        for pub, msg in pub_data_map.items():
            pub.publish(msg)

        self.get_logger().info("Motor information published.")



    # Reset faults function
    async def reset_faults(self):
        await self.transport.cycle([
            servo.make_stop() 
            for servo in self.servos.values()
        ])

    # Set motor gains
    async def set_gains(self, KP, KI, KD):
        for idx, stream in self.streams.items():
            self.get_logger().info("Setting gains for motor " + str(self.motor_ids[idx]))
            await stream.command(bytes(str("conf set servo.pid_position.kp " + str(KP[idx])), 'utf-8'))
            self.get_logger().info("Set KP = " + str(KP[idx]))
            await stream.command(bytes(str("conf set servo.pid_position.ki " + str(KI[idx])), 'utf-8'))
            self.get_logger().info("Set KI = " + str(KI[idx]))
            await stream.command(bytes(str("conf set servo.pid_position.kd " + str(KD[idx])), 'utf-8'))
            self.get_logger().info("Set KD = " + str(KD[idx]))

    async def reset_gains(self):
        for idx, stream in self.streams.items():
            self.get_logger().info("Resetting gains for motor " + str(self.motor_ids[idx]))
            await stream.command(bytes(str("conf set servo.pid_position.kp 0.0"), 'utf-8'))
            await stream.command(bytes(str("conf set servo.pid_position.ki 0.0"), 'utf-8'))
            await stream.command(bytes(str("conf set servo.pid_position.kd 0.0"), 'utf-8'))
        
    # Set flux brake
    async def set_flux_brake(self, V):
        for idx, stream in self.streams.items():
            self.get_logger().info("Setting flux brake for motor " + str(self.motor_ids[idx]))
            await stream.command(bytes("conf set servo.flux_brake_min_voltage " + str(V[idx]), 'utf-8'))
            self.get_logger().info("Set FB = " + str(V[idx]))

    # Set motor positions to zero
    async def set_as_zero(self, request, response):
        await self.transport.cycle([
            servo.make_set_output_exact(position = self.zero_positions[idx])
            for idx, servo in self.servos.items()
        ])
        self.get_logger().info('Motors Re-zeroed')


# ROS Entry
def main(args=None):
    rclpy.init(args=args)

    # Create a new event loop and set it as the default for the current context
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    motor_driver_node = loop.run_until_complete(MotorDriverNode.create())

    executor = SingleThreadedExecutor()
    executor.add_node(motor_driver_node)

    def spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=spin)
    spin_thread.start()

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        motor_driver_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
