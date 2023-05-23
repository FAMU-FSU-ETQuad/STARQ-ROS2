# Class for the Moteus motor controllers

import threading
import asyncio
import moteus
import math

from dataclasses import dataclass

# Struct class for motor object
@dataclass
class MoteusMotor:
    motor_id: int
    zero_position: float = 0.0
    max_torque: float = 2.5
    position: float = math.nan
    velocity: float = math.nan
    controller: moteus.Controller = moteus.Controller(motor_id)
    stream: moteus.Stream = moteus.Stream(controller)

class MoteusDriver:

    def __init__(self, motor_ids):

        # Default Params
        self.update_rate = 20 # Hz
        self.zero_on_start = True

        # Motor map
        self.motors = {idx: MoteusMotor(motor_ids[idx]) for idx in range(len(motor_ids))}

        # Motor feedback
        self.feedback = [None] * len(motor_ids)

        # Transport
        self.transport = moteus.Fdcanusb()

        # Update transport
        for motor in self.motors.values():
            motor.controller.transport = self.transport
        
    # Start the update loop in a thread
    def start(self):
        self.thread = threading.Thread(target=asyncio.run, args=(self.main(),))
        self.thread.start()

    # Update loop
    async def main(self):

        # Reset motors
        await self.reset_faults()
        if self.zero_on_start:
            await self.set_as_zero()

        # Motor loop
        while True:
            self.command_motor()
            await asyncio.sleep(1 / self.update_rate)

    # Command the motors to go to their current set state
    async def command_motor(self):
        commands = {
            motor.controller.make_position( 
                position=motor.position,
                velocity=motor.velocity,
                maximum_torque=motor.max_torque,
                query=True
                )
            for motor in self.motors.values()
        }
        self.feedback = await self.transport.cycle(commands)

    # Reset motor faults
    async def reset_faults(self):
        await self.transport.cycle([motor.controller.make_stop() for motor in self.motors.values()])

    # Reset motor positions
    async def set_as_zero(self):
        await self.transport.cycle([motor.controller.make_set_output_exact(position=motor.zero_position) for motor in self.motors.values()])

    ## SETTERS ##
    def set_update_rate(self, rate):
        self.update_rate = rate

    def set_zero_on_start(self, zero_on_start):
        self.zero_on_start = zero_on_start

    def set_zero_positions(self, zero_position):
        for idx, motor in self.motors.items():
            motor.zero_position = zero_position[idx]
    
    def set_max_torques(self, max_torque):
        for idx, motor in self.motors.items():
            motor.max_torque = max_torque[idx]

    def set_positions(self, positions):
        for idx, motor in self.motors.items():
            motor.position = positions[idx]

    def set_velocities(self, velocities):
        for idx, motor in self.motors.items():
            motor.velocity = velocities[idx]

    ## GETTERS ##
    def get_motors(self):
        return self.motors

    def get_motor_positions(self):
        return [float(state.values[moteus.Register.POSITION]) for state in self.feedback]
    
    def get_motor_velocities(self):
        return [float(state.values[moteus.Register.VELOCITY]) for state in self.feedback]
    
    def get_motor_torques(self):
        return [float(state.values[moteus.Register.TORQUE]) for state in self.feedback]
    
    def get_motor_temperatures(self):
        return [float(state.values[moteus.Register.TEMPERATURE]) for state in self.feedback]
    
    def get_motor_qcurrents(self):
        return [float(state.values[moteus.Register.Q_CURRENT]) for state in self.feedback]
    
    def get_motor_modes(self):
        return [int(state.values[moteus.Register.MODE]) for state in self.feedback]
    
    def get_motor_faults(self):
        return [int(state.values[moteus.Register.FAULT]) for state in self.feedback]