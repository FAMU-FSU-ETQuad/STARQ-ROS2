import can
import cantools

# Global objects
_candb = cantools.db.load_file("/home/pi/ros2_ws/src/boom_packages/motor_driver/docs/odrive-cansimple.dbc")
_canbus = can.Bus("can0", bustype="socketcan")

# Send a odrive message to CAN
def send_can_msg(can_id : int, msg_name : str, msg_data):
    can_msg = _candb.get_message_by_name("Axis0_" + msg_name)
    can_data = can_msg.encode(msg_data)
    can_msg = can.Message(arbitration_id=((can_id << 5) | can_msg.frame_id), is_extended_id=False, data=can_data)
    _canbus.send(can_msg)

# Recieve a odrive message from CAN
def recieve_can_msg(can_id : int, msg_name : str, msg_data):
    can_msg = _candb.get_message_by_name("Axis0_" + msg_name)
    for msg in _canbus:
        if msg.arbitration_id == ((can_id << 5) | can_msg.frame_id):
            return _candb.decode_message(msg_name, msg.data)
    return None

# Clear motor errors
def clear_errors(can_id : int):
    send_can_msg(can_id, 'Clear_Errors', {})

# Set State
def set_state(can_id : int, state : int):
    send_can_msg(can_id, 'Set_Axis_State', {'Axis_Requested_State': state})

# Set Control Type
def set_control_mode(can_id : int, mode : int):
    send_can_msg(can_id, 'Set_Controller_Mode', {'Control_Mode': mode, 'Input_Mode': 1})

# Closed Loop Position Control
def set_position(can_id : int, position : float, velocity_ff : float = 0.0, torque_ff : float = 0.0):
    send_can_msg(can_id, 'Set_Input_Pos', {'Input_Pos': position, 'Vel_FF': velocity_ff, 'Torque_FF': torque_ff})

# Closed Loop Velocity Control
def set_velocity(can_id : int, velocity: float, torque_ff : float = 0.0):
    send_can_msg(can_id, 'Set_Input_Vel', {'Input_Vel': velocity, 'Input_Torque_FF': torque_ff})

# Closed Loop Torque Control
def set_torque(can_id : int, torque: float):
    send_can_msg(can_id, 'Set_Input_Torque', {'Input_Torque': torque})

# Set Gains
def set_gains(can_id : int, pos_gain : float, vel_gain : float, vel_int_gain : float):
    send_can_msg(can_id, 'Set_Pos_Gain', {'Pos_Gain' : pos_gain})
    send_can_msg(can_id, 'Set_Vel_Gains', {'Vel_Gain' : vel_gain, 'Vel_Integrator_Gain': vel_int_gain})

# Encoder data
def get_encoder(can_id : int):
    return recieve_can_msg(can_id, 'Get_Encoder_Estimates', {'Pos_Estimate': 0, 'Vel_Estimate': 0})

# Voltage data
def get_voltage(can_id : int):
    return recieve_can_msg(can_id, 'Get_Bus_Voltage', {'Bus_Voltage': 0})['Bus_Voltage']

# Current data
def get_current(can_id : int):
    return recieve_can_msg(can_id, 'Get_Iq', {'Iq_Measured': 0})['Iq_Measured']

# Temperature data
def get_temperature(can_id : int):
    return recieve_can_msg(can_id, 'Get_Temperature', {'Motor_Temperature': 0})['Motor_Temperature']

def close():
    _canbus.shutdown()

