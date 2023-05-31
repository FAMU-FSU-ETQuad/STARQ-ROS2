import can
import cantools
from odrive.enums import ControlMode, AxisState

# Global objects
_candb = cantools.db.load_file("odrive-cansimple.dbc")
_canbus = can.Bus("can0", bustype="socketcan")

# Send a odrive message to CAN
def send_can_msg(can_id : int, msg_name : str, msg_data):
    can_msg = _candb.get_message_by_name(msg_name)
    can_data = can_msg.encode(msg_data)
    can_msg = can.Message(arbitration_id=((can_id << 5) | can_msg.frame_id), is_extended_id=False, data=can_data)
    _canbus.send(can_msg)

# Recieve a odrive message from CAN
def recieve_can_msg(can_id : int, msg_name : str, msg_data):
    can_msg = _candb.get_message_by_name(msg_name)
    can_data = can_msg.encode(msg_data)
    for msg in _canbus:
        if msg.arbitration_id == ((can_id << 5) | can_msg.frame_id):
            return _candb.decode_message(msg_name, can_data)
    return None


# Set State
def set_state(can_id : int, state : AxisState):
    send_can_msg(can_id, 'Set_Axis_State', {'Axis_Requested_State': state})

# Set Control Type
def set_control_mode(can_id : int, mode : ControlMode):
    send_can_msg(can_id, 'Set_Controller_Mode', {'Control_Mode': mode})

# Closed Loop Position Control
def set_position(can_id : int, position : float, velocity_ff : float = 0.0, torque_ff : float = 0.0):
    send_can_msg(can_id, 'Set_Input_Pos', {'Input_Pos': position, 'Vel_FF': velocity_ff, 'Torque_FF': torque_ff})

# Closed Loop Velocity Control
def set_velocity(can_id : int, velocity: float, torque_ff : float = 0.0):
    send_can_msg(can_id, 'Set_Input_Vel', {'Input_Vel': velocity, 'Input_Torque_FF': torque_ff})

# Closed Loop Torque Control
def set_torque(can_id : int, torque: float):
    send_can_msg(can_id, 'Set_Input_Torque', {'Input_Torque': torque})

# Encoder data
def get_encoder(can_id : int):
    return recieve_can_msg(can_id, 'Get_Encoder_Estimates', {'Pos_Estimate': 0.0, 'Vel_Estimate': 0.0})