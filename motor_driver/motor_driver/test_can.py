import can
import cantools

# Global objects
_candb = cantools.db.load_file("/home/pi/ros2_ws/src/boom_packages/motor_driver/docs/odrive-cansimple.dbc")
_canbus = can.Bus("can0", bustype="socketcan")

# Recieve a odrive message from CAN
def recieve_can_msg(can_id : int, msg_name : str, msg_data):
    send_can_msg(can_id, msg_name, msg_data)
    can_msg = _candb.get_message_by_name("Axis0_" + msg_name)
    for msg in _canbus:
        if msg.arbitration_id == ((can_id << 5) | can_msg.frame_id):
            return _candb.decode_message(msg_name, msg.data)
    return None

# Encoder data
def get_encoder(can_id : int):
    return recieve_can_msg(can_id, 'Get_Encoder_Estimates', {'Pos_Estimate': 0, 'Vel_Estimate': 0})

while True:
    print(get_encoder(0))