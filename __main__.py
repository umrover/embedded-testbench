import can
import odrive
import cantools
import time

db = cantools.database.load_file("odrive-cansimple.dbc")
bus = can.Bus("can0", bustype="socketcan")

# notes: 2 axes per odrive (default 0x0 and 0x1).
#        Each axis looks like a separate node on the bus. 
#        Thus, they both have the two properties can_node_id and can_node_id_extended. 
#        The node ID can be from 0 to 63 (0x3F) inclusive, or, 
#        if extended CAN IDs are used, from 0 to 16777215 (0xFFFFFF). 
#        If you want to connect more than one ODrive on a CAN bus, 
#        you must set different node IDs for the second ODrive or 
#        they will conflict and crash the bus.

# id from odrive documentation
INPUT_VEL = 0x00D
INPUT_POS = 0x00C

def main():
    # get this from json eventually
    axes = {{0x0, 0x1, 0x2}, {0x3, 0x4, 0x5}}
    # turn/s
    vel = 0.25 
    print("testing forwards and backwards\n")
    send_command(axes[0][0], vel)
    # sleep for 5 sec
    time.sleep(5)
    send_command(axes[0][0], -vel)
    time.sleep(5)
    send_command(axes[0][0], 0)
    


def send_command(axis_id, setvel):
    data = db.encode_message('Set_Input_Vel', {'Input_Vel':setvel, 'Torque_FF':0.0})
    msg = can.Message(arbitration_id=axis_id << 5 | INPUT_VEL, data=data, is_extended_id=False)
    bus.send(msg)

if __name__ == "__main__":
    main()