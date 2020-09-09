
import odrive
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, CTRL_MODE_VELOCITY_CONTROL, CTRL_MODE_CURRENT_CONTROL, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_IDLE, ENCODER_MODE_HALL
from odrive.utils import dump_errors
import sys
import select
import time
import fibre

id = "205F3883304E"

def reset():
    global odrv
    global test_motor

    test_motor.motor.config.pole_pairs = 15
    test_motor.motor.config.resistance_calib_max_voltage = 4
    test_motor.motor.config.requested_current_range = 40
    test_motor.motor.config.current_control_bandwidth = 100

    test_motor.encoder.config.mode = ENCODER_MODE_HALL
    test_motor.encoder.config.cpr = 90
    test_motor.encoder.config.bandwidth = 100
    test_motor.controller.config.pos_gain = 1
    test_motor.controller.config.vel_gain = 0.02
    test_motor.controller.config.vel_integrator_gain = 0.1
    test_motor.controller.config.vel_limit = 1000
    test_motor.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    try:
        odrv.reboot()
    except fibre.protocol.ChannelBrokenException:
        odrv = odrive.find_any(serial_number=id)
        test_motor = odrv.axis0
        print("found odrive")
        pass
    


def calibrate():
    global odrv
    global test_motor
    try:
        test_motor.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        test_motor.motor.config.pre_calibrated = True
        test_motor.encoder.config.pre_calibrated = True
        while(test_motor.requested_state != AXIS_STATE_IDLE):
            pass
        odrv.save_configuration()
        odrv.reboot()
    except fibre.protocol.ChannelBrokenException:
        odrv = odrive.find_any(serial_number=id)
        test_motor = odrv.axis0
        print("found odrive")
        pass

def errors():
    global odrv
    dump_errors(odrv, True)
    
def drive():
    global test_motor
    test_motor.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    # test_motor.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    # test_motor.controller.vel_setpoint = 500
    test_motor.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
    test_motor.controller.current_setpoint = 30
    
def stop():
    global test_motor
    test_motor.controller.current_setpoint = 0
    test_motor.requested_state = AXIS_STATE_IDLE
    
def get_current_draw():
    global test_motor
    return test_motor.motor.current_control.Iq_measured

def get_speed_estimate():
    global test_motor
    return test_motor.encoder.vel_estimate
    
def main():
    global odrv
    global test_motor
    odrv = odrive.find_any(serial_number=id)
    print("found odrive")
    test_motor = odrv.axis0

    while(1):
        cmd_input = select.select([sys.stdin], [], [], 1)[0]
        f = open("current_draw.txt", "a")
        g = open("speed.txt", "a")
        t = time.clock()

        if cmd_input:
            value = sys.stdin.readline().rstrip()
            if (value == 'q'):
                stop()
                print("stopped")
                break;
            elif (value == 'r'):
                reset()
                print("reset")
            elif (value == 'c'):
                calibrate()
                print("calibrate")
            elif (value == 'e'):
                errors()
            elif (value == 'd'):
                drive()
                print("driving")
            else:
                print("unknown input")
        else:
            f.write("time: " + str(time.clock() - t) + " current: " +  str(get_current_draw()))
            g.write("time: " + str(time.clock() - t) + "speed: " + str(get_speed_estimate()))

if __name__=="__main__":
    main()
    
