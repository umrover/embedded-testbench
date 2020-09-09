
import odrive
import sys
import select
import time


odrv = odrv0
test_motor = odrive.odrv0.axis0

def reset():
    test_motor.motor.config.pole_pairs = 15
    test_motor.motor.config.resistance_calib_max_voltage = 4
    test_motor.motor.config.requested_current_range = 25
    test_motor.motor.config.current_control_bandwidth = 100

    test_motor.encoder.config.mode = ENCODER_MODE_HALL
    test_motor.encoder.config.cpr = 90
    test_motor.encoder.config.bandwidth = 100
    test_motor.controller.config.pos_gain = 1
    test_motor.controller.config.vel_gain = 0.02
    test_motor.controller.config.vel_integrator_gain = 0.1
    test_motor.controller.config.vel_limit = 1000
    test_motor.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    odrv.reboot()
    


def calibrate():
    test_motor.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    test_motor.motor.config.pre_calibrated = True
    test_motor.encoder.config.pre_calibrated = True
    odrv.reboot()

def errors():
    odrive.dump_errors(test_motor, True)
    
def drive():
    test.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    test_motor.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    test_motor.controller.vel_setpoint = 500
    
def stop():
    test_motor.controller.vel_setpoint = 0
    test_motor.requested_state = AXIS_STATE_IDLE
    
def get_current_draw():
    return test_motor.motor.current_control.Iq_measured
    
def main():
    while(1):
        cmd_input = select.select([sys.stdin], [], [], 1)[0]
        f = open("current_draw.txt", "a")
        t = time.clock()

        if cmd_input:
            value = sys.stdin.readline().rstrip()
            if (value == 'q'):
                stop()
                print("stopped")
                break;
            elif (value == 'r'):
                reset()
                calibrate()
                print("reset")
            elif (value == 'c'):
                calibrate()
                print("calibrate")
            elif (value == 'e'):
                errors()
            elif (value == 'drive'):
                drive()
                print("driving")
            else:
                print("unknown input")
        else:
            f.write(str(time.clock() - t) + str(get_current_draw()))

if __name__=="__main__":
    main()
    
