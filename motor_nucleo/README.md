# MOTOR NUCLEO FIRMWARE
## Table of Contents
[Overview](#Overview) \
[Registers](#Registers) \
[Commands](#Commands) \
[IO](#IO) \
[Issues](#Issues)

## Overview
This firmware configures one nucleo f303re to act as a "motor nucleo".

Motor nucleos meet their functionality by internally splitting themselves into 6 distinct controller units, termed “channels”, labeled 0, 1, 2, 3, 4, and 5. **Each channel controls one motor independently**.

Each channel’s interfaces are exposed on the IO of the Nucleo F303RE. **Each channel’s interfaces have different IO. Some interfaces use shared IO**

Motor nucleos can control up to 6 motors, with each motor assigned to a "channel"
- All channels **share** an I2C bus to receive commands from a master device
- All channels **own** one PWM output (0%-100% duty cycle = throttle of motor)
- All channels **own** one DIR and NDIR output (logic HIGH/LOW = direction of motor, NDIR = !DIR)
- All channels **own** one LIMIT SWITCH input (logic HIGH = stop PWM)
- Channels 0, 1, 2 **own** one set of QUAD ENCODER A and QUAD ENCODER B inputs
- Channels 0, 1, 2 **share** an I2C bus to command ABSOLUTE ENCODERS

## Registers
Each channel has its own set of registers that dictate the channel’s behavior. **These registers are volatile (will not be saved under loss of power)**

| Register | Data Type | Value | Reset Value | Units |
| -------- | --------- | ----- | ----------- | ----- |
| control_mode | hidden | OPEN/CLOSED -> open/closed loop control is used. Set to OPEN upon OPEN or OPEN_PLUS command. Set to CLOSED upon CLOSED or CLOSED_PLUS command. | OPEN |
| dir | 8-bit value | 1 -> DIR/NDIR = HIGH/LOW, 0 -> DIR/NDIR = LOW/HIGH | 1 |
| open_setpoint | 16-bit unsigned integer | This register is an input for open-loop control | 0 | 0.125 us |
| closed_setpoint | 32-bit float | This register is an input for closed-loop control | 0 | quadrature encoder counts |
| closed_FF | 32-bit float | This register is an input for closed-loop control | 0 | |
| closed_KP | 32-bit float | This register is an input for closed-loop control | 0 | |
| closed_KI | 32-bit float | This register is an input for closed-loop control | 0 | |
| closed_KD | 32-bit float | This register is an input for closed-loop control | 0 | |
| quad_enc | 32-bit integer | This register is automatically incremented/decremented upon quadrature encoder feedback | 0 | quadrature encoder counts |
| abs_enc | 32-bit float | This register is a passthrough to the absolute encoder's position register. Reading from this register is equivalent to reading from an absolute encoder. | 0 | absolute encoder counts |
| pwm_max | 16-bit unsigned integer | VALUE = maximum PWM pulse width. Use this register to protect lower voltage motors. | 0 | 0.125 uS |

In closed-loop control mode, the Nucleo will use output from a standard PID loop. The error for the PID loop is calculated by *closed_setpoint – quad_enc*. The PWM pulse width and PWM direction will automatically be calculated, with a maximum pulse width of pwmMax.

In open-loop control mode, the Nucleo will output *open_setpoint* as the PWM pulse width, *dir* for DIR, and the ! *dir* for NDIR. 



## Commands
Each channel behaves as an I2C slave. The first three bits of the channel’s address correspond to the motor nucleo's designated number (plus 1) on the rover, while the last four bits of the address correspond to the channel’s number on the motor nucleo.

> For example, channel 2 on Nucleo 3 would have the I2C address of 100 0010.

The Nucleo expects transactions to adhere to the following format. For each transaction, the master should follow these steps:
1.	Issue a START condition
2.	Send the channel’s 7-bit address followed by a Write bit
3.	Send one byte to indicate the desired command (see "Command")
4.	Send the appropriate number of data bytes (see "Host Data"). (Could be 0)
5.  If no nucleo data is to be received, issue a STOP condition. End of transaction.
6.	Else, issue a RESTART condition, OR issue a STOP condition and then a START condition
7.	Send the channel’s 7-bit address followed by a Read bit
8.	Accept the appropriate number of data bytes (see "Nucleo Data")
9.	Issue a NACK
10.	Issue a STOP condition. End of transaction.

> These transactions are not based off of any smbus standard. It is recommended to use the open(), ioctl(), read(), and write() functions defined in linux/i2c-dev.h to control the host's I2C interface.

Host data sent to the channel will overwrite the appropriate registers in that channel. Nucleo data received from the channel will contain the most recent value of appropriate registers in that channel.

The table below lists the expected data order / number of bytes (in parenthesis) for every command.
>Transactions not following the proper format will be dropped in most cases.

| Command | Hex Code | Host Data | Nucleo Data |
| ------- | -------- | --------- | ----------- |
| OPEN | 0x10 | dir(1), open_setpoint(2) | (0) | 
| OPEN_PLUS | 0x1F | dir(1), open_setpoint(2) | quad_enc(4) | 
| CLOSED | 0x20 | closed_setpoint(4), closed_FF(4) | (0) |
| CLOSED_PLUS | 0x2F | closed_setpoint(4), closed_FF(4) | quad_enc(4) |
| SET_MAX_PWM | 0x30 | pwm_max(2) | (0) |
| SET_CLOSED_K | 0x3F | closed_KP(4), closed_KI(4), closed_KD(4) | (0) |
| QUAD_ENC | 0x40 | (0) | quad_enc(4) |
| QUAD_ENC_ADJUST | 0x4F | quad_enc(4) | (0) |
| ABS_ENC | 0X50 | (0) | abs_enc(2) |

>If the motor nucleo does not receive any commands from the host for a period longer than 1 second, the motor nucleo will reset all channels to open loop control, with an *open_setpoint* of 0, stopping all motors.

## IO
| Interface | Channel | IO |
| --------- | ------- | -- |
| HOST I2C - SCL | ALL | PB8 |
| HOST I2C - SDA | ALL | PB9 |
| PWM - PWM | 0 | PC0 | 
| PWM - PWM | 1 | PC1 | 
| PWM - PWM | 2 | PC2 | 
| PWM - PWM | 3 | PC6 | 
| PWM - PWM | 4 | PC7 | 
| PWM - PWM | 5 | PC8 | 
| PWM - DIR | 0 | PA10 | 
| PWM - DIR | 1 | PA11 | 
| PWM - DIR | 2 | PA12 | 
| PWM - DIR | 3 | PA13 | 
| PWM - DIR | 4 | PA14 | 
| PWM - DIR | 5 | PA15 | 
| PWM - NDIR | 0 | PC10 | 
| PWM - NDIR | 1 | PC11 | 
| PWM - NDIR | 2 | PC12 | 
| PWM - NDIR | 3 | PC13 | 
| PWM - NDIR | 4 | PC14 | 
| PWM - NDIR | 5 | PC15 |
| LIMIT SWITCH | 0 | PB10 | 
| LIMIT SWITCH | 1 | PB11 | 
| LIMIT SWITCH | 2 | PB12 | 
| LIMIT SWITCH | 3 | PB13 | 
| LIMIT SWITCH | 4 | PB14 | 
| LIMIT SWITCH | 5 | PB15 |
| ENCODER QUAD - A | 0 | PA0 |
| ENCODER QUAD - A | 1 | PA6 |
| ENCODER QUAD - A | 2 | PB7 |
| ENCODER QUAD - B | 0 | PA1 |
| ENCODER QUAD - B | 1 | PA4 |
| ENCODER QUAD - B | 2 | PB6 |
| ENCODER I2C - SCL | 0, 1, 2 | PF0 |
| ENCODER I2C - SDA | 0, 1, 2 | PF1 |

If any Master I2C IO is disconnected, all channels will not respond to commands. It is imperative this interface is connected.

If any PWM output IO is disconnected, there will be no internal effects on the channel.

If any quadrature input IO is disconnected, the affected channel will sense incorrect or no quadrature encoder input.

If any encoder I2C IO is disconnected, all channels will not respond to commands. It is imperative this interface is connected.

## Issues
### Linker failing 'multiple instances of...' CubeIDE version 1.9.0
Useful stackoverflow link: [link](https://stackoverflow.com/questions/71409313/stm-cubeide-1-9-0-has-linker-issues)

