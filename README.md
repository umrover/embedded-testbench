# PDB Bridge

## Current Status

Code is still being developed and is not ready. Check the TODO for current status

## About

This is the code controlling the power distribution board for the rover.
A STM32G050C8 chip is being used.
For each respective hardware type there is a design file, with their function names and usages specified
in the this README.
The code is written in C and data is transmitted/received over I2C (TODO: TBD if I2C).
Check the ESW - EHW ICD of 2021 (in MRover drive) for more information.

## Relevant Links

[Here](https://docs.google.com/document/d/1KNdXkzjw123RyxxfUVBfyk_eKc1iTBEvt-JHOC-5bYw/edit#) is a link to the custom MCU documentation.
The link includes the pinout (outdated - was for pdb of 2020 with different chip).

[Here](https://docs.google.com/document/d/1GrD6bYsV3d1v_Svg_e8F1hEzgNB-p2fankFGTrMYNW4/edit#heading=h.3jepa9v6kxlh) is a link to the PDB proposal (outdated - was for pdb of 2020 with different chip).

[STM32F103C6T6A Datasheet](https://www.st.com/content/ccc/resource/technical/document/datasheet/0d/93/e0/d7/77/bf/4c/54/CD00210843.pdf/files/CD00210843.pdf/jcr:content/translations/en.CD00210843.pdf)

[Temperature Sensor Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/25095A.pdf)

[Current Sensor Datasheet - Download](https://www.digikey.com/en/products/detail/allegro-microsystems/ACS722LLCTR-10AU-T/4915370)

## Table of Contents

- [Thermal](#Thermal)
- [Analog](#Analog)

## Thermal
### I2C data string (current plan)

Will send 32-bit float information back using HAL_I2C_Slave_Seq_Transmit_IT.
The information will represent temperature in Celsius.

There are three values that will be sent: Temp1, Temp2, and Temp3. 
These represent the 3.3V, 5V, and 12V rail, respectively.

## Analog 
### I2C data string (current plan)

Will send 32-bit float information back using HAL_I2C_Slave_Seq_Transmit_IT.

Will first send six values that will represent voltage in Volts.
Will then send six values that will represent current in Amps.

There are six values that will be sent: CS1, Voltage Divider 1, CS2, Voltage Divider 2, CS3, Voltage Divider 3. 
These represent the 12V, 12V, 5V, 5V, 3.3V, and 3.3V rail, respectively.

## TODO
 - [ ] Verify that correct readings can be obtained (check debugger)
 - [ ] Check to make sure that I2C2 is the desired way to communicate back to jetson
 - [ ] Make sure that sending back I2C messages makes sense (hint: it probably doesn't yet)
 - [ ] Add more to the TODO list once there are more issues...
