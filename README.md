# PDB Bridge

## Table of Contents

- [Current Status](#Current_Status)
- [About](#About)
- [Relevant Links](#Relevant_Links)
- [Thermal](#Thermal)
- [Current](#Current)
- [Voltage](#Voltage)
- [TODO](#TODO)

## Current Status

Code is still being developed and is not yet ready. Check the TODO for current status.

## About

This is the code controlling the power distribution board for the rover.
A STM32G050C8 chip is being used.
For each respective hardware type there is a design file, with their function names and usages specified
in the this README.

The code is written in C and data is transmitted/received over I2C.
Check the ESW - EHW ICD of 2021 (in MRover drive) for more information.

The corresponding bridge will be on the pdb_bridge branch on tabiosg/mrover-workspace.

## Thermal
### I2C data string (current plan)

- Format of the data string
  - `$THERMISTOR,<t0>,<t1>,<t2>`
  - String is 50 characters long

The information will represent temperature in Celsius.

It will be information on Temp1, Temp2, and Temp3 (representing the 3.3V, 5V, and 12V rail respectively). 

## Current 
### I2C data string (current plan)

- Format of the current data string
    - `$CURRENT,<c0>,<c1>,<c2>`
    - String is 50 characters long

The information will represent current in Amps.

It will be information on CS3, CS2, and CS1 (representing the 3.3V, 5V, and 12V rail respectively). 

## Voltage 
### I2C data string (current plan)

- Format of the voltage data string
    - `$VOLTAGE,<v0>,<v1>,<v2>`
    - String is 50 characters long

The information will represent voltage in Volts.

It will be information on the voltage dividers for the 3.3V, 5V, and 12V rails. 

## TODO
 - [x] Create analog.c and thermal_sens.c
 - [ ] Test analog.c and thermal_sens.c to obtain values
 - [x] Update the .ioc
 - [x] Create read data functions in main.c
 - [ ] Test read data functions in main.c
 - [ ] Create send data functions in main.c
 - [ ] Test send data functions in main.c
 - [ ] Check to make sure that I2C2 is the desired way to communicate back to jetson
 - [ ] Make sure that sending back I2C messages makes sense (hint: it probably doesn't yet)
 - [ ] Create bridge on tabiosg/mrover-workspace
 - [ ] Add more to the TODO list once there are more issues...
