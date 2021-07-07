# PDB Bridge

## Current Status

Code is still being developed and is not ready.

## About

This is the code controlling the power distribution board for the Rover.
For each respective hardware type there is a design file, with their function names and usages specified
in the this README.
The code is written in C and data is transmitted/received over UART.

## Relevant Links

[Here](https://docs.google.com/document/d/1KNdXkzjw123RyxxfUVBfyk_eKc1iTBEvt-JHOC-5bYw/edit#) is a link to the custom MCU documentation.
The link includes the pinout.

[Here](https://docs.google.com/document/d/1GrD6bYsV3d1v_Svg_e8F1hEzgNB-p2fankFGTrMYNW4/edit#heading=h.3jepa9v6kxlh) is a link to the PDB proposal.

A STM32F103C6T6A chip is being used.

[Analog Mux Datasheet](https://www.ti.com/lit/ds/symlink/cd74hc4067.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1625237825034)

[I2C Mux Datasheet](https://www.ti.com/lit/ds/symlink/tca9548a.pdf?HQS=dis-mous-null-mousermode-dsf-pf-null-wwe&ts=1621451323906&ref_url=https%253A%252F%252Fwww.mouser.com%252F)

[STM32F103C6T6A Datasheet](https://www.st.com/content/ccc/resource/technical/document/datasheet/0d/93/e0/d7/77/bf/4c/54/CD00210843.pdf/files/CD00210843.pdf/jcr:content/translations/en.CD00210843.pdf)

[Temperature Sensor Datasheet](https://www.mouser.com/datasheet/2/268/25095A-15487.pdf)

[Current Sensor Datasheet - Download](https://static6.arrow.com/aropdfconversion/dc872c84038254e6875bc4efc97640aaaa43f7f/acs722-datasheet.pdf)

## Table of Contents

- [Thermal](#Thermal)
- [Mux](#Mux)
- [Analog](#Analog)

## Thermal
### UART data string

- Format of the data string
  - `Thermal,<temp0>,<temp1>,<temp2>,<temp3>`
  - String is ???TODO??? characters long
  
## Mux 

### Public Members
- Mux Struct Variables 
  - `i2cBus`
    - SMBus Data structure for abstracting i2c smbus transactions 
  - `channel_list` 
    - list of the active channels of the mux in hex. Channels 0-7 are open 
  - `channels active`
    - The number of active channels on the mux 
- `Mux *new_mux(SMBus *i2cBus)` 
  - c++ style constructor. Takes an SMBus pointer and returns a Mux pointer
- `void add_channel(Mux *mux, int channel)`
  - initalizes a channel on the mux 
- `void del_mux(Mux *mux)`
  - c++ style destructor 
### Private Members
None

## Analog 
### UART Data String 
- Format of the data string
  - `ANALOG,<voltage0>,<voltage1>,<current0>,<current1>,`
