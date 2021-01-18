Science Nucleo 😀
---
## About 
Code for controlling all of the science sensors for the 2021 Rover. \
For each respective hardware type there is a design file, with their function names and usages specified \
in the this README. \
The code is written in C and data is transmitted/received over UART. \
[Here](https://docs.google.com/spreadsheets/d/1bnmS1VDOZzItH4A75VZc1xltQRsbM6IUoubj1wVaauA/edit#gid=0) is a link to the pinout of this nucleo. 

## Table of Contents

- [Thermistor](#Thermistor)
- [Mux](#Mux)
- [Spectral](#Spectral)
- [Mosfet](#Mosfet)
- [Ammonia Motor](#Ammonia Motor)

## Thermistor
### UART data string

- Format of the data string
  - `$THERMISTOR,<temp0>,<temp1>,<temp2>`
  - String is 50 characters long

### Public Members 😁

- Thermistor Struct Variables
  - `float constantArray[4][4]`
    - Stores all the constants need to convert from resistance to temperatuer
  - `ADC_HandleTypeDef* adcPins[3]`
    - Stores references to the three adc Objects
  - `int R1vals[3]`
    - Stores the resistor values for the three different 10kOhms resistors in the circuit
  - `float V1`
    - Stores the reference voltage of the circuit, normally 3.3V
  - `int R25`
    - Stores the resistance of the thermistor at 25C, normally 10kOhms
- `Thermistors* newThermistors(ADC_HandleTypeDef*, ADC_HandleTypeDef*, ADC_HandleTypeDef*);`
  - Returns a pointer to a block in memory of size Thermistors that contains a thermistors object, given the three ADC references you want to use for reading temperature values
  - C++ equivalent: constructor
- `float getTemp(const uint8_t, const Thermistors*);`
  - Returns a temperature in Celcius for one of the thermistors, specificy which thermistor you want to get a temperature from with either `0`, `1`, or `2` in the first parameter
- `void deleteThermistors(Thermistors*);`
    - Pass in a thermistor object to delete the object from memory
    - Make sure you call this before you exit your program to avoid leaking memory
    - C++ equivalent: destructor

### Private Members 😈

- `uint32_t readVoltage(ADC_HandleTypeDef*);`
  - Returns a completely unformated string from the ADC type given.
  - Data is between 0 and 4095 and is relative to the reference voltage given, defaults to 3.3V
  
  
## Mux 
### UART Data String
None

### Public Members 😁 
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
### Private Members 😈
None 


## Spectral 
### UART Data String 
- `$SPECTRAL, d0_msb_ch0, d0_lsb_ch0, d0_msb_ch1, d0_lsb_ch1, d0_msb_ch2, d0_lsb_ch2, d0_msb_ch3, d0_lsb_ch3, d0_msb_ch4, d0_lsb_ch4, d0_msb_ch5, d0_lsb_ch5, d1_msb_ch0, d1_lsb_ch0, d1_msb_ch1, d1_lsb_ch1, d1_msb_ch2, d1_lsb_ch2, d1_msb_ch3, d1_lsb_ch3, d1_msb_ch4, d1_lsb_ch4, d1_msb_ch5, d1_lsb_ch5,  d2_msb_ch0, d2_lsb_ch0, d2_msb_ch1, d2_lsb_ch1, d2_msb_ch2, d2_lsb_ch2, d2_msb_ch3, d2_lsb_ch3, d2_msb_ch4, d2_lsb_ch4, d2_msb_ch5, d2_lsb_ch5,`
### Public Members 😁
- Spectral Struct
  - `i2cBus`
    - SMBus object to abstract away HAL i2c transactions 
  - `channels`
    - array of length CHANNELS to store Channel objects 
- `Spectral *new_spectral(SMBus *i2cBus)`
  - c++ style spectral constructor, takes an SMBus object 
- `void enable_spectral(Spectral *spectral)`
  - Sets the enable bits of the spectral sensor 
- `void get_spectral_data(Spectral *spectral, uint16_t *data)`
  - Gets data of all 6 spectral channels 
- `del_spectral(Spectral *spectral)`
  - c++ style destructor 
### Private Members 😈 
- Channel Struct
  - the spectral data is split into two 8 bit registers 
  - `uint8_t lsb_register`
  - `uint8_t msb_register`
  - `uint16_t color_data`
    - spectral data from combining the data in the lsb and msb registers
- `CHANNELS` --> 6
- `Channel* new_channel(uint8_t lsb_r, uint8_t msb_r)`
  - c++ style Channel object constructor. Takes the lsb and msb registers for this color channel\
  return a Channel pointer 
- `void get_channel_data(Spectral *spectral)`
  - gets the color data of all the channels of the spectral sensor, no return type
- `uint16_t read_channel(Spectral *spectral, int channel)`
  - reads an individual channel on the spectral sensor
- `uint16_t get_val(Spectral *spectral, uint8_t virtual_reg_l, uint8_t virtual_reg_h)`
  - reads and combines the data from the msb and lsb registers of the spectral sensor 
- `uint8_t virtual_read(Spectral *spectral, uint8_t v_reg)`
  - i2c byte read as according to the virtual i2c protocol 
- `void virtual_write(Spectral *spectral, uint8_t v_reg, uint8_t data)`
  - i2c byte write as according to the virtual i2c protocol  
- `void del_channel(Channel *channel)`
  - c++ style destructor 

## Mosfet
### UART data string

- Format of the UART NMEA command
  - `$Mosfet,<device>,<enable>,<extra padding>`
  - String is 30 characters long

### Public Members 😁

- `void enableX(int enable)`
  - Turns on or off the gpio port linked to device x based on enable.
  - X is replaced with the appropriate device. i.e. enableRled
  - This includes the r/g/b leds, science UV, SA UV, white science led and the 2 peristaltic pumps


### Private Members 😈

- `void enablePin(int enable, GPIO_TypeDef *port, uint16_t pin)`
  - Writes to a certain gpio port given the port and a specific pin number
  - i.e. enablePin(1, GPIOC,GPIO_PIN_8) if you wanted to enable the C8 port.


## Ammonia Motor
### UART data string

- Format of the UART NMEA command
  - `$AMMONIA,<speed>,<padding commas>`
  - String is 13 characters long
  - Speed can range from -1 to 1 & is capped at those two points 

### Public Members 😁
- Motor Struct 
  - `fwd_port`
    - gpio pin series for forward signal 
  - `bwd_port`
    - gpio pin series for backwards signal 
	- `timer`
    - pwm timer object
  - `fwd_pin`
    - pin number in series for forward port
  - `bwd_pin`
    - pin number in series for backward port
- `Motor *new_motor(GPIO_TypeDef *fwd_port, uint16_t fwd_pin, GPIO_TypeDef *bwd_port, uint16_t bwd_pin, TIM_HandleTypeDef *timer)`
  - c++ style constructor, takes the gpio pins & pwm timer to control the motor 
- `void start(Motor *motor, int channel)`
  - enables the PWM timer
- `void set_speed(Motor *motor, double speed)`
  - sets the speed of the PWM, can be beteween -1 and 1

### Private Members 😈
none!
