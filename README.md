Science Nucleo 😀
---

## Table of Contents

- [Thermistor](#Thermistor)
- [Mux](#Mux)
- [Spectral](#Spectral)

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
