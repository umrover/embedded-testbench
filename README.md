Science Nucleo 😀
---

## Table of Contents

- [Thermistor](#Thermistor-😫)

## Thermistor 😫
### UART data string

- Format of the data string
  - `$THERMISTOR,[temp0],[temp1],[temp2]`
  - String is 50 characters long

### Public Functions

- `Thermistors* newThermistors(ADC_HandleTypeDef*, ADC_HandleTypeDef*, ADC_HandleTypeDef*);`
  - Returns a pointer to a block in memory of size Thermistors that contains a thermistors object, given the three ADC references you want to use for reading temperature values
  - C++ equivalent: constructor
- `float getTemp(const uint8_t, const Thermistors*);`
  - Returns a temperature in Celcius for one of the thermistors, specificy which thermistor you want to get a temperature from with either `0`, `1`, or `2` in the first parameter
- `void deleteThermistors(Thermistors*);`
    - Pass in a thermistor object to delete the object from memory
    - Make sure you call this before you exit your program to avoid leaking memory
    - C++ equivalent: destructor

### Private Functions

- `uint32_t readVoltage(ADC_HandleTypeDef*);`
  - Returns a completely unformated string from the ADC type given.
  - Data is between 0 and 4095 and is relative to the reference voltage given, defaults to 3.3V