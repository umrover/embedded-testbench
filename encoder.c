// Encoder 

#ifdef ENCODER_ENABLE

#include "encoder.h"

///////////////////

Encoder* new_encoder(bool A1_power, bool A2_power){
    Encoder* encoder = (Encoder*) malloc(sizeof(Encoder));

    if (A1_power & A2_power) {
        encoder->address = DEVICE_SLAVE_ADDRESS_BOTH_POWER;
    }
    else if (A1_power) {
        encoder->address = DEVICE_SLAVE_ADDRESS_A1_POWER;
    }
    else if (A2_power) {
        encoder->address = DEVICE_SLAVE_ADDRESS_A2_POWER;
    }
    else {
        encoder->address = DEVICE_SLAVE_ADDRESS_NONE_POWER;
    }

    return encoder;
}

uint16_t get_val_angle(Encoder* encoder) {
    
    uint32_t rawData = readVoltage(therms->adcPins[whichTherm]);


    ///////////////////////////////////
    ///////////////////////////////////
    // delete comments after

    AngleHighData = read_byte(i2cBus, encoder->address); // ? not sure how to read data
    AngleLowData = read_byte(i2cBus, encoder->address); // 
    
    LSBmodified = AngleLowData & 0x3F;

    AngleData = (AngleMostByte << 6) | LSBmodified;

    Degrees = 180 * AngleData / (RAW_TO_180_DEGREES_CONVERSION_FACTOR);

    return Degrees;
}

void deleteEncoder(Encoder* encoder){
    free(encoder);
}

#endif
