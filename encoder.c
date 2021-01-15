// Encoder 

#ifdef ENCODER_ENABLE

#include "encoder.h"

Encoder* new_encoder(SMBus* i2cBus, bool A1_power, bool A2_power){
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

    encoder->i2cBus = i2cBus;

    return encoder;
}

uint16_t read_raw_angle(Encoder* encoder) {

    uint8_t AngleHighData = read_byte_data(i2cBus, encoder->address, ANGLE_HIGH);
    uint8_t AngleLowData = read_byte_data(i2cBus, encoder->address, ANGLE_LOW);
    
    uint8_t LSBmodified = AngleLowData & 0x3F;

    uint16_t AngleData = (AngleMostByte << 6) | LSBmodified;

    return AngleData;
}

double get_angle_degrees(Encoder* encoder) {

    uint16_t AngleRaw = get_raw_val_angle(encoder);
    double Degrees = 180 * AngleRaw / (RAW_TO_180_DEGREES_CONVERSION_FACTOR);

    return Degrees
}


void deleteEncoder(Encoder* encoder){
    free(encoder);
}

#endif
