// Encoder

// #ifdef ENCODER_ENABLE

#include "encoder.h"
#include "smbus.h"

Encoder* new_encoder(SMBus* i2cBus, _Bool A1_power, _Bool A2_power){
    Encoder* encoder = (Encoder*) malloc(sizeof(Encoder));

    if ((A1_power) && (A2_power)) {
        encoder->address = device_slave_address_both_power;
    }
    else if (A1_power) {
        encoder->address = device_slave_address_a1_power;
    }
    else if (A2_power) {
        encoder->address = device_slave_address_a2_power;
    }
    else {
        encoder->address = device_slave_address_none_power;
    }

    encoder->i2cBus = i2cBus;

    return encoder;
}

int read_raw_angle(Encoder* encoder) {

    int angle_high_data = read_byte_data(encoder->i2cBus, encoder->address, 0xFE);
    int angle_low_data = read_byte_data(encoder->i2cBus, encoder->address, 0xFF);
    
    int angle_low_data_modified = angle_low_data & 0x3F;

    int angle_raw = (angle_high_data << 6) | angle_low_data_modified;

    return angle_raw;
}

float get_angle_degrees(Encoder* encoder) {

    int angle_raw = read_raw_angle(encoder);
    float angle_raw_float = (float)angle_raw;
    float degrees_proportion = 180.0 * angle_raw;
    float degrees = degrees_proportion / (RAW_TO_180_DEGREES_CONVERSION_FACTOR);

    return degrees;
}


void deleteEncoder(Encoder* encoder){
    free(encoder);
}

// #endif
