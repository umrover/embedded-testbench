#include "mux.h"
#include "smbus.h"

Mux *new_mux(SMBus *i2cBus) {
    Mux *mux = malloc(sizeof(Mux));
    mux->i2cBus = i2cBus;
    for (int i = 0; i < 8; ++i) {
        mux->channel_list[i] = 0x00;
    }
    mux->channels_active = 0;

    return mux;
}

void add_channel(Mux *mux, int channel) {
    if (channel > 7) {
        return;
    }
    mux->channel_list[channel] = channel_map[channel];
    mux->channels_active += 1;
}

void select(Mux *mux, int channel){
    write_byte_data(mux->i2cBus, I2C_MUX_ADDRESS, MUX_CMD, channel);
}