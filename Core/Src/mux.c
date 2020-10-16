#include "mux.h"
#include "smbus.h"

Mux *new_mux(Bus *i2cbus, int channels, int *channel_list) {
    Mux *mux = malloc(sizeof(Mux));
    mux->i2cbus = i2cbus;

    for (int i = 0; i < channels; ++i) {
        if (i > 7) {
            break;
        }
        // if someone is trying to turn on a channel that doesn't exist 
        mux->channel_list[i] = channel_map[channel_list[i]];
    }
    return mux;
}

void select(Mux *mux, int channel){
    write_byte_data(mux->bus, I2C_MUX_ADDRESS, MUX_CMD, channel);
}