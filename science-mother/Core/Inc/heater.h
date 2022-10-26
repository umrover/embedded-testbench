#pragma once

#include "mosfet.h"
#include "thermistor.h"

#define MAX_HEATER_TEMP 65.0f

typedef struct
{
    MosfetDevice *mosfet;
    Thermistor *thermistor;
    bool auto_shutoff;
    bool is_on;
} Heater;

// REQUIRES: mosfet_dev is a pointer to a MosfetDevice object and therm
// is a pointer to a Thermistor object
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Heater object
Heater *new_heater(MosfetDevice *_mosfet_dev, Thermistor *_thermistor);

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Updates temperature of heater thermistor
void update_heater_temperature(Heater *heater);

// REQUIRES: nothing
// MODIFIES: is_on
// EFFECTS:  Turns heater off if it is on, thermistor temperature exceeds
// permitted temperature, and auto_shutoff is enabled
void update_heater_state(Heater *heater);

// REQUIRES: state is either 0 or 1, representing off or on
// MODIFIES: is_on
// EFFECTS:  Turn heater off if state is 0. Turn heater on if state is 1 and
// either temperature is lower than permitted temperature OR auto_shutoff is
// disabled
void change_heater_state(int state);

// REQUIRES: state is either 0 or 1, representing off or on
// MODIFIES: auto_shutoff
// EFFECTS:  Turn auto_shutoff on if state is 1 OR off if state is 0
void change_auto_shutoff(Heater *heater, int state);
