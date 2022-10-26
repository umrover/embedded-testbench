#include "heater.h"

// REQUIRES: nothing
// MODIFIES: is_on
// EFFECTS:  Turns heater off
void turn_heater_off(Heater *heater)
{
    turn_mosfet_device_off(heater->mosfet);
    heater->is_on = false;
}

// REQUIRES: nothing
// MODIFIES: is_on
// EFFECTS:  Turns heater on
void turn_heater_on(Heater *heater)
{
    turn_mosfet_device_on(heater->mosfet);
    heater->is_on = true;
}

// REQUIRES: mosfet_dev is a pointer to a MosfetDevice object and therm
// is a pointer to a Thermistor object
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Heater object
Heater *new_heater(MosfetDevice *_mosfet_dev, Thermistor *_thermistor)
{
    Heater *heater = (Heater *)malloc(sizeof(Heater));

    heater->mosfet = _mosfet_dev;
    heater->thermistor = _thermistor;
    heater->auto_shutoff = true;
    heater->is_on = false;

    return heater;
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Updates temperature of heater thermistor
void update_heater_temperature(Heater *heater)
{
    update_thermistor_temperature(heater->thermistor);
}

// REQUIRES: nothing
// MODIFIES: is_on
// EFFECTS:  Turns heater off if it is on AND thermistor temperature exceeds
// permitted temperature AND auto_shutoff is enabled
void update_heater_state(Heater *heater)
{
    if (heater->is_on && get_thermistor_temperature(heater->thermistor) >= MAX_HEATER_TEMP && heater->auto_shutoff)
    {
        turn_heater_off(heater);
    }
}

// REQUIRES: state is either false or true, representing off or on
// MODIFIES: is_on
// EFFECTS:  Turn heater off if state is false. Turn heater on if state is true
// AND either temperature is lower than permitted temperature OR auto_shutoff is
// disabled
void change_heater_state(Heater *heater, bool state)
{
    if (!state)
    {
        turn_heater_off(heater);
    }
    else if (state && (get_thermistor_temperature(heater->thermistor) < MAX_HEATER_TEMP || !heater->auto_shutoff))
    {
        turn_heater_on(heater);
    }
}

// REQUIRES: state is either false or true, representing off or on
// MODIFIES: auto_shutoff
// EFFECTS:  Turn auto_shutoff on if state is true OR off if state is false
void change_heater_auto_shutoff(Heater *heater, bool state)
{
    if (!state)
    {
        heater->auto_shutoff = true;
    }
    else
    {
        heater->auto_shutoff = false;
    }
}