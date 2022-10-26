#include "heater.h"

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
    if (heater->is_on == true && get_thermistor_temperature(heater->thermistor) >= MAX_HEATER_TEMP && heater->auto_shutoff == true)
    {
        turn_mosfet_device_off(heater->mosfet);
        heater->is_on = false;
    }
}

// REQUIRES: state is either 0 or 1, representing off or on
// MODIFIES: is_on
// EFFECTS:  Turn heater off if state is 0. Turn heater on if state is 1 AND
// either temperature is lower than permitted temperature OR auto_shutoff is
// disabled
void change_heater_state(int state)
{
    if (!state)
    {
        turn_mosfet_device_off(heater->mosfet);
        heater->is_on = false;
    }
    else if (state == 1 && (get_thermistor_temperature(heater->thermistor) < MAX_HEATER_TEMP || heater->auto_shutoff == false))
    {
        turn_mosfet_device_on(heater->mosfet);
        heater->is_on = true;
    }
}

// REQUIRES: state is either 0 or 1, representing off or on
// MODIFIES: auto_shutoff
// EFFECTS:  Turn auto_shutoff on if state is 1 OR off if state is 0
void change_auto_shutoff(Heater *heater, int state)
{
    if (!state)
    {
        heater->auto_shutoff = true;
    }
    else if (state == 1)
    {
        heater->auto_shutoff = false;
    }
}
