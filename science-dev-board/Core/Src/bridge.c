#include "bridge.h"

// REQUIRES: uart is the uart channel
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Bridge object
Bridge *new_bridge(UART_HandleTypeDef *_uart)
{
    Bridge *bridge = (Bridge *)malloc(sizeof(Bridge));

    bridge->uart = _uart;

    return bridge;
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends diagnostic current and thermistor data in format:
// $DIAG,,<TEMP_0>,<TEMP_1>,<TEMP_2>,<CURR_0>,<CURR_1>,<CURR_2>
void bridge_send_diagnostic(Bridge *bridge, float temp[3], float curr[3])
{
    char msg[150];

    snprintf(msg, sizeof(msg), "$DIAG,%f,%f,%f,%f,%f,%f", temp[0], temp[1],
    		temp[2], curr[0], curr[1], curr[2]);

    HAL_Delay(100);
    HAL_UART_Transmit_IT(bridge->uart, (uint8_t *)msg, 150);
    HAL_Delay(100);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends spectral data in format:
// "$SPECTRAL, <ch0>, <ch1>, <ch2>, <ch3>, <ch4>, <ch5>""
void bridge_send_spectral(Bridge *bridge, uint32_t channel[6])
{
    char msg[150];

    snprintf(msg, sizeof(msg), "$SPECTRAL,%lu,%lu,%lu,%lu,%lu,%lu", channel[0],
    		channel[1], channel[2], channel[3], channel[4],
			channel[5]);

    HAL_Delay(100);
    HAL_UART_Transmit_IT(bridge->uart, (uint8_t *)msg, 150);
    HAL_Delay(100);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends science temperatures in format:
// "$SCIENCE_TEMP,<TEMP_0>,<TEMP_1>,<TEMP_2>"
void bridge_send_science_thermistors(Bridge *bridge, float temp[3])
{
    char msg[150];
    snprintf(msg, sizeof(msg), "$SCIENCE_TEMP,%f,%f,%f", temp[0], temp[1], temp[2]);
    HAL_Delay(100);
    HAL_UART_Transmit_IT(bridge->uart, (uint8_t *)msg, 40);
    HAL_Delay(100);
}

void send_uart_string(Bridge *bridge, char msg[150])
{
    HAL_Delay(300);
    HAL_UART_Transmit_IT(bridge->uart, (uint8_t *)msg, 150);
    HAL_Delay(100);
}
