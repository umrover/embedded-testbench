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
void bridge_send_diagnostic(Bridge *bridge, float temps[3], float currs[3])
{
    char msg[150];

    snprintf(msg, sizeof(msg), "$DIAG,%f,%f,%f,%f,%f,%f", temps[0], temps[1],
    		temps[2], currs[0], currs[1], currs[2]);

    HAL_Delay(100);
    HAL_UART_Transmit_IT(bridge->uart, (uint8_t *)msg, 150);
    HAL_Delay(100);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends spectral data in format:
// "$SPECTRAL, <ch0>, <ch1>, <ch2>, <ch3>, <ch4>, <ch5>""
void bridge_send_spectral(Bridge *bridge, uint16_t ch_data[6])
{
    char msg[150];

    snprintf(msg, sizeof(msg), "$SPECTRAL,%u,%u,%u,%u,%u,%u", ch_data[0],
    		ch_data[1], ch_data[2], ch_data[3], ch_data[4],
			ch_data[5]);

    HAL_Delay(100);
    HAL_UART_Transmit_IT(bridge->uart, (uint8_t *)msg, 150);
    HAL_Delay(100);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends science temperatures in format:
// "$SCIENCE_TEMP,<TEMP_0>,<TEMP_1>,<TEMP_2>"
void bridge_send_science_thermistors(Bridge *bridge, float temps[3])
{
    char msg[150];
    snprintf(msg, sizeof(msg), "$SCIENCE_TEMP,%f,%f,%f", temps[0], temps[1], temps[2]);
    HAL_Delay(100);
    HAL_UART_Transmit_IT(bridge->uart, (uint8_t *)msg, 150);
    HAL_Delay(100);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends heater auto shutoff in format:
// "$AUTOSHUTOFF,<VAL>"
void bridge_send_heater_auto_shutoff(Bridge *bridge, bool state) {
	char msg[150];
	snprintf(msg, sizeof(msg), "$AUTOSHUTOFF,%i", state);
	HAL_Delay(100);
	HAL_UART_Transmit_IT(bridge->uart, (uint8_t *)msg, 150);
	HAL_Delay(100);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends heater state in format:
// "$HEATER_DATA,<STATE_0>,<STATE_1>,<STATE_2>"
void bridge_send_heater_state(Bridge *bridge, bool states[3]) {
	char msg[150];
	snprintf(msg, sizeof(msg), "$HEATER_DATA,%i,%i,%i", states[0], states[1], states[2]);
	HAL_Delay(100);
	HAL_UART_Transmit_IT(bridge->uart, (uint8_t *)msg, 150);
	HAL_Delay(100);
}
