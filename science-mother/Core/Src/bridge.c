#include "bridge.h"

// Private Variables -----------------------------------------------------

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
// $DIAG,<TEMP_0>,<TEMP_1>,<TEMP_2>,<CURR_0>,<CURR_1>,<CURR_2>
void bridge_send_diagnostic(Bridge *bridge, float temp_0, float temp_1, float temp_2,
                            float current_0, float current_1, float current_2)
{
    char msg[150];

    /*
    float current_data[3];
    float temp_data[3];
    for (uint8_t i = 0; i < 3; ++i)
    {
        temp_data[i] = 0;
        current_data[i] = 0;
    }
    */

    snprintf(msg, sizeof(msg), "$DIAG,%f,%f,%f,%f,%f,%f", temp_0, temp_1,
             temp_2, current_0, current_1, current_2);
    /*temp_data[0],
     temp_data[1], temp_data[2], current_data[0], current_data[1],
     current_data[2]);
     */
    HAL_Delay(100);
    HAL_UART_Transmit_IT(bridge->uart, (uint8_t *)msg, 150);
    HAL_Delay(100);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends spectral data in format:
// "$SPECTRAL, <ch0>, <ch1>, <ch2>, <ch3>, <ch4>, <ch5>""
void bridge_send_spectral(Bridge *bridge, Spectral *spectral)
{
    char msg[150];

    uint32_t spectral_data[6];
    for (uint8_t i = 0; i < 6; ++i)
    {
        spectral_data[i] = get_spectral_channel_data(spectral, i);
    }

    snprintf(msg, sizeof(msg), "$SPECTRAL,%u,%u,%u,%u,%u,%u", spectral_data[0],
             spectral_data[1], spectral_data[2], spectral_data[3], spectral_data[4],
             spectral_data[5]);
    HAL_Delay(100);
    HAL_UART_Transmit_IT(bridge->uart, (uint8_t *)msg, 150);
    HAL_Delay(100);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends science temperatures in format:
// "$SCIENCE_TEMP,<TEMP_0>,<TEMP_1>,<TEMP_2>"
void bridge_send_science_thermistors(Bridge *bridge, float temp_0, float temp_1,
                                     float temp_2)
{
    char msg[40];
    snprintf(msg, sizeof(msg), "$SCIENCE_TEMP,%f,%f,%f", temp_0, temp_1, temp_2);
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
