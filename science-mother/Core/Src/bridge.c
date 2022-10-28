#include "bridge.h"

// Private Variables -----------------------------------------------------
ADC_HandleTypeDef hadc1; // used by adc_sensors
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1; // used by servos

// REQUIRES: uart is the uart channel
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Bridge object
Bridge *new_bridge(UART_HandleTypeDef *_uart)
{
    Bridge *bridge = (Bridge *)malloc(sizeof(Bridge));

    bridge->uart = _uart;

    int num_thermistors = 3;
    bridge->adc_sensor = new_adc_sensor(&hadc1, num_thermistors);
    for (size_t i = 0; i < num_thermistors; ++i)
    {
        bridge->science_thermistors[i] = new_thermistor(bridge->adc_sensor, i);
        bridge->science_temps[i] = 0;
    }

    int num_servos = 3;

    bridge->servos[0] = new_servo(&htim1, TIM_CHANNEL_1, &(TIM1->CCR1));
    bridge->servos[1] = new_servo(&htim1, TIM_CHANNEL_2, &(TIM1->CCR2));
    bridge->servos[2] = new_servo(&htim1, TIM_CHANNEL_3, &(TIM2->CCR3));

    for (size_t i = 0; i < num_servos; ++i)
    {
        initialize_servo(bridge->servos[i], 0);
    }

    bridge->mosfet_devices[0] = new_mosfet_device(MOSFET_0_GPIO_Port, MOSFET_0_Pin);
    bridge->mosfet_devices[1] = new_mosfet_device(MOSFET_1_GPIO_Port, MOSFET_1_Pin);
    bridge->mosfet_devices[2] = new_mosfet_device(MOSFET_2_GPIO_Port, MOSFET_2_Pin);
    bridge->mosfet_devices[3] = new_mosfet_device(MOSFET_3_GPIO_Port, MOSFET_3_Pin);
    bridge->mosfet_devices[4] = new_mosfet_device(MOSFET_4_GPIO_Port, MOSFET_4_Pin);
    bridge->mosfet_devices[5] = new_mosfet_device(MOSFET_5_GPIO_Port, MOSFET_5_Pin);
    bridge->mosfet_devices[6] = new_mosfet_device(MOSFET_6_GPIO_Port, MOSFET_6_Pin);
    bridge->mosfet_devices[7] = new_mosfet_device(MOSFET_7_GPIO_Port, MOSFET_7_Pin);

    return bridge;
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Updates ADC and Themistor temperatures, and sets servos and mosfets
void bridge_iterate(Bridge *bridge)
{
    update_adc_sensor_values(bridge->adc_sensor);
    for (size_t i = 0; i < 3; ++i)
    {
        update_thermistor_temperature(bridge->science_thermistors[i]);
        bridge->science_temps[i] = get_thermistor_temperature(bridge->science_thermistors[i]);
    }

    /* TODO: Add Heater Stuff. */

    for (size_t i = 0; i < 3; ++i)
    {
        set_servo_angle(bridge->servos[i], 0);
    }

    for (size_t i = 0; i < 8; ++i)
    {
        set_mosfet_device_state(bridge->mosfet_devices[i], 0);
    }
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends science temperatures in format:
// "$SCIENCE_TEMP,<TEMP_0>,<TEMP_1>,<TEMP_2>"
void bridge_send_science_thermistors(Bridge *bridge)
{
    char msg[150];
    snprintf(msg, sizeof(msg), "$SCIENCE_TEMP,%u,%u,%u",
             bridge->science_temps[0], bridge->science_temps[1],
             bridge->science_temps[2]);
    send_uart_string(bridge, msg);
}

void send_uart_string(Bridge *bridge, char msg[150])
{
    HAL_Delay(300);
    HAL_UART_Transmit_IT(bridge->uart, (uint8_t *)msg, 150);
    HAL_Delay(100);
}