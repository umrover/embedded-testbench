#ifndef MOSFET_H_
#define MOSFET_H_

#include "stm32g0xx_hal.h"

// define number of devices for the MOSFET
#define MOSFET_DEVICE_NUM 12

#include "main.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"

#define HEATER_0_Pin MOSFET_8_Pin
#define HEATER_1_Pin MOSFET_9_Pin
#define HEATER_2_Pin MOSFET_10_Pin

#define HEATER_0_GPIO_Port MOSFET_8_GPIO_Port
#define HEATER_1_GPIO_Port MOSFET_9_GPIO_Port
#define HEATER_2_GPIO_Port MOSFET_10_GPIO_Port


// Mapping of devices to their mosfet number
typedef enum
{
	RED_LED = 10,
	GREEN_LED = 4,
	BLUE_LED = 5,
	RA_LASER = 1,
	UV_LED = 4,
	WHITE_LED = 5,
	UV_BULB = 6,
	HEATER_0 = 7,
	HEATER_1 = 8,
	HEATER_2 = 9,
	RAMAN_LASER = 10,
} MOSFET_DEVICE_NAME;

typedef enum
{
	DISABLED,     // 0
	ENABLED,      // 1
} DEVICE_ENABLE_STATE;

typedef enum
{
	RED_STATE,
	GREEN_STATE,
	BLUE_STATE,
	NONE_STATE
} COLOR_STATE;


// Private Interface
// Enable or disable a certain pin
void enable_pin(int enable, GPIO_TypeDef *port, uint16_t pin);
void receive_mosfet_cmd(uint8_t *buffer, int *device,int*enable, char*mosfet_copy);
void receive_led_cmd(uint8_t *buffer, int *color_state, char*led_copy);
void receive_auto_shutoff_cmd(uint8_t *buffer, int *enable, char* shutdowncopy);
void send_heater_state(UART_HandleTypeDef* huart,int device,int enable);
void send_auto_shutoff_state(UART_HandleTypeDef* huart);
void LED_tick();


extern uint16_t pin_array[MOSFET_DEVICE_NUM];
extern GPIO_TypeDef* port_array[MOSFET_DEVICE_NUM];

extern char* mosfet_copy;
extern char *auto_shut_copy;
extern char *led_copy;
extern int send_heater_info;
extern int send_shutoff_info;

extern int heater_state_0;
extern int heater_state_1;
extern int heater_state_2;
extern int auto_shutoff;

extern int led_state;
extern int green_on;
extern uint16_t ms_tick;

#endif

