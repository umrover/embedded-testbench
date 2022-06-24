//#ifdef MOSFET_ENABLE
#include "mosfet.h"
#include "main.h"

// Array of all pins and ports for each device
// Put a better formatted list here to make it easier

uint16_t pin_array[MOSFET_DEVICE_NUM] = {
		MOSFET_1_Pin,            		// 1
		MOSFET_2_Pin,            		// 2
		MOSFET_3_Pin,            		// 3
		MOSFET_4_Pin,            		// 4
		MOSFET_5_Pin,            		// 5
		MOSFET_6_Pin,            		// 6
		MOSFET_7_Pin,            		// 7
		MOSFET_8_Pin,            		// 8
		MOSFET_9_Pin,            		// 9
		MOSFET_10_Pin,            		// 10
		MOSFET_11_Pin,            		// 11
		MOSFET_12_Pin,            		// 12
};

GPIO_TypeDef* port_array[MOSFET_DEVICE_NUM] = {
		MOSFET_1_GPIO_Port, 				// 1
		MOSFET_2_GPIO_Port, 				// 2
		MOSFET_3_GPIO_Port, 				// 3
		MOSFET_4_GPIO_Port, 				// 4
		MOSFET_5_GPIO_Port, 				// 5
		MOSFET_6_GPIO_Port, 				// 6
		MOSFET_7_GPIO_Port, 				// 7
		MOSFET_8_GPIO_Port, 				// 8
		MOSFET_9_GPIO_Port, 				// 9
		MOSFET_10_GPIO_Port, 				// 10
		MOSFET_11_GPIO_Port, 				// 11
		MOSFET_12_GPIO_Port, 				// 12
};

char *mosfet_copy;
char *auto_shut_copy;
char *led_copy;

int heater_state_0 = DISABLED;
int heater_state_1 = DISABLED;
int heater_state_2 = DISABLED;

int send_heater_info = 1;
int send_shutoff_info = 1;

int auto_shutoff = 1;

int led_state = NONE_STATE;
int green_on = DISABLED;
uint16_t ms_tick = 0;

void enable_pin(int enable, GPIO_TypeDef *port, uint16_t pin){
    if (enable){
         HAL_GPIO_WritePin(port,pin, GPIO_PIN_SET);
    }
    else {
        HAL_GPIO_WritePin(port,pin,GPIO_PIN_RESET);
    }
}

void receive_mosfet_cmd(uint8_t *buffer, int *device,int*enable, char*mosfet_copy){

  //Change to string
  char delim[] = ",";

  strncpy(mosfet_copy, buffer,30);

  if (mosfet_copy == NULL) {
  	return;
  }

  //Expected $MOSFET,<devicenum>,<enablenum>
  char *identifier = strtok(mosfet_copy,delim);
  if (!strcmp(identifier,"$MOSFET")){
	  *device = atoi(strtok(NULL,delim));
	  *enable = atoi(strtok(NULL,delim));
  }
}

void receive_led_cmd(uint8_t *buffer, int *color_state, char*led_copy){

  //Change to string
  char delim[] = ",";

  strncpy(led_copy, buffer,30);

  if (led_copy == NULL) {
  	return;
  }

  //Expected $MOSFET,<devicenum>,<enablenum>
  char *identifier = strtok(led_copy,delim);
  if (!strcmp(identifier,"$LED")){
	  *color_state = atoi(strtok(NULL,delim));
  }
}

void receive_auto_shutoff_cmd(uint8_t *buffer, int *enable, char* shutdowncopy){
	//Change to string
	char delim[] = ",";

	strncpy(shutdowncopy, buffer,30);

	if (shutdowncopy == NULL) {
		return;
	}
	//Expected $AUTOSHUTOFF,<devicenum>,<enablenum>
	char *identifier = strtok(shutdowncopy,delim);
	if (!strcmp(identifier,"$AUTOSHUTOFF")){
		*enable = atoi(strtok(NULL,delim));
	}
}
void send_auto_shutoff_state(UART_HandleTypeDef* huart){
	char string[30] = "";
	sprintf((char *)string, "$AUTOSHUTOFF,%i,\n",auto_shutoff);
	HAL_Delay(300);
	HAL_UART_Transmit_IT(huart, (uint8_t *)string, sizeof(string));
	HAL_Delay(15);
}

// Send the state of each heater(on/off)
// Changes on heater on/off
void send_heater_state(UART_HandleTypeDef* huart,int device,int enable){
	char string[30] = "";
	sprintf((char *)string, "$HEATER,%i,%i,\n",device,enable);
	HAL_Delay(500);
	HAL_UART_Transmit_IT(huart, (uint8_t *)string, sizeof(string));
	HAL_Delay(15);
}

void LED_tick() {
	if (led_state != GREEN_STATE) {
		return;
	}
	ms_tick += 1;
	if (ms_tick >= 60000)
	{
		// after 60 seconds, reset ms_tick so we don't get overflow
		ms_tick = 0;
	}
	int green_should_be_on = ms_tick % 2000 < 1000;
	if (green_should_be_on)
	{
		// in the middle of every even second, turn on
		if (green_on == ENABLED)
		{
			return;
		}
		green_on = ENABLED;
		enable_pin(ENABLED, port_array[GREEN_LED], pin_array[GREEN_LED]);
	}
	else
	{
		// in the middle of every odd second, turn off
		if (green_on == DISABLED)
		{
			return;
		}
		green_on = DISABLED;
		enable_pin(DISABLED, port_array[GREEN_LED], pin_array[GREEN_LED]);
	}
}

//#endif

