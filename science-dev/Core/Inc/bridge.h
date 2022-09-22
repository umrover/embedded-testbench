#ifndef BRIDGE_H_
#define BRIDGE_H_

// The communication bridge between the Jetson and the chip
typedef struct {
	UART_HandleTypeDef *uart;
} Bridge;

// REQUIRES: uart is the uart channel
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Bridge object
Bridge *new_bridge(UART_HandleTypeDef *_uart);

#endif

//#endif
