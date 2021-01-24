import serial

#defines
baud = 115200
uartPort = "/dev/ttyS4"

# configure the serial connections (set port yourself)
ser = serial.Serial(
    port=uartPort,
    baudrate=115200,
)
char c = ser.read()