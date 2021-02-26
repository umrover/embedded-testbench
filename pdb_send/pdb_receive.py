import serial
import Adafruit_BBIO.UART as UART

#defines
baud = 9600
uartPort = "/dev/ttyS4"
UART.setup("UART4")
# configure the serial connections (set port yourself)
ser = serial.Serial(
    port=uartPort,
    baudrate= baud
)
ser.close()
ser.open()
while 1 :
    if ser.isOpen():
        try:
            print("i am running1")
            s = ser.readline()
            print(">>" + s)
        except ser.SerialTimeoutException:
            print("read timeout")
    else:
            print("Serial is not open")

