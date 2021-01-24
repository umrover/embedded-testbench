import serial

#defines
baud = 115200
uartPort = "/dev/ttyS4"

# configure the serial connections (set port yourself)
ser = serial.Serial(
    port=uartPort,
    baudrate=115200,
)
ser.isOpen()
while 1 :
    while ser.inWaiting() > 0:
        s = ser.read()
        print ">>" + s

