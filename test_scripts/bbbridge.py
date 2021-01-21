import Adafruit_BBIO.UART as UART
import serial
import time

UART.setup("UART4")

ser = serial.Serial(port = "/dev/ttyO4", baudrate=38400)
ser.close()
ser.open()
if ser.isOpen():
    while(1):
        print("Serial is open!")
        ser.write("$AMMONIA,0,,,".encode('utf-8'))
        time.sleep(1)
ser.close()
 
# Eventually, you'll want to clean up, but leave this commented for now, 
# as it doesn't work yet
#UART.cleanup()