import Adafruit_BBIO.UART as UART
import serial
import time

UART.setup("UART4")

ser = serial.Serial(port = "/dev/ttyO4", baudrate=38400)
ser.close()
ser.open()
if ser.isOpen():
    try:
        ser.flushInput() #flush input buffer, discarding all its contents
        ser.flushOutput()#flush output buffer, aborting current output 
                 #and discard all that is in buffer
        while(1):
            tx=ser.readline()
            time.sleep(1)
            print(tx.decode('utf-8'))
        ser.close()
    except Exception:
        print("error communicating...'\n'")

else:
    print("cannot open serial port ")
 