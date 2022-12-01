import os
import subprocess
import re

def getiSerial(bus,device_number) :
	output = subprocess.getoutput("sudo lsusb -s {}:{} -v|grep -i iserial".format(bus,device_number))
	var = output.find("iSerial")
	iSerial = []	
	for i in range(20):
		try:
			#print(var+24+i)
			#print(output[var+24+i])
			iSerial.append(output[var+24+i])				
		except:
			break
			
	print(''.join(iSerial))		
	return ''.join(iSerial)
			
def main():
    getiSerial("003","001")

if __name__ == "__main__":
    main()
    
  

