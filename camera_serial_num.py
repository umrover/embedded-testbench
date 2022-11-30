import os
import subprocess
import re
#s.system("lsusb | grep \"Bus $1 Device $2\" | sed 's/\// /' | awk '{for(i=7;i<=NF;++i)print $i}'")

def getiSerial(bus,device_number) :
	output = subprocess.getoutput("sudo lsusb -s {}:{} -v|grep -i iserial".format(bus,device_number))
	var = output.find("iSerial")
	#for i in range(32):
	#	print(var+i)
	#	print(output[var+i])
		
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
    
  

