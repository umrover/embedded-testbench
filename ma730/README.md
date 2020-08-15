Code to test the ma730
======================
### About

The code prints out the degree to which the ma730 encoder has been spun.

### Usage

Hardware required: an ma730, motor, and arduino. 
First, attatch the ma730 to a motor (if it is not already attatched). 
Then, connect the ma730 to an arduino. To do this, first connect the VCC and GND pin on the ma730 to the corresponding pins on the arduino. 
In order to get data from the ma730, we are going to use SPI, so we will need MOSI (Master-Out-Slave-In), MISO (Master-In-Slave-Out), SCLK (Serial Clock), and SS (Slave Select) signals. You can use the ma730 datasheet to find the correct pins on the ma730 to connect to the arduino. The testing script specifies the correct pins on the arduino.  
Finally, plug the arduino into your computer. 
Then, upload the script and open the serial monitor on your computer to see the degrees of rotation. 


### Notes
Datasheet(Chip):
https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MA730/document_id/3563/
Datasheet(Board):
https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/TBMA730-Q-RD-01A/document_id/3550/

