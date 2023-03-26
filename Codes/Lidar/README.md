# Specifications
- Lidar : RPLidar A1M8
- Board : STM32-F446RE
- Library works with all RPLidar versions

# Version Details
- V0.1 : Version not working, First test done on lidar communication
- V0.2 : Version not working, Second test done on lidar communication using Lidar Protocole
- V1.1 : First Version using lidar protocole and having trouble to work (working 1/4 of the time)
- V1.2 : First Working version but not finished
- V2.1 : Final version, fully functionnal and using all function for communication with MotherBoard

# Communication Protocole
- Type : Serial
- Baud Rate : 19200
- Warning : You cannot send any data to the Lidar Board, the Lidar Board will only transmit data to the Mother Board and will never read any data you could send to it !

## Receiving :
Lidar will transmit to the Mother board the following Bytes regulary with 10ms spacing between each Byte

Data will arrive only when the Lidar Board finished sampling the environement
- First Byte will be either "N" in ascii or "S" in ascii representing for "N" "No obstacles detected" and for "S" "Start angle of obstacle" in degrees
- In case you received "N" :
  * No more Bytes will be transmitted
- In case you received "S" :
  * Second and Third Byte are the data of the start angle, keep in mind that the data you are receiving is transmitted lsb meaning if you are receiving for example the 2 bytes of value "11010100 10110001" your first byte would be "10110001" and the second one is "11010100"
  * Fourth Byte will be "E" for "End angle of obstacle"
  * Fifth and sixth Byte are the data of the end angle, keep in mind that the data you are receiving is transmitted lsb meaning if you are receiving for example the 2 bytes of value "11010100 10110001" your first byte would be "10110001" and the second one is "11010100"
  * seventh Byte will be "D" for "Minimum distance to obstacle" in millimeters
  *
- You can either recreate your own function or use the existing one called "void SendXY(int x, inty)" that does every previous steps for you and is fully operationnal
