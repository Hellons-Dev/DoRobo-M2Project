# Specifications
- Board : STM32-L476RG
- Rotary Encoders : Rotary encoder 16mm 24 detents switch
- DC motors controllers : PMod HB3 2A H-bridge Driver + F/B
- DC motors : DC Motor 4.5-15V 35mm 1:148 Low Noise
- PowerBank : PowerBank 10.0 PD

# Version Details
- V0.1 : Test version for PWM outputs on board
- V1.1 : First working version with errors on angle detection and distance measurement
- V2.1 : New version and fully fonctionnal one with stategy modification using radians instead of degrees, Fully operationnal and communicating with Mother Board

# Communication Protocole
- Type : Serial
- Baud Rate : 19200

## Transmitting :
To transmit to Odometry you must transmit the following bytes with at least 10 ms of spacing between bytes.
- First Byte must be either "X" in ascii or "Y" in ascii, depending on the X or Y coordinates you want to transmit
- Second Byte is the sign of the data, 0x01 is "+" and 0x00 is "-"
- Third Byte is the number of bytes coding the data
- Next Bytes are the data, keep in mind that the data you are transmitting should be transmitted lsb meaning if you are transmitting for example 2 bytes containing the value "11010100 10110001" your first byte would be "10110001" and the second one is "11010100"
- You can either recreate your own function or use the existing one called "void SendXY(int x, inty)" that does every previous steps for you and is fully operationnal

## Receiving :
Odometry will transmit to the Mother board the following Bytes regulary with 10ms spacing between each Byte
- First Byte will be either "X" in ascii, "Y" in ascii or "A" in ascii, representing on the X or Y coordinates or for the "A" the angle of the robot in degrees
- In case you received "X" or "Y" :
  * Second Byte is the sign of the data, 0x01 is "+" and 0x00 is "-"
  * Third Byte is the number of bytes coding the data
  * Next Bytes are the data, keep in mind that the data you are receiving is transmitted lsb meaning if you are receiving for example 2 bytes containing the value "11010100 10110001" your first byte would be "10110001" and the second one is "11010100"
- In case you received "A":
  * Second and Third Byte are the data, keep in mind that the data you are receiving is transmitted lsb meaning if you are receiving for example the 2 bytes of value "11010100 10110001" your first byte would be "10110001" and the second one is "11010100"
- You can either recreate your own function or use the existing one called "void OdometryReceive(void)" that does every previous steps for you and is fully operationnal when used as RawSerial Interrupt
