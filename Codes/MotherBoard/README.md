# Specifications
- Board : STM32-L476RG
- MBed OS version : MBed 5
- Antenna : Antenna 1/4 wave length SMA 90deg 868MHz
- FM Module : BRAVO-T868
- Gas Sensor 1 : Figaro TGS813-A00
- Gas Sensor 2 : MQ135

# Version Details
- V0.1 : Test of communication with Odometry Board, fully operationnal
- V1.1 : Test of communication with Lidar Board, fully operationnal
- V2.1 : Implementation of gas sensor codes, SOMO_II handling for sounds, communication with Odometry board that works perfectly, Lidar communication that does not work due to conflict with SOMO_II declaration And FM communication that does not work due to random bytes reversing caused by the FM module
- V2.2 : Addition of all V2.1 code except for FM communication and SOMO_II and Lidar conflict solved
- V3.1 : Addition of first test of strategy, not concluing
- V3.2 : Update of data management for the strategy, fix and upgrade of array filling of the strategy type array and edition of the angle of calculation for strategy
