/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"

#define LIDAR_MODE_STOP     0x25
#define LIDAR_MODE_RESET    0x40
#define LIDAR_MODE_EXPRESS  0x82
#define LIDAR_MODE_GETCONF  0x84
#define LIDAR_MODE_INFO     0x50
#define LIDAR_MODE_HEALTH   0x52
#define LIDAR_MODE_RATE     0x59
#define LIDAR_MODE_SCAN     0x20

#define LIDAR_ERROR         0x02
#define LIDAR_WARNING       0x01
#define LIDAR_GOOD          0x00

#define     NB_BYTE_INFO_DESC       7
#define     NB_BYTE_INFO_RESP       20
#define     NB_BYTE_RATE_DESC       7
#define     NB_BYTE_RATE_RESP       4
#define     NB_BYTE_HEALTH_DESC     7
#define     NB_BYTE_HEALTH_RESP     3
#define     NB_BYTE_FORCE_DESC      7
#define     NB_BYTE_FORCE_RESP      5
#define     NB_BYTE_SCAN_DESC       7
#define     NB_BYTE_SCAN_RESP       5
#define     NB_BYTE_CONF_DESC       7
#define     NB_BYTE_CONF_RESP       4

RawSerial lidar(PA_0, PA_1);
RawSerial PC(USBTX, USBRX);

PwmOut lidarCTRL(PB_4);

char pc_debug_data[128], received_descriptor[7], received_data[64], mode = LIDAR_MODE_STOP;

int data_nb = 0, data_scan_nb = 0;
int distance_scan[360] = {0}, quality_scan[360] = {0};
int distance_scan_old[360] = {0}, quality_scan_old[360] = {0};

bool Sending = false;

struct lidar_data{
    int quality;
    int angle;
    int distance;
    bool start;
    bool NStart;
    bool C;
};

lidar_data currentValues;

void SerialInterrupt(void);
void InitLidar(void);
void GetLidarHealth(void);
void GetLidarInfo(void);
void GetSampleRate(void);
void ResetLidar(void);
void StartSingleScan(void);
void Configure(void);
void Stop(void);

int main()
{
    InitLidar();
    StartSingleScan();
    while (true) {

    }
}

void SerialInterrupt(void){
    char data = 0x00;
    if(data_nb == 0){
        received_data[64] = {0};
        received_descriptor[7] = {0};
    }
    if(lidar.readable()){
        data = lidar.getc();
        switch(mode){

            case LIDAR_MODE_HEALTH:
                if(data_nb < NB_BYTE_HEALTH_DESC){
                    received_descriptor[data_nb] = data;
                } else {
                    received_data[data_nb-NB_BYTE_HEALTH_DESC] = data;
                }
                break;

            case LIDAR_MODE_INFO:
                if(data_nb < NB_BYTE_INFO_DESC){
                    received_descriptor[data_nb] = data;
                } else {
                    received_data[data_nb-NB_BYTE_INFO_DESC] = data;
                }
                break;

            case LIDAR_MODE_RATE:
                if(data_nb < NB_BYTE_RATE_DESC){
                    received_descriptor[data_nb] = data;
                } else {
                    received_data[data_nb-NB_BYTE_RATE_DESC] = data;
                }
                break;

            case LIDAR_MODE_SCAN:
                if(data_nb < NB_BYTE_SCAN_DESC){
                    received_descriptor[data_nb] = data;
                } else {
                    if(data_nb == (NB_BYTE_SCAN_DESC + NB_BYTE_SCAN_RESP)){
                        data_nb = NB_BYTE_SCAN_DESC;
                        currentValues.start = received_data[0] & 0x01;
                        currentValues.NStart = (received_data[0] >> 1) & 0x01;
                        currentValues.quality = received_data[0] >> 2;
                        currentValues.C = received_data[1] & 0x01;

                        float thisAngle = ((received_data[1] >> 1) | (received_data[2] << 7))*6.0/64.0;
                        if(fmod(thisAngle,1) > 0.5){
                            thisAngle = ceil(thisAngle);
                        }else{
                            thisAngle = floor(thisAngle);
                        }
                        currentValues.angle = thisAngle;
                        currentValues.distance = (received_data[3] | (received_data[4] << 8));
                        //PC.printf("\n\rAngle : %d \n\r", currentValues.angle);
                        distance_scan[currentValues.angle] = currentValues.distance;
                        quality_scan[currentValues.angle] = currentValues.quality;
                        PC.printf("\n\rQuality : %d , Angle : %d , Distance : %d , Start : %o , Nstart : %o , C : %o \n\r", currentValues.quality, currentValues.angle, currentValues.distance, currentValues.start, currentValues.NStart, currentValues.C);
                        
                    }
                    received_data[data_nb-NB_BYTE_SCAN_DESC] = data;
                    //PC.printf("\n\r%#X", data);
                }
                break;
        }    
        data_nb ++;  
        /*if(mode == LIDAR_MODE_SCAN && data_nb == (NB_BYTE_SCAN_RESP + NB_BYTE_SCAN_DESC)){
            currentValues.quality = received_data[7] >> 2;
            currentValues.angle = (received_data[8] >> 1) + (received_data[9]);
            currentValues.distance = (0x0000 | received_data[10]) | ((0x0000 | received_data[11]) << 8);
            
        }*/
    }
    return;
}

void InitLidar(void){
    lidar.baud(115200);
    wait(2);
    lidarCTRL.period(1/25000.0);
    lidarCTRL.write(0.5);
    wait(2);
    PC.printf("\r\nLIDAR Testing\r\n");
    lidar.attach(&SerialInterrupt);
    wait_ms(500);
    PC.printf("\r\nLIDAR OK\r\n");
 
    ResetLidar();
    GetLidarHealth();
    GetLidarInfo();
    GetSampleRate();
    //Configure();
}

void GetLidarHealth(){
    mode = LIDAR_MODE_HEALTH;
    PC.printf("\n\rChecking lidar Health state ...\n\r");
    char data[2] = {0xA5, LIDAR_MODE_HEALTH};
    lidar.putc(data[0]);
    lidar.putc(data[1]);
    data_nb = 0;
    while(data_nb != (NB_BYTE_HEALTH_DESC + NB_BYTE_HEALTH_RESP)){__nop();}
    //print_data("Health", received_data, (NB_BYTE_HEALTH_REQ + NB_BYTE_HEALTH_RESP));
    /*for(int i = 0; i < data_nb; i++){
        PC.printf("%#X\n\r",received_data[i]);
    }*/
    if(received_data[0] == LIDAR_GOOD) PC.printf("\r\nLidar is GOOD to go !\r\n");
    else if(received_data[0] == LIDAR_WARNING) PC.printf("\r\nLidar is in Warning State but will still run !\r\n");
    else   PC.printf("\r\nError, Lidar is in Bad state, please check connectivity and Lidar.\r\nExiting Code ...\n\r");
    
    Stop();
}

void GetLidarInfo(void){
    mode = LIDAR_MODE_INFO;
    PC.printf("\n\rAcquiring lidar infos ...\n\r");
    char data[5] = {0xA5, LIDAR_MODE_INFO, 0x21, 0x05, 0x24};
    for(int i = 0; i< 5; i++){
        lidar.putc(data[i]);
    }
    data_nb = 0;
    while(data_nb != (NB_BYTE_INFO_DESC + NB_BYTE_INFO_RESP)){__nop();}
    /*for(int i = 0; i < data_nb; i++){
        PC.printf("%#X\n\r",received_data[i]);
    }*/
    PC.printf("\n\rLidar model %d\n\r", received_data[0]);
    PC.printf("Lidar Firmware Version : %d.%d\n\r", received_data[2], received_data[1]);
    PC.printf("Lidar hardawe Version : %d\n\r", received_data[3]);
    PC.printf("Lidar Serial number : ");
    for(int i = 0; i < 16; i++){
        PC.printf("%d", received_data[4+i]);
    }
    PC.printf("\n\r");
    
    Stop();
}

void GetSampleRate(void){
    mode = LIDAR_MODE_RATE;
    PC.printf("\n\rAcquiring lidar Sample Rates ...\n\r");
    char data[2] = {0xA5, LIDAR_MODE_RATE};
    lidar.putc(data[0]);
    lidar.putc(data[1]);
    data_nb = 0;
    while(data_nb != (NB_BYTE_RATE_DESC + NB_BYTE_RATE_RESP)){__nop();}
    //print_data("Rate", received_data, (NB_BYTE_RATE_REQ + NB_BYTE_RATE_RESP));
    int rate = (received_data[1] << 8) + received_data[0];
    PC.printf("\n\rStandard mode Sample Rate (uS) : %d\n\r", rate);
    rate = (received_data[3] << 8) + received_data[2];
    PC.printf("\n\rExpress mode Sample Rate (uS) : %d\n\r ", rate);
    
    Stop();
}

void ResetLidar(void){
    mode = LIDAR_MODE_RESET;
    char data[2] = {0xA5, LIDAR_MODE_RESET};
    lidar.putc(data[0]);
    lidar.putc(data[1]);
    wait_ms(4);
}

void StartSingleScan(void){
    mode = LIDAR_MODE_SCAN;
    PC.puts("\n\rBeginning scan ...\n\r");
    char data[2] = {0xA5, LIDAR_MODE_SCAN};
    lidar.putc(data[0]);
    lidar.putc(data[1]);
    data_nb = 0;
    data_scan_nb = 0;
    
    PC.puts("\n\rScan Started !\n\r");
    /*for(int i = 0; i < data_nb; i++){
        PC.printf("%#X\n\r",received_data[i]);
    }*/
}

void Configure(void){
    mode = LIDAR_MODE_GETCONF;
    PC.printf("\n\rGetting lidar configuration\n\r");
    char data[8] = {0xA5, 0x84, 0x04, 0x75, 0x00, 0x00, 0x00, 0x00};
    char checksum = 0x00;
    for(int i = 0; i < 7; i++){
        checksum ^= data[i];
    }
    data[7] = checksum;
    //char S = ;
    //char check = 0xA5 ^ 0x84 ^ sum ^ 0x00 ^ 0x75 ^ 0x00 ^ 0x00;
    //char data[9] = {0xA5, LIDAR_MODE_GETCONF, 0x21, 0x00, 0x00, 0x00, 0x75, 0x00, check};
    for(int i = 0; i < 8; i++){
        lidar.putc(data[i]);
    }
    data_nb = 0;
    data_scan_nb = 0;
    while(data_nb != (NB_BYTE_CONF_DESC+NB_BYTE_CONF_RESP)){__nop();}
    for(int i = 0; i < data_nb; i++){
        PC.printf("%#X\n\r",received_data[i]);
    }
    PC.printf("\n\rLidar configuration acquired\n\r");
}

void Stop(void){
    mode = LIDAR_MODE_STOP;
    char data[2] = {0xA5, LIDAR_MODE_STOP};
    lidar.putc(data[0]);
    lidar.putc(data[1]);
    wait_ms(2);
}
