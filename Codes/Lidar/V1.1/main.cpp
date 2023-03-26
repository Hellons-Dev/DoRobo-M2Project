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
 
#define     NB_BYTE_INFO_REQ        7
#define     NB_BYTE_INFO_RESP       20
#define     NB_BYTE_RATE_REQ        7
#define     NB_BYTE_RATE_RESP       4
#define     NB_BYTE_HEALTH_REQ      7
#define     NB_BYTE_HEALTH_RESP     3
#define     NB_BYTE_FORCE_REQ       7
#define     NB_BYTE_FORCE_RESP      5
#define     NB_BYTE_SCAN_REQ        7

RawSerial lidar(PA_0, PA_1);
RawSerial debug_pc(USBTX, USBRX);

PwmOut lidar_ct(PB_4);

char            pc_debug_data[128];
char            received_data[64];
int             data_nb = 0;
int             data_scan_nb = 0;
char            mode = LIDAR_MODE_STOP;
char            scan_ok = 0;
int             distance_scan[360] = {0};
int             distance_scan_old[360] = {0};
char            tour_ok = 0;
char            trame_ok = 0;

struct lidar_data{
    int quality;
    int angle;
    int distance;
};

lidar_data   ld_current;

void initLidar(void);
void testLidar(void);
void print_int(const char *name, int ki);
void print_data(const char *name, char *datai, int sizedata);
void wait_s(float sec);
void findMax(int *int_data, int angle_min, int angle_max, int *value, int *indice);
void IT_Process(void);
void IT_lidar(void);
void sendResetReq(void);
void getHealthLidar(void);
void getInfoLidar(void);
void getSampleRate(void);
void startScan(void);
void stopScan(void);

int main()
{
    initLidar();
    //int i = 0;
    /*while(i < 6){
        startScan();
        i++;
    }*/
    
    testLidar();
    // Initialise the digital pin LED1 as an output

    while (true) {
        startScan();
        //debug_pc.printf("angle : %d , distance : %d , quality : %d\n\r", ld_current.angle, ld_current.distance, ld_current.quality);
    }
}

void initLidar(void){
    lidar.baud(115200);
    wait_ms(2000);
    lidar_ct.period(1/25000.0);
    lidar_ct.write(0.4);
    wait_ms(2000);
    debug_pc.printf("\r\nLIDAR Testing\r\n");
    lidar.attach(&IT_lidar);
    wait_ms(500);
    debug_pc.printf("\r\nLIDAR OK\r\n");
 
    getHealthLidar();
    getInfoLidar();
    getSampleRate(); 
}

// Fonction de test du Lidar
void testLidar(){
    if(tour_ok == 6){
        int maxDistance, maxAngle;
        tour_ok = 0;
        findMax(distance_scan_old, 0, 360, &maxDistance, &maxAngle);
        print_int("A", maxAngle);
    }
}
 
void print_int(const char *name, int ki){
    debug_pc.printf("\t %s = %d\r\n", name, ki);
}
 
void print_data(const char *name, char *datai, int sizedata){
    debug_pc.printf("\t %s = ", name);
    for(int i = 0; i < sizedata; i++){
        debug_pc.printf("%x ", datai[i]);
    }
    debug_pc.printf("\r\n");
}
 
void wait_s(float sec){
    wait_us(sec*1000000);
}
 
void findMax(int *int_data, int angle_min, int angle_max, int *value, int *indice){
    *value = 0;
    *indice = 0;
    for(int k = angle_min; k < angle_max; k++){
        if(int_data[k] > *value){
            *value = int_data[k];
            *indice = k;
        }
    }
}

void IT_Process(void){
    char data, startt, nostartt;
    data = lidar.getc();

    debug_pc.printf("%d\r\n",data);
 
    if(scan_ok){
        switch(data_scan_nb % 5){
            case 0 :
                if (((data&0X03)==0X01) || ((data&0X03)==0X02)) { 
                    trame_ok=1;
                } else {
                    trame_ok=0;
                }
                ld_current.quality = data >> 2;
                startt = data & 0x01;
                nostartt = (data & 0x02) >> 1;
                if((data & 0x01) == 0x01){
                    for(int k = 0; k < 360; k++){
                        distance_scan_old[k] = distance_scan[k];
                        distance_scan[k] = 0;
                    }
                    tour_ok++;
                }   
                if(startt == nostartt)      data_scan_nb = 0;
                break;            
            case 1 :
                if((data&0x01) == 0){
                    trame_ok = 0;
                    data_scan_nb = 0;
                }
                // angle_q6[6:0] / 64 and check (degre)
                ld_current.angle = data >> (1 + 6);
                // check ?
                break;            
            case 2 :
                // angle_q6[14:7] / 64 (degre)
                ld_current.angle += data << 1;
                break;            
            case 3 :
                // distance_q2[7:0] / 4 (mm)
                ld_current.distance = data >> 2;
                break;
            default :
                // distance_q2[15:8] / 4 (mm)
                ld_current.distance += data << 6;
                if(trame_ok){
                    distance_scan[ld_current.angle%360] = ld_current.distance;
                }         
        }
        data_scan_nb++;
    }
    else{
        received_data[data_nb] = data;
        data_nb++;
    }
}
 
void IT_lidar(void){
    IT_Process();
}
 
 
void sendResetReq(void){
    mode = LIDAR_MODE_RESET;
    char data[2] = {0xA5, 0x40};
    lidar.putc(data[0]);
    lidar.putc(data[1]);
    wait_ms(4);
}
 
void getHealthLidar(void){
    stopScan();
    mode = LIDAR_MODE_HEALTH;
    char data[2] = {0xA5, LIDAR_MODE_HEALTH};
    lidar.putc(data[0]);
    lidar.putc(data[1]);
    data_nb = 0;
    while(data_nb != (NB_BYTE_HEALTH_REQ + NB_BYTE_HEALTH_RESP)){
        __nop();
        /*if(lidar.readable()){
            IT_lidar();
        }*/
    }
    //print_data("Health", received_data, (NB_BYTE_HEALTH_REQ + NB_BYTE_HEALTH_RESP));
    if(received_data[7] == 0)   debug_pc.printf("\r\nGOOD\r\n");
    else   debug_pc.printf("\r\nBAD\r\n");
 
}
 
void getInfoLidar(void){
    stopScan();
    mode = LIDAR_MODE_INFO;
    char data[2] = {0xA5, LIDAR_MODE_INFO};
    lidar.putc(data[0]);
    lidar.putc(data[1]);
    data_nb = 0;
    while(data_nb != (NB_BYTE_INFO_REQ + NB_BYTE_INFO_RESP)){
        __nop();
        /*if(lidar.readable()){
            IT_lidar();
        }*/
    }
    print_data("Info", received_data, (NB_BYTE_INFO_REQ + NB_BYTE_INFO_RESP));
}
 
void getSampleRate(void){
    stopScan();
    mode = LIDAR_MODE_RATE;
    char data[2] = {0xA5, LIDAR_MODE_RATE};
    lidar.putc(data[0]);
    lidar.putc(data[1]);
    data_nb = 0;
    while(data_nb != (NB_BYTE_RATE_REQ + NB_BYTE_RATE_RESP)){
        __nop();
        /*if(lidar.readable()){
            IT_lidar();
        }*/
    }
    print_data("Rate", received_data, (NB_BYTE_RATE_REQ + NB_BYTE_RATE_RESP));
    int usRate = (received_data[8] << 8) + received_data[7];
    print_int("Standard (uS) ", usRate);
    usRate = (received_data[10] << 8) + received_data[9];
    print_int("Express (uS) ", usRate);
}
 
void startScan(void){
    //sendResetReq();
    stopScan();
    mode = LIDAR_MODE_SCAN;
    char data[2] = {0xA5, LIDAR_MODE_SCAN};
    lidar.putc(data[0]);
    lidar.putc(data[1]);
    data_nb = 0;
    data_scan_nb = 0;
    while(data_nb != (NB_BYTE_SCAN_REQ)){
        __nop();
        /*if(lidar.readable()){
            IT_lidar();
        }*/
    }
    //debug_pc.printf("over");
    scan_ok = 1;
}
 
void stopScan(void){
    mode = LIDAR_MODE_STOP;
    scan_ok = 0;
    char data[2] = {0xA5, LIDAR_MODE_STOP};
    lidar.putc(data[0]);
    lidar.putc(data[1]);
    wait_ms(2);
}
