#include "mbed.h"
#include "rplidar.h"
#include <cstdio>
 
void print_int(const char *name, int ki){
    pc.printf("\t %s = %d\r\n", name, ki);
}
 
void print_data(const char *name, char *datai, int sizedata){
    pc.printf("\t %s = ", name);
    for(int i = 0; i < sizedata; i++){
        pc.printf("%x ", datai[i]);
    }
    pc.printf("\r\n");
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
 
void IT_lidar(void){
    char data, startt, nostartt;
    debug_data = 1;
    data = lidar.getc();
 
    if(scan_ok){
        switch(data_scan_nb % 5){
            case 0 :
                data_ok = 0;
                data_ok_q = 0;
                if (((data&0X03)==0X01) || ((data&0X03)==0X02)) { 
                    trame_ok=1;
                } else {
                    trame_ok=0;
                }
                ld_current.quality = data >> 2;
                startt = data & 0x01;
                nostartt = (data & 0x02) >> 1;
                if((data & 0x01) == 0x01){
                    debug_out = 1;
                    for(int k = 0; k < 360; k++){
                        distance_scan_old[k] = distance_scan[k];
                        distance_scan[k] = 0;
                        angle_scan_old[k] = angle_scan[k];
                        angle_scan[k] = 0;
                    }
                    debug_out = 0;
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
                    angle_scan[ld_current.angle%360] = ld_current.angle%360;
                    data_ok = 1;
                    if(ld_current.quality > 0) data_ok_q = 1;
                }         
        }
        data_scan_nb++;
    }
    else{
        data_ok = 0;
        data_ok_q = 0;
        received_data[data_nb] = data;
        data_nb++;
    }
    debug_data = 0;
}
 
 
void sendResetReq(void){
    mode = LIDAR_MODE_RESET;
    char data[2] = {0xA5, 0x40};
    lidar.putc(data[0]);
    lidar.putc(data[1]);
    wait_us(10000);
}
 
void getHealthLidar(void){
    stopScan();
    mode = LIDAR_MODE_HEALTH;
    char data[2] = {0xA5, LIDAR_MODE_HEALTH};
    lidar.putc(data[0]);
    lidar.putc(data[1]);
    data_nb = 0;
    while(data_nb != (NB_BYTE_HEALTH_REQ + NB_BYTE_HEALTH_RESP)){__nop();}
    //print_data("Health", received_data, (NB_BYTE_HEALTH_REQ + NB_BYTE_HEALTH_RESP));
    if(received_data[7] == 0)   pc.printf("\r\nGOOD\r\n");
    else   pc.printf("\r\nBAD\r\n");
 
}
 
void getInfoLidar(void){
    stopScan();
    mode = LIDAR_MODE_INFO;
    char data[2] = {0xA5, LIDAR_MODE_INFO};
    lidar.putc(data[0]);
    lidar.putc(data[1]);
    data_nb = 0;
    while(data_nb != (NB_BYTE_INFO_REQ + NB_BYTE_INFO_RESP)){__nop();}
    print_data("Info", received_data, (NB_BYTE_INFO_REQ + NB_BYTE_INFO_RESP));
}
 
void getSampleRate(void){
    stopScan();
    mode = LIDAR_MODE_RATE;
    char data[2] = {0xA5, LIDAR_MODE_RATE};
    lidar.putc(data[0]);
    lidar.putc(data[1]);
    data_nb = 0;
    while(data_nb != (NB_BYTE_RATE_REQ + NB_BYTE_RATE_RESP)){__nop();}
    print_data("Rate", received_data, (NB_BYTE_RATE_REQ + NB_BYTE_RATE_RESP));
    int usRate = (received_data[8] << 8) + received_data[7];
    print_int("Standard (uS) ", usRate);
    usRate = (received_data[10] << 8) + received_data[9];
    print_int("Express (uS) ", usRate);
}
 
void startScan(void){
    stopScan();
    mode = LIDAR_MODE_SCAN;
    char data[2] = {0xA5, LIDAR_MODE_SCAN};
    lidar.putc(data[0]);
    lidar.putc(data[1]);
    data_nb = 0;
    data_scan_nb = 0;
    while(data_nb != (NB_BYTE_SCAN_REQ)){__nop();}
    pc.printf("over");
    scan_ok = 1;
}
 
void stopScan(void){
    mode = LIDAR_MODE_STOP;
    scan_ok = 0;
    char data[2] = {0xA5, LIDAR_MODE_STOP};
    lidar.putc(data[0]);
    lidar.putc(data[1]);
}
