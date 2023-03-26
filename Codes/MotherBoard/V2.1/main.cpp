/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "platform/mbed_thread.h"
#include <iostream>
#include <cmath>
#include "SOMO.h"

#define FMTiming    200
#define Vadc_5      0.0012
#define Rl          1000.0
// Blinking rate in milliseconds

DigitalOut  E1(PB_6, PullDown);
DigitalOut  E2(PC_7, PullDown);
DigitalOut  E3(PB_5, PullDown);
DigitalOut  E4(PA_10, PullDown);
DigitalOut  E5(PB_3, PullDown);
DigitalOut  E6(PB_4, PullDown);
DigitalOut  E7(PA_8, PullDown);
DigitalOut  E8(PA_9, PullDown);

AnalogIn MQ135(PC_0);
AnalogIn TGS813(PC_1);

SOMO somo2(PC_4, PC_5);

Timer Tp;
Timer TimingLidar;
Timer Start;

RawSerial pc(USBTX, USBRX, 115200);
RawSerial Odometry(PC_12, PD_2, 38400);
RawSerial Lidar(PC_10, PC_11, 38400);

int ReceivedDataLidar = 0;
int ReceivedDataOdometry = 0;
int StartAngleR = 0;
int EndAngleR = 0;
int DistanceR = 0;
int nbDataLidar = 0;
int nbObstacles = 0;

bool    ReceivingStartAngle = false,
        ReceivingDistance = false,
        ReceivingEndAngle = false,
        NoOsbtacles = false;

double Vrl;                                  // Output voltage
double Rs;                                   // Rs (Ohm) - Sensor resistance
double ppm;   
double ppm_init;                               // ppm
double resRatio;
double lgPPM_init;

float adcRaw_init;

struct Obstacle {
    int StartAngle;
    int EndAngle;
    int Distance;
};

Obstacle obstacles[360];

void SendXY(int x, int y){

}

void LidarReceive(void){
    char data = Lidar.getc();
    pc.printf("\n\rMaster Received %#X\n\r", data);
    //pc.printf("\n\r%#X\n\r",data);
    if(TimingLidar.read_ms() > 500){
        nbObstacles = 0;
    }

    TimingLidar.stop();
    TimingLidar.reset();

    if(ReceivedDataLidar == 0){
        if(data == 78){
            NoOsbtacles = true;
            ReceivingEndAngle = false;
            ReceivingDistance = false;
            ReceivingStartAngle = false;
            ReceivedDataLidar = 0;
        }else if(data == 83){
            NoOsbtacles = false;
            ReceivingEndAngle = false;
            ReceivingDistance = false;
            ReceivingStartAngle = true;
            ReceivedDataLidar++;
        }else if(data == 69){
            NoOsbtacles = false;
            ReceivingEndAngle = true;
            ReceivingDistance = false;
            ReceivingStartAngle = false;
            ReceivedDataLidar++;
        }else if(data == 68){
            NoOsbtacles = false;
            ReceivingEndAngle = false;
            ReceivingDistance = true;
            ReceivingStartAngle = false;
            ReceivedDataLidar++;
        }else;

    }else if(ReceivedDataLidar == 1){
        if(ReceivingStartAngle && !ReceivingDistance && !ReceivingEndAngle){
            StartAngleR = 0xFF & data;
            ReceivedDataLidar++;
        }else if(!ReceivingStartAngle && !ReceivingDistance && ReceivingEndAngle){
            EndAngleR = 0xFF & data;
            ReceivedDataLidar++;
        }else if(!ReceivingStartAngle && ReceivingDistance && !ReceivingEndAngle){
            nbDataLidar = 0xFF & data;
            ReceivedDataLidar++;
        }else{
            ReceivedDataLidar = 0;
        }
        
    }else if(ReceivedDataLidar == 2){
        if(ReceivingStartAngle && !ReceivingDistance && !ReceivingEndAngle){
            StartAngleR |= (data << 8);
            ReceivingStartAngle = false;
            obstacles[nbObstacles].StartAngle = StartAngleR;
            ReceivedDataLidar = 0;
        }else if(!ReceivingStartAngle && !ReceivingDistance && ReceivingEndAngle){
            EndAngleR |= (data << 8);
            ReceivingEndAngle = false;
            obstacles[nbObstacles].EndAngle = EndAngleR;
            ReceivedDataLidar = 0;
        }else if(!ReceivingStartAngle && ReceivingDistance && !ReceivingEndAngle){
            DistanceR = 0xFF & data;
            ReceivedDataLidar++;
        }else{
            ReceivedDataLidar = 0;
        }
    }else{
        if(!ReceivingStartAngle && ReceivingDistance && !ReceivingEndAngle){
            DistanceR |= (data << (8*(ReceivedDataLidar-2)));
            ReceivedDataLidar++;
            if((ReceivedDataLidar-2) == nbDataLidar){
                ReceivingDistance = false;
                obstacles[nbObstacles].Distance = DistanceR;
                pc.printf("\n\r AngleStart = %d -- AngleStop = %d -- Distance = %d \n\r", obstacles[nbObstacles].StartAngle, obstacles[nbObstacles].EndAngle, obstacles[nbObstacles].Distance);
                nbObstacles++;
                ReceivedDataLidar = 0;
            }
        }else{
            ReceivedDataLidar = 0;
        }
    }
    TimingLidar.start();
}

void OdometryReceive(void){
    char data = Lidar.getc();
    //pc.printf("\n\r%#X\n\r",data);
    pc.printf("\n\r%#X\n\r",data);
}

int MQ135_test(void){
    float adcRaw  = MQ135.read();
        
    double lgPPM;
    Vrl = (double)adcRaw*Vadc_5;             // For 5V Vcc use Vadc_5
    Rs = (5 - Vrl)/Vrl;                   // Calculate sensor resistance
    resRatio = Rs/Rl;                             // Calculate ratio
    lgPPM = (log10(resRatio) * -0.8)+ 0.9;        // Calculate ppm
    ppm = pow(10,lgPPM)*1000;                       // Calculate ppm
    //printf("The ppm value is %lf\n\r",ppm);
    //printf("The raw Rs is %2.3lf\n\r",resRatio);
    //printf("The raw voltage is %2.3lf\n\r",Vrl);

    //Detection de gaz :
    double diff = ppm - ppm_init;
    //butane
    if (diff >= 400 && diff < 1000){
        //printf("niveau de Co2 inquétant\n\r");
        return 3;
    }
    //Co2
    else if (diff >= 1000){
        //printf("niveau de butane inquiétant\n\r");
        return 2;
    }
    //NH3
    else if (diff <= -200){
        //printf("Fumée detectée\n\r");
        return 1;
    }
    else {
        //printf ("Rien à signaler\n\r");
        return 0;
    }
}

int TGS813_test(void){
    float tgsValue = TGS813.read();
    //pc.printf("value = %f\n\r", tgsValue);
    if(tgsValue > 0.08){
        return 1;
    }else{
        return 0;
    }
}

int main()
{
    //wait(20);
    // Initialise the digital pin LED1 as an output
    //char data[3] = {0xA5, 0x10};
    //transmitFM(data);
    pc.printf("\n\rMaster Starting Up\n\r");
    Start.start();
    adcRaw_init = MQ135.read();
    Vrl = (double)adcRaw_init*Vadc_5;             // For 5V Vcc use Vadc_5
    Rs = (5 - Vrl)/Vrl;                   // Calculate sensor resistance
    resRatio = Rs/Rl;                             // Calculate ratio
    lgPPM_init = (log10(resRatio) * -0.8)+ 0.9;        // Calculate ppm
    ppm_init = pow(10,lgPPM_init)*1000;

    bool alarm = false;

    somo2.init();
    somo2.volume(20);
    somo2.start();
    Tp.start();
    TimingLidar.start();
    Lidar.enable_input(true);
    Lidar.attach(&LidarReceive);
    Odometry.attach(&OdometryReceive);

    __enable_irq();

    while(Start.read() < 3);
    Start.stop();
    Start.reset();
    pc.printf("\n\rMaster Ready\n\r");
    
    while (true) {
        //pc.printf("\n\rMaster Waiting\n\r");
        if(Lidar.readable()){
            LidarReceive();
        }
        //transmitFM(data);
        /*if(MQ135_test() == 0 && TGS813_test() == 0){
            alarm = false;
            if(Tp.read()>5){
                somo2.volume(0x15);
                somo2.playTrackNum(0x01,0x02);
                Tp.stop();
                Tp.reset();
                Tp.start();
            }
        }else{
            if(alarm == false || Tp.read()>55){
                alarm = true;
                somo2.volume(0x0A);
                somo2.playTrackNum(0x01,0x03);
                Tp.stop();
                Tp.reset();
                Tp.start();
            }
        }*/
        //TGS813_test();
    }
}

