/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "platform/mbed_thread.h"

Timer TimingLidar;
RawSerial pc(USBTX, USBRX, 115200);
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

struct Obstacle {
    int StartAngle;
    int EndAngle;
    int Distance;
};

Obstacle obstacles[360];

void LidarReceive(void){
    char data = Lidar.getc();
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

int main()
{
    // Initialise the digital pin LED1 as an output
    Lidar.enable_input(true);
    Lidar.attach(&LidarReceive);

    while (true) {
        
    }
}
