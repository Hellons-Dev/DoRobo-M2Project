/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "platform/mbed_thread.h"
#include <cmath>
#include <ctime>

// Defining constants
#define EntraxeD            14.00
#define EntraxeL            15.00
#define pi                  3.14159265358979323846
#define wheelDiameter       4.35
#define ticksPerRotation    72
#define IntDelay            10
#define Precision           100

// Defining Pwm Outputs

PwmOut ENR(PB_10);
PwmOut ENL(PB_5);

// Defining Digital Outputs

DigitalOut DIRR(PB_4, PullDown);
DigitalOut DIRL(PB_3, PullDown);

// Defining Inputs Interruptions

InterruptIn EncCLKR(PA_5);
InterruptIn EncCLKL(PC_7);

// Defining Variables for distances, positions and angles

double  DL =            0.0, 
        DR =            0.0,
        x =             0.0, 
        y =             0.0, 
        theta =         0.0, 
        x_goal =        0.0, 
        y_goal =        0.0, 
        theta_goal =    0.0, 
        goalDist =      0.0, 
        theta_err =     0.0;

// Defining Flags

bool    flagAL =        false, 
        flagBL =        false, 
        flagAR =        false, 
        flagBR =        false, 
        dirLFlag =      true, 
        dirRFlag =      true, 
        theta_Flag =    false, 
        Dest_Flag =     false,
        XY =            false,
        Signe =         false;

int     ReceivedData =  0,
        nbData =        0;
int contheta;
char    tempX[50] = {0};
char    tempY[50] = {0};

// Defining Timers for interruption bounding and data transmition

Timer leftIsr;
Timer rightIsr;
Timer Send;
Timer Transmit;

// Defining Seriale communication for pc

RawSerial pc(USBTX, USBRX);
RawSerial Master(PC_10, PC_11, 19200);

// Defining typedef enumeration for motors and robot and creating related objects

typedef enum{Standby, Forward, Backward}Motor;
typedef enum{Sleep, RotateR, RotateL, Go}Robot;

Motor LeftMotor = Standby;
Motor RightMotor = Standby;

Robot Dorobo = Sleep;

/**
 * Function isrL
 * Usage : Interruption called function for left encoder impulses
 * input : none
 * output : none
**/

void isrL(void){
    if(dirLFlag == true){
        DL += (1.0/ticksPerRotation)*pi*wheelDiameter;
        flagBL = true;
    } else {
        DL -= (1.0/ticksPerRotation)*pi*wheelDiameter;
        flagBL = false;
    }
    EncCLKL.rise(NULL);
    leftIsr.start();
    flagAL = true;
    return;
}

/**
 * Function isrR
 * Usage : Interruption called function for Right encoder impulses
 * input : none
 * output : none
**/

void isrR(void){
    if(dirRFlag == true){
        DR += (1.0/ticksPerRotation)*pi*wheelDiameter;
        flagBR = true;
    } else {
        DR -= (1.0/ticksPerRotation)*pi*wheelDiameter;
        flagBR = false;
    }
    EncCLKR.rise(NULL);
    rightIsr.start();
    flagAR = true;
    return;
}

/**
 * Function MR
 * Usage : Function used for right motor managing
 * input : none
 * output : none
**/

void MR(void){
    switch(RightMotor){
        case Standby:
            ENR.write(0);
            DIRR.write(0);
            if(LeftMotor == Forward) dirRFlag = true;
            else dirRFlag = false;
            break;

        case Forward:
            ENR.write(1);
            DIRR.write(1);
            dirRFlag = true;
            break;

        case Backward:
            ENR.write(1);
            DIRR.write(0);
            dirRFlag = false;
            break;

        default:
            RightMotor = Standby;
            break;
    }
    return;
}

/**
 * Function ML
 * Usage : Function used for left motor managing
 * input : none
 * output : none
**/

void ML(void){
    switch(LeftMotor){
        case Standby:
            ENL.write(0);
            DIRL.write(0);
            if(RightMotor == Forward) dirLFlag = true;
            else dirLFlag = false;
            break;

        case Forward:
            ENL.write(1);
            DIRL.write(1);
            dirLFlag = true;
            break;

        case Backward:
            ENL.write(1);
            DIRL.write(0);
            dirLFlag = false;
            break;

        default:
            LeftMotor = Standby;
            break;
    }
    return;
}

/**
 * Function RobotManager
 * Usage : FSM for robot
 * Input : none
 * Output : none
**/

void RobotManager(void){
    switch(Dorobo){
        case Sleep:
            RightMotor = Standby;
            LeftMotor = Standby;
            ML();
            MR();
            if(goalDist >= 2.0){
                if(theta_goal < theta - pi/Precision || theta_goal > theta + pi/Precision){
                    if((theta_goal - theta) < 0){
                        if((theta_goal - theta) < -1*pi) Dorobo = RotateL;
                        else Dorobo = RotateR;
                    } else {
                        if((theta_goal - theta) > pi) Dorobo = RotateR;
                        else Dorobo = RotateL;
                    }
                }else{
                    Dorobo = Go;
                }
            }
            break;

        case RotateR:
            RightMotor = Backward;
            LeftMotor = Forward;
            ML();
            MR();
            if((theta_goal - theta) < pi/Precision && (theta_goal - theta) > -pi/Precision){
                Dorobo = Go;
            } else {
                if((theta_goal - theta) < 0){
                    if((theta_goal - theta) < -1*pi) Dorobo = RotateL;
                    else Dorobo = RotateR;
                } else {
                    if((theta_goal - theta) > pi) Dorobo = RotateR;
                    else Dorobo = RotateL;
                }
            }
            break;

        case RotateL:
            RightMotor = Forward;
            LeftMotor = Backward;
            ML();
            MR();
            if((theta_goal - theta) < pi/Precision && (theta_goal - theta) > -pi/Precision){
                Dorobo = Go;
            } else {
                if((theta_goal - theta) < 0){
                    if((theta_goal - theta) < -1*pi) Dorobo = RotateL;
                    else Dorobo = RotateR;
                } else {
                    if((theta_goal - theta) > pi) Dorobo = RotateR;
                    else Dorobo = RotateL;
                }
            }
            break;

        case Go:
            RightMotor = Forward;
            LeftMotor = Forward;
            ML();
            MR();
            if((theta_goal - theta) < pi/Precision && (theta_goal - theta) > -pi/Precision){
                Dorobo = Go;
            } else {
                if((theta_goal - theta) < 0){
                    if((theta_goal - theta) < -1*pi) Dorobo = RotateL;
                    else Dorobo = RotateR;
                } else {
                    if((theta_goal - theta) > pi) Dorobo = RotateR;
                    else Dorobo = RotateL;
                }
            }
            if(goalDist < 2.0){
                Dorobo = Sleep;
            }
            break;

        default:
            Dorobo = Sleep;
            break;
    }
    return;
}

/**
 * Function AngleCalculator
 * Usage : Calculation of robot orientation
 * input : none
 * output : none
**/

void AngleCalculator(void){
    double entraxe = 0;
    if(LeftMotor == Backward && RightMotor == Forward) entraxe = EntraxeL;
    else entraxe = EntraxeD;
    if(DR-DL == 0) theta = 0;
    else theta = fmod(((DR-DL)/entraxe),2*pi);

    if(theta == -0.0) theta = 0;

    if(theta >= 0){
        if(theta > pi) theta = fmod(theta, pi)-pi;
    } else {
        if(theta <= -1*pi) theta = fmod(theta, pi)+pi;
    }
    return;
}

/**
 * Function PositionCalculator
 * Usage : Calculation of robot x and y position
 * input : none
 * output : none
**/


void PositionCalculator(void){
    double adjustedAngle = abs(fmod(theta-(3*pi/2),-2*pi));
    x = x + (cos(adjustedAngle)*pi*wheelDiameter/(2*ticksPerRotation));
    y = y - (sin(adjustedAngle)*pi*wheelDiameter/(2*ticksPerRotation));
    return;
}

void SerialSend(void){
    char signX = (x >= 0 ? 0x01 : 0x00);
    char signY = (y >= 0 ? 0x01 : 0x00);

    int absX = abs(x);
    int absY = abs(y);
    char nbitX;
    char nbitY;
    if(absX >= 1 && absY >=1){
        nbitX = log2(absX)+1;
        nbitY = log2(absY)+1;
    }else{
        nbitX = 0;
        nbitY = 0;
    }

    char nByteX = (8 - fmod(nbitX,8) + nbitX)/ 8;
    char nByteY = (8 - fmod(nbitY,8) + nbitY)/ 8;
    char nByteAngle = 2;  

    for(int i = 0; i < 3+nByteX ; i++){
        if(i == 0){
            Master.putc(88);
        }else if(i == 1){
            Master.putc(signX);
        }else if(i == 2){
            Master.putc(nByteX);
        }else{
            Master.putc((0xFF & (absX >> ((i-3)*8))));
        }
        wait_ms(10);
    }

    for(int i = 0; i < 3+nByteY ; i++){
        if(i == 0){
            Master.putc(89);
        }else if(i == 1){
            Master.putc(signY);
        }else if(i == 2){
            Master.putc(nByteY);
        }else{
            Master.putc((0xFF & (absY >> ((i-3)*8))));
        }
        wait_ms(10);
    }

    Master.putc(65);
    wait_ms(10);
    Master.putc(0xFF & contheta);
    wait_ms(10);
    Master.putc(0xFF & (contheta >> 8));
    wait_ms(10);
    
    return;
}

void Conversion(void){
    contheta=theta*180/pi;
    if(contheta<0){
        contheta=360-abs(contheta);
    }
    return;
}

void SerialReceive(void){
    char data = Master.getc();
    pc.printf("\n\r%#X\n\r",data);
    if(ReceivedData == 0){
        if(data == 88){
            XY = true;
            ReceivedData++;
        }else if(data == 89){
            XY = false;
            ReceivedData++;
        }else;

    }else if(ReceivedData == 1){
        Signe = (data == 0x01 ? true : false);
        ReceivedData++;
    }else if(ReceivedData == 2){
        nbData = data;
        ReceivedData++;
    }else{
        
        if(XY == true){
            tempX[ReceivedData-3] = data;
            ReceivedData++;
        }else{
            tempY[ReceivedData-3] = data;
            ReceivedData++;
        }
        if((ReceivedData - 3) == nbData){
            if(XY == true){
                int value = 0;
                for(int i = 0; i < nbData; i++){
                    value = value | (tempX[i] << (i*8));
                }
                x_goal = value * (Signe == true ? 1 : -1);
            }else{
                int value = 0;
                for(int i = 0; i < nbData; i++){
                    value = value | (tempY[i] << (i*8));
                }
                y_goal = value * (Signe == true ? 1 : -1);
            }
            nbData = 0;
            ReceivedData = 0;
        }
    }
}

int main()
{
    char msg[50];

    EncCLKL.mode(PullUp);
    EncCLKR.mode(PullUp);

    EncCLKL.rise(&isrL);
    EncCLKR.rise(&isrR);

    EncCLKL.fall(NULL);
    EncCLKR.fall(NULL);

    Transmit.start();
    Master.attach(&SerialReceive);

    Send.start();

    while (true) {
        double deltaX = x_goal - x;
        double deltaY = y_goal - y;
        goalDist = sqrt(pow(deltaX,2)+pow(deltaY,2));
        double ThetaDest = atan2(deltaY, deltaX);
        theta_goal = fmod(ThetaDest-pi/2,2*pi);

        if(theta_goal >= 0){
            if(theta_goal > pi) theta_goal = fmod(theta_goal, pi)-pi;
        } else {
            if(theta_goal <= -1*pi) theta_goal = fmod(theta_goal, pi)+pi;
        }

        if(flagAL == true){
            flagAL = false;
            //sprintf(msg, "%s\n\r", flagBL? "true" : "false");
            //pc.printf(msg,sizeof(msg));
        }
        if(flagAR == true){
            flagAR = false;
            //sprintf(msg, "%s\n\r", flagBR? "true" : "false");
            //pc.printf(msg,sizeof(msg));
        }

        if(Send.read_ms() > 1000){
            pc.printf("Distance L: %.2f\n\r", DL);
            pc.printf("Distance R: %.2f\n\r", DR);
            pc.printf("Angle: %.2f\n\r",theta);
            pc.printf("X: %.2f\n\r",x);
            pc.printf("Y: %.2f\n\r",y);
            pc.printf("Theta_goal : %.2f\n\r",theta_goal);
            pc.printf("Theta_err : %.2f\n\r", theta_err);
            pc.printf("Dest_goal : %.2f\n\r", goalDist);
            pc.printf("ThetaDest : %.2f\n\r", ThetaDest);
            pc.printf("\n\r");
            Send.stop();
            Send.reset();
            Send.start();
        }

        if(leftIsr.read_ms() > IntDelay){
            EncCLKL.rise(&isrL);
            leftIsr.stop();
            leftIsr.reset();
            AngleCalculator();
            PositionCalculator();
        }

        if(rightIsr.read_ms() > IntDelay){
            EncCLKR.rise(&isrR);
            rightIsr.stop();
            rightIsr.reset();
            AngleCalculator();
            PositionCalculator();
        }

        /*if(Master.readable()){
            //SerialReceive();
        }*/
        if(Transmit.read_ms() > 250){
            Conversion();
            SerialSend();
            Transmit.stop();
            Transmit.reset();
            Transmit.start();
        }

        RobotManager();
    }
}
