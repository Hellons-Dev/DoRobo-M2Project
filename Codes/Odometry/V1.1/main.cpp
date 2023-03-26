/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "platform/mbed_thread.h"
#include <chrono>

#define Entraxe 10.5
#define pi 3.14159265358979323846
#define wheelDiameter 4.35
#define ticksPerRotation 72

using namespace std::chrono;

// main() runs in its own thread in the OS

PwmOut ENR(D11);
DigitalOut DIRR(D8, PullDown);
PwmOut ENL(D2);
DigitalOut DIRL(D4, PullDown);

InterruptIn EncCLKL(D12);
InterruptIn EncCLKR(D13);

DigitalIn EncL(D14, PullUp);
DigitalIn EncR(D15, PullUp);

double DL = 0.0, DR = 0.0, x = 0.0, y = 0.0, theta = 0.0, x_goal = 10.0, y_goal = 0.0, theta_goal = 0.0, goalDist = 0.0, theta_err = 0.0;

bool flagAL = false, flagBL = false, flagAR = false, flagBR = false, dirLFlag = true, dirRFlag = true, theta_Flag = false, Dest_Flag = false;

Timer leftIsr;
Timer rightIsr;
Timer Send;

Serial pc(USBTX, USBRX);

typedef enum{Standby, Forward, Forward50, Backward, Backward50 }Motor;

Motor LeftMotor = Standby;
Motor RightMotor = Standby;

void PositionCalculator(void);
void AngleCalculator(void);
void isrL(void);
void isrR(void);
void GotoDestination(void);
void GotoDestTheta(void);
void GotoDestDist(void);
void MR(void);
void ML(void);

int main()
{
    ENR.period_ms(100);
    ENL.period_ms(100);
    DIRR = 1;
    DIRL = 1;
    char msg[50];
    EncCLKL.rise(NULL);
    EncCLKR.rise(NULL);
    EncCLKL.fall(&isrL);
    EncCLKR.fall(&isrR);
    EncCLKL.mode(PullUp);
    EncCLKR.mode(PullUp);
    Send.start();
    
    while (true) {
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
        
        if(leftIsr.read_ms() > 50){
            EncCLKL.fall(&isrL);
            leftIsr.stop();
            leftIsr.reset();
            AngleCalculator();
            PositionCalculator();
        }

        if(rightIsr.read_ms() > 50){
            EncCLKR.fall(&isrR);
            rightIsr.stop();
            rightIsr.reset();
            AngleCalculator();
            PositionCalculator();
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
            pc.printf("\n\r");
            Send.stop();
            Send.reset();
            Send.start();
        }
        GotoDestination();
        ML();
        MR();
    }
}

void PositionCalculator(void){
    if(theta >= 0){
        if(90-theta < 0 && theta <= 180){
            double temp = (cos((90-theta)*-1)*180/pi)*((1.0/ticksPerRotation)*pi*wheelDiameter);
            x = x + temp;
        } else {
            double temp = (cos(90-theta)*180/pi)*((1.0/ticksPerRotation)*pi*wheelDiameter);
            x = x + temp;
        }
    } else {
        if(90-(theta*-1) < 0 && (theta*-1) <= 180){
            double temp = (cos((90-(theta*-1))*-1)*180/pi)*((1.0/ticksPerRotation)*pi*wheelDiameter);
            x = x - temp;
        } else {
            double temp = (cos(90-(theta*-1))*180/pi)*((1.0/ticksPerRotation)*pi*wheelDiameter);
            x = x - temp;
        }
    }

    if(theta >= -90 && theta <=90){
        if(theta >=0){
            double temp = (cos(theta)*180/pi)*((1.0/ticksPerRotation)*pi*wheelDiameter);
            y = y + temp;
        } else {
            double temp = (cos((theta * -1))*180/pi)*((1.0/ticksPerRotation)*pi*wheelDiameter);
            y = y + temp;
        }
    } else {
        if(theta <= 180 && theta > 0){
            double temp = (cos(180-theta)*180/pi)*((1.0/ticksPerRotation)*pi*wheelDiameter);
            y = y - temp;
        } else {
            double temp = (cos(180-(theta * -1))*180/pi)*((1.0/ticksPerRotation)*pi*wheelDiameter);
            y = y - temp;
        }
    }
}

void AngleCalculator(void){
    /*theta = ((DL - DR)/Entraxe)*180/pi;
    if(theta < 0 && theta >= -180){
        theta *= -1;
        theta = fmod(theta,360);
        theta *= -1;
    } else if(theta > 0 && theta > 180){
        theta = fmod(theta,360);
    }

    if(theta <= -180){
        theta += 360;
    } else if(theta > 180){
        theta -= 360;
    }*/
    /*

    if(fmod(theta,1) > 0.75){
        if(fmod(theta,1) > 0.875){
            theta = ceil(theta);
        } else {
            theta = ceil(theta)-0.25;
        }
    } else if(fmod(theta,1) > 0.50) {
        if(fmod(theta,1) > 0.625){
            theta = ceil(theta)-0.25;
        } else {
            theta = ceil(theta)-0.50;
        }
    } else if(fmod(theta,1) > 0.25) {
        if(fmod(theta,1) > 0.375){
            theta = ceil(theta)-0.50;
        } else {
            theta = ceil(theta)-0.75;
        }
    } else {
        if(fmod(theta,1) > 0.125){
            theta = ceil(theta)-0.75;
        } else {
            theta = floor(theta);
        }
    }
    */
    if(DL >= DR){
        theta = fmod((((DL - DR)/(Entraxe*0.5))*180/pi),360.0);
        /*if(fmod(theta,1) > 0.5){
            theta = ceil(theta);
        } else {
            theta = ceil(theta)-0.5;
        }*/
        if(theta > 180.0){
            theta -= 360;
        }
    } else {
        theta = fmod((((DR - DL)/(Entraxe*0.5))*180/pi),360.0);
        /*if(fmod(theta,1) > 0.5){
            theta = ceil(theta);
        } else {
            theta = ceil(theta)-0.5;
        }*/
        if(theta > 180.0){
            theta -= 360;
        }
        theta = theta * -1.0;
    }
}

void isrL(void){
    if(dirLFlag == true){
        DL+=(1.0/ticksPerRotation)*pi*wheelDiameter;
        flagBL = true;
    } else {
        DL-=(1.0/ticksPerRotation)*pi*wheelDiameter;
        flagBL = false;
    }
    EncCLKL.fall(NULL);
    leftIsr.start();
    flagAL = true;
}

void isrR(void){
    if(dirRFlag == true){
        DR+=(1.0/ticksPerRotation)*pi*wheelDiameter;
        flagBR = true;
    } else {
        DR-=(1.0/ticksPerRotation)*pi*wheelDiameter;
        flagBR = false;
    }
    EncCLKR.fall(NULL);
    rightIsr.start();
    flagAR = true;
}

void GotoDestination(void){
    /*if(y != 0 && x != 0){
        theta_goal = (atan((y_goal - y)/(x_goal - x))*180)/pi;
    } else if(y == 0 && x != 0) {
        theta_goal = (acos((x_goal - x)/goalDist)*180)/pi;
    } else {
        theta_goal = (asin((y_goal - y)/goalDist)*180)/pi;
    }*/
    double deltaX = x_goal - x;
    double deltaY = y_goal - y;

    if (deltaX < 3.0 && deltaX > -3.0) {
        // deltaX est nul, on traite séparément ce cas
        if (deltaY > 0) {
            theta_goal = 0; // 90 degrés
        } else if(deltaY < 0) {
            theta_goal = 180; // -90 degrés
        } else {
            theta_goal = 0;
        }
    } else if(deltaY < 3.0 && deltaY > -3.0) {
        // deltaX est nul, on traite séparément ce cas
        if (deltaX > 0) {
            theta_goal = 90; // 90 degrés
        } else if (deltaX < 0) {
            theta_goal = -90; // -90 degrés
        } else {
            theta_goal = 0;
        }
    } else {
        // calculer l'angle à partir des coordonnées de destination et de la position actuelle
        theta_goal = (atan(deltaY/ deltaX) * 180) / pi;
    }



    if(theta_goal > 180.0){
        theta_goal -= 360.0;
    }

    goalDist = sqrt(pow(x_goal - x,2)+pow(y_goal - y,2));

    if(theta_goal < theta+1.5 && theta_goal > theta-1.5){
        theta_Flag = true;
    } else {
        theta_Flag = false;
    }

    if(theta_Flag == false){
        GotoDestTheta();
    }

    if(goalDist > 1.5 && theta_Flag == true){
        Dest_Flag = false;
        GotoDestDist();
        RightMotor = Forward;
        LeftMotor = Forward;
    } else if(theta_Flag == true) {
        Dest_Flag = true;
    }

    if(Dest_Flag == true && theta_Flag == true){
        RightMotor = Standby;
        LeftMotor = Standby;
    }
}

void GotoDestTheta(){
    theta_err = theta_goal - theta;

    /*if(fmod(theta_err,1) > 0.75){
        if(fmod(theta_err,1) > 0.875){
            theta_err = ceil(theta_err);
        } else {
            theta_err = ceil(theta_err)-0.25;
        }
    } else if(fmod(theta_err,1) > 0.50) {
        if(fmod(theta_err,1) > 0.625){
            theta_err = ceil(theta_err)-0.25;
        } else {
            theta_err = ceil(theta_err)-0.50;
        }
    } else if(fmod(theta_err,1) > 0.25) {
        if(fmod(theta_err,1) > 0.375){
            theta_err = ceil(theta_err)-0.50;
        } else {
            theta_err = ceil(theta_err)-0.75;
        }
    } else {
        if(fmod(theta_err,1) > 0.125){
            theta_err = ceil(theta_err)-0.75;
        } else {
            theta_err = floor(theta_err);
        }
    }*/

    if(theta_err < 0){
        if(theta_err < -180){
            theta_err += 360;
        }
    }else{
        if(theta_err > 180){
            theta_err -= 360;
        }
    }

    if(theta_err < -1.5){
        RightMotor=Forward;
        LeftMotor=Backward;
    }else if(theta_err > 1.5){
        RightMotor=Backward;
        LeftMotor=Forward;
    }else{
        RightMotor=Standby;
        LeftMotor=Standby;
    }
}

void GotoDestDist(){
    RightMotor = Forward;
    LeftMotor = Forward;
}

void MR(void){
    switch(RightMotor){
        case Standby:
            ENR.write(0.0);
            DIRR = 0;
            dirRFlag = false;
            break;

        case Forward:
            ENR.write(1.0);
            DIRR = 1;
            dirRFlag = true;
            break;

        case Forward50:
            ENR.write(0.10);
            DIRR = 1;
            dirRFlag = true;
            break;

        case Backward:
            ENR.write(1.0);
            DIRR = 0;
            dirRFlag = false;
            break;

        case Backward50:
            ENR.write(0.10);
            DIRR = 0;
            dirRFlag = false;
            break;

        default:
            RightMotor = Standby;
            break;
    }
}

void ML(void){
    switch(LeftMotor){
        case Standby:
            ENL.write(0.0);
            DIRL = 0;
            dirLFlag = false;
            break;

        case Forward:
            ENL.write(1.0);
            DIRL = 1;
            dirLFlag = true;
            break;

        case Forward50:
            ENL.write(0.10);
            DIRL = 1;
            dirLFlag = true;
            break;

        case Backward:
            ENL.write(1.0);
            DIRL = 0;
            dirLFlag = false;
            break;

        case Backward50:
            ENL.write(0.10);
            DIRL = 0;
            dirLFlag = false;
            break;

        default:
            LeftMotor = Standby;
            break;
    }
}
