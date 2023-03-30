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
#define pi          3.14159265358979323846

Timer TimingLidar;
Timer Tp;
Timer Ready;

SOMO somo2(PA_0, PA_1);

RawSerial pc(USBTX, USBRX, 115200);
RawSerial Lidar(PC_10, PC_11, 19200);
RawSerial Odometry(PC_12, PD_2, 19200);

AnalogIn MQ135(PC_0);
AnalogIn TGS813(PC_1);

int ReceivedDataLidar = 0;
int ReceivedDataOdometry = 0;
int StartAngleR = 0;
int EndAngleR = 0;
int DistanceR = 0;
int nbDataLidar = 0;
int nbDataOdometry = 0;
int nbObstacles = 0;
int prevObsNb = 0;
int x = 0;
int y = 0;
int NP = 0;
int Angle = 0;

double Vrl;                                  // Output voltage
double Rs;                                   // Rs (Ohm) - Sensor resistance
double ppm;   
double ppm_init;                               // ppm
double resRatio;
double lgPPM_init;
double theta = 0.0;

char    tempX[50] = {0};
char    tempY[50] = {0};

float adcRaw_init;

bool    ReceivingStartAngle = false,
        ReceivingDistance = false,
        ReceivingEndAngle = false,
        NoOsbtacles = false,
        Signe = false,
        XY = false,
        ROk = true,
        ReceivingAngle = false;

struct Point{
    int x=0;
    int y=0;
};

Point prevPoint;

struct Obstacle {
    int StartAngle;
    int EndAngle;
    int Distance;
};

Obstacle obstacles[360];
Obstacle tab_angle[180];

void SendXY(int x, int y){
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

    for(int i = 0; i < 3+nByteX ; i++){
        if(i == 0){
            Odometry.putc(88);
        }else if(i == 1){
            Odometry.putc(signX);
        }else if(i == 2){
            Odometry.putc(nByteX);
        }else{
            Odometry.putc((0xFF & (absX >> ((i-3)*8))));
        }
        wait_ms(10);
    }

    for(int i = 0; i < 3+nByteY ; i++){
        if(i == 0){
            Odometry.putc(89);
        }else if(i == 1){
            Odometry.putc(signY);
        }else if(i == 2){
            Odometry.putc(nByteY);
        }else{
            Odometry.putc((0xFF & (absY >> ((i-3)*8))));
        }
        wait_ms(10);
    }
    return;
}

void OdometryReceive(void){
    char data = Odometry.getc();

    if(ReceivedDataOdometry == 0){
        if(data == 88){
            XY = true;
            ReceivedDataOdometry++;
        }else if(data == 89){
            XY = false;
            ReceivedDataOdometry++;
        }else if(data == 65){
            ReceivingAngle = true;
            ReceivedDataOdometry++;
        }else;

    }else if(ReceivedDataOdometry == 1){
        if(ReceivingAngle){
            Angle = 0xFF & data;
            ReceivedDataOdometry++;
        }else{
            Signe = (data == 0x01 ? true : false);
            ReceivedDataOdometry++;
        }
    }else if(ReceivedDataOdometry == 2){
        if(ReceivingAngle){
            Angle = Angle | (data << 8);
            theta = Angle * pi / 180;
            ReceivingAngle = false;
            ReceivedDataOdometry = 0;
        }else{
            nbDataOdometry = data;
            ReceivedDataOdometry++;
        }
    }else{
        
        if(XY == true){
            tempX[ReceivedDataOdometry-3] = data;
            ReceivedDataOdometry++;
        }else{
            tempY[ReceivedDataOdometry-3] = data;
            ReceivedDataOdometry++;
        }
        if((ReceivedDataOdometry - 3) == nbDataOdometry){
            if(XY == true){
                int value = 0;
                for(int i = 0; i < nbDataOdometry; i++){
                    value = value | (tempX[i] << (i*8));
                }
                x = value * (Signe == true ? 1 : -1);
            }else{
                int value = 0;
                for(int i = 0; i < nbDataOdometry; i++){
                    value = value | (tempY[i] << (i*8));
                }
                y = value * (Signe == true ? 1 : -1);
            }
            nbDataOdometry = 0;
            ReceivedDataOdometry = 0;
        }
    }
}

void LidarReceive(void){
    char data = Lidar.getc();
    ROk = false;
    //pc.printf("\n\r%#X\n\r",data);
    if(TimingLidar.read_ms() > 500){
        nbObstacles = 0;
    }
    Ready.stop();
    Ready.reset();
    Ready.start();
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
        //pc.printf("\n\r0\n\r");

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
        //pc.printf("\n\r1\n\r");
        
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
        //pc.printf("\n\r2\n\r");
    }else{
        if(!ReceivingStartAngle && ReceivingDistance && !ReceivingEndAngle){
            DistanceR |= (data << (8*(ReceivedDataLidar-2)));
            ReceivedDataLidar++;
            if((ReceivedDataLidar-2) >= nbDataLidar){
                ReceivingDistance = false;
                obstacles[nbObstacles].Distance = DistanceR;
                pc.printf("\n\r AngleStart = %d -- AngleStop = %d -- Distance = %d \n\r", obstacles[nbObstacles].StartAngle, obstacles[nbObstacles].EndAngle, obstacles[nbObstacles].Distance);
                nbObstacles++;
                pc.printf("\n\r Obstacle number %d\n\r", nbObstacles);
                ReceivedDataLidar = 0;
            }
        }else{
            ReceivedDataLidar = 0;
        }
        //pc.printf("\n\relse\n\r");
    }
    TimingLidar.start();
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

Point multi_obs(Obstacle* thisAngle, int nbr_obs)
{
    int i = 0;
    int nv_angle;
    Point new_coord;
    bool ok;
    double adjustedAngle = abs(fmod(theta-(3*pi/2),-2*pi));

    while (i != nbr_obs){

		if (thisAngle[i].EndAngle - thisAngle[i+1].StartAngle < -35){

            nv_angle = thisAngle[i].EndAngle + 20;
            new_coord.y = y - (sin(adjustedAngle) * (thisAngle[i].Distance/10.0));
            new_coord.x = x + (cos(adjustedAngle) * (thisAngle[i].Distance/10.0) * tan(nv_angle));
			//On passe entre les deux -> réussir à indiquer x-y en fonction de l’angle cible trouvé
			ok = 1;
			i = nbr_obs;
        } else {
			i++;
            ok = 0;
        }

    }

    if (ok == 0){

        new_coord.y = y + (sin(adjustedAngle) * 80);
        new_coord.x = x - (cos(adjustedAngle) * 80);

    }

    return new_coord;
}

Point gestion_tab(Obstacle *angle,int nbr_obs)
{
    int Obs_traj = 0;
    int deb, fin;
    Point new_coord;
    int a,b = 0;
    double c;

    //définir le tableau tab_angle

    int i=0;
    while (angle[i].EndAngle < 90)
    {
        pc.printf ("1. Obstacle :[ %d : %d ] et D = %lf\n\r", angle[i].StartAngle, angle[i].EndAngle, angle[i].Distance);

        deb = angle[i].StartAngle;
        fin = angle[i].EndAngle;
        c = angle[i].Distance;

        Obs_traj += 1;
        pc.printf ("1INT Obstacle :[ %d : %d ] et D = %lf\n\r", deb, fin, c);
        tab_angle[i].StartAngle = deb;
        tab_angle[i].EndAngle = fin;
        tab_angle[i].Distance = c;
        pc.printf ("1Fin Obstacle :[ %d : %d ] et D = %lf\n\r", tab_angle[i].StartAngle, tab_angle[i].EndAngle, tab_angle[i].Distance);
        i++;

    }

    a = 90;
    b = 89;
    c = tab_angle[Obs_traj-1].Distance;
    tab_angle[Obs_traj].EndAngle = a;
    tab_angle[Obs_traj].StartAngle = b;
    tab_angle[Obs_traj].Distance = c;
    Obs_traj += 1;
    pc.printf (" 90 Obstacle :[ %d : %d ] et D = %lf\n\r", tab_angle[Obs_traj].StartAngle, tab_angle[Obs_traj].EndAngle, tab_angle[Obs_traj].Distance);

    tab_angle[Obs_traj+1].StartAngle = 269;
    tab_angle[Obs_traj+1].EndAngle = 270;
    tab_angle[Obs_traj+1].Distance = tab_angle[Obs_traj-1].Distance;
    Obs_traj++;


    for(int j=Obs_traj; j<= nbr_obs-Obs_traj+2; j++){


        if (angle[j].EndAngle > 270) {

            deb = angle[j].StartAngle;
            fin = angle[j].EndAngle;
            Obs_traj =  Obs_traj + 1;
            tab_angle[j].StartAngle = deb;
            tab_angle[j].EndAngle = fin;
            tab_angle[j].Distance = angle[j].Distance;

        }
        pc.printf ("2; Obstacle :[ %d : %d ] et D = %lf\n\r", tab_angle[j].StartAngle, tab_angle[j].EndAngle, tab_angle[j].Distance);
    }

    new_coord = multi_obs(tab_angle, Obs_traj);
    return new_coord;
}

void StrategyManager(void){
    //printf ("Obstacle :[ %d : %d ] et D = %lf", angle[0].debut, angle[0].fin, angle[0].distance);

    Point new_coord = gestion_tab(obstacles ,nbObstacles);

    prevPoint.x = new_coord.x;
    prevPoint.y = new_coord.y;
    SendXY(new_coord.x, new_coord.y);

    pc.printf ("Les nouvelles coordonnees sont :[ %d : %d ] et D = %d\n\r", new_coord.x, new_coord.y, obstacles[0].Distance);
    nbObstacles = 0;
}

int main()
{
    bool alarm = false;
    // Initialise the digital pin LED1 as an output
    adcRaw_init = MQ135.read();
    Vrl = (double)adcRaw_init*Vadc_5;             // For 5V Vcc use Vadc_5
    Rs = (5 - Vrl)/Vrl;                   // Calculate sensor resistance
    resRatio = Rs/Rl;                             // Calculate ratio
    lgPPM_init = (log10(resRatio) * -0.8)+ 0.9;        // Calculate ppm
    ppm_init = pow(10,lgPPM_init)*1000;

    somo2.init();
    wait_ms(100);
    somo2.volume(20);
    wait_ms(100);
    somo2.playTrackNum(1,1);
    wait_ms(100);

    Lidar.enable_input(true);
    Lidar.attach(&LidarReceive, Serial::RxIrq);
    Odometry.enable_input(true);
    Odometry.enable_output(true);
    Odometry.attach(&OdometryReceive, Serial::RxIrq);
    Tp.start();
    TimingLidar.start();
    Ready.start();
    while (true) {
        if(MQ135_test() == 0 && TGS813_test() == 0){
            alarm = false;
            if(Tp.read()>5){
                somo2.volume(20);
                wait_ms(100);
                somo2.playTrackNum(1,2);
                Tp.stop();
                Tp.reset();
                Tp.start();
            }
        }else{
            if(alarm == false || Tp.read()>55){
                alarm = true;
                somo2.volume(30);
                wait_ms(100);
                somo2.playTrackNum(1,3);
                Tp.stop();
                Tp.reset();
                Tp.start();
            }
        }
        if(alarm == false){
            if(ROk == false){
                if(Ready.read_ms() > 200 && !NoOsbtacles && nbObstacles != 0){
                    Ready.stop();
                    Ready.reset();
                    Ready.start();
                    StrategyManager();
                    ROk = true;
                }
            } else {
                bool diff1, diff2;
                diff1 = ((prevPoint.x >= x-30 && prevPoint.x <= x+30) ? true : false);
                diff2 = ((prevPoint.y >= y+30 && prevPoint.y <= y+30)? true : false);
                Point new_coord;
                if(diff1 && diff2){
                    double adjustedAngle = abs(fmod(theta-(3*pi/2),-2*pi));
                    new_coord.y = y - (sin(adjustedAngle) * 900);
                    new_coord.x = x + (cos(adjustedAngle) * 900);
                    prevPoint.x = new_coord.x;
                    prevPoint.y = new_coord.y;
                    SendXY(new_coord.x, new_coord.y);
                }
            }
        } else {
            SendXY(x, y);
        }
    }
}
