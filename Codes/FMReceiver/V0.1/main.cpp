#include "mbed.h"
#include <iostream>
#include <string>

DigitalIn led1(PB_4);
DigitalIn led2(PB_5);
DigitalIn led3(PA_11);
DigitalIn led4(PA_8);
//DigitalIn led5(PF_1);
//DigitalIn led6(PF_0);
DigitalIn led7(PB_1);
DigitalIn led8(PB_6);

//On utilise cette partie seulement si on veut vérifier avec des
//led que le message est bien passé
DigitalOut led_out1(PA_0);
DigitalOut led_out2(PA_1);
DigitalOut led_out3(PA_3);
DigitalOut led_out4(PA_4);
DigitalOut led_out5(PA_5);
DigitalOut led_out6(PA_6);
DigitalOut led_out7(PA_7);
DigitalOut led_out8(PA_2);

char a,b,c,d,e,f,g,h;

int main()
{
    // Initialise the digital pin LED1 as an input
    while (true) {
       if (led1==1){
            a = '1';
            led_out1=1;
       }
       else{
           led_out1=0;
           a = '0';
       }
       if (led2==1){
            b = '1';
           led_out2=1;
       }
       else{
           led_out2 = 0;
            b = '0';
       }
       if (led3==1){
            c = '1';
           led_out3=1;
       }
       else{
           led_out3=0;
            c = '0';
       }
       if (led4==1){
            d = '1';
           led_out4=1;
       }
       else{
           led_out4 = 0;
            d = '0';
       }
       /*if (led5==1){
            e = '1';
           led_out5=1;
       }
       else{
           led_out5=0;
            e = '0';
       }
       if (led6==1){
            f = '1';
           led_out6=1;
       }
       else{
           led_out6= 0;
            f = '0';
       }*/
       if (led7==1){
            g = '1';
           led_out7=1;
       }
       else{
           led_out7= 0;
            g = '0';
       }
       if (led8==1){
            h = '1';
           led_out8=1;
       }
       else{
           led_out8= 0;
            h = '0';
       }
    }
}
