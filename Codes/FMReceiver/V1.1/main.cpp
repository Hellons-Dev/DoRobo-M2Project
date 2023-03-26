#include "mbed.h"
#include "mbed2/299/TARGET_NUCLEO_F303K8/TARGET_STM/TARGET_STM32F3/TARGET_NUCLEO_F303K8/PinNames.h"
#include <iostream>
#include <string>
#include <string.h> 
#include <ctype.h> 
#include "C12832.h" 

DigitalIn led1(PB_4);
DigitalIn led2(PB_5);
DigitalIn led3(PA_11);
DigitalIn led4(PA_8);
DigitalIn led5(PF_1);
DigitalIn led6(PF_0);
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

bool a,b,c,d,e,f,g,h;
char mot = 0x00;
Serial vC(USBTX, USBRX);

int main()
{
    // Initialise the digital pin LED1 as an input
    while (true) {
       
       mot = (a &0x01) << 7 | (b&0x01) << 6 | (c&0x01) << 5 | (d&0x01) << 4 | (e&0x01) << 3 | (f&0x01) << 2 | (g&0x01) << 1 | (h&0x01);
       vC.printf("%c\n\r", mot);
    }
}
