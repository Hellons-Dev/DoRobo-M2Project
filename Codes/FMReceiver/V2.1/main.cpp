#include "mbed.h"
// Blinking rate in milliseconds
#define FMRate 10

RawSerial pc(USBTX, USBRX);

InterruptIn FM1(PB_4);
InterruptIn FM2(PB_5);
InterruptIn FM3(PA_11);
InterruptIn FM4(PA_8);
InterruptIn FM5(PF_0);
InterruptIn FM6(PB_1);
InterruptIn FM7(PB_6);
InterruptIn FM8(PB_7);



bool R = false;

void interruptFunc(void){
    FM8.rise(NULL);
    FM8.fall(NULL);
    FM7.rise(NULL);
    FM7.fall(NULL);
    FM6.rise(NULL);
    FM6.fall(NULL);
    FM5.rise(NULL);
    FM5.fall(NULL);
    FM4.rise(NULL);
    FM4.fall(NULL);
    FM3.rise(NULL);
    FM3.fall(NULL);
    FM2.rise(NULL);
    FM2.fall(NULL);
    FM1.rise(NULL);
    FM1.fall(NULL);
    wait_ms(10);
    R = true;
}

int main()
{
    FM8.rise(&interruptFunc);
    FM8.fall(&interruptFunc);
    FM7.rise(&interruptFunc);
    FM7.fall(&interruptFunc);
    FM6.rise(&interruptFunc);
    FM6.fall(&interruptFunc);
    FM5.rise(&interruptFunc);
    FM5.fall(&interruptFunc);
    FM4.rise(&interruptFunc);
    FM4.fall(&interruptFunc);
    FM3.rise(&interruptFunc);
    FM3.fall(&interruptFunc);
    FM2.rise(&interruptFunc);
    FM2.fall(&interruptFunc);
    FM1.rise(&interruptFunc);
    FM1.fall(&interruptFunc);
    bool Start = false, Sync = false, Stop = false, NStop = false;

    bool b1, b2, b3, b4, b5, b6, b7, b8;
    char word = 0x00;

    char words[200] = {0};
    

    int dataNb = 0;

    while (true) {
        //pc.printf("\n\rTest\n\r");
        if(R){
            FM8.rise(&interruptFunc);
            FM8.fall(&interruptFunc);
            FM7.rise(&interruptFunc);
            FM7.fall(&interruptFunc);
            FM6.rise(&interruptFunc);
            FM6.fall(&interruptFunc);
            FM5.rise(&interruptFunc);
            FM5.fall(&interruptFunc);
            FM4.rise(&interruptFunc);
            FM4.fall(&interruptFunc);
            FM3.rise(&interruptFunc);
            FM3.fall(&interruptFunc);
            FM2.rise(&interruptFunc);
            FM2.fall(&interruptFunc);
            FM1.rise(&interruptFunc);
            FM1.fall(&interruptFunc);
            

            R = false;
            b1 = FM1;
            b2 = FM2;
            b3 = FM3;
            b4 = FM4;
            b5 = FM5;
            b6 = FM6;
            b7 = FM7;
            b8 = FM8;
            word = 0x00;

            word = b1 | (b2<<1) | (b3<<2) | (b4<<3) | (b5<<4) | (b6<<5) | (b7<<6) | (b8<<7);
            pc.printf("\n\r%#X\n\r",word);
            if(Start == false){
                if(word == 0xA3){
                    Start = true;
                }else{
                    Start = false;
                    Sync = false;
                    Stop = false;
                    NStop = false;
                }
            }else if(Sync == false){
                if(word == 0x3A){
                    Sync = true;
                }else{
                    Start = false;
                    Sync = false;
                    Stop = false;
                    NStop = false;
                }
            }else{
                if(Stop == false && NStop == false){
                    if(word == 0xD0){
                        Stop = true;
                    }else{
                        words[dataNb] = word;
                        dataNb++;
                    }
                }else if(Stop == true && NStop == false){
                    if(word == 0x0D){
                        NStop = true;
                    }else{
                        words[dataNb] = 0xD0;
                        dataNb++;
                        words[dataNb] = word;
                        dataNb++;
                        Stop = false;
                    }
                }else{
                    Start = false;
                    Sync = false;
                    Stop = false;
                    NStop = false;
                    pc.printf("\n\r5\n\r");
                    pc.printf("\n\r");
                    for(int i = 0; i< dataNb; i++){
                        pc.printf("%c", words[i]);
                    }
                    pc.printf("\n\r");
                    dataNb = 0;
                }
            }
        }
    }
}
