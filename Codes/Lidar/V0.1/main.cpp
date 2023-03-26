#include "mbed.h"


// Blinking rate in milliseconds
PwmOut LidarCTRL(PB_3);

Serial PC(USBTX, USBRX);
Serial Lidar(PB_10, PC_5);

char i[35];


int main()
{
    // Initialise the digital pin LED1 as an output
    DigitalOut led(LED1);
    LidarCTRL.period_ms(10);
    PC.set_baud(115200);
    Lidar.set_baud(115200);
    LidarCTRL.write(0.5);
    char a = 0xff;
    Lidar.putc(0xff);
    while (true) {
        
        Lidar.read(i,35);
        PC.write(i,35);
    }
}
