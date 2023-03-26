#include "mbed.h"
#include "platform/mbed_thread.h"


// Blinking rate in milliseconds
PwmOut LidarCTRL(PB_3);

Serial PC(USBTX, USBRX);
Serial Lidar(PB_10, PC_5);


int main()
{
    // Initialise the digital pin LED1 as an output
    LidarCTRL.period_ms(10);
    PC.baud(9600);
    Lidar.baud(115200);
    LidarCTRL.write(0.4);
    char a = 0xff;
    Lidar.putc(0xA5);
    Lidar.putc(0x25);
    wait_ms(2);
    Lidar.putc(0xA5);
    Lidar.putc(0x40);
    wait_ms(2);
    Lidar.putc(0xA5);
    Lidar.putc(0x20);
    while (true) {
        if(Lidar.read()){
            PC.putc(0x01);
            PC.putc(Lidar.getc());
        } else {
            PC.putc(0x02);
            Lidar.putc(0xA5);
            Lidar.putc(0x20);
        }
    }
}
