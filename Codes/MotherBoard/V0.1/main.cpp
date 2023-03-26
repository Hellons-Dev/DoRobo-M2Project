/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "platform/mbed_thread.h"


// Blinking rate in milliseconds
Serial Odometry(PC_10, PC_11, 38400);


int main()
{
    Odometry.putc(88);
    wait_ms(10);
    Odometry.putc(0x00);
    wait_ms(10);
    Odometry.putc(0x02);
    wait_ms(10);
    Odometry.putc(0b00101100);
    wait_ms(10);
    Odometry.putc(0b00000001);
    wait_ms(10);

    while (true) {
    }
}
