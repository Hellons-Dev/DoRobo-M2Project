/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "platform/mbed_thread.h"


PwmOut ENR(D11);
DigitalOut DIRR(D8, PullDown);
PwmOut ENL(D2);
DigitalOut DIRL(D4, PullDown);


int main()
{
    // Initialise the digital pin LED1 as an output
    
    ENL.period_ms(100);
    ENR.period_ms(100);
    ENL.write(0.0);
    ENR.write(0.0);
    DIRR = 1;
    DIRL = 1;

    while (true) {
        ENL.write(0.1);
        ENR.write(0.5);
        //ENR = 1;
    }
}
