// SOMO II sound module  http://www.4dsystems.com.au/product/SOMO_II/
// Datasheet http://www.4dsystems.com.au/productpages/SOMO-II/downloads/SOMO-II_datasheet_R_1_2.pdf

#ifndef SOMO_H
#define SOMO_H

#include <mbed.h>

//**************************************************************************
// class SOMO SOMO.h
// classe principale. exemple d'utilisation : SOMO audio(p13,p14);

class SOMO
{
    
    public:
    
        SOMO(PinName TXPin, PinName RXPin);
    
        void init();
    
        void playTrackNum(unsigned char, unsigned char);
        
        void start();
           
        void stopTrack();  
    
        void volume(unsigned char);
    
        void reset(); 
        
        void continuous(unsigned char, unsigned char);  
        
        void pause();
        
        void resume();
        
        void set_EQ(unsigned char);
        
        void play_random();
        
        void next();
        
        void previous();
        
        void repeat();
    
    
    protected :
    
        Serial     _serialSOMO;
    
};

#endif
