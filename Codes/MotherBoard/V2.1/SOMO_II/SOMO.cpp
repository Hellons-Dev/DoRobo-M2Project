// SOMO II sound module  http://www.4dsystems.com.au/product/SOMO_II/
// Datasheet http://www.4dsystems.com.au/productpages/SOMO-II/downloads/SOMO-II_datasheet_R_1_2.pdf
#include "mbed.h"
#include "SOMO.h"
#include "CAN.h"
#define CHKSUM 0xFFFF               // CHKSUM par défaut
//#define myID 0x4A0                  // id de départ du can en accord avec la librairie du CRAC

Ticker ticker;
CANMessage msg;



bool sendflag = 0;                  // flag permettant de réguler la communication de la carte par le can
int data_can = 0;                   // variable qui récupère l'ensemble des informations reçues
short commande = 0;                  // variable récupérant le premier octet de la partie DATA sur le can
char message1 = 0;                  // variable récupérant le second octet de la partie DATA sur le can
char message2 = 0;                  // variable récupérant le troisième octet de la partie DATA sur le can
int calc_chksum;                    // variable qui récupère la valeur du chksum une fois ce dernier calculer
char chksum[2]={0,0};               // tableau venant splitter en deux char les deux octets de chksum
unsigned char is_busy = 0x00;       // état de la carte
char tab_ident[3];
short myID = 0x4A0;
        
//construction
SOMO::SOMO(PinName TXPin, PinName RXPin): _serialSOMO(TXPin,RXPin)
{    
    init();
}


void SOMO::init()                               //configure la liaison avec le SOMO-II et le CAN 
{
    _serialSOMO.baud(9600);
}

/*
les fichiers mp3 doivent commencer par 001 jusqu'a 254
Les nom des dossier de repertoir pour le musique doivent être nommées par un nombre entre 01 et 99
*/

// **************************************************************************************************
// Joue une musique via son numéro et son répertoire

void SOMO::playTrackNum(unsigned char repository, unsigned char track)    
{  
    // calcule du CHKSUM
    calc_chksum=CHKSUM-(0x19+0x01)+1;           
    
    // séparation des deux octets du chksum pour l'envois
    chksum[0]=0xFF&(calc_chksum>>8);            
    chksum[1]=0xFF&(calc_chksum);
    unsigned char play2[8] = {0x7E, 0x19, 0x00, 0x00, 0x01, chksum[0], chksum[1], 0xEF}; 
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(play2[i]); 
    }
    wait(0.2);
    calc_chksum=CHKSUM-(0x0F+repository+track)+1;
    chksum[0]=0xFF&(calc_chksum>>8);
    chksum[1]=0xFF&(calc_chksum);
    unsigned char  play[8] = {0x7E, 0x0F, 0x00, repository, track, chksum[0], chksum[1], 0xEF}; 
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(play[i]); 
    }
    wait(0.2);
}

// **************************************************************************************************
// fonction de démarrage permettant l'émission d'un son au démarrage 
// de la carte pour avertir de sa mise sous tension

void SOMO::start()    
{  
    // calcule du CHKSUM
    calc_chksum=CHKSUM-(0x19+0x01)+1;
    
    // séparation des deux octets du chksum pour l'envois
    chksum[0]=0xFF&(calc_chksum>>8);
    chksum[1]=0xFF&(calc_chksum);
    
    // envois des octets sur la liaison série du SOMO-II
    unsigned char play2[8] = {0x7E, 0x19, 0x00, 0x00, 0x01, chksum[0], chksum[1], 0xEF}; 
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(play2[i]); 
    }
    wait(0.2);
    
    // calcule du CHKSUM
    calc_chksum=CHKSUM-(0x0F+0x01+0x01)+1;
    
    // séparation des deux octets du chksum pour l'envois
    chksum[0]=0xFF&(calc_chksum>>8);
    chksum[1]=0xFF&(calc_chksum);
    
    // envois des octets sur la liaison série du SOMO-II
    unsigned char play1[8] = {0x7E, 0x0F, 0x00, 0x01, 0x01, chksum[0], chksum[1], 0xEF}; 
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(play1[i]); 
    }
    wait(0.2);
    
    // calcule du CHKSUM
    calc_chksum=CHKSUM-(0x0D)+1;
    
    // séparation des deux octets du chksum pour l'envois
    chksum[0]=0xFF&(calc_chksum>>8);
    chksum[1]=0xFF&(calc_chksum);
    
    // envois des octets sur la liaison série du SOMO-II
    unsigned char play3[8] = {0x7E, 0x0D, 0x00, 0x00, 0x00, chksum[0], chksum[1], 0xEF}; 
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(play3[i]); 
    }
    wait(0.2);
}

// **************************************************************************************************
// stop la lecture de n'importe quelle musique

void SOMO::stopTrack()                         
{ 
    // calcule du CHKSUM
    calc_chksum=CHKSUM-(0x16)+1;
    
    // séparation des deux octets du chksum pour l'envois
    chksum[0]=0xFF&(calc_chksum>>8);
    chksum[1]=0xFF&(calc_chksum);
    
    // envois des octets sur la liaison série du SOMO-II
    unsigned char  stop[8] = {0x7E, 0x16, 0x00, 0x00, 0x00, chksum[0], chksum[1], 0xEF};
    for (int i = 0; i <8; i++)
    {
      _serialSOMO.putc(stop[i]); 
    }
}

// **************************************************************************************************
// change le volume (de 0 a 30)

void SOMO::volume(unsigned char vol)            
{
    if (vol>0 && vol<31)
    {
        // calcule du CHKSUM
        calc_chksum=CHKSUM-(0x06 + vol)+1;
        
        // séparation des deux octets du chksum pour l'envois
        chksum[0]=0xFF&(calc_chksum>>8);
        chksum[1]=0xFF&(calc_chksum);   
        
        // envois des octets sur la liaison série du SOMO-II
        unsigned char  changeVol[8] = {0x7E, 0x06, 0x00, 0x00, vol, chksum[0], chksum[1], 0xEF};         
        for (int i = 0; i <8; i++)
        {
            _serialSOMO.putc(changeVol[i]); 
        }
    }
}   

// **************************************************************************************************
// remise a zéro du module

void SOMO::reset()                              
{
    // envois des octets sur la liaison série du SOMO-II
    unsigned char reset[8] = {0x7E, 0x0C, 0x00, 0x00, 0x00, 0xFF, 0xF4, 0xEF};
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(reset[i]); 
    }    
}

// **************************************************************************************************
// Fonction pour jouer tout les sons de la carte en continus en prennant pour point de départ
// le répertoire et la chansons données

void SOMO::continuous(unsigned char repository, unsigned char track)
{
    calc_chksum=CHKSUM-(0x0F+repository+track)+1;
    chksum[0]=0xFF&(calc_chksum>>8);
    chksum[1]=0xFF&(calc_chksum);
    
    // envois des octets sur la liaison série du SOMO-II
    unsigned char play1[8] = {0x7E, 0x0F, 0x00, repository, track, chksum[0], chksum[1], 0xEF}; 
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(play1[i]); 
    }
    wait(0.2);
    calc_chksum=CHKSUM-(0x11+0x01)+1;
    chksum[0]=0xFF&(calc_chksum>>8);
    chksum[1]=0xFF&(calc_chksum);
    
    // envois des octets sur la liaison série du SOMO-II
    unsigned char play2[8] = {0x7E, 0x11, 0x00, 0x00, 0x01, chksum[0], chksum[1], 0xEF}; 
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(play2[i]); 
    }
    wait(0.2);
    calc_chksum=CHKSUM-(0x0D)+1;
    chksum[0]=0xFF&(calc_chksum>>8);
    chksum[1]=0xFF&(calc_chksum);
    
    // envois des octets sur la liaison série du SOMO-II
    unsigned char  play[8] = {0x7E, 0x0D, 0x00, 0x00, 0x00, chksum[0], chksum[1], 0xEF}; 
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(play[i]); 
    }
    wait(0.2);
}

// **************************************************************************************************
// Mise en pause de la lecture

void SOMO::pause()
{
    calc_chksum=CHKSUM-(0x0E)+1;
    chksum[0]=0xFF&(calc_chksum>>8);
    chksum[1]=0xFF&(calc_chksum);
    
    // envois des octets sur la liaison série du SOMO-II
    unsigned char  play[8] = {0x7E, 0x0E, 0x00, 0x00, 0x00, chksum[0], chksum[1], 0xEF}; 
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(play[i]); 
    }
    wait(0.2);
}

// **************************************************************************************************
// Reprise de la lecture

void SOMO::resume()
{
    calc_chksum=CHKSUM-(0x0D)+1;
    chksum[0]=0xFF&(calc_chksum>>8);
    chksum[1]=0xFF&(calc_chksum);
    
    // envois des octets sur la liaison série du SOMO-II
    unsigned char  play[8] = {0x7E, 0x0D, 0x00, 0x00, 0x00, chksum[0], chksum[1], 0xEF}; 
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(play[i]); 
    }
    wait(0.2);
}

// **************************************************************************************************
// Réglage de l'equilizer

void SOMO::set_EQ(unsigned char mod)
{
    calc_chksum=CHKSUM-(0x07+mod)+1;
    chksum[0]=0xFF&(calc_chksum>>8);
    chksum[1]=0xFF&(calc_chksum);
    
    // envois des octets sur la liaison série du SOMO-II
    unsigned char  play[8] = {0x7E, 0x07, 0x00, 0x00, mod, chksum[0], chksum[1], 0xEF}; 
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(play[i]); 
    }
    wait(0.2);
}

// **************************************************************************************************
// Joue les piste aléatoirement

void SOMO::play_random()
{
    calc_chksum=CHKSUM-(0x18)+1;
    chksum[0]=0xFF&(calc_chksum>>8);
    chksum[1]=0xFF&(calc_chksum);
    
    // envois des octets sur la liaison série du SOMO-II
    unsigned char  play[8] = {0x7E, 0x18, 0x00, 0x00, 0x00, chksum[0], chksum[1], 0xEF}; 
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(play[i]); 
    }
    wait(0.2);
    calc_chksum=CHKSUM-(0x0D)+1;
    chksum[0]=0xFF&(calc_chksum>>8);
    chksum[1]=0xFF&(calc_chksum);
    
    // envois des octets sur la liaison série du SOMO-II
    unsigned char  play1[8] = {0x7E, 0x0D, 0x00, 0x00, 0x00, chksum[0], chksum[1], 0xEF}; 
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(play1[i]); 
    }
    wait(0.2);
}

// **************************************************************************************************
// Joue la piste suivante de la liste

void SOMO::next()
{
    calc_chksum=CHKSUM-(0x01)+1;
    chksum[0]=0xFF&(calc_chksum>>8);
    chksum[1]=0xFF&(calc_chksum);
    
    // envois des octets sur la liaison série du SOMO-II
    unsigned char  play1[8] = {0x7E, 0x01, 0x00, 0x00, 0x00, chksum[0], chksum[1], 0xEF}; 
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(play1[i]); 
    }
    wait(0.2);
}

// **************************************************************************************************
// Joue la pise précédante de la liste

void SOMO::previous()
{
    calc_chksum=CHKSUM-(0x02)+1;
    chksum[0]=0xFF&(calc_chksum>>8);
    chksum[1]=0xFF&(calc_chksum);
    
    // envois des octets sur la liaison série du SOMO-II
    unsigned char  play1[8] = {0x7E, 0x02, 0x00, 0x00, 0x00, chksum[0], chksum[1], 0xEF}; 
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(play1[i]); 
    }
    wait(0.2);
}

// **************************************************************************************************
// Lecture en boucle de la dernière piste jouée

void SOMO::repeat()
{
    calc_chksum=CHKSUM-(0x0D)+1;
    chksum[0]=0xFF&(calc_chksum>>8);
    chksum[1]=0xFF&(calc_chksum);
    
    // envois des octets sur la liaison série du SOMO-II
    unsigned char  play[8] = {0x7E, 0x0D, 0x00, 0x00, 0x00, chksum[0], chksum[1], 0xEF}; 
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(play[i]); 
    }
    wait(0.2);
    calc_chksum=CHKSUM-(0x19)+1;
    chksum[0]=0xFF&(calc_chksum>>8);
    chksum[1]=0xFF&(calc_chksum);
    
    // envois des octets sur la liaison série du SOMO-II
    unsigned char  play1[8] = {0x7E, 0x19, 0x00, 0x00, 0x00, chksum[0], chksum[1], 0xEF}; 
    for (int i = 0; i <8; i++)
    {
        _serialSOMO.putc(play1[i]); 
    }
    wait(0.2);
}
