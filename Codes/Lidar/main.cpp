#include "mbed.h"
#include "rplidar.h"
#include <cstdio>
 
#define BLINKING_RATE_MS        500
#define NB_DATA_MAX             20
#define AFF_DATA                0
 
char            pc_debug_data[128];
char            received_data[64];

int             data_nb = 0;
int             data_scan_nb = 0;
int             distance_scan[360] = {0};
int             distance_scan_old[360] = {0};
int             angle_scan[360] = {0};
int             angle_scan_old[360] = {0};
int             obs_deb;
int             nb_obs;
int             dis_min;

char            mode = LIDAR_MODE_STOP;
char            scan_ok = 0;
char            tour_ok = 0;
char            trame_ok = 0;
//variable obstacle verification
bool            obs_data[360];
bool            obs_verif = false;

struct Obstacle {
    int StartAngle;
    int EndAngle;
    long Distance;
};

Obstacle obstacles[360];

DigitalOut          led(LED1);
DigitalOut          debug_data(D10);
DigitalOut          debug_tour(D9);
DigitalOut          debug_out(D7);
DigitalOut          data_ok(D5);
DigitalOut          data_ok_q(D4);

RawSerial           Master(PC_10, PC_11, 19200);

Serial              pc(USBTX, USBRX, 115200);
Serial              lidar(PA_0, PA_1, 115200);

PwmOut              rotation(PB_4);
 
struct lidar_data   ld_current;
 

void SerialSend(void){
    //angle debut angle fin distance min nbr obst.
    char nBitStart , nBitEnd, nBitDist ;
    
    //nbitdeb = log2(obstacles[i].debut)+1;
    //nbitfin= log2(obstacles[i].fin)+1;
    //nbitdist= log2(obstacles[i].dist)+1;
    pc.printf("\n\rnb obstacles %d\n\r", nb_obs);
    if(nb_obs == 0){
        Master.putc(78);
    }else{
        for(int i = 0; i < nb_obs; i++){
            if(obstacles[nb_obs].Distance >=1){
                nBitDist= log2(obstacles[i].Distance)+1;
            }else{
                nBitDist=0;
            }
            char nBytedist=(8-fmod(nBitDist,8)+nBitDist)/8;
            Master.putc(83);
            wait_ms(10);
            Master.putc(obstacles[i].StartAngle & 0xFF);
            wait_ms(10);
            Master.putc((obstacles[i].StartAngle >> 8) & 0xFF);
            wait_ms(10);
            Master.putc(69);
            wait_ms(10);
            Master.putc(obstacles[i].EndAngle & 0xFF);
            wait_ms(10);
            Master.putc((obstacles[i].EndAngle >> 8) & 0xFF);
            wait_ms(10);
            Master.putc(68);
            wait_ms(10);
            Master.putc(nBytedist);
            wait_ms(10);
            
            for(int j = 0; j < nBytedist ; j++){
                Master.putc(0xFF & (obstacles[i].Distance >> (8*j)));
                wait_ms(10);
            }
            wait_ms(100);
        }
    }
    nb_obs = 0;
    return;
}
 
/** MAIN FUNCTION
 */
int main()
{
    int nb_tour = 0;
    wait_s(3.0);
    rotation.period(1/25000.0);
    rotation.write(0.5);
    wait_s(2.0);
    pc.printf("\r\nLIDAR Testing\r\n");
    lidar.attach(&IT_lidar);
    wait_s(1.0);
    pc.printf("\r\nLIDAR OK\r\n");
 
    getHealthLidar();
    getInfoLidar();
    getSampleRate();  
    // Start a new scan
    startScan(); 
    // Infinite Loop
    while (true) {
        if(trame_ok){
            debug_tour = !debug_tour;
        }
 
        if(tour_ok == 6){
            //
            stopScan();
            int maxDistance, maxAngle;
          
            

            tour_ok = 0;
            /*for(int i = 0; i< 360; i++){
                char str[50];
                sprintf(str, "angle %d : ", angle_scan_old[i]);
                print_int(str, distance_scan_old[i]);
            }*/
            print_int("Angle 0 has distance ", distance_scan_old[0]);
            findMax(distance_scan_old, 0, 360, &maxDistance, &maxAngle);
            print_int("A", maxAngle);
            print_int("D", maxDistance);

            for ( int i = 0; i < 360; i++ )
            { 
                if( distance_scan_old[i] != 0 && distance_scan_old[i] <= 800){
                    obs_data[i]=true;
                }else{
                    obs_data[i]=false;
                }
            }
            nb_obs=0;
            for(int i = 0; i < 360; i++)
            {
                if(obs_data[i] == true && obs_verif==false){
                    obs_deb=i;
                    obs_verif= true;
                }else if(obs_data[i] == false && obs_verif==true){
                //obstacles[nbostacle]
                    obstacles[nb_obs].StartAngle = obs_deb;
                    obstacles[nb_obs].EndAngle = i-1;
                    dis_min=800;
                    for(int j = obs_deb; j<i; j++){
                        if(dis_min > distance_scan_old[j]){
                            dis_min = distance_scan_old[j];
                        }
                    } 
                    obstacles[nb_obs].Distance = dis_min;
                    obs_verif= false;
                    nb_obs++;
                }else if(i==359 && obs_data[i]==true && obs_verif==true){
                    obstacles[0].StartAngle = obs_deb;
                    obs_verif= false;
                }
            }

            SerialSend();
            startScan();
        }
    }
}
