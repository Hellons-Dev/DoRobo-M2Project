#ifndef __include_rplidar_h__
#define __include_rplidar_h__
 
#include "mbed.h"
#define     LIDAR_MODE_STOP     0x25
#define     LIDAR_MODE_RESET    0x40
#define     LIDAR_MODE_SCAN     0x20
#define     LIDAR_MODE_FORCE    0x21
#define     LIDAR_MODE_INFO     0x50
#define     LIDAR_MODE_HEALTH   0x52
#define     LIDAR_MODE_RATE     0x59
 
#define     NB_BYTE_INFO_REQ        7
#define     NB_BYTE_INFO_RESP       20
#define     NB_BYTE_RATE_REQ        7
#define     NB_BYTE_RATE_RESP       4
#define     NB_BYTE_HEALTH_REQ      7
#define     NB_BYTE_HEALTH_RESP     3
#define     NB_BYTE_FORCE_REQ       7
#define     NB_BYTE_FORCE_RESP      5
#define     NB_BYTE_SCAN_REQ        7
 
extern char     pc_debug_data[128];
 
extern  Serial    lidar;
extern  Serial    pc;
extern  DigitalOut          debug_data;
extern  DigitalOut          debug_out;
extern  DigitalOut          data_ok;
extern  DigitalOut          data_ok_q;
extern  int                 data_nb;
extern  int                 data_scan_nb;
extern  char                received_data[];
extern  char                mode;
extern  char                scan_ok;
extern  int                 distance_scan[];
extern  int                 distance_scan_old[];
extern  int                 angle_scan[];
extern  int                 angle_scan_old[];
extern  char                tour_ok;
extern  char                trame_ok;
extern  struct lidar_data   ld_current;
 
/* Data Structure of lidar */
struct lidar_data{
    int quality;
    int angle;
    int distance;
};
 
/*********************************************************************** GENERAL FUNCTIONS */
 
/** Print int value and its name
 */ 
void print_int(const char *name, int ki);
/** Print data from serial communication
 */ 
void print_data(const char *name, char *datai, int sizedata);
/** Wait seconds
 */ 
void wait_s(float sec);
 
/** Find max in an integer array
 */ 
void findMax(int *int_data, int angle_min, int angle_max, int *value, int *indice);
 
/************************************************************************* LIDAR FUNCTIONS */
 
/** IT_lidar
        interrupt function on serial receiving
 */
void IT_lidar(void);
 
/** Reset request
        send command to core reset of the lidar
        this action took 2ms
 */
void sendResetReq(void);
 
/** Health request
        get device health information
 */
void getHealthLidar(void);
 
/** Info request
        get device information
        model / firmware _ LSB / MSB / Hardware / SerialNumber (15 octets)
 */
void getInfoLidar(void);
 
/** Sample Rate
        get sample rate
 */
void getSampleRate(void);
 
/** Start Scan
        start standard scan
 */
void startScan(void);
 
/** Stop Scan
        stop standard scan
 */
void stopScan(void);
 
#endif /* #ifndef __include_rplidar_h__ */
