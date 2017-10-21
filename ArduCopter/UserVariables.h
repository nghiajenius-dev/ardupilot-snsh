// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES
#define BUFFER_FRAME_SIZE   15

uint16_t ips_bytes;
int32_t ips_data[4];
uint16_t c_buff;
uint16_t c_state;
char ips_char[BUFFER_FRAME_SIZE];
uint32_t ips_delay_ms; 
float air_temperature;
Vector3f ips_gyro, ips_accel;
Vector2f opt_flowRate;
Vector2f opt_bodyRate;
uint32_t opt_integration_timespan;

double ips_pos[3] = {123,456,789};
double ips_flag;
double opt_flow[2];
double opt_gyro[3];
double lidar_h;
double k_pos[3];
uint32_t k_timer;

double R_OP[3];
double nlsRCM[15] = {105, 204.3, 105.5, 5.9, 105, 5.8, 105.5, 204.5, 105, 105, 206.4, 206.4, 206.4, 206.4, 190};
double nlsMR[5] = {186.3900,  187.6800,  208.5700,  209.0400,  154.1600};


// #if WII_CAMERA == 1
// WiiCamera           ircam;
// int                 WiiRange=0;
// int                 WiiRotation=0;
// int                 WiiDisplacementX=0;
// int                 WiiDisplacementY=0;
// #endif  // WII_CAMERA

#endif  // USERHOOK_VARIABLES


