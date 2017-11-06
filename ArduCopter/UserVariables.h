#include "pid.h"
// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES
#define BUFFER_FRAME_SIZE   500
#define MAX_REV_NODE		5

// IPS
#define RUN_TRILATERATION
uint16_t ips_bytes;
uint16_t ips_data[MAX_REV_NODE];
char ips_char[BUFFER_FRAME_SIZE];

double ips_pos[3];
double ips_flag;
uint32_t ips_timer; 
uint16_t c_buff;
uint16_t c_state;

// SENSORS
float air_temperature;
Vector3f ips_gyro, ips_accel;
Vector2f opt_flowRate;
Vector2f opt_bodyRate;
uint32_t opt_integration_timespan;

// KALMAN
double opt_flow[2];
double opt_gyro[3];
double lidar_h;
double k_pos[3];
uint32_t k_timer;
double ins_att[3];		//roll, pitch, yaw
double yaw_angle;		//[rad]
double frame_yaw_offset;

// NLS
const double calib_a[5] = { 0.8641, 0.8672, 0.8641, 0.8652, 0.8649 };      // CD->G
const double calib_k[5] = { 4.891, 3.848, 5.756, 4.71, 5.226 };             // CD->G
double R_OP[3];
const double nlsRCM[15] = {1050, 2043, 1055, 59, 1050, 58, 1055, 2045, 1050, 1050, 2064, 2064, 2064, 2064, 1900};
double nlsMR[5];
double tempRCM[15];
double tempMR[5];
int err_cnt;

bool nls_healthy;
// double sortRCM[15];
// uint16_t sortMR[5];
// uint16_t maxMR;
// int c_i, c_j, c_k;
// int max_NOR, max_index;

volatile uint16_t s16_range_finder = 123;

Vector3f v3f_target_control;
float target_roll = 0.0f;
float target_pitch = 0.0f;
bool is_armed = false;

PID::PID_PARAMETERS pid_pos_x_param = {.Kp = 1.2, .Ki = 0.0, .Kd = 0.05,
		.Ts = 0.01, .PID_Saturation = 250, .e=0,  .e_=0, .e__=0, .u =0,  .u_=0};
PID::PID_PARAMETERS pid_pos_y_param = {.Kp = 1.2, .Ki = 0.0, .Kd = 0.05,
		.Ts = 0.01, .PID_Saturation = 250, .e=0,  .e_=0, .e__=0, .u =0,  .u_=0};

PID pid_posx;
PID pid_posy;

// #if WII_CAMERA == 1
// WiiCamera           ircam;
// int                 WiiRange=0;
// int                 WiiRotation=0;
// int                 WiiDisplacementX=0;
// int                 WiiDisplacementY=0;
// #endif  // WII_CAMERA

#endif  // USERHOOK_VARIABLES


