#include "pid.h"
// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES
#define BUFFER_FRAME_SIZE   250
#define MAX_REV_NODE		7
#define PI_NUMBER 			3.14159f
#define CENTER_X			180		//cm
#define CENTER_Y			210		//cm
#define MIN_FENCE_CM 		40
#define MAX_FENCE_CM 		400
#define MAX_XY_POS    		450
#define MAX_Z_POS    		350
#define MAX_DELTA_DISTANCE	50 //cm
// IPS
#define RUN_TRILATERATION
uint16_t ips_bytes;
uint16_t ips_data[MAX_REV_NODE];
char ips_char[BUFFER_FRAME_SIZE];

double ips_pos[3];
int ips_flag;
uint32_t ips_timer; 
uint16_t c_buff;
uint16_t c_state;

// CONTROL LOOP
bool update_loop;
// bool update_slowloop;
// SENSORS
float air_temperature;
// Vector3f ips_gyro, ips_accel;
Vector2f opt_flowRate;
Vector2f opt_bodyRate;
uint32_t opt_integration_timespan;

// IMU ODOMETRY
Vector3f imu_euler;
Vector3f imu2_velNED;
Vector3f imu3_velNED;

// KALMAN
double opt_flow[2];
double opt_gyro[2];
double lidar_h;
double k_pos[3];
double k_vel[3];
uint32_t k_timer;
// double ins_att[3];		//roll, pitch, yaw
double yaw_angle;		//[rad]
double frame_yaw_offset;
double max_inno_m[3];
double nls_timeout_s,_nls_timeout_s; //s

// NLS
double calib_a;
double calib_k;
double R_OP[3];
// const double nlsRCM[15] = {	0,180,360,180,180, 		//x[cm]
// 							180,60,180,330,180, 	//y
// 							300,300,300,300,301};	//z: must be different

const double nlsRCM[MAX_REV_NODE*3] = {				
	291.0698,  198.1481,   16.7031,  381.8444,  198.0192,  104.9728,  199.7348,	// 6calib_pts	
  	191.6734,  339.9091,  188.5640,  192.0087,   68.3208,  189.5930,  190.2275,
  	338.1022,  336.8074,  336.6327,  338.2026,  337.4751,  337.1002,  312.6070

	// 292.2629,  195.8769,   13.6291,  381.4104,  200.8723,  100.5476,  199.7435,	//7pts
 //  	193.2697,  345.9433,  191.8182,  196.0412,   70.8111,  192.8986,  192.9073,
 //  	338.1330,  337.1258,  336.0217,  338.5910,  338.5729,  336.9318,  313.5814

};


double nlsMR[MAX_REV_NODE];
double pre_nlsMR[MAX_REV_NODE];
double tempRCM[MAX_REV_NODE*3];
double tempMR[MAX_REV_NODE];
int err_cnt;
bool first_data;

bool nls_healthy;
float kalman_type;
volatile uint16_t s16_range_finder;

// TRAJECTORY
float circle_r, circle_w, circle_step, circle_heading, pre_circle_x, pre_circle_y;
float circle_T, circle_alpha;
float circle_x, circle_y, circle_cnt;
Vector2f circle_v, circle_a;
float lean_angle_max;
float trajectory_type;

// GUI/PLANNER
uint16_t gui_bytes;
char gui_char[BUFFER_FRAME_SIZE];
int gui_target;
int gui_flag;
uint16_t gui_buff;
uint16_t gui_state;

// PID
Vector3f v3f_target_control;
float target_roll = 0.0f;
float target_pitch = 0.0f;
bool is_armed = false;
float error_deadband;

PID::PID_PARAMETERS pid_pos_x_param = {.Kp = 0.05, .Ki = 0.0, .Kd = 0.005,
		.Ts = 0.01, .PID_Saturation = 250, .e=0,  .e_=0, .e__=0, .u =0,  .u_=0};
PID::PID_PARAMETERS pid_pos_y_param = {.Kp = 0.05, .Ki = 0.0, .Kd = 0.005,
		.Ts = 0.01, .PID_Saturation = 250, .e=0,  .e_=0, .e__=0, .u =0,  .u_=0};

PID pid_posx;
PID pid_posy;


// PID 2
float d2_target, pos_kp, h_accel_cms, h_speed_cms, linear_d, d_target, vel_target_x, vel_target_y;
float h_dt, accel_feedforward_x, accel_feedforward_y, h_vel_last_x, h_vel_last_y, h_vel_error_x, h_vel_error_y, h_vel_error_x_, h_vel_error_y_, accel_target_x, accel_target_y;
float lean_ang_max, accel_max, h_accel_total, accel_target_x_, accel_target_y_, h_accel_forward, h_accel_right, h_pitch_target, cos_pitch_target, h_roll_target;
float pid_roll, pid_pitch, pid_mode;
float pid_roll_, pid_pitch_;
float h_vel_xy_p_x, h_vel_xy_p_y ,h_vel_xy_i_x, h_vel_xy_i_y, h_vel_xy_imax;
float h_vel_xy_p_x_, h_vel_xy_p_y_;
Vector2f h_accel_target_jerk_limited;
Vector2f h_accel_target_filtered,h_accel_target_filtered_;
float vel_kp, vel_ki, kff;
Vector2f target_vel_desire;
Vector2f target_acc_desire;
int en_feedforward;

float heading_mode, heading_ctrl, heading_ctrl_mp;

size_t j_valen;
const char* j_value;

// #if WII_CAMERA == 1
// WiiCamera           ircam;
// int                 WiiRange=0;
// int                 WiiRotation=0;
// int                 WiiDisplacementX=0;
// int                 WiiDisplacementY=0;
// #endif  // WII_CAMERA

#endif  // USERHOOK_VARIABLES


