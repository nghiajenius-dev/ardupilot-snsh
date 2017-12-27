#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    c_buff = 0;     //0: temp-buf
    c_state = 0;    //0: wait-start-buff, 1: data-buffer, 2: end-buffer-->0
    ips_bytes = 0;

    optflow.init();
    frame_yaw_offset = 0.0f;
    LPF_pos_initialize();
    multirate_kalman_v4_initialize();
    hal.uartD->begin(115200);
   
#ifdef RUN_TRILATERATION
    LeastSquare_NJ_initialize();
#endif
    pid_posx.init(pid_pos_x_param);
    pid_posy.init(pid_pos_y_param);

    // DEFAULT HOVER TARGET
    v3f_target_control.x = g.user_hover_x;
    v3f_target_control.y = g.user_hover_y;


    circle_T = 24;// 24s/round
    circle_w = 2*PI_NUMBER/circle_T;       
    circle_step = 0;
    circle_r = 40;
    lean_angle_max = 1000;
    max_inno_m[0] = 10;
    max_inno_m[1] = 10;
    max_inno_m[2] = 10;
}
#endif
 

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // if( frame_yaw_offset == 0.0f){
    //      frame_yaw_offset = (double)ToRad(ahrs.yaw_sensor)/100;
    // }
    // put your 100Hz code here
    // uartF: serial5, baud 115200
    //================================IPS====================================//
    // Get available bytes
    ips_bytes = hal.uartF->available();

    while (ips_bytes-- > 0) {
        // Get data string here
        ips_char[0] = hal.uartF->read();
        // start-of-frame
        if(ips_char[0] == 's'){
            c_buff = 1;
            c_state = 1;
        }
        else if((c_state==1)&&(ips_char[0] == '\n')){   // end-of-frame: start parsing
            // data format: "s123,056,789,012\r\n"
                
            ips_data[0] = (ips_char[1]-0x30)*100 + (ips_char[2]-0x30)*10 + (ips_char[3]-0x30);  
            ips_data[1] = (ips_char[5]-0x30)*100 + (ips_char[6]-0x30)*10 + (ips_char[7]-0x30);
            ips_data[2] = (ips_char[9]-0x30)*100 + (ips_char[10]-0x30)*10 + (ips_char[11]-0x30);
            ips_data[3] = (ips_char[13]-0x30)*100 + (ips_char[14]-0x30)*10 + (ips_char[15]-0x30);
            ips_data[4] = (ips_char[17]-0x30)*100 + (ips_char[18]-0x30)*10 + (ips_char[19]-0x30);

            for(int i = 0; i < MAX_REV_NODE; i++){
                nlsMR[i] = (calib_a[i] * ips_data[i] + calib_k[i]) * 10;    //mm
            }
       
            // cliSerial->printf("R:%d,%d,%d,%d,%d\r\n",ips_data[0],ips_data[1],ips_data[2],ips_data[3],ips_data[4]);
            // cliSerial->printf("D:%.0f,%.0f,%.0f,%.0f,%.0f\r\n",nlsMR[0],nlsMR[1],nlsMR[2],nlsMR[3],nlsMR[4]);
            ips_flag = 1;   // finish convert data --> start NLS
            c_buff = 0;
            c_state = 0;
        }
        else{   // fill buffer after catch start header
            if(c_state == 1){
                ips_char[c_buff] = ips_char[0];
                // hal.uartF->printf("%c",ips_char[c_buff]);
                c_buff++;
            }
        }
    }


//================================NLS====================================//
  // ips_flag = 1;
  // [121.8, 120.4, 185.4, 185.5, 92.9];
  // nlsMR[0] = 1218;
  // nlsMR[1] = 1204;
  // nlsMR[2] = 1854;
  // nlsMR[3] = 1855;
  // nlsMR[4] = 929;

    if (ips_flag == 1){
        err_cnt = 0;
        for(int i = 0; i<5; i++){
            if(nlsMR[i]>3500){
                err_cnt++;
            }
        }
        if(err_cnt<2){      // at least 4 valid node             
            for(int i = 0; i < 15; i++){
                tempRCM[i] = nlsRCM[i];    //temp value
            }
            for(int i = 0; i < 5; i++){
                tempMR[i] = nlsMR[i];    //mm
            }

            LeastSquare_NJ(5,tempMR,tempRCM, 2, R_OP); 
            if((R_OP[0]>0)&&(R_OP[1]>0)&&(R_OP[2]>0)&&(R_OP[0]<2500)&&(R_OP[1]<2500)&&(R_OP[2]<2500)){
                nls_healthy = true;
                // cliSerial->printf("NLS:%d,%d,%d\r\n",(int)R_OP[0],(int)R_OP[1],(int)R_OP[2]);
            }
            else{
                nls_healthy = false; 
            }   
        }
        else{
            nls_healthy = false; 
        }

        
        // ips_timer = AP_HAL::millis() - ips_timer;   
        // hal.uartF->printf("NLS: %d, %d, %d, %d\r\n",(int)R_OP[0],(int)R_OP[1],(int)R_OP[2],ips_timer);  
        // hal.uartF->printf("NLS: %d,%d,%d,%d,%d\r\n",nlsMR[0],nlsMR[1],nlsMR[2],nlsMR[3],nlsMR[4]);  
    }
    //reset ips flag
    ips_flag = 0; 

//==============================PX4FLOW======================================//
    optflow.update();
    Vector2f opt_flowRate = optflow.flowRate();
    Vector2f opt_bodyRate = optflow.bodyRate();
    // uint32_t opt_integration_timespan = optflow.integration_timespan();
    // float bodyRateZ = optflow.bodyRateZ();

    opt_flow[0]= opt_flowRate.x;
    opt_flow[1]= opt_flowRate.y;

    opt_gyro[0] = opt_bodyRate.x;
    opt_gyro[1] = opt_bodyRate.y;
    // opt_gyro[2] = bodyRateZ;
    // cliSerial->printf("%f,%f,%f,%f,%d\r\n",opt_flowRate.x,opt_flowRate.y,opt_bodyRate.x,opt_bodyRate.y,opt_integration_timespan);

//==============================INS======================================//

    // ips_gyro = ins.get_gyro();
    // ips_accel = ins.get_accel();
    // ins_att[0] = (ahrs.roll);
    // ins_att[1] = (ahrs.pitch);
    // ins_att[2] = (double)ToRad(ahrs.yaw_sensor)/100;
    // calc yaw angle
    // yaw_angle = -(ins_att[2] - frame_yaw_offset);
    // if(yaw_angle>2*M_PI){
    //     yaw_angle -= 2*M_PI;
    // }
    // if(yaw_angle<0){
    //     yaw_angle += 2*M_PI;
    // }
    // cliSerial->printf("%.1f,%.1f\r\n",(double)ToDeg(frame_yaw_offset),(double)ToDeg(yaw_angle));
    // hal.uartF->printf("INS:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",ips_gyro.x,ips_gyro.y,ips_gyro.z,ips_accel.x,ips_accel.y,ips_accel.z);

//==============================YAW-ANGLE======================================//
    // calc yaw angle
    yaw_angle = (double)ToRad(ahrs.yaw_sensor)/100 - frame_yaw_offset;  //rad
//==============================KALMAN======================================//
    // k_timer = AP_HAL::micros();

    if(nls_healthy){
        // Convert mm -> m
        ips_timer = AP_HAL::millis() - ips_timer; 
        // cliSerial->printf("%d\r\n",ips_timer);
        R_OP[0] /= 1000; 
        R_OP[1] /= 1000;
        R_OP[2] /= 1000; 

        nls_timeout_s = (double)(AP_HAL::millis() - _nls_timeout_s)/1000;
        _nls_timeout_s = (double)AP_HAL::millis();
        // cliSerial->printf("%.3f\r\n",nls_timeout_s);
    }
    else{
        nls_timeout_s = (double)(AP_HAL::millis() - _nls_timeout_s)/1000;
        // cliSerial->printf("%.3f\r\n",nls_timeout_s);
    //     R_OP[0] = 0; 
    //     R_OP[1] = 0;
    //     R_OP[2] = 0;
    }     
    
    // KALMAN
    if(kalman_type == 0){
        LPF_pos(R_OP,nls_healthy,20,max_inno_m, nls_timeout_s,k_pos,k_vel);
    }
    else{
        multirate_kalman_v4(R_OP, nls_healthy, opt_flow, opt_gyro, -yaw_angle, k_pos, k_vel);

    }
    
    // DEBUG LOG
    if(g.user_raw_log == 1){    
        ahrs.get_NavEKF2().getEulerAngles(-1,imu_euler);
        ahrs.get_NavEKF2().getVelNED(-1,imu2_velNED);
        ahrs.get_NavEKF3().getVelNED(-1,imu3_velNED);
        // roll    : (int16_t)(100*degrees(euler.x)), // roll angle (centi-deg, displayed as deg due to format string)
        // pitch   : (int16_t)(100*degrees(euler.y)), // pitch angle (centi-deg, displayed as deg due to format string)
        // yaw     : (uint16_t)wrap_360_cd(100*degrees(euler.z)), // yaw angle (centi-deg, displayed as deg due to format string)
        // velN    : (float)(velNED.x), // velocity North (m/s)
        // velE    : (float)(velNED.y), // velocity East (m/s)
        // velD    : (float)(velNED.z), // velocity Down (m/s)

        hal.uartD->printf("%d %.2f %.2f %.2f %d %.2f %.2f %.2f %.2f %.2f ",AP_HAL::millis64(),R_OP[0],R_OP[1],R_OP[2],nls_healthy,opt_flow[0],opt_flow[1],opt_gyro[0],opt_gyro[1],(double)ToRad(ahrs.yaw_sensor)/100);
        hal.uartD->printf("%.2f %.2f %.2f ",imu_euler.x,imu_euler.y,imu_euler.z);
        hal.uartD->printf("%.2f %.2f %.2f ",imu2_velNED.x,imu2_velNED.y,imu2_velNED.z);
        // hal.uartD->printf("%.2f %.2f %.2f ",imu3_velNED.x,imu3_velNED.y,imu3_velNED.z);
        hal.uartD->printf("\r\n");
    }

    s16_range_finder = (int)(k_pos[2]*100);    
    AP_Notify::flags.ips_x = (int)(k_pos[0]*100);
    AP_Notify::flags.ips_y = (int)(k_pos[1]*100);
    AP_Notify::flags.ips_z = (int)(k_pos[2]*100);
    // k_timer = AP_HAL::micros()-k_timer;  
    //DATA Flash
    // Log_Write_NLS_KAL(R_OP[0],R_OP[1],R_OP[2],(float)nls_healthy);
    // reset nls_healthy after kalman
    // cliSerial->printf("%.2f,%.2f,%.2f,%.1f\r\n",k_pos[0],k_pos[1],k_pos[2],(double)ToDeg(yaw_angle));
    if(nls_healthy){
        nls_healthy = false; 
    }
	// hal.uartF->printf("K: %.2f, %.2f, %.2f, %d\r\n",k_pos[0],k_pos[1],k_pos[2],k_timer);

    // SET VALUE WHEN ARMED
    if (motors->armed() && !is_armed)
    {
        is_armed = true;

        target_roll = ahrs.roll;
        target_pitch = ahrs.pitch;
        frame_yaw_offset = (double)ToRad(ahrs.yaw_sensor)/100;

        //reduce max_inno_m on flight
        max_inno_m[0] = 0.8;
        max_inno_m[1] = 0.8;
        max_inno_m[2] = 0.8;

        // Reset PID value
        pid_posx.pid_reset();
        pid_posy.pid_reset();

        h_vel_xy_i_x = 0;
        h_vel_xy_i_y = 0;

        // cliSerial->printf("TARGET_POS: %.2f, %.2f \n",v3f_target_control.x , v3f_target_control.y);
    } else if (!motors->armed() && is_armed ) is_armed = false;
    
    // ADD TRAJECTORY
    if(g.user_trajectory == 0){                      // HOVER [cm]
        circle_step = 0;
        // v3f_target_control.x = g.user_hover_x;
        // v3f_target_control.y = g.user_hover_y;
        // Update from GUI
    }

    else if(g.user_trajectory == 1){                 // CIRCLE [cm]
        circle_x  = 110 + circle_r * cos(circle_w * circle_step - PI_NUMBER);
        circle_y  = 110 - circle_r * sin(circle_w * circle_step - PI_NUMBER);
        
        circle_step = circle_step + 0.01;  //increase @100Hz
        if(circle_step > circle_T){
            circle_step -= circle_T;
        }

        // Update target pos
        v3f_target_control.x = circle_x;
        v3f_target_control.y = circle_y;           
    }

    else{                                        // HOVER [cm]
        circle_step = 0;
        // v3f_target_control.x = g.user_hover_x;
        // v3f_target_control.y = g.user_hover_y;
        // Update from GUI
    }

}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // debug PID 2
    // cliSerial->printf("p:%f %f %f\r\n",pid_pitch, pid_roll, d_target);

    //set pid
    pid_posx.pid_set_k_params(g.user_rll_kp,g.user_rll_ki,g.user_rll_kd);
    pid_posy.pid_set_k_params(g.user_pit_kp,g.user_pit_ki,g.user_pit_kd);

    pid_posx.pid_lpf_value = g.user_pid_lpf_value;
    pid_posy.pid_lpf_value = g.user_pid_lpf_value;
    pid_mode = g.user_pid_mode;
    
    pos_kp = g.user_pid2_kp;
    vel_kp = g.user_pid2_kp;
    vel_ki = g.user_pid2_ki;


    h_accel_cms = g.user_accel_max;
    h_speed_cms = g.user_speed_max;

    circle_r = g.user_circle_r;
    circle_T = g.user_circle_T;
    circle_w = 2*PI_NUMBER/circle_T; 

    lean_angle_max = g.user_lean_max;
    kalman_type = g.user_kalman_type;
    // put your 20Hz code here
//==============================TEMPERATURE======================================//
    // air_temperature = barometer.get_temperature();
    // hal.uartF->printf("temp:%f",air_temperature);

//==============================IPS_TRANSMIT======================================//
    hal.uartE->printf("{PARAM,TRIGGER_US}\n");
    ips_timer = AP_HAL::millis();    // trigger IPS_transmission on Tiva C

//==============================GUI_PLANNER======================================//
    // SEND POSITION TO GUI
    if(g.user_raw_log == 0){    
        hal.uartD->printf("{\"x\":%d,\"y\":%d}\r\n",(int)(k_pos[0]*100),(int)(k_pos[1]*100));
        // hal.uartD->printf("{\"x\":%d,\"y\":%d,\"z\":%d,\"tx\":%d,\"ty\":%d}\r\n",(int)(k_pos[0]*100),(int)(k_pos[1]*100),(int)(k_pos[2]*100),(int)v3f_target_control.x,(int)v3f_target_control.y);
   
    }
    // GET TARGET FROM GUI
    gui_bytes = hal.uartD->available();
    // 
    // value = js0n("x", 0, gui_str , strlen(gui_str), &valen);
    // atoi(value);

    while (gui_bytes-- > 0) {
        // Get data string here
        gui_char[0] = hal.uartD->read();
        // start-of-frame
        if(gui_char[0] == '{'){
            gui_char[1] = gui_char[0];
            gui_buff = 2;
            gui_state = 1;
        }
        else if((gui_state==1) && (gui_char[0] == '}')){   // end-of-frame: start parsing
            gui_char[gui_buff] = gui_char[0];

            gui_flag = 1;   // NEW TARGET UPDATE
            // hal.uartD->printf("%s",&gui_char[1]);

            j_value = js0n("tx", 0, &gui_char[1] , strlen(&gui_char[1]), &j_valen);
            if(j_value != NULL){
                // hal.uartD->printf("x:%d",(int)atoi(j_value));
                gui_target = (int)atoi(j_value);
                if((gui_target > MIN_FENCE_CM) && (gui_target < MAX_FENCE_CM)){                    
                    // ONLY ACCEPT TARGET ON GUIDE MODE
                    if(g.user_trajectory == 0){     
                        v3f_target_control.x = gui_target;
                    }
                }
            }

            j_value = js0n("ty", 0, &gui_char[1] , strlen(&gui_char[1]), &j_valen);
            if(j_value != NULL){
                // hal.uartD->printf("y:%d",(int)atoi(j_value));
                gui_target = (int)atoi(j_value);
                if((gui_target > MIN_FENCE_CM) && (gui_target < MAX_FENCE_CM)){
                    // ONLY ACCEPT TARGET ON GUIDE MODE
                    if(g.user_trajectory == 0){     
                        v3f_target_control.y = gui_target;
                    }
                }
            }
            gui_buff = 0;
            gui_state = 0;
        }
        else{   // fill buffer after catch start header
            if(gui_state == 1){
                gui_char[gui_buff] = gui_char[0];
                gui_buff++;
            }
        }
    }    

}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{

    // put your 1Hz code here
}
#endif
