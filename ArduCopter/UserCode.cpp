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

    circle_T = 16;// 24s/round
    circle_w = 2*PI_NUMBER/circle_T;       
    circle_step = 0;
    circle_r = 45;
    lean_angle_max = 4500;
    max_inno_m[0] = 10;
    max_inno_m[1] = 10;
    max_inno_m[2] = 10;
    update_loop = 0;
    kalman_type = 0;
    pid_mode = 0;
    en_feedforward = 0;
    heading_mode = 0;
}
#endif
 

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    // uartF: serial5, baud 115200
    // k_timer = AP_HAL::micros();
//===========================CONTROLLER_FLAG=============================//
    update_loop = 1;
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
        // hal.uartF->printf("NLS: %d, %d, %d, %d\r\n",(int)R_OP[0],(int)R_OP[1],(int)R_OP[2],ips_timer);  
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



//=================================KALMAN======================================//
    // NLS_TIMER
    if(nls_healthy){
        // Convert mm -> m
        ips_timer = AP_HAL::millis() - ips_timer; 
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

    // // DEBUG LOG
    // if(g.user_raw_log == 1){    
    //     ahrs.get_NavEKF2().getEulerAngles(-1,imu_euler);
    //     ahrs.get_NavEKF2().getVelNED(-1,imu2_velNED);
    //     ahrs.get_NavEKF3().getVelNED(-1,imu3_velNED);
    //     // roll    : (int16_t)(100*degrees(euler.x)), // roll angle (centi-deg, displayed as deg due to format string)
    //     // pitch   : (int16_t)(100*degrees(euler.y)), // pitch angle (centi-deg, displayed as deg due to format string)
    //     // yaw     : (uint16_t)wrap_360_cd(100*degrees(euler.z)), // yaw angle (centi-deg, displayed as deg due to format string)
    //     // velN    : (float)(velNED.x), // velocity North (m/s)
    //     // velE    : (float)(velNED.y), // velocity East (m/s)
    //     // velD    : (float)(velNED.z), // velocity Down (m/s)
    //     hal.uartD->printf("%d %.2f %.2f %.2f %d %.2f %.2f %.2f %.2f %.2f ",AP_HAL::millis64(),R_OP[0],R_OP[1],R_OP[2],nls_healthy,opt_flow[0],opt_flow[1],opt_gyro[0],opt_gyro[1],(double)ToRad(ahrs.yaw_sensor)/100);
    //     hal.uartD->printf("%.2f %.2f %.2f ",imu_euler.x,imu_euler.y,imu_euler.z);
    //     hal.uartD->printf("%.2f %.2f %.2f ",imu2_velNED.x,imu2_velNED.y,imu2_velNED.z);
    //     // hal.uartD->printf("%.2f %.2f %.2f ",imu3_velNED.x,imu3_velNED.y,imu3_velNED.z);
    //     hal.uartD->printf("\r\n");
    // }
    
    //DATA Flash
    // Log_Write_NLS_KAL(R_OP[0],R_OP[1],R_OP[2],(float)nls_healthy);

    s16_range_finder = (int)(k_pos[2]*100);
    AP_Notify::flags.ips_x = (int)(k_pos[0]*100);
    AP_Notify::flags.ips_y = (int)(k_pos[1]*100);
    AP_Notify::flags.ips_z = (int)(k_pos[2]*100);   
    
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
    } else if (!motors->armed() && is_armed ) is_armed = false;

//=================================TRAJECTORY======================================//    
    if(trajectory_type == 0){                      // HOVER [cm]
        circle_step = 0;
        // zero feedforward velocity in hover
        target_vel_desire.x = 0.0f;
        target_vel_desire.y = 0.0f;
        // Update from GUI
        // v3f_target_control.x = g.user_hover_x;
        // v3f_target_control.y = g.user_hover_y;  
    }

    else if(trajectory_type == 1){                 // CIRCLE [cm]
        //increase step @100Hz
        circle_step = circle_step + 0.01;  
        if(circle_step > circle_T){
            circle_step -= circle_T;
        }
        // // store previous pos
        // pre_circle_x = circle_x;
        // pre_circle_y = circle_y;
        // update new pos
        circle_alpha = circle_w * circle_step;
        circle_x  = circle_r * cos(circle_alpha);
        circle_y  = circle_r * sin(circle_alpha);
        circle_x += CENTER_X;
        circle_y += CENTER_Y;

        circle_v.x = -circle_r * circle_w * sin(circle_alpha);
        circle_v.y = circle_r * circle_w * cos(circle_alpha);
        circle_a.x = -circle_r * circle_w*circle_w * cos(circle_alpha);
        circle_a.y = -circle_r * circle_w*circle_w * sin(circle_alpha);
        // calc circle_heading
        circle_heading = atan2(circle_v.x,circle_v.y);
        // update target pos
        v3f_target_control.x = circle_x;
        v3f_target_control.y = circle_y;   
        // update feedforward accel
        target_acc_desire.x = circle_a.x;
        target_acc_desire.y = circle_a.y;
        // update feedforward vel
        target_vel_desire.x = circle_v.x;
        target_vel_desire.y = circle_v.y;
    }
    else if(trajectory_type == 2){                // INFINITY - type 1: lemniscate of Bernoulli
        //increase step @100Hz
        circle_step = circle_step + 0.01;  
        if(circle_step > circle_T){
            circle_step -= circle_T;
        }
        // // store previous pos
        // pre_circle_x = circle_x;
        // pre_circle_y = circle_y;
        // update new pos
        circle_alpha = circle_w * circle_step;
        circle_x = circle_r * cos(circle_alpha) / (1+pow(sin(circle_alpha),2));
        circle_y = circle_x * sin(circle_alpha);
        circle_x += CENTER_X;
        circle_y += CENTER_Y;
        circle_v.x = (circle_r*circle_w*sin(circle_alpha)*(pow(sin(circle_alpha),2) - 3))/pow((pow(sin(circle_alpha),2) + 1),2);
        circle_v.y = -(circle_r*circle_w*(3*pow(sin(circle_alpha),2) - 1))/pow((pow(sin(circle_alpha),2) + 1),2);
        circle_a.x = (circle_r*pow(circle_w,2)*cos(circle_alpha)*(10*pow(cos(circle_alpha),2) + pow(cos(circle_alpha),4) - 8))/(pow(pow(cos(circle_alpha),2) - 2,3));
        circle_a.y = (circle_r*pow(circle_w,2)*(14*sin(2*circle_alpha) + 3*sin(4*circle_alpha)))/(4*pow(pow(cos(circle_alpha),2) - 2,3));
        // calc circle_heading
        circle_heading = atan2(circle_v.x, circle_v.y);
        // update target pos
        v3f_target_control.x = circle_x;
        v3f_target_control.y = circle_y; 
        // calc feedforward accel
        target_acc_desire.x = circle_a.x;
        target_acc_desire.y = circle_a.y;
        // update feedforward vel
        target_vel_desire.x = circle_v.x;
        target_vel_desire.y = circle_v.y;
    }

    else if(trajectory_type == 3){                // INFINITY 8 - type 2
        //increase step @100Hz
        circle_step = circle_step + 0.01;  
        if(circle_step > circle_T){
            circle_step -= circle_T;
        }
        // // store previous pos
        // pre_circle_x = circle_x;
        // pre_circle_y = circle_y;
        // update new pos
        circle_alpha = circle_w * circle_step;
        circle_x = circle_r * cos(circle_alpha);
        circle_y = circle_x * sin(circle_alpha);
        circle_x += CENTER_X;
        circle_y += CENTER_Y;

        circle_v.x = -circle_r*circle_w*sin(circle_alpha);
        circle_v.y = circle_r*circle_w*cos(2*circle_alpha);
        circle_a.x = -circle_r*pow(circle_w,2)*cos(circle_alpha);
        circle_a.y = -2*circle_r*pow(circle_w,2)*sin(2*circle_alpha);
        // calc circle_heading
        circle_heading = atan2(circle_v.x, circle_v.y);
        // update target pos
        v3f_target_control.x = circle_x;
        v3f_target_control.y = circle_y; 
        // calc feedforward accel
        target_acc_desire.x = circle_a.x;
        target_acc_desire.y = circle_a.y;
        // update feedforward vel
        target_vel_desire.x = circle_v.x;
        target_vel_desire.y = circle_v.y;
    } 
    else{                                           // HOVER [cm]
        circle_step = 0;
        // zero feedforward velocity in loiter
        target_vel_desire.x = 0.0f;
        target_vel_desire.y = 0.0f;
        // Update from GUI
        // v3f_target_control.x = g.user_hover_x;
        // v3f_target_control.y = g.user_hover_y;
    }
    
    // heading_ctrl [centi-degree]
    if(heading_mode == 0){                      // HOLD: all mode
        heading_ctrl = ToDeg(frame_yaw_offset) * 100;
        // attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pid_roll, pid_pitch, target_yaw_rate, get_smoothing_gain());
    }
    else if(heading_mode == 1){                 // ALONG: exclude HOVER mode
        heading_ctrl = ToDeg(circle_heading + frame_yaw_offset)*100;  
        // attitude_control->input_euler_angle_roll_pitch_yaw(pid_roll, pid_pitch, heading_ctrl, true, get_smoothing_gain());
    }
    else{                                       // CENTRIPETAL: only in CIRCLE
        heading_ctrl = ToDeg(circle_heading + frame_yaw_offset - M_PI/2)*100;
        // attitude_control->input_euler_angle_roll_pitch_yaw(pid_roll, pid_pitch, heading_ctrl, true, get_smoothing_gain());
    }
    // heading_ctrl_mp [degree]: no_offset
    heading_ctrl_mp = heading_ctrl/100 - ToDeg(frame_yaw_offset);   //degree
    // Est loop duration
    // k_timer = AP_HAL::micros()-k_timer;
    // cliSerial->printf("t %f\r\n",(float)k_timer);
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

    // set pid
    pid_posx.pid_set_k_params(g.user_rll_kp,g.user_rll_ki,g.user_rll_kd);
    pid_posy.pid_set_k_params(g.user_pit_kp,g.user_pit_ki,g.user_pit_kd);

    pid_posx.pid_lpf_value = g.user_pid_lpf_value;
    pid_posy.pid_lpf_value = g.user_pid_lpf_value;
    // pid_mode = g.user_pid_mode;

    // h_accel_cms = g.user_accel_max;
    // h_speed_cms = g.user_speed_max;

    // circle_r = g.user_circle_r;
    // circle_T = g.user_circle_T;
    // circle_w = 2*PI_NUMBER/circle_T; 

    lean_angle_max = g.user_lean_max;
    // kalman_type = g.user_kalman_type;
    // put your 20Hz code here
//==============================TEMPERATURE======================================//
    // air_temperature = barometer.get_temperature();
    // hal.uartF->printf("temp:%f",air_temperature);

//==============================IPS_TRANSMIT======================================//
    hal.uartE->printf("{PARAM,TRIGGER_US}\n");
    ips_timer = AP_HAL::millis();    // trigger IPS_transmission on Tiva C
    // update_slowloop = 1;
        // Update MP parameter
    pos_kp = g.user_pid2_kp;
    vel_kp = g.user_pid2_kd;
    vel_ki = g.user_pid2_ki;
    kff = g.user_pid2_kff;

    //==============================GUI_PLANNER======================================//
    // SEND POSITION TO GUI 20Hz
    // hal.uartD->printf("{\"x\":%d,\"y\":%d}\r\n",(int)(k_pos[0]*100),(int)(k_pos[1]*100));
    hal.uartD->printf("{\"x\":%d,\"y\":%d,\"tx\":%d,\"ty\":%d,\"a\":%d}\r\n",(int)(k_pos[0]*100),(int)(k_pos[1]*100),(int)v3f_target_control.x,(int)v3f_target_control.y,(int)heading_ctrl_mp);
    // GET TARGET FROM GUI
    gui_bytes = hal.uartD->available();
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
                if((gui_target >= MIN_FENCE_CM) && (gui_target <= MAX_FENCE_CM) && (trajectory_type == 0)){                    
                    // ONLY ACCEPT TARGET ON GUIDE MODE
                    v3f_target_control.x = gui_target;           
                }
            }
            j_value = js0n("ty", 0, &gui_char[1] , strlen(&gui_char[1]), &j_valen);
            if(j_value != NULL){
                // hal.uartD->printf("y:%d",(int)atoi(j_value));
                gui_target = (int)atoi(j_value);
                if((gui_target >= MIN_FENCE_CM) && (gui_target <= MAX_FENCE_CM) && (trajectory_type == 0)){
                    // ONLY ACCEPT TARGET ON GUIDE MODE
                    v3f_target_control.y = gui_target;
                }
            }
            j_value = js0n("tj", 0, &gui_char[1] , strlen(&gui_char[1]), &j_valen);
            if(j_value != NULL){
                // hal.uartD->printf("y:%d",(int)atoi(j_value));
                gui_target = (int)atoi(j_value);
                // TJ: 0 -> 3
                if((gui_target >= 0) && (gui_target <= 3)){
                    trajectory_type = gui_target;
                }
            }
            j_value = js0n("r", 0, &gui_char[1] , strlen(&gui_char[1]), &j_valen);
            if(j_value != NULL){
                // hal.uartD->printf("y:%d",(int)atoi(j_value));
                gui_target = (int)atoi(j_value);
                // R: 10 -> 80 cm
                if((gui_target >= 10) && (gui_target <= 80)){
                    circle_r = gui_target;
                }
            }
            j_value = js0n("t", 0, &gui_char[1] , strlen(&gui_char[1]), &j_valen);
            if(j_value != NULL){
                // hal.uartD->printf("y:%d",(int)atoi(j_value));
                gui_target = (int)atoi(j_value);
                // T: 2 -> 20 s
                if((gui_target >= 2) && (gui_target <= 20)){
                    circle_T = gui_target;
                    circle_w = 2*PI_NUMBER/circle_T;
                }
            }
            j_value = js0n("op", 0, &gui_char[1] , strlen(&gui_char[1]), &j_valen);
            if(j_value != NULL){
                // hal.uartD->printf("y:%d",(int)atoi(j_value));
                gui_target = (int)atoi(j_value);
                // kalman_type
                // 0: us 
                // 1: us+opt
                if((gui_target >= 0) && (gui_target <= 1)){
                    kalman_type = gui_target;     
                }
            }
            j_value = js0n("ff", 0, &gui_char[1] , strlen(&gui_char[1]), &j_valen);
            if(j_value != NULL){
                // hal.uartD->printf("y:%d",(int)atoi(j_value));
                gui_target = (int)atoi(j_value);
                // kalman_type
                // 0: pid 
                // 1: pid+ff
                if((gui_target >= 0) && (gui_target <= 1)){
                    // en_feedforward = gui_target; 
                    pid_mode = gui_target;     
                }
            }
            j_value = js0n("hd", 0, &gui_char[1] , strlen(&gui_char[1]), &j_valen);
            if(j_value != NULL){
                // hal.uartD->printf("y:%d",(int)atoi(j_value));
                gui_target = (int)atoi(j_value);
                // heading_mode
                // 0: HOLD
                // 1: ALONG
                // 2: CENTRIPETAL
                if((gui_target >= 0) && (gui_target <= 2)){
                    if(gui_target == 0){            // 0: HOLD
                       heading_mode = gui_target;   
                    }
                    else if(gui_target == 1){       // 1: ALONG: exclude HOVER
                        if(trajectory_type != 0){
                            heading_mode = gui_target;
                        }
                    }
                    else{                           // 2: CENTRIPETAL: only in CIRCLE
                        if(trajectory_type == 1){
                            heading_mode = gui_target;
                        }
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
