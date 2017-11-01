#include "Copter.h"


#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    c_buff = 0;     //0: temp-buf
    c_state = 0;    //0: wait-start-buff, 1: data-buffer, 2: end-buffer-->0
    ips_bytes = 0;
    max_NOR = 5;
    optflow.init();
    multirate_kalman_initialize();
#ifdef RUN_TRILATERATION
    LeastSquare_NJ_initialize();
#endif
    pid_posx.init(pid_pos_x_param);
    pid_posy.init(pid_pos_y_param);
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
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
        else if(c_state==1&&(ips_char[0] == '\n')&&(c_buff>10)){   // end-of-frame: start parsing
            // number-of-receiver: 2bytes - 1 node
                // parse data: int16_t, distance in mm
            for(uint8_t i = 0; i < max_NOR; i++){
                ips_data[i] = (ips_char[2*i + 2] * 256) + (ips_char[2*i + 1]); //NODE i (0->4)
                if(i<max_NOR){
                    nlsMR[i] = ips_data[i];
                }
            }
            // cliSerial->printf("R:%d,%d,%d,%d,%d\r\n",ips_data[0],ips_data[1],ips_data[2],ips_data[3],ips_data[4]);
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
  // // [121.8, 120.4, 185.4, 185.5, 92.9];
  // nlsMR[0] = 1218;
  // nlsMR[1] = 1204;
  // nlsMR[2] = 1854;
  // nlsMR[3] = 1855;
  // nlsMR[4] = 929;

    maxMR = 0;
    for(c_i = 0; c_i < max_NOR; c_i++){
        if(maxMR < nlsMR[c_i]){
            maxMR = nlsMR[c_i];
            max_index = c_i;
        }
        sortMR[c_i] = nlsMR[c_i];
    }
    for(c_i = 0; c_i < 15; c_i++){
        sortRCM[c_i] = nlsRCM[c_i];
    }
    if(max_index < 4){
        sortMR[4] = nlsMR[max_index];
        sortMR[max_index] = nlsMR[4];
        for(c_i = 0; c_i < 3; c_i++){
            sortRCM[4 + 5*c_i] = nlsRCM[max_index + 5*c_i];
            sortRCM[max_index + 5*c_i] = nlsRCM[4 + 5*c_i];
        }
    }
    // cliSerial->printf("D:%d,%d,%d,%d,%d\r\n",sortMR[0],sortMR[1],sortMR[2],sortMR[3],sortMR[4]);

    if (ips_flag == 1){
        if((sortMR[0]<3500)&&(sortMR[1]<3500)&&(sortMR[2]<3500)&&(sortMR[3]<3500)){
            
            LeastSquare_NJ(g.new_parameter_1,sortRCM, sortMR, 2, R_OP); 
            if((R_OP[0]>0)&&(R_OP[1]>0)&&(R_OP[2]>0)){
                nls_healthy = true;
            }
            else{
                nls_healthy = false; 
            }
            // cliSerial->printf("NLS:%d,%d,%d\r\n",(int)R_OP[0],(int)R_OP[1],(int)R_OP[2]);
        }
        else{
            nls_healthy = false; 
        }

        // cliSerial->printf("NLS:%d,%d,%d\r\n",(int)R_OP[0],(int)R_OP[1],(int)R_OP[2]);
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
    uint32_t opt_integration_timespan = optflow.integration_timespan();
    float bodyRateZ = optflow.bodyRateZ();

    opt_flow[0]= opt_flowRate.x;
    opt_flow[1]= opt_flowRate.y;

    opt_gyro[0] = opt_bodyRate.x;
    opt_gyro[1] = opt_bodyRate.y;
    opt_gyro[2] = bodyRateZ;
    // cliSerial->printf("%f,%f,%f,%f,%d\r\n",opt_flowRate.x,opt_flowRate.y,opt_bodyRate.x,opt_bodyRate.y,opt_integration_timespan);

//==============================INS======================================//

    ips_gyro = ins.get_gyro();
    ips_accel = ins.get_accel();
    // hal.uartF->printf("INS:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",ips_gyro.x,ips_gyro.y,ips_gyro.z,ips_accel.x,ips_accel.y,ips_accel.z);

//==============================KALMAN======================================//
    k_timer = AP_HAL::micros();

    if(nls_healthy){
        // Convert mm -> m
        R_OP[0] /= 1000; 
        R_OP[1] /= 1000;
        R_OP[2] /= 1000;
        lidar_h = R_OP[2];     //ips_z  
    }
    else{
        R_OP[0] = 0; 
        R_OP[1] = 0;
        R_OP[2] = 0;
    }     
    multirate_kalman(R_OP, nls_healthy, opt_flow, opt_gyro, lidar_h, k_pos);
    // reset nls_healthy after kalman
    if(nls_healthy){
        nls_healthy = false; 
    }

    s16_range_finder = (int)(k_pos[2]*100);


    cliSerial->printf("%.2f,%.2f,%.2f\r\n",k_pos[0],k_pos[1],k_pos[2]);
    AP_Notify::flags.ips_x = (int)(k_pos[0]*100);
    AP_Notify::flags.ips_y = (int)(k_pos[1]*100);
    AP_Notify::flags.ips_z = (int)(k_pos[2]*100);
    k_timer = AP_HAL::micros()-k_timer;    

	// hal.uartF->printf("K: %.2f, %.2f, %.2f, %d\r\n",k_pos[0],k_pos[1],k_pos[2],k_timer);

    if (motors->armed() && !is_armed)
    {
        is_armed = true;
        v3f_target_control.x = k_pos[0]*100;
        v3f_target_control.y = k_pos[1]*100;

        target_roll = ahrs.roll;
        target_pitch = ahrs.pitch;

        cliSerial->printf("TARGET_POS: %.2f, %.2f \n",v3f_target_control.x , v3f_target_control.y);
    } else if (!motors->armed() && is_armed ) is_armed = false;
    

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
    // put your 20Hz code here
//==============================TEMPERATURE======================================//
    air_temperature = barometer.get_temperature();
    // hal.uartF->printf("temp:%f",air_temperature);

//==============================IPS_TRANSMIT======================================//
    hal.uartE->printf("{PARAM,TRIGGER_US}\n");
    ips_timer = AP_HAL::millis();    // trigger IPS_transmission on Tiva C
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
