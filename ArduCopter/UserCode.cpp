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
        if(ips_char[0] == 's'){
            c_buff = 1;
            c_state = 1;
        }
        else if(ips_char[0] == 'e'){
            // end-of-frame: get ips_pos & time_stamp
            if((ips_char[4] ==',') && (ips_char[8] ==',') && (c_buff == 12)){
                // valid frame
                ips_data[0] = (ips_char[1]-0x30)*100 + (ips_char[2]-0x30)*10 + (ips_char[3]-0x30); //pos_x
                ips_data[1] = (ips_char[5]-0x30)*100 + (ips_char[6]-0x30)*10 + (ips_char[7]-0x30); //pos_y
                ips_data[2] = (ips_char[9]-0x30)*100 + (ips_char[10]-0x30)*10 + (ips_char[11]-0x30); //pos_z
                ips_data[3] = AP_HAL::millis()-ips_delay_ms;
                hal.uartF->printf("%d,%d,%d,%d",ips_data[0],ips_data[1],ips_data[2],ips_data[3]);
                
            }
            hal.uartF->printf("\r\n");
            c_buff = 0;
            c_state = 0;
        }
        else{
            if(c_state == 1){
                ips_char[c_buff] = ips_char[0];
                // hal.uartF->printf("%c",ips_char[c_buff]);
                c_buff++;
            }
        }
    }
//==============================PX4FLOW======================================//
    optflow.update();
    Vector2f opt_flowRate = optflow.flowRate();
    Vector2f opt_bodyRate = optflow.bodyRate();
    hal.uartF->printf("%f,%f,%f,%f\r\n",opt_flowRate.x,opt_flowRate.y,opt_bodyRate.x,opt_bodyRate.y);

    // const v &gyro = ins.get_gyro();
    // hal.uartF->printf("F:%d,%f,%f,%f,%f,%f,%f\r\n",AP_HAL::millis(),(double)ToDeg(ahrs.roll),(double)ToDeg(ahrs.pitch),(double)ToDeg(ahrs.yaw),gyro.x,gyro.y,gyro.z);


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
    // put your 10Hz code here
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
