#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Copter::headcutter_init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter Alt Hold if the Rotor Runup is not complete
    if (!ignore_checks && !motors->rotor_runup_complete()){
        return false;
    }
#endif

    // initialize vertical speeds and leash lengths
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // stop takeoff if running
    takeoff_stop();

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Copter::headcutter_run()
{
    float pid_roll_tmp, pid_pitch_tmp, error_x, error_y, x_gain = 0.1f, y_gain = 0.1f;

    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors->rotor_runup_complete());
#else
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);
#endif

    // Alt Hold State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        althold_state = AltHold_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
#if FRAME_CONFIG == HELI_FRAME    
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
#else
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case AltHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        avoid.adjust_roll_pitch(target_roll, target_pitch, aparm.angle_max);
#endif

        //THIS IS A BIG HACK
    ////////////////////////////////////PID CONTROL 1////////////////////////////////////////////////
        if(pid_mode == 0){
            error_x = v3f_target_control.x - k_pos[0]*100;   //cm
            error_y = v3f_target_control.y - k_pos[1]*100;

            pid_roll = pid_posx.pid_process(error_x);           //Uc: control angle [degree]
            pid_pitch = pid_posy.pid_process(-error_y);

            pid_roll *= 100; // centi degree
            pid_pitch *= 100;

            // lean_angle_max limit [centi-deg]
            h_accel_total = sqrt(pid_roll*pid_roll + pid_pitch*pid_pitch);
            if (h_accel_total > lean_angle_max * sqrt(2)) {
            pid_roll = lean_angle_max * pid_roll/h_accel_total;
            pid_pitch = lean_angle_max * pid_pitch/h_accel_total;
            } 

            // Coordinate Rotate
            pid_roll_tmp = pid_roll;
            pid_pitch_tmp = pid_pitch;

            h_roll_target = pid_roll_tmp * cos(yaw_angle) + pid_pitch_tmp * sin(yaw_angle);     //centi-degree
            h_pitch_target = - pid_roll_tmp * sin(yaw_angle) + pid_pitch_tmp * cos(yaw_angle);

            h_roll_target = h_roll_target * M_PI/18000;     //rad
            h_pitch_target = h_pitch_target * M_PI/18000;

            h_accel_forward = h_pitch_target *(18000/M_PI); //centi-deg
            float cos_pitch_target = cosf(h_pitch_target);
            //centi-deg
            h_accel_right = atanf( (tanf(h_roll_target) * GRAVITY_MSS) * cos_pitch_target / GRAVITY_MSS) * (18000/M_PI);

            pid_roll = h_accel_right;
            pid_pitch = h_accel_forward;
            // cliSerial->printf("pid1:%f %f\n",pid_roll,pid_pitch);
        }
     ////////////////////////////////////PID-CONTROL-2////////////////////////////////////////////////
        else{
            //==========POS-TO-RATE=======================
            error_x = (v3f_target_control.x - k_pos[0]*100);   //cm
            error_y = (v3f_target_control.y - k_pos[1]*100);

            // calc distance to target
            d2_target = sqrt(error_x*error_x + error_y*error_y);     //cm
            // pos_kp = 1;
            // h_accel_cms = 500; 
            // h_speed_cms = 500;
            
            // calculate the distance at which we swap between linear and sqrt velocity response
            linear_d = h_accel_cms/(2.0f*pos_kp*pos_kp);

            if (d2_target > 2.0f*linear_d) {
                // velocity response grows with the square root of the distance
                float h_vel_sqrt = safe_sqrt(2.0f*h_accel_cms*(d2_target-linear_d));
                vel_target_x = h_vel_sqrt * error_x/d2_target;
                vel_target_y = h_vel_sqrt * error_y/d2_target;
            }else{
                // velocity response grows linearly with the distance
                vel_target_x = pos_kp * error_x;
                vel_target_y = pos_kp * error_y;
            }

             // scale velocity within speed limit
            float h_vel_total = sqrt(vel_target_x*vel_target_x + vel_target_y*vel_target_y);
            if (h_vel_total > h_speed_cms) {
                vel_target_x = h_speed_cms * vel_target_x/h_vel_total;
                vel_target_y = h_speed_cms * vel_target_y/h_vel_total;
            }

            //==========RATE-TO-ACCEL==================
            // check if vehicle velocity is being overridden
        
            // feed forward desired acceleration calculation
            h_dt = 0.0025;
            accel_feedforward_x = (vel_target_x - h_vel_last_x)/h_dt;
            accel_feedforward_y = (vel_target_y - h_vel_last_y)/h_dt;

            // store this iteration's velocities for the next iteration
            h_vel_last_x = vel_target_x;
            h_vel_last_y = vel_target_y;

            // calculate velocity error
            h_vel_error_x = vel_target_x - k_vel[0]*100; //cm
            h_vel_error_y = vel_target_y - k_vel[1]*100;

            // call pi controller
            // _pi_vel_xy.set_input(_vel_error);

            // --> filter vel input 5Hz
            h_vel_error_x = h_vel_error_x_ + 0.01242 * (h_vel_error_x - h_vel_error_x_);
            h_vel_error_x_ = h_vel_error_x;

            h_vel_error_y = h_vel_error_y_ + 0.01242 * (h_vel_error_y - h_vel_error_y_);
            h_vel_error_y_ = h_vel_error_y;

            // get p 

            // combine feed forward accel with PID output from velocity error and scale PID output to compensate for optical flow measurement induced EKF noise
            accel_target_x = accel_feedforward_x + h_vel_error_x * 1.0f;    //kp=1 , cmss
            accel_target_y = accel_feedforward_y + h_vel_error_y * 1.0f;

            //==========ACCEL-TO-LEAN==================

            lean_ang_max = lean_angle_max;    //centi-degree
            accel_max = GRAVITY_MSS * 100.0f * tanf(ToRad(30));  //cmss - 30deg limit

            // scale desired acceleration if it's beyond acceptable limit
            h_accel_total = sqrt(accel_target_x*accel_target_x + accel_target_y*accel_target_y);
        
            // constrain
            if (h_accel_total > accel_max) {
            accel_target_x = accel_max * accel_target_x/h_accel_total;
            accel_target_y = accel_max * accel_target_y/h_accel_total;
            } 

            // lowpass filter on NE accel: 10hz
            accel_target_x = accel_target_x_ + 0.02469 * (accel_target_x - accel_target_x_);
            accel_target_x_ = accel_target_x;

            accel_target_y = accel_target_y_ + 0.02469 * (accel_target_y - accel_target_y_);
            accel_target_y_ = accel_target_y;

            // rotate accelerations into body forward-right frame
            h_accel_right = accel_target_x * cos(yaw_angle) + accel_target_y * sin(yaw_angle);
            h_accel_forward = -accel_target_x * sin(yaw_angle) + accel_target_y * cos(yaw_angle);

            // update angle targets that will be passed to stabilize controller
            h_pitch_target = atanf(-h_accel_forward/(GRAVITY_MSS * 100))*(18000/M_PI);  //centi-degree
            float cos_pitch_target = cosf(h_pitch_target*M_PI/18000);
            h_roll_target = atanf(h_accel_right * cos_pitch_target/(GRAVITY_MSS * 100))*(18000/M_PI);

            // compatible
            pid_pitch = h_pitch_target;   //0---1000
            pid_roll = h_roll_target;

            // cliSerial->printf("pid2:%f %f\n",pid_roll,pid_pitch);
            // RC INPUT
            // ^
            // |
            // | pitch < 0

            // < --- roll <0

        }
        ////////////////////////////////////////////////////////////////////////////////////
        if(g.user_parm1 == 1){
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pid_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        }
        else if(g.user_parm1 == 2){
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, pid_pitch, target_yaw_rate, get_smoothing_gain());

        }
        else{
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pid_roll, pid_pitch, target_yaw_rate, get_smoothing_gain());

        }
        //END OF HACK

        // call attitude controller
        // attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        break;
    }
}
