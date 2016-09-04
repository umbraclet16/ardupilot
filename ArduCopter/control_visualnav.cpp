/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_visualnav.cpp - init and run calls for visual navigation flight mode
 *
 * This flight mode is based on loiter flight mode.
 * I created this independent flight mode due to that as long as the auxiliary switch
 * doesn't change position, a set_mode() can just do the job.
 * It's better to make this mode as part of AUTO mode, but it's much harder to develop that way.
 *
 * The modification from loiter mode is:
 * _pos_target is updated based on info from camera(UART) rather than rc input.
 * More specifically, it's based on the position of the target in the picture
 * relative to the center of the picture.
 * The copter descends if the target is within a certain area right below the copter.
 *
 * This flight mode should be triggered by set_mode() in AUTO mode when the target
 * shows up in the photo.
 * Also we need a FLAG(visualnav_enabled) in AUTO mode to make sure we can only change to
 * this mode when we want to(target could be found during both taking off and landing stage, but we
 * definitely don't want it to land when it has just taken off.)
 * This can be done by modifying the function that deals with a GCS do_command which we never need.
 *
 * AUTO mode desired flight status: altitude: 8~10m. velocity: 3m/s? gimbal is needed!
 * 
 * We may not want to modify MissionPlanner, so I just replace Drift mode with this new mode
 * by removing control_drift.cpp and defining drift_init() and drift_run() in this file,
 * rather than add a new enumeration. This saves a lot efforts.
 *
 * I expect to receive a flag 'target_in_image' indicating target is in the image.
 *
 * XXX: what's the baudrate and format of UART input?
 * Format: target_in_image(1 bit), target_coord_x(int), target_coord_y(int)
 *
 * I think it's good to read UART input regularly in a userhook(100/50/10Hz?),
 * what's the frequency of UART input???    Around 5Hz.
 *
 * what's the delay of target recognition???    0.0X s.
 *
 * TODO: how does the controller recognize LANDING?
 * We must lower RC throttle to lower limit, then hope update_land_detector() works fine.
 * But I worry that the moving platform cannot meet the acceleration requirement
 * (<=1m/s/s, see land_detector.cpp line64).
 * If so, we may need to call Copter::init_disarm_motors() in motors.cpp manually.
 * Or, the easiest way is just lower the throttle and change to STABILIZE mode!
 *
 * TODO: choose a function that deal with Mavlink do_command and modify it to set/reset
 * visualnav_enabled FLAG.
 */

// image pixels: 320 * 240. coordination range: x:[-160, 160], y:[-120,120]
#define COORD_RANGE_LIMIT_X 160
#define COORD_RANGE_LIMIT_Y 120
// visualnav_init - initialise visualnav controller
//bool Copter::visualnav_init(bool ignore_checks)
bool Copter::drift_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {

        // set target to current position
        wp_nav.init_loiter_target();

        // initialize vertical speed and acceleration
        pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control.set_accel_z(g.pilot_accel_z);

        // initialise position and desired velocity
        pos_control.set_alt_target(inertial_nav.get_altitude());
        pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

        return true;
    }else{
        return false;
    }
}

// visualnav_run - runs the visualnav controller
// should be called at 100hz or more
//void Copter::visualnav_run()
void Copter::drift_run()
{
    LoiterModeState loiter_state;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speed and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // change back to AUTO mode if we are in radio failsafe, or target coords are invalid.
    if (failsafe.radio || abs(target_coord_x) > COORD_RANGE_LIMIT_X || abs(target_coord_y) > COORD_RANGE_LIMIT_Y) {
        bool ret = set_mode(AUTO, MODE_REASON_UNKNOWN);
        if(!ret) {
            set_mode(LOITER, MODE_REASON_UNKNOWN);
        }
    }

    const float curr_alt = inertial_nav.get_altitude();
    // update _pos_target.x/y by simulating pilot roll and pitch input(set _pilot_accel_fwd/rgt_cmss)
    // based on position error of the target in the image and current altitude of the copter.
    // TODO: I guess setting accel will have pretty lagging reaction with 10Hz data?
    //       If so, we will have to change to set _pos_target or _vel_desired directly.
    //       Then we should call _pos_control.update_xy_controller() rather than wp_nav.update_loiter().
    // >>>>>>>>>>>>>>>>>>>>
    // TODO: this needs to be adjusted based on practical experiments!
    float x_accel, y_accel;
    if(curr_alt > 500) {    // max accel = _loiter_accel_cmss = 250 cm/s/s
        x_accel = (float)target_coord_x / COORD_RANGE_LIMIT_X * 4500;
        y_accel = (float)target_coord_y / COORD_RANGE_LIMIT_Y * 4500;
    } else if(curr_alt > 200) {
        x_accel = (float)target_coord_x / COORD_RANGE_LIMIT_X * 4500 / 2;
        y_accel = (float)target_coord_y / COORD_RANGE_LIMIT_Y * 4500 / 2;
    } else {
        x_accel = (float)target_coord_x / COORD_RANGE_LIMIT_X * 4500 / 4;
        y_accel = (float)target_coord_y / COORD_RANGE_LIMIT_Y * 4500 / 4;
    }

    wp_nav.set_pilot_desired_acceleration(x_accel, y_accel);
    // <<<<<<<<<<<<<<<<<<<<

    // update _pos_target.z:
    // 1. if target is near the center of the image, descend at a speed of 50 cm/s;
    // 2. if target is far from the center but still in the image, keep current altitude;
    // 3. if lose target, ascend until target appears in the image again;
    // 4. if reach an altitude of 15 m and still cannot find target, mission FAILED,
    //    return to AUTO mode. TODO: maybe reload last waypoint?
    // >>>>>>>>>>>>>>>>>>>>
    // TODO: parameters need to be optimized based on practical experiments.
    int16_t coord_near_center = 50;     // range: [-120, 120]
    float   descend_velocity  = -50;    // cm/s
    float   ascend_velocity   = 100;    // cm/s
    float   altitude_limit    = 1500;   // cm
    // <<<<<<<<<<<<<<<<<<<<
    if(target_in_image) {
        // case 1
        if(abs(target_coord_x) <= coord_near_center && abs(target_coord_y) <= coord_near_center) {
            target_climb_rate = descend_velocity;
        } else{ // case 2
            target_climb_rate = 0;
        }
    } else{ // case 3
        if(curr_alt < altitude_limit) {
            target_climb_rate = ascend_velocity;
        }
        else{   // case 4
            bool ret = set_mode(AUTO, MODE_REASON_UNKNOWN);
            if(!ret) {
                set_mode(LOITER, MODE_REASON_UNKNOWN);
            }
        }
    }

    // constrain target_climb_rate
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

    // TODO: decision making: land or lifebuoy delivery?
    float decision_making_alt = 200.0f;
    // if(target_in_image == LIFEBUOY_DELIVERY && curr_alt < decision_making_alt)
    // 
    // DELIVER THE LIFEBUOY!
    //

    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
        wp_nav.loiter_soften_for_landing();
    }

    // Loiter State Machine Determination
    if (!motors.armed() || !motors.get_interlock()) {
        loiter_state = Loiter_MotorStopped;
    } else if (!ap.auto_armed) {
        loiter_state = Loiter_NotAutoArmed;
    //} else if (takeoff_state.running || (ap.land_complete && (channel_throttle->get_control_in() > get_takeoff_trigger_throttle()))){
        //loiter_state = Loiter_Takeoff;
    } else if (ap.land_complete){
        loiter_state = Loiter_Landed;
    } else {
        loiter_state = Loiter_Flying;
    }

    // Loiter State Machine
    switch (loiter_state) {

    case Loiter_MotorStopped:

        motors.set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        wp_nav.init_loiter_target();
        // multicopters do not stabilize roll/pitch/yaw when motors are stopped
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->get_control_in())-throttle_average);
        break;

    case Loiter_NotAutoArmed:

        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        wp_nav.init_loiter_target();
        // Multicopters do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->get_control_in())-throttle_average);
        break;

        // We will never take off in this flight mode.
    case Loiter_Takeoff:

        break;

    case Loiter_Landed:

        wp_nav.init_loiter_target();
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(0, 0, 0, get_smoothing_gain());
        // move throttle to between minimum and non-takeoff-throttle to keep us on the ground
        attitude_control.set_throttle_out(get_throttle_pre_takeoff(channel_throttle->get_control_in()),false,g.throttle_filt);
        // if throttle zero reset attitude and exit immediately
        if (ap.throttle_zero) {
            motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->get_control_in())-throttle_average);
        break;

    case Loiter_Flying:

        // set motors to full range
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // run loiter controller
        wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // update altitude target and call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control.update_z_controller();
        break;
    }
}
