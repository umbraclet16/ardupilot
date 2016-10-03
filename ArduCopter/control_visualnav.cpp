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
 * I modified DO_PARACHUTE() in commands_logic.cpp to set flag visualnav_enabled.
 * If more do_command is needed, DO_GRIPPER() can be modified.
 *
 * AUTO mode desired flight status: altitude: 8~10m. velocity: 3m/s? gimbal is needed!
 * 
 * We may not want to modify MissionPlanner, so I just replace Drift mode with this new mode
 * by removing control_drift.cpp and defining drift_init() and drift_run() in this file,
 * rather than add a new enumeration. This saves a lot efforts.
 *
 * I expect to receive a flag 'target_in_image' indicating target is in the image.
 *
 * the baudrate and format of UART input:
 * baudrate: 57600
 * Format: target_in_image(1 bit), target_coord_x(int), target_coord_y(int)
 *
 * I think it's good to read UART input regularly in a userhook(100/50/10Hz?),
 * what's the frequency of UART input???    Around 5Hz.
 *
 * what's the delay of target recognition???    0.X s.
 *
 * TODO: how does the controller recognize LANDING?
 * We must lower RC throttle to lower limit, then hope update_land_detector() works fine.
 * But I worry that the moving platform cannot meet the acceleration requirement
 * (<=1m/s/s, see land_detector.cpp line64).
 * If so, we may need to call Copter::init_disarm_motors() in motors.cpp manually.
 * Or, the easiest way is just lower the throttle and change to STABILIZE mode!
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
    VisualNavModeState visualnav_state;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speed and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // change back to AUTO mode if we are in radio failsafe.
    if (failsafe.radio) {
        bool ret = set_mode(AUTO, MODE_REASON_RADIO_FAILSAFE);
        if(!ret) {
            set_mode(LOITER, MODE_REASON_RADIO_FAILSAFE);
        }
        AP_Notify::events.user_mode_change = 1;
    }

    // check the validity of input target coord. If invalid or target not found, then set to 0.
    if (abs(target_coord_x) > COORD_RANGE_LIMIT_X || !target_in_image) target_coord_x = 0;
    if (abs(target_coord_y) > COORD_RANGE_LIMIT_Y || !target_in_image) target_coord_y = 0;

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
        x_accel = (float)target_coord_x / COORD_RANGE_LIMIT_X * 4500 / 4;
        y_accel = (float)target_coord_y / COORD_RANGE_LIMIT_Y * 4500 / 4;
    } else if(curr_alt > 200) {
        x_accel = (float)target_coord_x / COORD_RANGE_LIMIT_X * 4500 / 4;
        y_accel = (float)target_coord_y / COORD_RANGE_LIMIT_Y * 4500 / 4;
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
            bool ret = set_mode(AUTO, MODE_REASON_UNKNOWN); // Mission failed, so we record reason as UNKNOWN(=0).
            if(!ret) {
                set_mode(LOITER, MODE_REASON_UNKNOWN);
            }
            AP_Notify::events.user_mode_change = 1;
        }
    }

    // constrain target_climb_rate
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

    // TODO: decision making: land or lifebuoy delivery?
    float decision_making_alt = 200.0f;
    if(target_in_image == LIFEBUOY_DELIVERY && curr_alt < decision_making_alt && !delivery_over_and_rise) {
        // 
        // DELIVER THE LIFEBUOY!
        // (TODO: maybe loiter for a few seconds before do the job?)
        //

        // Unset visualnav flag when lifebuoy delivery is finished.
        // If the flag is still set, the copter will change to visualnav mode again.
        visualnav_enabled = false;
        // Set this flag so relay will not be triggered again after the delivery.
        delivery_over_and_rise = true;
    }

    // When lifebuoy delivery is over, rise up to routine flight altitude(8m),
    // then set back to AUTO mode to continue other missions.
    float routine_flight_alt = 800.0f;
    if(delivery_over_and_rise) {
        // No longer need visual info to nav during ascending procedure, so clear it.
        wp_nav.set_pilot_desired_acceleration(0, 0);
        target_climb_rate = ascend_velocity;
        // Set back to AUTO mode.
        if(curr_alt > routine_flight_alt - 100) {
            delivery_over_and_rise = false;
            gcs_send_text(MAV_SEVERITY_CRITICAL,"Setting back to AUTO...");
            bool ret = set_mode(AUTO, MODE_REASON_TX_COMMAND); // Mission finished, so we record reason as TX command(=1).
            if(!ret) {
                set_mode(LOITER, MODE_REASON_TX_COMMAND);
            }
            AP_Notify::events.user_mode_change = 1;
        }
    }

    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
        wp_nav.loiter_soften_for_landing();
    }

    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);

    // Loiter State Machine Determination
    if (!motors.armed() || !motors.get_interlock()) {
        visualnav_state = VisualNav_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        visualnav_state = VisualNav_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        visualnav_state = VisualNav_Landed;
    } else {
        visualnav_state = VisualNav_Flying;
    }

    // Loiter State Machine
    switch (visualnav_state) {

    case VisualNav_MotorStopped:

        motors.set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        wp_nav.init_loiter_target();
        attitude_control.reset_rate_controller_I_terms();
        attitude_control.set_yaw_target_to_current_heading();
        pos_control.relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate, get_smoothing_gain());
        pos_control.update_z_controller();
        break;

    case VisualNav_Takeoff:
        // set motors to full range
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }

        // get takeoff adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // run loiter controller
        wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate, get_smoothing_gain());

        // update altitude target and call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control.add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control.update_z_controller();
        break;

    case VisualNav_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }
        wp_nav.init_loiter_target();
        attitude_control.reset_rate_controller_I_terms();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        pos_control.relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control.update_z_controller();
        break;

    case VisualNav_Flying:

        // set motors to full range
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // run loiter controller
        wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate, get_smoothing_gain());

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
