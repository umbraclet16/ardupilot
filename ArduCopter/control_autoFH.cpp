/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_autoFH.cpp
 *                          by Fang You
 *                            June, 2016
 *
 *     a combination of AUTOF and AUTOH.
 *     use channel2(pitch) to control the movement of the vehicle along the planned path.
 *     use channel3(throttle) to control the vertical speed of the vehicle.
 */

/*
 * control_auto.pde - init and run calls for auto flight mode
 *
 * This file contains the implementation for Land, Waypoint navigation and Takeoff from Auto mode
 * Command execution code (i.e. command_logic.pde) should:
 *      a) switch to Auto flight mode with set_mode() function.  This will cause auto_init to be called
 *      b) call one of the three auto initialisation functions: auto_wp_start(), auto_takeoff_start(), auto_land_start()
 *      c) call one of the verify functions auto_wp_verify(), auto_takeoff_verify, auto_land_verify repeated to check if the command has completed
 * The main loop (i.e. fast loop) will call update_flight_modes() which will in turn call auto_run() which, based upon the auto_mode variable will call
 *      correct auto_wp_run, auto_takeoff_run or auto_land_run to actually implement the feature
 */

/*
 *  While in the auto flight mode, navigation or do/now commands can be run.
 *  Code in this file implements the navigation commands
 */


// autoF_run - runs the autoF controller
//      should be called at 100hz or more
//      relies on run_autopilot being called at 10hz which handles decision making and non-navigation related commands
void Copter::autoFH_run()
{
    // call the correct auto controller
    switch (auto_mode) {

    case Auto_TakeOff:
        auto_takeoff_run();
        break;

    case Auto_WP:
    case Auto_CircleMoveToEdge:
        autoFH_wp_run();                 //# This func is modified from auto_wp_run().
        break;

    case Auto_Land:
        auto_land_run();
        break;

    case Auto_RTL:
        auto_rtl_run();
        break;

    case Auto_Circle:
        auto_circle_run();
        break;

    case Auto_Spline:
        auto_spline_run();
        break;

    case Auto_NavGuided:
#if NAV_GUIDED == ENABLED
        auto_nav_guided_run();
#endif
        break;

    case Auto_Loiter:
        auto_loiter_run();
        break;
    }
}


// autoF_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
void Copter::autoFH_wp_run()
{
    //#>>>>>>>>>>>>>>>>>>>>
    //# This is not a proper position for this.
    //# TODO: Consider move it to somewhere like autoFH_init() or set_mode()[flight_mode.cpp].
    if (wp_nav.flag_AUTOFH() != 3) {
            wp_nav.set_flag_AUTOFH();
    }
    //#<<<<<<<<<<<<<<<<<<<<
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors.armed() || !ap.auto_armed || !motors.get_interlock()) {
        // To-Do: reset waypoint origin to current location because copter is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        // clear i term when we're taking off
        set_throttle_takeoff();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    //#>>>>>>>>>>>>>>>>>>>>
    float target_climb_rate = 0.0f;
    //#<<<<<<<<<<<<<<<<<<<<
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }

        //#>>>>>>>>>>>>>>>>>>>>
        //# get pilot's pitch input.
        //# !!!!! NOTICE that pitch input is REVERSE so we need to multiply it by -1 !!!!!
        //# Will be used to limit the update of _pos_target so as to indirectly control the movement along the track
        float pitch_input = channel_pitch->get_control_in() / 4500.0f * (-1);
        //# constrain the input. -1: lower limit; 0: neutral position; 1: upper limit
        pitch_input = constrain_float(pitch_input, -1.0, 1.0);
        //# send input to WP controller
        wp_nav.set_track_desired_change_limit(pitch_input);

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        //#<<<<<<<<<<<<<<<<<<<<
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    failsafe_terrain_set_status(wp_nav.update_wpnav());

    //#>>>>>>>>>>>>>>>>>>>>
    // adjust climb rate using rangefinder
    if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
    }

    // update altitude target and call z-axis position controller
    pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    //#<<<<<<<<<<<<<<<<<<<<
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.input_euler_angle_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(),true);
    }
}

