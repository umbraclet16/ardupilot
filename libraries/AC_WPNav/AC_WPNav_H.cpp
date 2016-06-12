/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>
#include "AC_WPNav.h"

extern const AP_HAL::HAL& hal;

/// advance_wp_target_along_track - move target location along track from origin to destination
bool AC_WPNav::advance_wp_target_along_track_H(float dt)
{
    float track_covered;        // distance (in cm) along the track that the vehicle has traveled.  Measured by drawing a perpendicular line from the track to the vehicle.
    Vector3f track_error;       // distance error (in cm) from the track_covered position (i.e. closest point on the line to the vehicle) and the vehicle
    float track_desired_max;    // the farthest distance (in cm) along the track that the leash will allow
    float track_leash_slack;    // additional distance (in cm) along the track from our track_covered position that our leash will allow
    bool reached_leash_limit = false;   // true when track has reached leash limit and we need to slow down the target point

    // get current location
    Vector3f curr_pos = _inav.get_position();

    // calculate terrain adjustments
    float terr_offset = 0.0f;
    if (_terrain_alt && !get_terrain_offset(terr_offset)) {
        return false;
    }

    // calculate 3d vector from segment's origin
    Vector3f curr_delta = (curr_pos - Vector3f(0,0,terr_offset)) - _origin;

    // calculate how far along the track we are
    track_covered = curr_delta.x * _pos_delta_unit.x + curr_delta.y * _pos_delta_unit.y + curr_delta.z * _pos_delta_unit.z;

    Vector3f track_covered_pos = _pos_delta_unit * track_covered;
    track_error = curr_delta - track_covered_pos;

    // calculate the horizontal error
    float track_error_xy = norm(track_error.x, track_error.y);

    // calculate the vertical error
    float track_error_z = fabsf(track_error.z);

    // get position control leash lengths
    float leash_xy = _pos_control.get_leash_xy();
    float leash_z;
    if (track_error.z >= 0) {
        leash_z = _pos_control.get_leash_up_z();
    }else{
        leash_z = _pos_control.get_leash_down_z();
    }

    // calculate how far along the track we could move the intermediate target before reaching the end of the leash
    track_leash_slack = MIN(_track_leash_length*(leash_z-track_error_z)/leash_z, _track_leash_length*(leash_xy-track_error_xy)/leash_xy);
    if (track_leash_slack < 0) {
        track_desired_max = track_covered;
    }else{
        track_desired_max = track_covered + track_leash_slack;
    }

    // check if target is already beyond the leash
    if (_track_desired > track_desired_max) {
        reached_leash_limit = true;
    }

    // get current velocity
    const Vector3f &curr_vel = _inav.get_velocity();
    // get speed along track
    float speed_along_track = curr_vel.x * _pos_delta_unit.x + curr_vel.y * _pos_delta_unit.y + curr_vel.z * _pos_delta_unit.z;

    // calculate point at which velocity switches from linear to sqrt
    float linear_velocity = _wp_speed_cms;
    float kP = _pos_control.get_pos_xy_kP();
    if (kP >= 0.0f) {   // avoid divide by zero
        linear_velocity = _track_accel/kP;
    }

    // let the limited_speed_xy_cms be some range above or below current velocity along track
    if (speed_along_track < -linear_velocity) {
        // we are traveling fast in the opposite direction of travel to the waypoint so do not move the intermediate point
        _limited_speed_xy_cms = 0;
    }else{
        // increase intermediate target point's velocity if not yet at the leash limit
        if(dt > 0 && !reached_leash_limit) {
            _limited_speed_xy_cms += 2.0f * _track_accel * dt;
        }
        // do not allow speed to be below zero or over top speed
        _limited_speed_xy_cms = constrain_float(_limited_speed_xy_cms, 0.0f, _track_speed);

        // check if we should begin slowing down
        if (!_flags.fast_waypoint) {
            float dist_to_dest = _track_length - _track_desired;
            if (!_flags.slowing_down && dist_to_dest <= _slow_down_dist) {
                _flags.slowing_down = true;
            }
            // if target is slowing down, limit the speed
            if (_flags.slowing_down) {
                _limited_speed_xy_cms = MIN(_limited_speed_xy_cms, get_slow_down_speed(dist_to_dest, _track_accel));
            }
        }

        // if our current velocity is within the linear velocity range limit the intermediate point's velocity to be no more than the linear_velocity above or below our current velocity
        if (fabsf(speed_along_track) < linear_velocity) {
            _limited_speed_xy_cms = constrain_float(_limited_speed_xy_cms,speed_along_track-linear_velocity,speed_along_track+linear_velocity);
        }
    }
    // advance the current target
    if (!reached_leash_limit) {
    	_track_desired += _limited_speed_xy_cms * dt;

    	// reduce speed if we reach end of leash
        if (_track_desired > track_desired_max) {
        	_track_desired = track_desired_max;
        	_limited_speed_xy_cms -= 2.0f * _track_accel * dt;
        	if (_limited_speed_xy_cms < 0.0f) {
        	    _limited_speed_xy_cms = 0.0f;
        	}
    	}
    }

    // do not let desired point go past the end of the track unless it's a fast waypoint
    if (!_flags.fast_waypoint) {
        _track_desired = constrain_float(_track_desired, 0, _track_length);
    } else {
        _track_desired = constrain_float(_track_desired, 0, _track_length + WPNAV_WP_FAST_OVERSHOOT_MAX);
    }

    // recalculate the desired position
    Vector3f final_target = _origin + _pos_delta_unit * _track_desired;
    // convert final_target.z to altitude above the ekf origin
    final_target.z += terr_offset;
    _pos_control.set_pos_target(final_target);

    // check if we've reached the waypoint
    if( !_flags.reached_destination ) {
        if( _track_desired >= _track_length ) {
            // "fast" waypoints are complete once the intermediate point reaches the destination
            if (_flags.fast_waypoint) {
                _flags.reached_destination = true;
            }else{
                // regular waypoints also require the copter to be within the waypoint radius
                Vector3f dist_to_dest = (curr_pos - Vector3f(0,0,terr_offset)) - _destination;
                if( dist_to_dest.length() <= _wp_radius_cm ) {
                    _flags.reached_destination = true;
                }
            }
        }
    }

    // successfully advanced along track
    return true;
}

