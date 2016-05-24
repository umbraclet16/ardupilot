/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

// read_inertia - read inertia in from accelerometers
void Copter::read_inertia()	//# update _relpos_cm, _abspos, _velocity_cm, _pos_z_rate.
{
    // inertial altitude estimates
    inertial_nav.update(G_Dt);

    // pull position from interial nav library
    current_loc.lng = inertial_nav.get_longitude();
    current_loc.lat = inertial_nav.get_latitude();

		//# current_loc.alt = _relpos_cm.z+origin和home之差(alt above home), climb_rate = _velocity_cm.z.
        //# ??? EKF origin和home的差别??
    // exit immediately if we do not have an altitude estimate
    if (!inertial_nav.get_filter_status().flags.vert_pos) {
        return;
    }

    // without home return alt above the EKF origin
    if (ap.home_state == HOME_UNSET) {
        // with inertial nav we can update the altitude and climb rate at 50hz
        current_loc.alt = inertial_nav.get_altitude();
    } else {
        // with inertial nav we can update the altitude and climb rate at 50hz
        current_loc.alt = pv_alt_above_home(inertial_nav.get_altitude());
    }

    // set flags and get velocity
    current_loc.flags.relative_alt = true;
    climb_rate = inertial_nav.get_velocity_z();		//# 用于Log和GCS_Mavlink
}
