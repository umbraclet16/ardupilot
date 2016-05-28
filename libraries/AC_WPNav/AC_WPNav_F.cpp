/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>
#include "AC_WPNav.h"

extern const AP_HAL::HAL& hal;


void AC_WPNav::set_track_desired_change_limit(float proportion)
{
        _track_desired_change_limit = proportion;
}

