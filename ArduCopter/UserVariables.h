/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

bool visualnav_enabled = false;

// Set this flag so relay will not be triggered more than 1 time,
// and quad will rise up to route flight alt(8m) before change back
// to AUTO mode.
bool delivery_over_and_rise = false;

// flag = 0: no target in the image;
// flag = 1: find lifebuoy delivery target in the image;
// flag = 2: find landing platform in the image.
#define NO_TARGET_IN_IMAGE  0
#define LIFEBUOY_DELIVERY   1
#define LANDING_PLATFORM    2
uint8_t target_in_image = NO_TARGET_IN_IMAGE;

int16_t target_coord_x;
int16_t target_coord_y;

// record the time(in 0.1s) that we fail to update coords from serial port.
uint8_t serial_no_input_cnt = 0;

// tell NanoPi which program to run:
// nanopi_target = 1, look for delivery target;
// nanopi_target = 2, look for landing target;
// nanopi_target = 0, NanoPi will do nothing.
uint8_t nanopi_target = 0;

#if WII_CAMERA == 1
WiiCamera           ircam;
int                 WiiRange=0;
int                 WiiRotation=0;
int                 WiiDisplacementX=0;
int                 WiiDisplacementY=0;
#endif  // WII_CAMERA

#endif  // USERHOOK_VARIABLES


