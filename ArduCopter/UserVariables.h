/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

bool visualnav_enabled = false;

// flag = 0: no target in the image;
// flag = 1: find lifebuoy delivery target in the image;
// flag = 2: find landing platform in the image.
#define NO_TARGET_IN_IMAGE  0
#define LIFEBUOY_DELIVERY   1
#define LANDING_PLATFORM    2
uint8_t target_in_image = NO_TARGET_IN_IMAGE;

int16_t target_coord_x;
int16_t target_coord_y;

#if WII_CAMERA == 1
WiiCamera           ircam;
int                 WiiRange=0;
int                 WiiRotation=0;
int                 WiiDisplacementX=0;
int                 WiiDisplacementY=0;
#endif  // WII_CAMERA

#endif  // USERHOOK_VARIABLES


