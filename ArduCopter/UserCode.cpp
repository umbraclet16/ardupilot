/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
//>>>>>>>>>>>>>>>>>>>>
// I don't want to define more variables in Copter class, so make them static.
static char    _term[5];            ///< buffer for the current term within the current sentence
static uint8_t _term_number = 0;    ///< term index within the current sentence
static uint8_t _term_offset = 0;    ///< character offset with the term being received
static int16_t _array[3];
//<<<<<<<<<<<<<<<<<<<<
/*
 * Merge two bytes to one integer
 *
 * Parameters:
 * uint8_t a: high-byte
 * uint8_t b: low-byte
 *
 * Returnvalue:
 * int Merged bytes
 */
int mergeBytes(uint8_t a, uint8_t b)    //# we must use uint8_t as parameters. int8_t will cause error!!!
{
	int c = a;                          //# expand high-byte to 16 bits before shifting!!!
	return (c << 8) | b;
}

bool _term_complete()
{
    //hal.uartE->printf("_term_number=%d,_term_offset=%d\n",_term_number,_term_offset);
    int16_t temp;
    uint8_t low_byte;
    uint8_t high_byte;

    if (_term_number == 1) {            // first parameter: target_in_image, 1 bit[0/1/2]
        temp = atoi(_term);
        if (temp >= 0 && temp <= 2)
            _array[0] = temp;
        return false;
    }
    if (_term_number == 2) {            // second parameter: target_coord_x, 1~4 bit[-160,160]
        //temp = atoi(_term);             // atoi can handle negative sign!!!
        low_byte = _term[0];
        high_byte = _term[1];
        temp = mergeBytes(high_byte,low_byte);
        if (temp >= -160 && temp <= 160)
            _array[1] = temp;
        return false;
    }
    if (_term_number == 3) {            // third parameter: target_coord_y, 1~4 bit[-120,120]
        //temp = atoi(_term);
        low_byte = _term[0];
        high_byte = _term[1];
        temp = mergeBytes(high_byte,low_byte);
        if (temp >= -120 && temp <= 120)
            _array[2] = temp;
        return true;
    }
    return false;
}

bool _decode(char c)
{
    bool valid_sentence = false;

    switch (c) {
    case ',': // term terminators
        /* no break */
    //case '\r':
    //case '\n':
    case '*':
        ++_term_number;
        if (_term_offset < sizeof(_term)) {
            //_term[_term_offset] = 0;    // add '\0'
            valid_sentence = _term_complete();
        }
        _term_offset = 0;
        return valid_sentence;

    case '$': // sentence begin
        _term_number = _term_offset = 0;
        return valid_sentence;
    }

    // ordinary characters      // we only expect numbers and +/- signs, other characters(e.g. space) will cause problem!
    if (_term_offset < sizeof(_term) - 1)
        _term[_term_offset++] = c;

    return valid_sentence;
}

bool _read(void)
{
    AP_HAL::UARTDriver *port = hal.uartE;   // serial 4

    int16_t numc;
    bool parsed = false;

    numc = port->available();
    while (numc--) {
        char c = port->read();
        if (_decode(c)) {
            parsed = true;
        }
    }
    return parsed;
}

void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
    // read from serial 4
    if (_read()) {
        target_in_image = _array[0];
        target_coord_x  = _array[1];
        target_coord_y  = _array[2];
    }
#define DEBUG_VISUALNAV
#ifdef DEBUG_VISUALNAV
    // debug
    hal.uartE->printf("target_in_image:%d, target_coord_x:%d, target_coord_y:%d\n",target_in_image, target_coord_x, target_coord_y);
#endif
}
#endif

#ifdef USERHOOK_SLOWLOOP
#include <string.h>
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
    // If in visualnav mode, and delivery/landing is not finished,
    // print target coord and alt info.
//#define DEBUG_PRINT_ON_HUD
#ifndef DEBUG_PRINT_ON_HUD
    if (control_mode == VISUALNAV && !delivery_over_and_rise && !ap.land_complete) {
#endif /*DEBUG_PRINT_ON_HUD*/
        const char *str_target = (target_in_image == 1 ? "delivery:" : (target_in_image == 2 ? "landing:" : "lost target!"));
        float alt = inertial_nav.get_altitude() / 100; // cm -> m
        char str[30];
        sprintf(str,"x=%d,y=%d,alt=%.1fm",target_coord_x,target_coord_y,alt);
        char str_send[40];
        strcpy(str_send,str_target);
        strcat(str_send,str);

        gcs_send_text(MAV_SEVERITY_CRITICAL,str_send);
#ifndef DEBUG_PRINT_ON_HUD
    }
#endif /*DEBUG_PRINT_ON_HUD*/
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    if (control_mode == VISUALNAV) {
        if (delivery_over_and_rise) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"Delivery over, rising...");
        }
    }
}
#endif
