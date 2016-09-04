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
bool _term_complete()
{
    //hal.uartD->printf("_term_number=%d,_term_offset=%d\n",_term_number,_term_offset);
    int16_t temp;

    if (_term_number == 1) {            // first parameter: target_in_image, 1 bit[0/1/2]
        temp = atoi(_term);
        if (temp >= 0 && temp <= 2)
            _array[0] = temp;
        return false;
    }
    if (_term_number == 2) {            // second parameter: target_coord_x, 1~4 bit[-160,160]
        temp = atoi(_term);             // atoi can handle negative sign!!!
        if (temp >= -160 && temp <= 160)
            _array[1] = temp;
        return false;
    }
    if (_term_number == 3) {            // third parameter: target_coord_y, 1~4 bit[-120,120]
        temp = atoi(_term);
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
            _term[_term_offset] = 0;    // add '\0'
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
    AP_HAL::UARTDriver *port = hal.uartD;   // telem 2

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
    // read from telem 2
    if (visualnav_enabled) {
        if (_read()) {
            target_in_image = _array[0];
            target_coord_x  = _array[1];
            target_coord_y  = _array[2];
        }
    }
//#define DEBUG_VISUALNAV
#ifdef DEBUG_VISUALNAV
    // debug
    visualnav_enabled = 1;
    hal.uartD->printf("target_in_image:%d, target_coord_x:%d, target_coord_y:%d\n",target_in_image, target_coord_x, target_coord_y);
#endif
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
