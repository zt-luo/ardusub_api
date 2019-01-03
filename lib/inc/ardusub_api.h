#pragma once

#include <stdint.h>

// Auto Pilot Modes enumeration
typedef enum control_mode_enum
{
    STABILIZE = 0, // manual angle with manual depth/throttle
    ACRO = 1,      // manual body-frame angular rate with manual depth/throttle
    ALT_HOLD = 2,  // manual angle with automatic depth/throttle
    AUTO = 3,      // not implemented in sub // fully automatic waypoint control using mission commands
    GUIDED = 4,    // not implemented in sub // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    CIRCLE = 7,    // not implemented in sub // automatic circular flight with automatic throttle
    SURFACE = 9,   // automatically return to surface, pilot maintains horizontal control
    POSHOLD = 16,  // automatic position hold with manual override, with automatic throttle
    MANUAL = 19    // Pass-through input with no stabilization
}control_mode_t;

extern void ardusub_api_init();
extern void ardusub_api_deinit();
extern void ardusub_api_run();

void ardusub_api_do_set_servo(float servo_no, float pwm);
void ardusub_api_do_motor_test(float motor_no, float pwm);
void ardusub_api_do_set_mode(control_mode_t mode_);
void ardusub_api_request_param_list(void);
void ardusub_api_send_rc_channels_override(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4, 
        uint16_t ch5, uint16_t ch6, uint16_t ch7, uint16_t ch8);
