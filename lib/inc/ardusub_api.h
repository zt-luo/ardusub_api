#include <stdint.h>

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------


// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

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
} control_mode_t;

typedef struct Debug_Info_Bite_s
{
    uint64_t b000_b063;
    uint64_t b064_b127;
    uint64_t b128_b191;
    uint64_t b192_b255;
    uint64_t b256_b319;
    uint64_t b320_b383;
} Debug_Info_Bite_t;

#ifdef __cplusplus
extern "C"
{
#endif

    extern void as_api_init(char* subnet_address);
    extern void as_api_deinit();
    extern void as_api_run();

    extern void vehicle_arm();
    extern void vehicle_disarm();

    extern int as_api_check_active_sys(uint8_t sysid);
    extern void as_api_manual_control(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, ...);

    extern void do_set_servo(float servo_no, float pwm);

#ifdef __cplusplus
}
#endif