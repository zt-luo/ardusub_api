#pragma once

#include <stdint.h>

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------
#ifndef MAVLINK_H

typedef struct __mavlink_statustext_t
{
    uint8_t severity; /*<  Severity of status. Relies on the definitions within RFC-5424.*/
    char text[50];    /*<  Status text message, without null termination character*/
} mavlink_statustext_t;

#endif // MAVLINK_H

typedef struct Vehicle_Data_s
{
    // Heartbeat
    uint32_t custom_mode;    /*<  A bitfield for use for autopilot-specific flags*/
    uint8_t type;            /*<  Type of the system (quadrotor, helicopter, etc.). Components use the same type as their associated system.*/
    uint8_t autopilot;       /*<  Autopilot type / class.*/
    uint8_t base_mode;       /*<  System mode bitmap.*/
    uint8_t system_status;   /*<  System status flag.*/
    uint8_t mavlink_version; /*<  MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version*/

    // System Status
    uint32_t onboard_control_sensors_present; /*<  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.*/
    uint32_t onboard_control_sensors_enabled; /*<  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.*/
    uint32_t onboard_control_sensors_health;  /*<  Bitmap showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled.*/
    uint16_t load;                            /*< [d%] Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000*/
    uint16_t voltage_battery;                 /*< [mV] Battery voltage*/
    int16_t current_battery;                  /*< [cA] Battery current, -1: autopilot does not measure the current*/
    uint16_t drop_rate_comm;                  /*< [c%] Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)*/
    uint16_t errors_comm;                     /*<  Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)*/
    uint16_t errors_count1;                   /*<  Autopilot-specific errors*/
    uint16_t errors_count2;                   /*<  Autopilot-specific errors*/
    uint16_t errors_count3;                   /*<  Autopilot-specific errors*/
    uint16_t errors_count4;                   /*<  Autopilot-specific errors*/
    int8_t battery_remaining;                 /*< [%] Remaining battery energy, -1: autopilot estimate the remaining battery*/

    // Battery Status
    int32_t current_consumed;    /*< [mAh] Consumed charge, -1: autopilot does not provide consumption estimate*/
    int32_t energy_consumed;     /*< [hJ] Consumed energy, -1: autopilot does not provide energy consumption estimate*/
    int16_t temperature_bs;      /*< [cdegC] Temperature of the battery. INT16_MAX for unknown temperature.*/
    uint16_t voltages[10];       /*< [mV] Battery voltage of cells. Cells above the valid cell count for this battery should have the UINT16_MAX value.*/
    int16_t current_battery_bs;  /*< [cA] Battery current, -1: autopilot does not measure the current*/
    uint8_t id;                  /*<  Battery ID*/
    uint8_t battery_function;    /*<  Function of the battery*/
    uint8_t type_bs;             /*<  Type (chemistry) of the battery*/
    int8_t battery_remaining_bs; /*< [%] Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery.*/
    int32_t time_remaining;      /*< [s] Remaining battery time, 0: autopilot does not provide remaining battery time estimate*/
    uint8_t charge_state;        /*<  State for extent of discharge, provided by autopilot for warning or external reactions*/

    // Power Status
    uint16_t Vcc;    /*< [mV] 5V rail voltage.*/
    uint16_t Vservo; /*< [mV] Servo rail voltage.*/
    uint16_t flags;  /*<  Bitmap of power supply status flags.*/

    // System Time
    uint64_t time_unix_usec; /*< [us] Timestamp (UNIX epoch time).*/
    uint32_t time_boot_ms;   /*< [ms] Timestamp (time since system boot).*/

    // Attitude
    uint32_t time_boot_ms_at; /*< [ms] Timestamp (time since system boot).*/
    float roll;               /*< [rad] Roll angle (-pi..+pi)*/
    float pitch;              /*< [rad] Pitch angle (-pi..+pi)*/
    float yaw;                /*< [rad] Yaw angle (-pi..+pi)*/
    float rollspeed;          /*< [rad/s] Roll angular speed*/
    float pitchspeed;         /*< [rad/s] Pitch angular speed*/
    float yawspeed;           /*< [rad/s] Yaw angular speed*/

    // Scaled Pressure
    uint32_t time_boot_ms_sp; /*< [ms] Timestamp (time since system boot).*/
    float press_abs;          /*< [hPa] Absolute pressure*/
    float press_diff;         /*< [hPa] Differential pressure*/
    int16_t temperature_sp;   /*< [cdegC] Temperature measurement*/

    // Scaled Pressure2 (depth sensor)
    uint32_t time_boot_ms_sp2; /*< [ms] Timestamp (time since system boot).*/
    float press_abs2;          /*< [hPa] Absolute pressure*/
    float press_diff2;         /*< [hPa] Differential pressure*/
    int16_t temperature2;      /*< [cdegC] Temperature measurement*/

    // Servo Output Raw
    uint32_t time_usec_sor; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.*/
    uint16_t servo1_raw;    /*< [us] Servo output 1 value*/
    uint16_t servo2_raw;    /*< [us] Servo output 2 value*/
    uint16_t servo3_raw;    /*< [us] Servo output 3 value*/
    uint16_t servo4_raw;    /*< [us] Servo output 4 value*/
    uint16_t servo5_raw;    /*< [us] Servo output 5 value*/
    uint16_t servo6_raw;    /*< [us] Servo output 6 value*/
    uint16_t servo7_raw;    /*< [us] Servo output 7 value*/
    uint16_t servo8_raw;    /*< [us] Servo output 8 value*/
    uint8_t port;           /*<  Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.*/
    uint16_t servo9_raw;    /*< [us] Servo output 9 value*/
    uint16_t servo10_raw;   /*< [us] Servo output 10 value*/
    uint16_t servo11_raw;   /*< [us] Servo output 11 value*/
    uint16_t servo12_raw;   /*< [us] Servo output 12 value*/
    uint16_t servo13_raw;   /*< [us] Servo output 13 value*/
    uint16_t servo14_raw;   /*< [us] Servo output 14 value*/
    uint16_t servo15_raw;   /*< [us] Servo output 15 value*/
    uint16_t servo16_raw;   /*< [us] Servo output 16 value*/

    // Raw IMU
    uint64_t time_usec_ri; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.*/
    int16_t xacc;          /*<  X acceleration (raw)*/
    int16_t yacc;          /*<  Y acceleration (raw)*/
    int16_t zacc;          /*<  Z acceleration (raw)*/
    int16_t xgyro;         /*<  Angular speed around X axis (raw)*/
    int16_t ygyro;         /*<  Angular speed around Y axis (raw)*/
    int16_t zgyro;         /*<  Angular speed around Z axis (raw)*/
    int16_t xmag;          /*<  X Magnetic field (raw)*/
    int16_t ymag;          /*<  Y Magnetic field (raw)*/
    int16_t zmag;          /*<  Z Magnetic field (raw)*/

    // RC channels
    uint32_t time_boot_ms_rc; /*< [ms] Timestamp (time since system boot).*/
    uint16_t chan1_raw;    /*< [us] RC channel 1 value.*/
    uint16_t chan2_raw;    /*< [us] RC channel 2 value.*/
    uint16_t chan3_raw;    /*< [us] RC channel 3 value.*/
    uint16_t chan4_raw;    /*< [us] RC channel 4 value.*/
    uint16_t chan5_raw;    /*< [us] RC channel 5 value.*/
    uint16_t chan6_raw;    /*< [us] RC channel 6 value.*/
    uint16_t chan7_raw;    /*< [us] RC channel 7 value.*/
    uint16_t chan8_raw;    /*< [us] RC channel 8 value.*/
    uint16_t chan9_raw;    /*< [us] RC channel 9 value.*/
    uint16_t chan10_raw;   /*< [us] RC channel 10 value.*/
    uint16_t chan11_raw;   /*< [us] RC channel 11 value.*/
    uint16_t chan12_raw;   /*< [us] RC channel 12 value.*/
    uint16_t chan13_raw;   /*< [us] RC channel 13 value.*/
    uint16_t chan14_raw;   /*< [us] RC channel 14 value.*/
    uint16_t chan15_raw;   /*< [us] RC channel 15 value.*/
    uint16_t chan16_raw;   /*< [us] RC channel 16 value.*/
    uint16_t chan17_raw;   /*< [us] RC channel 17 value.*/
    uint16_t chan18_raw;   /*< [us] RC channel 18 value.*/
    uint8_t chancount;     /*<  Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.*/
    uint8_t rssi;          /*< [%] Receive signal strength indicator. Values: [0-100], 255: invalid/unknown.*/

    // Named value
} Vehicle_Data_t;

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

// indicate the status controled from this api
typedef enum system_status_enum
{
    SYS_UN_INIT = 0,
    SYS_INITIATING = 1,
    SYS_DISARMED = 2,
    SYS_ARMED = 3,
} system_status_t;

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

    extern void as_api_init(char *subnet_address);
    extern void as_api_deinit();

    extern void as_api_vehicle_arm(uint8_t target_system, uint8_t target_autopilot);
    extern void as_api_vehicle_disarm(uint8_t target_system, uint8_t target_autopilot);

    //! NULL-able return value
    extern mavlink_statustext_t *as_api_statustex_queue_pop(uint8_t target_system);
    //!

    extern Vehicle_Data_t *as_api_get_vehicle_data(uint8_t target_system);

    extern int as_api_statustex_cpunt(uint8_t target_system);

    extern int as_api_check_vehicle(uint8_t sysid);
    extern void as_api_manual_control(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, ...);

#ifdef __cplusplus
}
#endif