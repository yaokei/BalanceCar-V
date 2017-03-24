
#pragma once

#include "stdint.h"

typedef void *orb_advert_t;

typedef uint8_t byte;

#define BATTERY_LOW     3.3f
#define BATTERY_FULL    4.2f

// navigation state
enum {

    NAVIGATION_STATE_MANUAL = 0,            // manual mode
    NAVIGATION_STATE_STAB,                  // stable mode
    NAVIGATION_STATE_ALTCTL,                // altitude control mode
    NAVIGATION_STATE_POSCTL,                // position control mode

    NAVIGATION_STATE_AUTO_MISSION,          // auto mission mode
    NAVIGATION_STATE_AUTO_LOITER,           // auto loiter mode
    NAVIGATION_STATE_AUTO_RTL,              // auto return to launch mode
    NAVIGATION_STATE_AUTO_LANDGPSFAIL,      // auto land on gps failure (e.g. open loop loiter down)
    NAVIGATION_STATE_AUTO_TAKEOFF,          // auto take off
    NAVIGATION_STATE_AUTO_LAND,             // auto land
    NAVIGATION_STATE_AUTO_FOLLOW_TARGET,    // auto follow

    NAVIGATION_STATE_ACRO,                  // acro mode

    NAVIGATION_STATE_OFFBOARD,              // off board control mode
    OFFBOARD_MODE
};

struct status_s {
    uint8_t armed;          // current arming state
    uint8_t nav_state;      // navigation state
    uint8_t landed;         // land
    uint8_t landed_maybe;   // land maybe
    uint8_t falling;        // falling
    uint8_t reset;          // reset system

    float voltage;          // battery voltage in volt
};

struct manual_s {
    uint32_t timestamp; // update time in us
    float x;            // pitch control    -1..1
    float y;            // roll control     -1..1
    float z;            // throttle control  0..1
    float r;            // yaw control      -1..1
    uint16_t aux[5];    // auxiliary channel    1000..2000
    uint8_t rssi;       // signal strength      0..100
};

struct accel_s {
    uint32_t timestamp;             // update time in us
    float temperature;              // temperature in degrees celsius
    float x, y, z;                  // body frame acceleration in m/s^s
    int16_t x_raw, y_raw, z_raw;    // raw sensor value of acceleration
};

struct gyro_s {
    uint32_t timestamp;             // update time in us
    float temperature;              // temperature in degrees celsius
    float x, y, z;                  // body frame angular rate in rad/s
    int16_t x_raw, y_raw, z_raw;    // raw sensor value of angular rate
};

struct mag_s {
    uint32_t timestamp;             // update time in us
    float temperature;              // temperature in degrees celsius
    float x, y, z;                  // body frame magetic field in Guass
    int16_t x_raw, y_raw, z_raw;    // raw sensor value of magnetic field
};

struct mag_calibration_s {
	float	x_offset;
	float	x_scale;
	float	y_offset;
	float	y_scale;
	float	z_offset;
	float	z_scale;
};

struct baro_s {
    uint32_t timestamp;             // update time in us
    float temperature;              // temperature in degrees celsius
    float pressure;                 // barometric pressure, already temp. comp.
    float altitude;                 // altitude, already temp. comp.
};

struct sonar_s {
	uint32_t timestamp;             // update time in us
	float min_distance;             // valid minimal distance
    float max_distance;             // valid maximum distance
    float current_distance;         // current distance
    float covariance;
    uint8_t orientation;
};

struct gps_s {
    uint32_t timestamp;             // update time in us
    int32_t lat;                    // latitude in 1e-7 degrees
    int32_t lon;                    // longitude in 1e-7 degrees
    int32_t alt;                    // altitude in 1e-3 meters above MSL (millimeters)
    int32_t alt_ellipsoid;          // altitude in 1e-3 meters blove Ellipsoid (millimeters)
    float s_variance_m_s;           // gps speed accuracy estimate (m/s)
    float c_variance_rad;           // gps course accuracy estimate (radians)
    uint8_t fix_type;               // 0-1: no fix   2: 2D fix   3: 3D fix   4: RTCM code diffenrential   5: RTK float   6: RTK fixed
    float eph;                      // gps horizontal position accuracy (meters)
    float epv;                      // gps vertical position accuracy (meters)
    float hdop;                     // horizontal dilution of precision
    float vdop;                     // vertical dilution of precision
    int32_t noise_per_ms;           // gps noise per millisecond
    int32_t jamming_indicator;      // indicates jamming is occurring
    float vel_m_s;                  // gps ground speed (m/s)
    float vel_n_m_s;                // gps north velocity (m/s)
    float vel_e_m_s;                // gps east velocity (m/s)
    float vel_d_m_s;                // gps down velocity (m/s)
    float cog_rad;                  // course over ground (not heading, but direction of movement), -PI..PI (radians)
    bool vel_ned_valid;             // true if NED velocity is valid
    uint64_t time_utc_usec;         // timestamp (ms, UTC), this is the timestamp which comes from th gps module, It might be unvailable right after cold start, indicated by a value of 0
    uint8_t satellites_used;        // number of satellites used
};

struct flow_s {
    uint32_t timestamp;             // update time (us)
    float pixel_flow_x_integral;    // accumulated optical flow around x axis (rad)
    float pixel_flow_y_integral;    // accumulated optical flow around y axis (rad)
    float pixel_flow_x_distance;
    float pixel_flow_y_distance;
    float gyro_x_rate_integral;     // accumulated gyro value around x axis (rad)
    float gyro_y_rate_integral;     // accumulated gyro value around y axis (rad)
    float gyro_z_rate_integral;     // accumulated gyro value around z axis (rad)
    float ground_distance_m;        // distance to ground in meters
    uint32_t integration_timespan;  // accumulation timespan (us)
    uint8_t quality;                // average of quality of accumulated frames, 0: bad quality, 255: maxium quality
};

struct attitude_s {
    uint32_t timestamp;                     // update time in us
    float roll, pitch, yaw;                 // angle in rad
    float roll_rate, pitch_rate, yaw_rate;  // angular rate in rad/s
    float rate_offsets[3];                  // offsets of the body angular rates from zero
    float R[9];                             // rotation matrix, body to world
    float q[4];                             // quaternion (NED)
    bool R_valid;                           // rotation matrix valid
    bool q_valid;                           // quaternion valid
};

struct attitude_setpoint_s {
    uint32_t timestamp;         // update time in us
    float roll_body;            // body frame angle setpoint in rad
    float pitch_body;           // body frame angle setpoint in rad
    float yaw_body;             // body frame angle setpoint in rad
    float yaw_sp_move_rate;     // yaw rate setpoint in rad/s (commanded by user)

    float R_body[9];            // rotation matrix describing the setpoint as rotation from the current body frame
    bool R_valid;               // set to true if rotation matrix is valid
    float q_d[4];               // desired quaternion for quaternion control
    bool q_d_valid;             // set to true if quaternion vector is valid
    float q_e[4];               // attitude error in quaternion
    bool q_e_valid;             // set to true if quaternion error vector is valid

    float thrust;               // thrust in Newton the power system should generate

    bool roll_reset_integral;   // reset roll integral part (navigation logic change)
    bool pitch_reset_integral;  // reset pitch integral part (navigation logic change)
    bool yaw_reset_integral;    // reset yaw integral part (navigation logic change)

    bool fw_control_yaw;            // control heading with rudder (used for auto takeoff on runway)
    bool disable_mc_yaw_control;    // control yaw for mc (used for vtol weather-vane mode)
};

struct local_position_s {
    uint32_t timestamp;         // update time (us)
    uint32_t ref_timestamp;     // ref time
    double ref_lat, ref_lon;    // reference point in latitude & longitute
    float ref_alt;              // reference altitude AMSL, must be set to current (not at reference point) ground level (m)
    float baro_offset;          // baro altitude offset
    float x, y, z;              // position in NED earth-fixed frame (m)
    float vx, vy, vz;           // velocity in NED earth-fixed frame (m/s)
    float ax, ay, az;           // acceleration in NED earth-fixed frame (m/s^2)
    float dist_bottom;
    float dist_bottom_rate;
    float yaw;                  // euler yaw angle
    float eph;
    float epv;
    bool xy_valid, z_valid;     // set to true if xy(z) position valid
    bool v_xy_valid, v_z_valid; // set to true if xy(z) velocity valid
    bool dist_bottom_valid;
};

struct local_position_setpoint_s {
    uint32_t timestamp;         // update time (us)
    float x, y, z;              // position setpoint in NED earth-fixed frame (m)
    float yaw;                  // yaw setpoint in NED (rad) -PI..+PI
    float vx, vy, vz;           // velocity setpoint in NED earth-fixed frame (m/s)
    float acc_x, acc_y, acc_z;  // acceleration setpoint  in NED earth-fixed frame (m/s^2)
};

struct global_position_s {
    uint32_t time_stamp;        // update time (us)
    uint32_t time_utc_usec;     // gps utc (us)
    double lat;                 // latitude (degrees)
    double lon;                 // longitude (degrees)
    float alt;                  // altitude AMSL (meters)
    float vel_n;                // north velocity in NED earth-fixed frame (m/s)
    float vel_e;                // east velocity in NED earth-fixed frame (m/s)
    float vel_d;                // down velocity in NED earth-fixed frame (m/s)
    float yaw;                  // euler yaw angle relative to NED earth-fixed frame, -PI..PI (radians)
    float eph;                  // standard deviation of horizontal position error (meters)
    float epv;                  // standard deviation of vertical position error (meters)
    float terrain_alt;          // terrain altitude WGS84 (meters)
    bool terrain_alt_valid;     // terrain altitude estimate is valid
    bool dead_reckoning;        // true if this position is estimated throught dead-reckoning
    float pressure_alt;         // pressure altitude AMSL (meters)
};

struct rates_setpoint_s {
    uint32_t timestamp;         // update time (us)
    float roll;                 // body frame angular rates (rad/s)
    float pitch;                // body frame angular rates (rad/s)
    float yaw;                  // body frame angular rates (rad/s)
    float thrust;               // thrust normalized to 0..1
};

struct attitude_control_status_s {
    uint32_t timestamp;         // update time (us)
    float roll_rate_integ;      // roll rate integrator
    float pitch_rate_integ;     // pitch rate integrator
    float yaw_rate_integ;       // yaw rate integrator
};

struct control_state_s {
    uint32_t timestamp;         // update time (us)
    float x_acc, y_acc, z_acc;  // acceleration in body frame
    float x_vel, y_vel, z_vel;  // velocity in body frame
    float x_pos, y_pos, z_pos;  // position in local earth frame
    float vel_variance[3];      // variance in body velocity estimate
    float pos_variance[3];      // variance in body position estimate
    float q[4];                 // attitude quaternion
    float roll_rate, pitch_rate, yaw_rate;  // low pass filtered angular rate in body frame (rad/s)
    float horz_acc_mag;         // low pass filtered magnitude of the horizontal acceleration
};

struct control_mode_s {
    uint32_t timestamp;                     // update time (us)
    bool flag_control_manual_enabled;       // true if manual input is mixed up
    bool flag_control_auto_enabled;         // true if onboard autopilot should act
    bool flag_control_offboard_enabled;     // true if offboard control should used
    bool flag_control_rates_enabled;        // true if rates are stabilized
    bool flag_control_attitude_enabled;     // true if attitude stabilization in mixed in
    bool flag_control_rattitude_enabled;    // true if rate/attitude stabilization is enabled
    bool flag_control_acceleration_enabled; // true if acceleration is controlled
    bool flag_control_velocity_enabled;     // true if horizontal velocity is controlled
    bool flag_control_position_enabled;     // true if position is controlled
    bool flag_control_climb_rate_enabled;   // true if climb rate is controlled
    bool flag_control_altitude_enabled;     // ture if altitude is controlled
};

struct motor_limits_s {
    uint32_t timestamp;         // update time (us)
    uint8_t lower_limit;        // at least one actuator command has saturated on the lower limit
    uint8_t upper_limit;        // at least one actuator command has saturated on the upper limit
    uint8_t yaw;                // yaw limit reached
    uint8_t reserved;           // reserved
};

struct actuator_s {
    uint32_t timestamp;         // update time (us)
    float control[4];           // actuator output [0]-[1]-[2]:roll-pitch-yaw -1..1    [3]:throttle 0..1
};

struct actuator_output_s {
    uint32_t timestamp;         // update time (us)
    unsigned rotor_count;       // effective rotor numbers
    float control[16];          // output data, in natual output uints
};

struct vehicle_command_s {
	uint32_t timestamp;         // required for logger
	double param5;
	double param6;
	float param1;
	float param2;
	float param3;
	float param4;
	float param7;
	uint32_t command;
	uint32_t target_system;
	uint32_t target_component;
	uint32_t source_system;
	uint32_t source_component;
	uint8_t confirmation;

#ifdef __cplusplus
	static const uint32_t VEHICLE_CMD_CUSTOM_0 = 0;
	static const uint32_t VEHICLE_CMD_CUSTOM_1 = 1;
	static const uint32_t VEHICLE_CMD_CUSTOM_2 = 2;
	static const uint32_t VEHICLE_CMD_NAV_WAYPOINT = 16;
	static const uint32_t VEHICLE_CMD_NAV_LOITER_UNLIM = 17;
	static const uint32_t VEHICLE_CMD_NAV_LOITER_TURNS = 18;
	static const uint32_t VEHICLE_CMD_NAV_LOITER_TIME = 19;
	static const uint32_t VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20;
	static const uint32_t VEHICLE_CMD_NAV_LAND = 21;
	static const uint32_t VEHICLE_CMD_NAV_TAKEOFF = 22;
	static const uint32_t VEHICLE_CMD_NAV_ROI = 80;
	static const uint32_t VEHICLE_CMD_NAV_PATHPLANNING = 81;
	static const uint32_t VEHICLE_CMD_NAV_VTOL_TAKEOFF = 84;
	static const uint32_t VEHICLE_CMD_NAV_VTOL_LAND = 85;
	static const uint32_t VEHICLE_CMD_NAV_GUIDED_LIMITS = 90;
	static const uint32_t VEHICLE_CMD_NAV_GUIDED_MASTER = 91;
	static const uint32_t VEHICLE_CMD_NAV_GUIDED_ENABLE = 92;
	static const uint32_t VEHICLE_CMD_NAV_LAST = 95;
	static const uint32_t VEHICLE_CMD_CONDITION_DELAY = 112;
	static const uint32_t VEHICLE_CMD_CONDITION_CHANGE_ALT = 113;
	static const uint32_t VEHICLE_CMD_CONDITION_DISTANCE = 114;
	static const uint32_t VEHICLE_CMD_CONDITION_YAW = 115;
	static const uint32_t VEHICLE_CMD_CONDITION_LAST = 159;
	static const uint32_t VEHICLE_CMD_DO_SET_MODE = 176;
	static const uint32_t VEHICLE_CMD_DO_JUMP = 177;
	static const uint32_t VEHICLE_CMD_DO_CHANGE_SPEED = 178;
	static const uint32_t VEHICLE_CMD_DO_SET_HOME = 179;
	static const uint32_t VEHICLE_CMD_DO_SET_PARAMETER = 180;
	static const uint32_t VEHICLE_CMD_DO_SET_RELAY = 181;
	static const uint32_t VEHICLE_CMD_DO_REPEAT_RELAY = 182;
	static const uint32_t VEHICLE_CMD_DO_SET_SERVO = 183;
	static const uint32_t VEHICLE_CMD_DO_REPEAT_SERVO = 184;
	static const uint32_t VEHICLE_CMD_DO_FLIGHTTERMINATION = 185;
	static const uint32_t VEHICLE_CMD_DO_GO_AROUND = 191;
	static const uint32_t VEHICLE_CMD_DO_REPOSITION = 192;
	static const uint32_t VEHICLE_CMD_DO_PAUSE_CONTINUE = 193;
	static const uint32_t VEHICLE_CMD_DO_CONTROL_VIDEO = 200;
	static const uint32_t VEHICLE_CMD_DO_SET_ROI = 201;
	static const uint32_t VEHICLE_CMD_DO_DIGICAM_CONTROL = 203;
	static const uint32_t VEHICLE_CMD_DO_MOUNT_CONFIGURE = 204;
	static const uint32_t VEHICLE_CMD_DO_MOUNT_CONTROL = 205;
	static const uint32_t VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST = 206;
	static const uint32_t VEHICLE_CMD_DO_FENCE_ENABLE = 207;
	static const uint32_t VEHICLE_CMD_DO_PARACHUTE = 208;
	static const uint32_t VEHICLE_CMD_DO_INVERTED_FLIGHT = 210;
	static const uint32_t VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT = 220;
	static const uint32_t VEHICLE_CMD_DO_GUIDED_MASTER = 221;
	static const uint32_t VEHICLE_CMD_DO_GUIDED_LIMITS = 222;
	static const uint32_t VEHICLE_CMD_DO_LAST = 240;
	static const uint32_t VEHICLE_CMD_PREFLIGHT_CALIBRATION = 241;
	static const uint32_t VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242;
	static const uint32_t VEHICLE_CMD_PREFLIGHT_STORAGE = 245;
	static const uint32_t VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246;
	static const uint32_t VEHICLE_CMD_OVERRIDE_GOTO = 252;
	static const uint32_t VEHICLE_CMD_MISSION_START = 300;
	static const uint32_t VEHICLE_CMD_COMPONENT_ARM_DISARM = 400;
	static const uint32_t VEHICLE_CMD_START_RX_PAIR = 500;
	static const uint32_t VEHICLE_CMD_DO_TRIGGER_CONTROL = 2003;
	static const uint32_t VEHICLE_CMD_DO_VTOL_TRANSITION = 3000;
	static const uint32_t VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY = 30001;
	static const uint32_t VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY = 30002;
	static const uint32_t VEHICLE_CMD_PREFLIGHT_UAVCAN = 243;
	static const uint32_t VEHICLE_CMD_LOGGING_START = 2510;
	static const uint32_t VEHICLE_CMD_LOGGING_STOP = 2511;
	static const uint32_t VEHICLE_CMD_RESULT_ACCEPTED = 0;
	static const uint32_t VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED = 1;
	static const uint32_t VEHICLE_CMD_RESULT_DENIED = 2;
	static const uint32_t VEHICLE_CMD_RESULT_UNSUPPORTED = 3;
	static const uint32_t VEHICLE_CMD_RESULT_FAILED = 4;
	static const uint32_t VEHICLE_CMD_RESULT_ENUM_END = 5;
	static const uint32_t VEHICLE_MOUNT_MODE_RETRACT = 0;
	static const uint32_t VEHICLE_MOUNT_MODE_NEUTRAL = 1;
	static const uint32_t VEHICLE_MOUNT_MODE_MAVLINK_TARGETING = 2;
	static const uint32_t VEHICLE_MOUNT_MODE_RC_TARGETING = 3;
	static const uint32_t VEHICLE_MOUNT_MODE_GPS_POINT = 4;
	static const uint32_t VEHICLE_MOUNT_MODE_ENUM_END = 5;
	static const uint32_t VEHICLE_ROI_NONE = 0;
	static const uint32_t VEHICLE_ROI_WPNEXT = 1;
	static const uint32_t VEHICLE_ROI_WPINDEX = 2;
	static const uint32_t VEHICLE_ROI_LOCATION = 3;
	static const uint32_t VEHICLE_ROI_TARGET = 4;
	static const uint32_t VEHICLE_ROI_ENUM_END = 5;
	static const uint32_t ORB_QUEUE_LENGTH = 3;

#endif
};

struct mavlink_log_s {
	uint32_t timestamp;     // required for logger
	uint8_t text[50];
	uint8_t severity;
};

struct vehicle_command_ack_s {
    uint32_t timestamp;
    uint16_t command;
    uint8_t result;
#ifdef __cplusplus
	static const uint8_t VEHICLE_RESULT_ACCEPTED = 0;
	static const uint8_t VEHICLE_RESULT_TEMPORARILY_REJECTED = 1;
	static const uint8_t VEHICLE_RESULT_DENIED = 2;
	static const uint8_t VEHICLE_RESULT_UNSUPPORTED = 3;
	static const uint8_t VEHICLE_RESULT_FAILED = 4;
	static const uint32_t ORB_QUEUE_LENGTH = 3;

#endif
};

extern int topics_config(void);
extern int publish(char *name, void *data);
extern int subscribe(char *name, void *data);
