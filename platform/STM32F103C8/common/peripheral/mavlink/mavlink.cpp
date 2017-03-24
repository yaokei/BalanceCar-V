#include "library.h"
#include "mavlink_bridge_header.h"
#include "mavlink.h"

#include "platform.h"
#include "topics.h"
#include "param.h"
#include "math.hpp"

#ifndef MAVLINK_PATH
    #define MAVLINK_PATH    dev_uart1
#endif

class Mavlink;

namespace namespace_mavlink
{
    Mavlink *instance;
}

static mavlink_system_t mavlink_system;

class Mavlink
{
public:
    Mavlink();

    int start();
    static void task_main_trampoline(void *arg);
    void task_main();

protected:
    inline int FLOAT_EQ_FLOAT(float f1, float f2) { return (f1 == f2); }

    void init();
    void heartbeat_send();
    void sys_status_send();
    void parameter_send();
    void stream_send();
    void handle_message(mavlink_channel_t chan, mavlink_message_t* msg);
    void usart_receive();

    union custom_mode_u {
        struct {
            uint16_t reserved;
            uint8_t main_mode;
            uint8_t sub_mode;
        };
        uint32_t data;
        float data_float;
    };

    bool _inited;

    uint32_t _param_count;
    uint32_t _param_index;

    uint32_t _t;
    status_s _status;
};

Mavlink::Mavlink() :
    _inited(false),
    _t(0)
{
    _param_count = param_count();
    _param_index = _param_count;
    memset(&mavlink_system, 0, sizeof(mavlink_system));
    memset(&_status, 0, sizeof(_status));
}

int Mavlink::start()
{
    schedule_register(&task_main_trampoline, this, 102);
    return 0;
}

void Mavlink::task_main_trampoline(void *arg)
{
    namespace_mavlink::instance->task_main();
}

void Mavlink::task_main()
{
    static uint32_t timestamp[4];

    _t = time();
    
    if (!_inited) {
        init();
        return;
    }

    subscribe("status", &_status);

    if (_t > timestamp[0] + 2e4) {
        timestamp[0] = _t;
        parameter_send();
    }

    if (_t > timestamp[1] + 1e5) {
        timestamp[1] = _t;
        usart_receive();
    }

    if (_t > timestamp[2] + 1e6) {
        timestamp[2] = _t;
        heartbeat_send();
        sys_status_send();
    }

    if (_t > timestamp[3] + 2e4) {
        timestamp[3] = _t;
        stream_send();
    }
}

void Mavlink::init()
{
    mavlink_system.sysid = 1;
    mavlink_system.compid = 1;
    mavlink_system.type = 2;

    _inited = true;
}

void Mavlink::heartbeat_send()
{
    uint8_t mavlink_status = 0;
    uint8_t mavlink_base_mode = 0;
    uint32_t mavlink_custom_mode = 0;

    custom_mode_u custom_mode;
    custom_mode.data = 0;

    // arming state
    if (_status.armed) {
        mavlink_base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // main state
    mavlink_base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    switch (_status.nav_state) {
        case NAVIGATION_STATE_MANUAL:
            mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
            custom_mode.main_mode = 1;
            break;

        case NAVIGATION_STATE_STAB:
            mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED;
            custom_mode.main_mode = 7;
            break;

        case NAVIGATION_STATE_ALTCTL:
            mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
                            | MAV_MODE_FLAG_STABILIZE_ENABLED;
            custom_mode.main_mode = 2;
            break;

        case NAVIGATION_STATE_POSCTL:
            mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
                            | MAV_MODE_FLAG_STABILIZE_ENABLED
                            | MAV_MODE_FLAG_GUIDED_ENABLED;
            custom_mode.main_mode = 3;
            break;
    }

    mavlink_custom_mode = custom_mode.data;

    // set system state
    if (_status.armed) {
        mavlink_status = MAV_STATE_ACTIVE;

    } else {
        mavlink_status = MAV_STATE_STANDBY;
    }

    mavlink_msg_heartbeat_send(MAVLINK_COMM_1, mavlink_system.type, MAV_AUTOPILOT_PX4, mavlink_base_mode, mavlink_custom_mode, mavlink_status);
}

void Mavlink::sys_status_send()
{
    uint32_t onboard_control_sensors_present = 0;
    uint32_t onboard_control_sensors_enabled = 0;
    uint32_t onboard_control_sensors_health = 0;
    uint16_t load = 0;
    uint16_t voltage_battery = _status.voltage * 1000.0f;
    int16_t current_battery = -1;
    int8_t battery_remaining = 100.0f * (_status.voltage - 6.8f) / (8.4f - 6.8f);

    mavlink_msg_sys_status_send(MAVLINK_COMM_1, onboard_control_sensors_present, onboard_control_sensors_enabled, onboard_control_sensors_health, load, voltage_battery, current_battery, battery_remaining, 0, 0, 0, 0, 0, 0);
}

void Mavlink::parameter_send()
{
    if (_param_index < _param_count) {

        const char *name = param_name(_param_index);
        param_type_t type = param_type(_param_index);
        mavlink_message_type_t mav_type;
        float val;
        param_get(_param_index, &val);

        if (type == PARAM_TYPE_INT32) {
            mav_type = MAVLINK_TYPE_INT32_T;

        } else if (type == PARAM_TYPE_FLOAT) {
            mav_type = MAVLINK_TYPE_FLOAT;
        }

        mavlink_msg_param_value_send(MAVLINK_COMM_1, name, val, mav_type, _param_count, _param_index);

        _param_index++;
    }
}

void Mavlink::stream_send()
{
    static int state(0);

    switch (state) {

        case 0 :
        {
            attitude_s angle = {};
						subscribe("attitude", &angle);
//						mavlink_msg_attitude_send(MAVLINK_COMM_1, angle.timestamp, att.roll, att.pitch, att.yaw, att.roll_rate, att.pitch_rate, att.yaw_rate);
//							
//							
//						 flow_s flow = {};
//            subscribe("flow", &flow);

//            mavlink_msg_optical_flow_rad_send(MAVLINK_COMM_1, flow.timestamp, 0, flow.integration_timespan, flow.pixel_flow_x_integral, flow.pixel_flow_y_integral,
//                flow.gyro_x_rate_integral, flow.gyro_y_rate_integral, flow.gyro_z_rate_integral, 0.0f, flow.quality, 0.0f, flow.ground_distance_m);

							
            mavlink_msg_named_value_float_send(MAVLINK_COMM_1, time(), "ANGLE_EULER", _sensor.euler[1]);
						mavlink_msg_named_value_float_send(MAVLINK_COMM_1, time(), "ANGLE_ACC", _sensor.acc_angle[1]);
							
						mavlink_msg_named_value_float_send(MAVLINK_COMM_1, time(), "ANGLE_GY_X", _sensor.gyro[0]);
						mavlink_msg_named_value_float_send(MAVLINK_COMM_1, time(), "ANGLE_GY_Y", _sensor.gyro[1]);
						mavlink_msg_named_value_float_send(MAVLINK_COMM_1, time(), "ANGLE_GY_Z", _sensor.gyro[2]);
							
 //           mavlink_msg_named_value_float_send(MAVLINK_COMM_1, time(), "ANGLE", flow.pixel_flow_y_distance);
            state++;

            break;
        }

        case 1 :
        {
//            attitude_s att = {};
//            subscribe("attitude", &att);

//            // send attitude message
//            mavlink_msg_attitude_send(MAVLINK_COMM_1, att.timestamp, att.roll, att.pitch, att.yaw, att.roll_rate, att.pitch_rate, att.yaw_rate);

//            manual_s manual = {};
//            subscribe("manual", &manual);
//            // send manual control
//            mavlink_msg_manual_control_send(MAVLINK_COMM_1, mavlink_system.sysid, manual.x * 1000.0f, manual.y * 1000.0f, manual.z * 1000.0f, manual.r * 1000.0f, manual.aux[0]);

            state++;

            break;
        }

        case 2 :
        {
           

            state++;

            break;
        }

        case 3 :
        {

            

            state++;

            break;
        }

        case 4 :
        {
           

            state = 0;

            break;
        }
    }
}

void Mavlink::handle_message(mavlink_channel_t chan, mavlink_message_t* msg)
{
	// handling messages
	switch (msg->msgid)
	{
		case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        {
			mavlink_param_request_read_t set;
			mavlink_msg_param_request_read_decode(msg, &set);

			// Check if this message is for this system 
			if ((uint8_t) set.target_system == (uint8_t) mavlink_system.sysid
                && (uint8_t) set.target_component == (uint8_t) mavlink_system.compid) {

				char* key = (char*) set.param_id;

				if (set.param_id[0] != (char)-1) {
					// Choose parameter based on index
					if ((set.param_index >= 0) && (set.param_index < param_count())) {
						// report back value
                        const char *name = param_name(set.param_index);
                        param_type_t type = param_type(set.param_index);
                        if (type == PARAM_TYPE_INT32) {
                            int val;
                            param_get(set.param_index, &val);
                            mavlink_msg_param_value_send(chan, name, val, MAVLINK_TYPE_INT32_T, param_count(), set.param_index);

                        } else if (type == PARAM_TYPE_FLOAT) {
                            float val;
                            param_get(set.param_index, &val);
                            mavlink_msg_param_value_send(chan, name, val, MAVLINK_TYPE_FLOAT, param_count(), set.param_index);
                        }
					}

				} else {
					for (int i = 0; i < param_count(); i++) {
                        const char *name = param_name(i);

						bool match = !strcmp(name, key);

						// check if matched
						if (match) {
							// report back value
                            param_type_t type = param_type(i);
                            if (type == PARAM_TYPE_INT32) {
                                int val;
                                param_get(set.param_index, &val);
                                mavlink_msg_param_value_send(chan, name, val, MAVLINK_TYPE_INT32_T, param_count(), set.param_index);

                            } else if (type == PARAM_TYPE_FLOAT) {
                                float val;
                                param_get(set.param_index, &val);
                                mavlink_msg_param_value_send(chan, name, val, MAVLINK_TYPE_FLOAT, param_count(), set.param_index);
                            }
						}
					}
				}
			}
        }
        break;

		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        {
			// start sending parameters
			_param_index = 0;
        }
        break;

		case MAVLINK_MSG_ID_PARAM_SET:
		{
			mavlink_param_set_t set;
			mavlink_msg_param_set_decode(msg, &set);

			// Check if this message is for this system
			if ((uint8_t) set.target_system == (uint8_t) mavlink_system.sysid
                && (uint8_t) set.target_component == (uint8_t) mavlink_system.compid) {
				char* key = (char*) set.param_id;

				for (int i = 0; i < param_count(); i++) {
                    const char *name = param_name(i);

					bool match = !strcmp(name, key);

					// Check if matched
					if (match) {
						// Only write and emit changes if there is actually a difference
                        // AND only write if new value is NOT "not-a-number" AND is NOT infinity
							float val;
							param_get(i, &val);

						if (!FLOAT_EQ_FLOAT(val, set.param_value) && !isnan(set.param_value) && !isinf(set.param_value)) {
							val = set.param_value;
                            param_set(i, &val);

                            write(dev_flash, NULL, 0);

							// report back new value
							mavlink_msg_param_value_send(MAVLINK_COMM_1, name, val, MAVLINK_TYPE_FLOAT, param_count(), i);
						} else {
							// send back current value because it is not accepted or not write access
							mavlink_msg_param_value_send(MAVLINK_COMM_1, name, val, MAVLINK_TYPE_FLOAT, param_count(), i);
						}
					}
				}
			}

            break;
		}

		case MAVLINK_MSG_ID_PING: {
			mavlink_ping_t ping;
			mavlink_msg_ping_decode(msg, &ping);
			if (ping.target_system == 0 && ping.target_component == 0) {
				/* Respond to ping */
				uint32_t r_timestamp = time();
				mavlink_msg_ping_send(chan, ping.seq, msg->sysid, msg->compid, r_timestamp);
			}

            break;
		}

		case MAVLINK_MSG_ID_COMMAND_LONG: {
			mavlink_command_long_t cmd;
			mavlink_msg_command_long_decode(msg, &cmd);

			switch (cmd.command) {
				case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
				if (((int)(cmd.param1)) == 1) {
					mavlink_msg_command_ack_send(chan, cmd.command, MAV_RESULT_ACCEPTED);
					/* reboot */
//					systemreset(false);

				} else if (((int)(cmd.param1)) == 3) {
					mavlink_msg_command_ack_send(chan, cmd.command, MAV_RESULT_ACCEPTED);
					/* reboot to bootloader */
//					systemreset(true);

				} else {
					/* parameters are wrong */
					mavlink_msg_command_ack_send(chan, cmd.command, MAV_RESULT_FAILED);
					// XXX add INVALID INPUT to MAV_RESULT
				}
				break;

				default:
					mavlink_msg_command_ack_send(chan, cmd.command, MAV_RESULT_UNSUPPORTED);
				break;
			}

            break;
		}

		default:
			break;
	}
}

void Mavlink::usart_receive(void)
{
	mavlink_message_t msg;
	mavlink_status_t status = { 0 };

    uint8_t c;

	while (read(MAVLINK_PATH, &c, 1) == 0) {

		// Try to get a new message
		if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message
			handle_message(MAVLINK_COMM_0, &msg);
		}
	}
}

/**
 * @brief Send multiple chars (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use
 * @param ch Character to send
 */
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t * ch, uint16_t length)
{
	write(MAVLINK_PATH, ch, length);
}

/*
* Internal function to give access to the channel status for each channel
*/
mavlink_status_t* mavlink_get_channel_status(uint8_t channel)
{
    static mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
    return &m_mavlink_status[channel];
}

/*
* Internal function to give access to the channel buffer for each channel
*/
mavlink_message_t* mavlink_get_channel_buffer(uint8_t channel)
{
    static mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];
    return &m_mavlink_buffer[channel];
}

int mavlink_main(int argc, char *argv[])
{
    if (argc < 2) {
        return 1;
    }

    if (!strcmp(argv[1], "start")) {
        if (namespace_mavlink::instance) {
            // already running
            return -1;
        }

        // 32 byte
        namespace_mavlink::instance = new Mavlink;

        if (!namespace_mavlink::instance) {
            // alloc failed
            return -1;
        }

        if (namespace_mavlink::instance->start()) {
            // start failed
            return -1;
        }

        return 0;
    }

    return 0;
}
