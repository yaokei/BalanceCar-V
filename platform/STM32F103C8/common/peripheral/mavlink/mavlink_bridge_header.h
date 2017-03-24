
#pragma once

#include <stdbool.h>
#include <inttypes.h>

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

// use efficient approach, see mavlink_helpers.h
#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes
#define MAVLINK_GET_CHANNEL_STATUS mavlink_get_channel_status
#define MAVLINK_GET_CHANNEL_BUFFER mavlink_get_channel_buffer

#include <mavlink_types.h>

extern mavlink_system_t mavlink_system;

// defined in communication.c
extern void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t * ch, uint16_t length);

extern mavlink_status_t* mavlink_get_channel_status(uint8_t chan);
extern mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan);
