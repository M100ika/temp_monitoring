#pragma once
#include <stdint.h>

// Unique node identifiers
#define NODE_ID_TOP     1
#define NODE_ID_BOTTOM  2

// Maximum number of sensors per node
#define MAX_SENSORS     4

// Status flags
#define STATUS_OK       0x00
#define STATUS_ERR_SENS 0x01  // at least one sensor fault

/**
 * ESP-NOW data packet shared between nodes and gateway.
 * Size: 1 + 4 + 16 + 1 + 1 = 23 bytes (well within 250-byte ESP-NOW limit).
 */
typedef struct __attribute__((packed)) {
    uint8_t  node_id;               // NODE_ID_TOP or NODE_ID_BOTTOM
    uint32_t timestamp_ms;          // esp_timer_get_time() / 1000 at send time
    float    temps[MAX_SENSORS];    // temperature readings in °C (NAN = fault)
    uint8_t  num_sensors;           // valid entries in temps[]
    uint8_t  status;                // STATUS_* bitmask
} espnow_packet_t;
