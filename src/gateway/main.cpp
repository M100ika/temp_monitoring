/**
 * Gateway Firmware
 * ESP32-C6 | ESP-NOW RX | Serial JSON output @ 115200
 *
 * Receives packets from Node_Top (id=1) and Node_Bottom (id=2),
 * merges them and outputs one JSON line every 200 ms:
 *
 *   {"timestamp":12345,"top":[25.1,25.2,25.4,25.0],"bottom":[26.1],"status":"OK"}
 *
 * If a node hasn't been heard from in the last 1 s, its values are
 * reported as null and status shows "TIMEOUT".
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "espnow_packet.h"

static const char* TAG = "GATEWAY";

#define NODE_TIMEOUT_MS  1000   // declare node lost after this many ms
#define JSON_PERIOD_MS    200   // output interval (5 Hz)

// ──────────────────────────────────────────────────────────────
// Shared state updated by ESP-NOW callback, read by output task
// ──────────────────────────────────────────────────────────────
typedef struct {
    espnow_packet_t pkt;
    int64_t         rx_time_ms;   // esp_timer_get_time()/1000 at last RX
    bool            valid;
} node_state_t;

static node_state_t s_nodes[3];    // index 1 = top, 2 = bottom
static SemaphoreHandle_t s_mutex;

// ──────────────────────────────────────────────────────────────
// ESP-NOW receive callback (runs in Wi-Fi task context)
// ──────────────────────────────────────────────────────────────
static void espnow_recv_cb(const esp_now_recv_info_t* info,
                           const uint8_t* data, int len)
{
    if (len < (int)sizeof(espnow_packet_t)) return;

    const espnow_packet_t* pkt = (const espnow_packet_t*)data;
    uint8_t id = pkt->node_id;
    if (id < 1 || id > 2) return;

    int64_t now_ms = esp_timer_get_time() / 1000LL;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_nodes[id].pkt       = *pkt;
        s_nodes[id].rx_time_ms = now_ms;
        s_nodes[id].valid     = true;
        xSemaphoreGive(s_mutex);
    }
}

// ──────────────────────────────────────────────────────────────
// Wi-Fi / ESP-NOW init
// ──────────────────────────────────────────────────────────────
static void wifi_espnow_init(void)
{
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    esp_now_init();
    esp_now_register_recv_cb(espnow_recv_cb);

    // Print own MAC so nodes can target it directly if needed
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "Gateway MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}

// ──────────────────────────────────────────────────────────────
// Helper: append a float or "null" into a char buffer
// Returns number of characters written.
// ──────────────────────────────────────────────────────────────
static int append_float(char* buf, int pos, float v)
{
    if (isnan(v) || isinf(v))
        return pos + sprintf(buf + pos, "null");
    return pos + sprintf(buf + pos, "%.2f", (double)v);
}

// ──────────────────────────────────────────────────────────────
// JSON output task (5 Hz)
// ──────────────────────────────────────────────────────────────
static void output_task(void* /*arg*/)
{
    static char line[256];

    for (;;) {
        int64_t now_ms = esp_timer_get_time() / 1000LL;

        // Snapshot under mutex
        node_state_t top_snap, bot_snap;
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        top_snap = s_nodes[NODE_ID_TOP];
        bot_snap = s_nodes[NODE_ID_BOTTOM];
        xSemaphoreGive(s_mutex);

        bool top_alive = top_snap.valid &&
                         (now_ms - top_snap.rx_time_ms < NODE_TIMEOUT_MS);
        bool bot_alive = bot_snap.valid &&
                         (now_ms - bot_snap.rx_time_ms < NODE_TIMEOUT_MS);

        // Determine status string
        const char* status;
        if (!top_alive && !bot_alive)      status = "NO_SIGNAL";
        else if (!top_alive)               status = "TOP_TIMEOUT";
        else if (!bot_alive)               status = "BOT_TIMEOUT";
        else if (top_snap.pkt.status || bot_snap.pkt.status)
                                           status = "SENSOR_ERR";
        else                               status = "OK";

        // Build JSON
        int p = 0;
        p += sprintf(line + p, "{\"timestamp\":%lld,\"top\":[",
                     (long long)now_ms);

        if (top_alive) {
            for (int i = 0; i < top_snap.pkt.num_sensors; i++) {
                if (i > 0) line[p++] = ',';
                p = append_float(line, p, top_snap.pkt.temps[i]);
            }
        } else {
            p += sprintf(line + p, "null,null,null,null");
        }

        p += sprintf(line + p, "],\"bottom\":[");

        if (bot_alive) {
            for (int i = 0; i < bot_snap.pkt.num_sensors; i++) {
                if (i > 0) line[p++] = ',';
                p = append_float(line, p, bot_snap.pkt.temps[i]);
            }
        } else {
            p += sprintf(line + p, "null");
        }

        p += sprintf(line + p, "],\"status\":\"%s\"}\n", status);

        // Output to Serial (UART0 / USB-CDC on ESP32-C6)
        fputs(line, stdout);
        fflush(stdout);

        vTaskDelay(pdMS_TO_TICKS(JSON_PERIOD_MS));
    }
}

// ──────────────────────────────────────────────────────────────
// Entry point
// ──────────────────────────────────────────────────────────────
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting Gateway");

    s_mutex = xSemaphoreCreateMutex();
    memset(s_nodes, 0, sizeof(s_nodes));

    wifi_espnow_init();

    xTaskCreate(output_task, "output_task", 4096, nullptr, 4, nullptr);

    ESP_LOGI(TAG, "Gateway ready – waiting for nodes");
}
