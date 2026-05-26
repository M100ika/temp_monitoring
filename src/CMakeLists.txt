# ──────────────────────────────────────────────────────────────
# Main component – selects ONE source file based on NODE_TARGET.
#
# NODE_TARGET is injected via platformio.ini:
#   board_build.cmake_extra_args = -DNODE_TARGET=node_top
#                                  -DNODE_TARGET=node_bottom
#                                  -DNODE_TARGET=gateway
# ──────────────────────────────────────────────────────────────

if("${NODE_TARGET}" STREQUAL "node_top")
    message(STATUS "[TempMon] Building target: node_top")
    set(APP_SRCS       "node_top/main.cpp")
    set(APP_PRIV_REQS  esp_wifi nvs_flash esp_netif LovyanGFX)

elseif("${NODE_TARGET}" STREQUAL "node_bottom")
    message(STATUS "[TempMon] Building target: node_bottom")
    set(APP_SRCS       "node_bottom/main.cpp")
    set(APP_PRIV_REQS  esp_wifi nvs_flash esp_netif LovyanGFX)

elseif("${NODE_TARGET}" STREQUAL "monitor")
    message(STATUS "[TempMon] Building target: monitor (ESP32-WROOM-32)")
    set(APP_SRCS       "monitor/main.cpp")
    set(APP_PRIV_REQS  driver esp_timer LovyanGFX)

else()
    # Default / explicit: gateway (no display, no LovyanGFX)
    message(STATUS "[TempMon] Building target: gateway (default)")
    set(APP_SRCS       "gateway/main.cpp")
    set(APP_PRIV_REQS  esp_wifi nvs_flash esp_netif)

endif()

idf_component_register(
    SRCS         ${APP_SRCS}
    INCLUDE_DIRS "." "../include"
    PRIV_REQUIRES ${APP_PRIV_REQS}
)
