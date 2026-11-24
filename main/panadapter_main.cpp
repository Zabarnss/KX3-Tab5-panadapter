// v1.6.24: show LVGL charging symbol beside battery voltage while charging.
// v1.6.29: separate charging-symbol label; visibility updates independently of battery text.
// v1.6.30: replace font glyph with custom LVGL-drawn lightning bolt.
// v1.6.32: reconcile touch-tune preview against actual CAT VFO changes; avoid stale/doubled local offsets.
// v1.6.23: battery voltage beside frequency; frequency display precision reduced to 1 kHz.
// v1.6.22: unified top-row text styling; removed dropdown/CAT diagnostic logging.
// v1.6.8: waterfall restored; diagnostic monitor/performance readouts omitted.
#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <mutex>
#include <string>

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "logger.hpp"
#include "m5stack-tab5.hpp"
#include "usb_host.hpp"
#include "usb/uac_host.h"
#include "usb/uac2_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/cdc_acm_host_ops.h"
#include "usb/usb_types_cdc.h"
#include "usb/vcp_ftdi.h"

// Use only Montserrat fonts enabled by the project's LVGL configuration.
// Montserrat 14 is known to be available in this project; larger sizes fall
// back to the next available size instead of causing an undeclared-symbol error.
#if defined(LV_FONT_MONTSERRAT_14) && LV_FONT_MONTSERRAT_14
#define APP_FONT_14 (&lv_font_montserrat_14)
#else
#error "LV_FONT_MONTSERRAT_14 must be enabled in lv_conf.h"
#endif

#if defined(LV_FONT_MONTSERRAT_20) && LV_FONT_MONTSERRAT_20
#define APP_FONT_20 (&lv_font_montserrat_20)
#else
#define APP_FONT_20 APP_FONT_14
#endif

#if defined(LV_FONT_MONTSERRAT_28) && LV_FONT_MONTSERRAT_28
#define APP_FONT_28 (&lv_font_montserrat_28)
#elif defined(LV_FONT_MONTSERRAT_20) && LV_FONT_MONTSERRAT_20
#define APP_FONT_28 APP_FONT_20
#else
#define APP_FONT_28 APP_FONT_14
#endif

using namespace std::chrono_literals;

namespace {
constexpr char TAG[] = "KX3_PANADAPTER";
// Performance v2.7 experiment: custom RGB565 spectrum raster instead of LVGL chart widget.

uint32_t s_uac_transfer_error_count = 0;
std::atomic<int> s_uac2_input_gain_db{0};


enum class EventKind : uint8_t {
    RxConnected, TxConnected, DeviceDisconnected, RxReady, TransferError,
    Uac2RxConnected, Uac2TxConnected, Uac2DeviceDisconnected,
    Uac2RxReady, Uac2TransferError, Uac2StreamError
};

struct UacEvent {
    EventKind kind;
    uint8_t address;
    uint8_t interface_number;
    uac_host_device_handle_t handle;
    uac2_host_device_handle_t handle_uac2;
};

struct UiState {
    std::string host_status{"Starting Tab5..."};
    std::string audio_status{"No USB audio input detected"};
    std::string format_status{"Format: waiting for USB audio device"};
    std::string detail_status{"Connect the Syba SD-AUD20101 to the USB-A port"};
    std::string meter_status{"I: -- dBFS     Q: -- dBFS"};
    std::string count_status{"PCM frames: 0"};
};

QueueHandle_t s_event_queue = nullptr;
std::mutex s_ui_mutex;
UiState s_ui;
uac_host_device_handle_t s_rx_handle = nullptr;
uac2_host_device_handle_t s_uac2_rx_handle = nullptr;

// FTDI KXUSB CAT connection. The USB-host callback only queues a notification;
// opening/configuring the device is done from ftdi_device_task().
QueueHandle_t s_ftdi_event_queue = nullptr;
QueueHandle_t s_cat_rx_queue = nullptr;
QueueHandle_t s_tune_queue = nullptr;
QueueHandle_t s_radio_control_queue = nullptr;
cdc_acm_dev_hdl_t s_ftdi_handle = nullptr;

// Latest successfully decoded radio state. CAT runs on its own task; the
// LVGL task and FFT renderer take short, mutex-protected snapshots.
struct CatDisplayState {
    uint64_t vfo_a_hz = 0;
    uint64_t vfo_b_hz = 0;
    char mode_code = 0;
    bool vfo_a_valid = false;
    bool vfo_b_valid = false;
    bool mode_valid = false;
};
std::mutex s_cat_state_mutex;
CatDisplayState s_cat_state{};

const char *kx3_band_name(uint64_t hz);

enum class RadioControlKind : uint8_t { Band, Mode };
struct RadioControlRequest { RadioControlKind kind; uint8_t value; };

static constexpr const char *kBandOptions = "160m\n80m\n60m\n40m\n30m\n20m\n17m\n15m\n10m\n6m";
static constexpr const char *kModeOptions = "USB\nLSB\nCW\nDigital\nAM\nFM";
static constexpr uint8_t kBandNumbers[] = {0, 1, 2, 3, 4, 5, 6, 7, 9, 10};
static constexpr char kModeCommands[] = {'2', '1', '3', '6', '5', '4'};

int band_dropdown_index(uint64_t hz) {
    const char *band = kx3_band_name(hz);
    if (!band) return 0;
    if (std::strcmp(band, "160m") == 0) return 0;
    if (std::strcmp(band, "80m") == 0) return 1;
    if (std::strcmp(band, "60m") == 0) return 2;
    if (std::strcmp(band, "40m") == 0) return 3;
    if (std::strcmp(band, "30m") == 0) return 4;
    if (std::strcmp(band, "20m") == 0) return 5;
    if (std::strcmp(band, "17m") == 0) return 6;
    if (std::strcmp(band, "15m") == 0) return 7;
    if (std::strcmp(band, "10m") == 0) return 8;
    if (std::strcmp(band, "6m") == 0) return 9;
    return 0;
}

int mode_dropdown_index(char mode) {
    switch (mode) {
    case '2': return 0; // USB
    case '1': return 1; // LSB
    case '3': case '7': return 2; // CW / CW reverse
    case '6': case '9': return 3; // Digital DATA variants
    case '5': return 4; // AM
    case '4': return 5; // FM
    default: return 0;
    }
}

lv_obj_t *s_top_band_label = nullptr;
lv_obj_t *s_top_mode_label = nullptr;

// Keep the user's dropdown selection visible while the KX3 is processing the
// CAT command. The periodic radio-state refresh must not immediately revert the
// control to the old band/mode before the radio confirms the change.
bool s_band_selection_pending = false;
uint8_t s_pending_band_index = 0;
TickType_t s_band_selection_tick = 0;
bool s_mode_selection_pending = false;
uint8_t s_pending_mode_index = 0;
TickType_t s_mode_selection_tick = 0;
static constexpr TickType_t RADIO_SELECTION_CONFIRM_TIMEOUT = pdMS_TO_TICKS(5000);

// Style the dropdown's popup list using the LVGL 8 API. LV_PART_LIST is not
// available in this project version; the popup is a separate object returned
// by lv_dropdown_get_list() after it has been created/opened.
void radio_dropdown_style_popup(lv_obj_t *dropdown) {
    if (!dropdown) return;
    lv_obj_t *list = lv_dropdown_get_list(dropdown);
    if (!list) return;

    lv_obj_set_style_text_font(list, APP_FONT_28, LV_PART_MAIN);
    lv_obj_set_style_text_font(list, APP_FONT_28, LV_PART_SELECTED);
    lv_obj_set_style_pad_ver(list, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(list, 8, LV_PART_SELECTED);
}

void radio_dropdown_popup_style_event(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    lv_obj_t *dropdown = static_cast<lv_obj_t *>(lv_event_get_target(e));
    radio_dropdown_style_popup(dropdown);
}

void radio_dropdown_event(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if (!s_radio_control_queue) {
        ESP_LOGW(TAG, "Dropdown selection ignored: radio control queue is unavailable");
        return;
    }

    lv_obj_t *dropdown = static_cast<lv_obj_t *>(lv_event_get_target(e));
    RadioControlRequest request{};
    const TickType_t now = xTaskGetTickCount();
    if (dropdown == s_top_band_label) {
        const uint16_t selected = lv_dropdown_get_selected(dropdown);
        if (selected >= (sizeof(kBandNumbers) / sizeof(kBandNumbers[0]))) {
            ESP_LOGW(TAG, "Invalid Band dropdown index: %u", (unsigned)selected);
            return;
        }
        request.kind = RadioControlKind::Band;
        request.value = kBandNumbers[selected];
        s_band_selection_pending = true;
        s_pending_band_index = static_cast<uint8_t>(selected);
        s_band_selection_tick = now;
    } else if (dropdown == s_top_mode_label) {
        const uint16_t selected = lv_dropdown_get_selected(dropdown);
        if (selected >= (sizeof(kModeCommands) / sizeof(kModeCommands[0]))) {
            ESP_LOGW(TAG, "Invalid Mode dropdown index: %u", (unsigned)selected);
            return;
        }
        request.kind = RadioControlKind::Mode;
        request.value = static_cast<uint8_t>(kModeCommands[selected]);
        s_mode_selection_pending = true;
        s_pending_mode_index = static_cast<uint8_t>(selected);
        s_mode_selection_tick = now;
    } else {
        return;
    }

    // Use a short bounded wait instead of silently dropping the request when
    // the CAT task is busy.
    if (xQueueSend(s_radio_control_queue, &request, pdMS_TO_TICKS(100)) != pdTRUE) {
        if (request.kind == RadioControlKind::Band) s_band_selection_pending = false;
        else s_mode_selection_pending = false;
    }
}

lv_obj_t *s_top_frequency_label = nullptr;
lv_obj_t *s_top_battery_label = nullptr;
lv_obj_t *s_top_charging_icon = nullptr;
bool s_battery_monitor_initialized = false;
int64_t s_last_battery_update_us = 0;

// Draw a charging bolt using LVGL's drawing API instead of a font glyph.
// This avoids symbol-font coverage/rendering issues on the Tab5.
void charging_bolt_draw_event(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) return;
    lv_layer_t *layer = lv_event_get_layer(e);
    lv_obj_t *obj = lv_event_get_target_obj(e);
    if (!layer || !obj) return;

    lv_area_t area;
    lv_obj_get_coords(obj, &area);
    lv_point_precise_t points[] = {
        {area.x2 - 3, area.y1 + 1},
        {area.x1 + 5, area.y1 + 10},
        {area.x1 + 11, area.y1 + 10},
        {area.x1 + 3, area.y2 - 1},
    };
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.points = points;
    line_dsc.point_cnt = 4;
    line_dsc.color = lv_color_hex(0xFFD400);
    line_dsc.width = 3;
    line_dsc.round_start = 1;
    line_dsc.round_end = 1;
    lv_draw_line(layer, &line_dsc);
}
lv_obj_t *s_reset_button_label = nullptr;
bool s_reset_requested = false;
lv_obj_t *s_tuning_cursor = nullptr;
uint64_t s_preview_frequency_hz = 0;
bool s_tuning_gesture_active = false;
// Phase-1 touch gesture state: tap/hold tunes; horizontal drag pans the view.
int32_t s_pan_offset_hz = 0;
static bool s_touch_gesture_active = false;
static bool s_touch_pan_mode = false;
static bool s_touch_tune_mode = false;
static int s_touch_start_x = 0;
static int s_touch_last_x = 0;
static uint32_t s_touch_start_tick = 0;
static int32_t s_touch_start_pan_hz = 0;
static uint64_t s_touch_start_frequency_hz = 0;
static bool s_pan_tune_pending = false;
static uint64_t s_pan_tune_target_hz = 0;
// Frequency at the radio immediately before the queued touch-tune command.
// Reconcile against an actual CAT VFO change, not an exact requested value:
// the KX3 may quantize/ignore the 1 Hz digit depending on FINE tuning mode.
static uint64_t s_pan_tune_start_hz = 0;
static uint32_t s_pan_tune_request_tick = 0;
constexpr uint32_t PAN_TUNE_RECONCILE_TIMEOUT_MS = 1500;
constexpr int TOUCH_PAN_THRESHOLD_PX = 24;
constexpr uint32_t TOUCH_TUNE_HOLD_MS = 300;
// Set by the CAT task when VFO A changes; consumed by the LVGL/UI task.
std::atomic<bool> s_frequency_change_pending{false};
uint32_t s_active_rate = 0;

constexpr size_t FFT_SIZE = 1024;
// LVGL chart resolution: fewer points than the FFT, while retaining the full FFT for DSP.
constexpr uint16_t SPECTRUM_CHART_POINTS = 512;
static std::array<lv_coord_t, SPECTRUM_CHART_POINTS> s_spectrum_chart_values{};
// Panadapter FFT/display update cadence.
constexpr uint32_t PANADAPTER_REFRESH_MS = 50;
constexpr float FFT_PI = 3.14159265358979323846f;
std::mutex s_iq_mutex;
std::array<float, FFT_SIZE> s_i_ring{};
std::array<float, FFT_SIZE> s_q_ring{};
size_t s_iq_write = 0;
size_t s_iq_count = 0;
lv_obj_t *s_spectrum_chart = nullptr; // Generic container retained for touch/frequency geometry.
lv_obj_t *s_spectrum_image = nullptr;
uint16_t *s_spectrum_pixels = nullptr;
uint16_t s_spectrum_width = 0;
lv_image_dsc_t s_spectrum_dsc{};
lv_obj_t *s_spectrum_peak_label = nullptr;
lv_obj_t *s_spectrum_axis_label = nullptr; // ruler container, directly below the chart
lv_obj_t *s_center_marker = nullptr;
constexpr size_t FREQUENCY_TICK_COUNT = 19; // -45 kHz through +45 kHz, 5 kHz spacing
std::array<lv_obj_t *, FREQUENCY_TICK_COUNT> s_frequency_labels{};
std::array<lv_obj_t *, FREQUENCY_TICK_COUNT> s_frequency_grid_lines{};

// RGB565 waterfall history. New rows are inserted at the top of a mirrored ring.
uint16_t s_waterfall_width = 0;
constexpr uint16_t WATERFALL_HEIGHT = 251;
constexpr int PANADAPTER_HEIGHT = 330;
uint16_t *s_waterfall_pixels = nullptr;
static uint16_t s_waterfall_head = 0;
// LVGL image descriptor is RGB565, so each stored pixel MUST be uint16_t.
// lv_color_t is not guaranteed to be RGB565 (LVGL 9 commonly uses RGB888).
static std::array<float, FFT_SIZE> s_averaged_power{};
static bool s_averaging_initialized = false;
// After a VFO change, use fast spectrum decay for a short transition so the
// previous frequency context disappears smoothly without blanking the display.
static int s_frequency_transition_frames = 0;
constexpr int FREQUENCY_TRANSITION_FRAMES = 8;
constexpr float FREQUENCY_TRANSITION_AVERAGING_SECONDS = 0.10f;
// Set the initial display low levels from the first valid FFT after audio starts.
static bool s_auto_low_levels_initialized = false;
// User-adjustable display settings. The settings panel is an LVGL overlay.
struct DisplaySettings {
    int frequency_offset_hz = 0;
    int pan_low_db = -100;
    int pan_high_db = -20;
    int waterfall_low_db = -95;
    int waterfall_high_db = -35;
    float averaging_seconds = 0.2f;
    int display_width_khz = 48;
    int waterfall_speed_hz = 40; // Waterfall history rows per second (30-60 Hz).
    int uac2_input_gain_db = 0;  // Applied only to UAC2/Xonar PCM; UAC1 remains unchanged.
};
DisplaySettings s_settings{};
bool s_settings_nvs_ready = false;

void load_persistent_audio_settings() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Do not erase the NVS partition automatically: preserve any other
        // application data and continue with the safe 0 dB default.
        ESP_LOGW(TAG, "NVS unavailable (%s); UAC2 gain will use default 0 dB",
                 esp_err_to_name(err));
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS initialization failed (%s); UAC2 gain will use default 0 dB",
                 esp_err_to_name(err));
        return;
    }

    nvs_handle_t handle{};
    err = nvs_open("kx3_pan", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not open settings storage (%s)", esp_err_to_name(err));
        return;
    }
    s_settings_nvs_ready = true;

    int32_t saved_gain_db = 0;
    err = nvs_get_i32(handle, "uac2_gain_db", &saved_gain_db);
    if (err == ESP_OK) {
        s_settings.uac2_input_gain_db = std::clamp(static_cast<int>(saved_gain_db), 0, 24);
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Could not read saved UAC2 gain (%s); using 0 dB",
                 esp_err_to_name(err));
    }
    nvs_close(handle);
    s_uac2_input_gain_db.store(s_settings.uac2_input_gain_db, std::memory_order_relaxed);
    ESP_LOGI(TAG, "Loaded saved UAC2 input gain: %+d dB",
             s_settings.uac2_input_gain_db);
}

void save_persistent_audio_settings() {
    if (!s_settings_nvs_ready) return;
    nvs_handle_t handle{};
    esp_err_t err = nvs_open("kx3_pan", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not open settings storage to save UAC2 gain (%s)",
                 esp_err_to_name(err));
        return;
    }
    err = nvs_set_i32(handle, "uac2_gain_db", s_settings.uac2_input_gain_db);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Saved UAC2 input gain: %+d dB",
                 s_settings.uac2_input_gain_db);
    } else {
        ESP_LOGW(TAG, "Failed to save UAC2 input gain (%s)", esp_err_to_name(err));
    }
}
lv_obj_t *s_settings_panel = nullptr;
lv_obj_t *s_settings_backdrop = nullptr;
// Modal settings state: pauses FFT/panadapter and waterfall work, but leaves
// USB audio, CAT communications, LVGL, and touch handling active.
std::atomic<bool> s_settings_open{false};

enum class SettingId : uint8_t {
    FrequencyOffset, PanLow, PanHigh, WaterfallLow, WaterfallHigh,
    Averaging, DisplayWidth, WaterfallSpeed, Uac2InputGain
};
struct SettingControl {
    SettingId id;
    lv_obj_t *value_label;
    lv_obj_t *slider;
};
std::array<SettingControl, 8> s_setting_controls{};

lv_image_dsc_t s_waterfall_dsc{};
lv_obj_t *s_waterfall_image = nullptr;
uint32_t s_last_fft_tick = 0;

// Defined below the FFT update routine; used to refresh the settings overlay
// if it has already been opened when the first audio frame arrives.
void update_setting_value_labels();

// Pack RGB888 components into the RGB565 format declared by the LVGL image.
uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return static_cast<uint16_t>(((uint16_t(r) & 0xF8u) << 8) |
                                 ((uint16_t(g) & 0xFCu) << 3) |
                                  (uint16_t(b) >> 3));
}

void spectrum_draw_pixel(int x, int y, uint16_t color) {
    if (!s_spectrum_pixels || x < 0 || y < 0 || x >= s_spectrum_width || y >= PANADAPTER_HEIGHT) return;
    s_spectrum_pixels[static_cast<size_t>(y) * s_spectrum_width + static_cast<size_t>(x)] = color;
}

void spectrum_draw_line(int x0, int y0, int x1, int y1, uint16_t color) {
    // Integer Bresenham line drawing, avoiding LVGL chart-series rendering.
    const int dx = std::abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        spectrum_draw_pixel(x0, y0, color);
        // Two-pixel trace to match the previous chart's line width.
        spectrum_draw_pixel(x0, y0 + 1, color);
        if (x0 == x1 && y0 == y1) break;
        const int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

constexpr uint32_t SPECTRUM_RENDER_MIN_INTERVAL_MS = 100; // Limit image refresh to 10 FPS.
static std::array<int16_t, SPECTRUM_CHART_POINTS> s_previous_spectrum_x{};
static std::array<int16_t, SPECTRUM_CHART_POINTS> s_previous_spectrum_y{};
static bool s_previous_spectrum_trace_valid = false;
void render_spectrum_image(bool force = false) {
    if (!s_spectrum_pixels || !s_spectrum_image || s_spectrum_width == 0) return;
    static int64_t last_render_us = 0;
    const int64_t now_us = esp_timer_get_time();
    if (!force && last_render_us != 0 &&
        now_us - last_render_us < static_cast<int64_t>(SPECTRUM_RENDER_MIN_INTERVAL_MS) * 1000) {
        return;
    }
    last_render_us = now_us;
    const uint16_t background = rgb565(8, 16, 24); // Near-black blue.
    const uint16_t horizontal_grid = rgb565(41, 64, 82);
    const uint16_t trace_color = rgb565(66, 230, 164);

    const float low = static_cast<float>(s_settings.pan_low_db);
    const float high = static_cast<float>(s_settings.pan_high_db);
    const float range = std::max(1.0f, high - low);
    std::array<int16_t, SPECTRUM_CHART_POINTS> new_x{};
    std::array<int16_t, SPECTRUM_CHART_POINTS> new_y{};
    int dirty_y_min = PANADAPTER_HEIGHT - 1;
    int dirty_y_max = 0;

    for (uint16_t point = 0; point < SPECTRUM_CHART_POINTS; ++point) {
        const int x = (SPECTRUM_CHART_POINTS > 1)
            ? static_cast<int>(point) * (s_spectrum_width - 1) / (SPECTRUM_CHART_POINTS - 1)
            : 0;
        const float db = static_cast<float>(s_spectrum_chart_values[point]);
        const float normalized = std::clamp((db - low) / range, 0.0f, 1.0f);
        const int y = PANADAPTER_HEIGHT - 1 - static_cast<int>(std::lround(normalized * (PANADAPTER_HEIGHT - 1)));
        new_x[point] = static_cast<int16_t>(x);
        new_y[point] = static_cast<int16_t>(y);
        dirty_y_min = std::min(dirty_y_min, y);
        dirty_y_max = std::max(dirty_y_max, y + 1);
    }

    // First render or a forced refresh: rebuild the static background completely.
    if (force || !s_previous_spectrum_trace_valid) {
        std::fill(s_spectrum_pixels,
                  s_spectrum_pixels + static_cast<size_t>(s_spectrum_width) * PANADAPTER_HEIGHT,
                  background);
        for (int division = 1; division < 5; ++division) {
            const int y = division * (PANADAPTER_HEIGHT - 1) / 5;
            uint16_t *row = s_spectrum_pixels + static_cast<size_t>(y) * s_spectrum_width;
            std::fill(row, row + s_spectrum_width, horizontal_grid);
        }
        dirty_y_min = 0;
        dirty_y_max = PANADAPTER_HEIGHT - 1;
    } else {
        // Erase only the old trace. The background and grid are otherwise static.
        for (uint16_t point = 1; point < SPECTRUM_CHART_POINTS; ++point) {
            spectrum_draw_line(s_previous_spectrum_x[point - 1], s_previous_spectrum_y[point - 1],
                              s_previous_spectrum_x[point], s_previous_spectrum_y[point], background);
        }
        for (uint16_t point = 0; point < SPECTRUM_CHART_POINTS; ++point) {
            dirty_y_min = std::min(dirty_y_min, static_cast<int>(s_previous_spectrum_y[point]));
            dirty_y_max = std::max(dirty_y_max, static_cast<int>(s_previous_spectrum_y[point]) + 1);
        }
        // Restore grid rows intersecting the dirty trace area.
        for (int division = 1; division < 5; ++division) {
            const int y = division * (PANADAPTER_HEIGHT - 1) / 5;
            if (y >= dirty_y_min && y <= dirty_y_max) {
                uint16_t *row = s_spectrum_pixels + static_cast<size_t>(y) * s_spectrum_width;
                std::fill(row, row + s_spectrum_width, horizontal_grid);
            }
        }
    }

    // Draw the newest trace over the restored background.
    for (uint16_t point = 1; point < SPECTRUM_CHART_POINTS; ++point) {
        spectrum_draw_line(new_x[point - 1], new_y[point - 1], new_x[point], new_y[point], trace_color);
    }
    s_previous_spectrum_x = new_x;
    s_previous_spectrum_y = new_y;
    s_previous_spectrum_trace_valid = true;

    // Invalidate only the vertical band occupied by the old/new traces.
    // LVGL invalidation areas are in screen coordinates, so translate the
    // local image band using the image object's current position.
    lv_area_t image_coords;
    lv_obj_get_coords(s_spectrum_image, &image_coords);
    lv_area_t dirty_area;
    dirty_area.x1 = image_coords.x1;
    dirty_area.x2 = image_coords.x2;
    dirty_area.y1 = image_coords.y1 + static_cast<lv_coord_t>(std::max(0, dirty_y_min - 2));
    dirty_area.y2 = image_coords.y1 + static_cast<lv_coord_t>(std::min(PANADAPTER_HEIGHT - 1, dirty_y_max + 2));
    lv_obj_invalidate_area(s_spectrum_image, &dirty_area);
}

uint16_t waterfall_color(float above_noise_db) {
    // NaP3-like palette: quiet blue background, bright cyan/yellow signals.
    // 'above_noise_db' uses a temporally averaged linear-power spectrum.
    if (above_noise_db < 5.0f) return rgb565(0, 7, 28);
    if (above_noise_db < 10.0f) {
        const float t = (above_noise_db - 5.0f) / 5.0f;
        return rgb565(0, static_cast<uint8_t>(8 + 18*t),
                         static_cast<uint8_t>(30 + 75*t));
    }
    if (above_noise_db < 18.0f) {
        const float t = (above_noise_db - 10.0f) / 8.0f;
        return rgb565(0, static_cast<uint8_t>(26 + 175*t),
                         static_cast<uint8_t>(105 + 140*t));
    }
    if (above_noise_db < 28.0f) {
        const float t = (above_noise_db - 18.0f) / 10.0f;
        return rgb565(static_cast<uint8_t>(255*t),
                      static_cast<uint8_t>(201 + 54*t),
                      static_cast<uint8_t>(245*(1.0f-t)));
    }
    if (above_noise_db < 38.0f) {
        const float t = (above_noise_db - 28.0f) / 10.0f;
        return rgb565(255, static_cast<uint8_t>(255 - 190*t), 0);
    }
    return rgb565(255, 45, 0);
}


struct ComplexValue { float re; float im; };

void fft_in_place(std::array<ComplexValue, FFT_SIZE> &a) {
    // Iterative radix-2 decimation-in-time FFT.
    size_t j = 0;
    for (size_t i = 1; i < FFT_SIZE; ++i) {
        size_t bit = FFT_SIZE >> 1;
        while (j & bit) { j ^= bit; bit >>= 1; }
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= FFT_SIZE; len <<= 1) {
        const float angle = -2.0f * FFT_PI / static_cast<float>(len);
        const float wlen_re = std::cos(angle);
        const float wlen_im = std::sin(angle);
        for (size_t i = 0; i < FFT_SIZE; i += len) {
            float w_re = 1.0f, w_im = 0.0f;
            for (size_t k = 0; k < len / 2; ++k) {
                const ComplexValue u = a[i + k];
                const ComplexValue v0 = a[i + k + len / 2];
                const ComplexValue v{v0.re * w_re - v0.im * w_im,
                                     v0.re * w_im + v0.im * w_re};
                a[i + k] = {u.re + v.re, u.im + v.im};
                a[i + k + len / 2] = {u.re - v.re, u.im - v.im};
                const float next_re = w_re * wlen_re - w_im * wlen_im;
                w_im = w_re * wlen_im + w_im * wlen_re;
                w_re = next_re;
            }
        }
    }
}

// Convert a signed frequency offset to the FFT's *natural* bin order.
// fft_in_place() leaves DC at bin 0, positive frequencies at bins 1..N/2-1,
// and negative frequencies at bins N/2+1..N-1. The display is fftshifted,
// so signed negative frequencies must wrap to the high-numbered FFT bins.
size_t fft_bin_for_frequency(float frequency_hz, float bin_width_hz) {
    int bin = static_cast<int>(std::lround(frequency_hz / bin_width_hz));
    bin %= static_cast<int>(FFT_SIZE);
    if (bin < 0) bin += static_cast<int>(FFT_SIZE);
    return static_cast<size_t>(bin);
}

// The Tab5's current L/R I/Q wiring produces the opposite complex-spectrum
// orientation from the conventional panadapter frequency axis. Keep the
// screen/ruler coordinates increasing from low frequency on the left to high
// frequency on the right, but mirror the signed frequency when looking up the
// natural-order FFT bin. Apply this to both spectrum and waterfall lookups.
size_t fft_bin_for_display_frequency(float display_frequency_hz, float bin_width_hz) {
    return fft_bin_for_frequency(-display_frequency_hz, bin_width_hz);
}

int kx3_iq_bin_correction_hz(char mode_code);

const char *kx3_band_name(uint64_t hz) {
    if (hz >= 1800000 && hz <= 2000000) return "160m";
    if (hz >= 3500000 && hz <= 4000000) return "80m";
    if (hz >= 5330000 && hz <= 5410000) return "60m";
    if (hz >= 7000000 && hz <= 7300000) return "40m";
    if (hz >= 10100000 && hz <= 10150000) return "30m";
    if (hz >= 14000000 && hz <= 14350000) return "20m";
    if (hz >= 18068000 && hz <= 18168000) return "17m";
    if (hz >= 21000000 && hz <= 21450000) return "15m";
    if (hz >= 24890000 && hz <= 24990000) return "12m";
    if (hz >= 28000000 && hz <= 29700000) return "10m";
    return "--";
}

uint64_t touch_frequency_from_chart_x(lv_obj_t *chart, int x) {
    CatDisplayState radio{};
    { std::lock_guard<std::mutex> lock(s_cat_state_mutex); radio = s_cat_state; }
    if (!radio.vfo_a_valid || !chart) return 0;
    const int w = lv_obj_get_width(chart);
    const int pl = lv_obj_get_style_pad_left(chart, LV_PART_MAIN);
    const int pr = lv_obj_get_style_pad_right(chart, LV_PART_MAIN);
    const int usable = std::max(1, w - pl - pr);
    const float f = std::max(0.0f, std::min(1.0f, (x - pl) / static_cast<float>(usable)));
    const double offset = (f - 0.5) * s_settings.display_width_khz * 1000.0;
    const double hz = static_cast<double>(radio.vfo_a_hz) + s_settings.frequency_offset_hz + s_pan_offset_hz + offset;
    return hz > 0 ? static_cast<uint64_t>(std::llround(hz)) : 0;
}

void update_frequency_ruler();

void spectrum_touch_event(lv_event_t *e) {
    const lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev || !s_spectrum_chart || s_settings_open.load()) return;

    lv_point_t point{};
    lv_indev_get_point(indev, &point);
    lv_area_t chart_area{};
    lv_obj_get_coords(s_spectrum_chart, &chart_area);
    const int x = point.x - chart_area.x1;
    const uint64_t hz = touch_frequency_from_chart_x(s_spectrum_chart, x);
    if (!hz) return;
    const uint32_t now = lv_tick_get();

    auto show_cursor_at = [&](int local_x) {
        if (!s_tuning_cursor) return;
        lv_area_t chart{};
        lv_obj_get_coords(s_spectrum_chart, &chart);
        const int cursor_x = lv_obj_get_x(s_spectrum_chart) + local_x - 1;
        const int cursor_y = lv_obj_get_y(s_spectrum_chart);
        lv_area_t waterfall{};
        if (s_waterfall_image) lv_obj_get_coords(s_waterfall_image, &waterfall);
        else waterfall = chart;
        const int cursor_height = std::max(1, static_cast<int>(waterfall.y2 - chart.y1 + 1));
        lv_obj_set_pos(s_tuning_cursor, cursor_x, cursor_y);
        lv_obj_set_height(s_tuning_cursor, cursor_height);
        lv_obj_clear_flag(s_tuning_cursor, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(s_tuning_cursor);
    };

    if (code == LV_EVENT_PRESSED) {
        s_touch_gesture_active = true;
        s_touch_pan_mode = false;
        s_touch_tune_mode = false;
        s_touch_start_x = s_touch_last_x = x;
        s_touch_start_tick = now;
        s_touch_start_pan_hz = s_pan_offset_hz;
        CatDisplayState radio_start{};
        { std::lock_guard<std::mutex> lock(s_cat_state_mutex); radio_start = s_cat_state; }
        s_touch_start_frequency_hz = radio_start.vfo_a_valid ? radio_start.vfo_a_hz : hz;
        s_tuning_gesture_active = true;
        s_preview_frequency_hz = hz;
        show_cursor_at(x);
        return;
    }
    if (code == LV_EVENT_PRESSING && s_touch_gesture_active) {
        const int dx = x - s_touch_start_x;
        if (!s_touch_pan_mode && !s_touch_tune_mode && std::abs(dx) >= TOUCH_PAN_THRESHOLD_PX) {
            s_touch_pan_mode = true;
            s_tuning_gesture_active = false;
            if (s_tuning_cursor) lv_obj_add_flag(s_tuning_cursor, LV_OBJ_FLAG_HIDDEN);
        }
        if (!s_touch_pan_mode && !s_touch_tune_mode &&
            (now - s_touch_start_tick) >= TOUCH_TUNE_HOLD_MS &&
            std::abs(dx) < TOUCH_PAN_THRESHOLD_PX) {
            s_touch_tune_mode = true;
        }
        if (s_touch_pan_mode) {
            const int chart_w = std::max<int32_t>(1, lv_obj_get_width(s_spectrum_chart));
            const int32_t shift_hz = static_cast<int32_t>(std::lround(
                -dx * (s_settings.display_width_khz * 1000.0 / chart_w)));
            const uint64_t target_hz = static_cast<uint64_t>(std::max<int64_t>(1,
                static_cast<int64_t>(s_touch_start_frequency_hz) + shift_hz));
            CatDisplayState radio_now{};
            { std::lock_guard<std::mutex> lock(s_cat_state_mutex); radio_now = s_cat_state; }
            s_pan_offset_hz = static_cast<int32_t>(static_cast<int64_t>(target_hz) -
                static_cast<int64_t>(radio_now.vfo_a_valid ? radio_now.vfo_a_hz : s_touch_start_frequency_hz) -
                static_cast<int64_t>(s_settings.frequency_offset_hz));
            s_pan_tune_pending = true;
            s_pan_tune_target_hz = target_hz;
            s_pan_tune_start_hz = radio_now.vfo_a_valid ? radio_now.vfo_a_hz : s_touch_start_frequency_hz;
            s_pan_tune_request_tick = lv_tick_get();
            if (s_tune_queue) xQueueOverwrite(s_tune_queue, &target_hz);
            update_frequency_ruler();
        } else {
            // Tap previews immediately; hold mode remains tuning while dragged.
            s_preview_frequency_hz = hz;
            show_cursor_at(x);
        }
        s_touch_last_x = x;
        return;
    }
    if (code == LV_EVENT_RELEASED && s_touch_gesture_active) {
        const int dx = x - s_touch_start_x;
        if (!s_touch_pan_mode && std::abs(dx) >= TOUCH_PAN_THRESHOLD_PX) {
            // A release can arrive before a PRESSING event classifies the drag.
            s_touch_pan_mode = true;
            const int chart_w = std::max<int32_t>(1, lv_obj_get_width(s_spectrum_chart));
            const int32_t shift_hz = static_cast<int32_t>(std::lround(
                -dx * (s_settings.display_width_khz * 1000.0 / chart_w)));
            const uint64_t target_hz = static_cast<uint64_t>(std::max<int64_t>(1,
                static_cast<int64_t>(s_touch_start_frequency_hz) + shift_hz));
            CatDisplayState radio_now{};
            { std::lock_guard<std::mutex> lock(s_cat_state_mutex); radio_now = s_cat_state; }
            s_pan_offset_hz = static_cast<int32_t>(static_cast<int64_t>(target_hz) -
                static_cast<int64_t>(radio_now.vfo_a_valid ? radio_now.vfo_a_hz : s_touch_start_frequency_hz) -
                static_cast<int64_t>(s_settings.frequency_offset_hz));
            s_pan_tune_pending = true;
            s_pan_tune_target_hz = target_hz;
            s_pan_tune_start_hz = radio_now.vfo_a_valid ? radio_now.vfo_a_hz : s_touch_start_frequency_hz;
            s_pan_tune_request_tick = lv_tick_get();
            if (s_tune_queue) xQueueOverwrite(s_tune_queue, &target_hz);
            update_frequency_ruler();
        }
        s_touch_gesture_active = false;
        s_tuning_gesture_active = false;
        if (s_tuning_cursor) lv_obj_add_flag(s_tuning_cursor, LV_OBJ_FLAG_HIDDEN);
        if (!s_touch_pan_mode) {
            s_preview_frequency_hz = hz;
            CatDisplayState radio_now{};
            { std::lock_guard<std::mutex> lock(s_cat_state_mutex); radio_now = s_cat_state; }
            s_pan_offset_hz = static_cast<int32_t>(static_cast<int64_t>(s_preview_frequency_hz) -
                static_cast<int64_t>(radio_now.vfo_a_valid ? radio_now.vfo_a_hz : s_preview_frequency_hz) -
                static_cast<int64_t>(s_settings.frequency_offset_hz));
            s_pan_tune_pending = true;
            s_pan_tune_target_hz = s_preview_frequency_hz;
            s_pan_tune_start_hz = radio_now.vfo_a_valid ? radio_now.vfo_a_hz : s_preview_frequency_hz;
            s_pan_tune_request_tick = lv_tick_get();
            if (s_tune_queue) xQueueOverwrite(s_tune_queue, &s_preview_frequency_hz);
            update_frequency_ruler();
        }
        s_touch_pan_mode = false;
        s_touch_tune_mode = false;
    }
}

void update_frequency_ruler() {
    if (!s_spectrum_axis_label || !s_spectrum_chart) return;

    // Use the live KX3 VFO A frequency as the absolute reference. The
    // user-adjustable frequency offset shifts the displayed span relative to
    // that dial frequency, just as it does in the FFT-bin lookup.
    CatDisplayState radio_state{};
    {
        std::lock_guard<std::mutex> cat_lock(s_cat_state_mutex);
        radio_state = s_cat_state;
    }

    if (!radio_state.vfo_a_valid) {
        for (size_t i = 0; i < FREQUENCY_TICK_COUNT; ++i) {
            if (s_frequency_labels[i]) {
                lv_obj_add_flag(s_frequency_labels[i], LV_OBJ_FLAG_HIDDEN);
            }
            if (s_frequency_grid_lines[i]) {
                lv_obj_add_flag(s_frequency_grid_lines[i], LV_OBJ_FLAG_HIDDEN);
            }
        }
        return;
    }

    constexpr double TICK_SPACING_HZ = 5000.0;
    const double width_hz = static_cast<double>(s_settings.display_width_khz) * 1000.0;
    const double center_hz = static_cast<double>(radio_state.vfo_a_hz) +
                             static_cast<double>(s_settings.frequency_offset_hz + s_pan_offset_hz);
    const double left_hz = center_hz - width_hz * 0.5;
    const double right_hz = center_hz + width_hz * 0.5;
    const double first_tick_hz = std::ceil(left_hz / TICK_SPACING_HZ) * TICK_SPACING_HZ;

    // LVGL's built-in chart grid uses equal divisions unrelated to radio
    // frequency. Draw our own grid at absolute 5 kHz boundaries instead.
    const int chart_width = lv_obj_get_width(s_spectrum_chart);
    const int chart_height = lv_obj_get_height(s_spectrum_chart);
    const int pad_left = lv_obj_get_style_pad_left(s_spectrum_chart, LV_PART_MAIN);
    const int pad_right = lv_obj_get_style_pad_right(s_spectrum_chart, LV_PART_MAIN);
    const int plot_width = std::max(1, chart_width - pad_left - pad_right);

    for (size_t i = 0; i < FREQUENCY_TICK_COUNT; ++i) {
        if (!s_frequency_labels[i] || !s_frequency_grid_lines[i]) continue;

        const double tick_hz = first_tick_hz + static_cast<double>(i) * TICK_SPACING_HZ;
        if (tick_hz > right_hz) {
            lv_obj_add_flag(s_frequency_labels[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_frequency_grid_lines[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }

        const double fraction = (tick_hz - left_hz) / width_hz;
        const int chart_x = pad_left + static_cast<int>(std::lround(fraction * plot_width));
        const uint64_t tick_frequency_hz = static_cast<uint64_t>(std::llround(tick_hz));
        const unsigned long long whole_mhz =
            static_cast<unsigned long long>(tick_frequency_hz / 1000000ULL);
        const unsigned long long fractional_khz =
            static_cast<unsigned long long>((tick_frequency_hz % 1000000ULL) / 1000ULL);

        char text[20];
        std::snprintf(text, sizeof(text), "%llu.%03llu", whole_mhz, fractional_khz);
        lv_label_set_text(s_frequency_labels[i], text);
        lv_obj_set_width(s_frequency_labels[i], 64);
        lv_obj_set_pos(s_frequency_labels[i], chart_x - 32, 0);
        lv_obj_clear_flag(s_frequency_labels[i], LV_OBJ_FLAG_HIDDEN);

        // Each subtle vertical line is at the exact absolute frequency
        // printed in the ruler directly below the spectrum.
        lv_obj_set_pos(s_frequency_grid_lines[i], chart_x, 0);
        lv_obj_set_size(s_frequency_grid_lines[i], 1, chart_height);
        lv_obj_clear_flag(s_frequency_grid_lines[i], LV_OBJ_FLAG_HIDDEN);
    }
}

void update_spectrum_chart(uint8_t waterfall_rows) {
    if (!s_spectrum_chart || s_active_rate == 0) return;
    // Static storage keeps the 16 KB FFT working set off the 16 KB main-task stack.
    static std::array<float, FFT_SIZE> i_samples{};
    static std::array<float, FFT_SIZE> q_samples{};
    static std::array<ComplexValue, FFT_SIZE> spectrum{};
    {
        std::lock_guard<std::mutex> lock(s_iq_mutex);
        if (s_iq_count < FFT_SIZE) return;
        for (size_t n = 0; n < FFT_SIZE; ++n) {
            const size_t index = (s_iq_write + n) % FFT_SIZE;
            i_samples[n] = s_i_ring[index];
            q_samples[n] = s_q_ring[index];
        }
    }


    float window_sum = 0.0f;
    for (size_t n = 0; n < FFT_SIZE; ++n) {
        const float window = 0.5f - 0.5f * std::cos(
            2.0f * FFT_PI * static_cast<float>(n) /
            static_cast<float>(FFT_SIZE - 1));
        window_sum += window;
        spectrum[n] = {i_samples[n] * window, q_samples[n] * window};
    }
    fft_in_place(spectrum);

    // Advance the mirrored circular history. The visible rows remain contiguous
    // for LVGL, avoiding a full-history memmove on every update.
    if (waterfall_rows > 0 && s_waterfall_pixels && s_waterfall_image) {
        waterfall_rows = std::min<uint8_t>(waterfall_rows, WATERFALL_HEIGHT);
        s_waterfall_head = static_cast<uint16_t>(
            (s_waterfall_head + WATERFALL_HEIGHT - waterfall_rows) % WATERFALL_HEIGHT);
    }

    static std::array<float, FFT_SIZE> db_bins{};
    const float normalization = std::max(window_sum, 1.0f);
    const float sample_rate = static_cast<float>(s_active_rate);
    const float bin_width = sample_rate / static_cast<float>(FFT_SIZE);
    const float shown_width_hz = static_cast<float>(s_settings.display_width_khz) * 1000.0f;
    const float refresh_seconds = static_cast<float>(PANADAPTER_REFRESH_MS) / 1000.0f;
    const float active_averaging_seconds = (s_frequency_transition_frames > 0)
        ? FREQUENCY_TRANSITION_AVERAGING_SECONDS : s_settings.averaging_seconds;
    const float alpha = (active_averaging_seconds <= 0.001f)
        ? 1.0f : 1.0f - std::exp(-refresh_seconds / active_averaging_seconds);
    char active_mode_code = 0;
    {
        std::lock_guard<std::mutex> cat_lock(s_cat_state_mutex);
        if (s_cat_state.mode_valid) active_mode_code = s_cat_state.mode_code;
    }
    static char previous_mode_code = 0;
    if (active_mode_code != 0 && previous_mode_code == 0) {
        // Initial CAT mode discovery is not a radio mode change. Keep the
        // startup average intact; only a later transition should reseed it.
        previous_mode_code = active_mode_code;
    } else if (active_mode_code != 0 && active_mode_code != previous_mode_code) {
        s_averaging_initialized = false;
        previous_mode_code = active_mode_code;
    }
    const float mode_correction_hz =
        static_cast<float>(kx3_iq_bin_correction_hz(active_mode_code));

    for (size_t x = 0; x < FFT_SIZE; ++x) {
        const float fraction = static_cast<float>(x) / static_cast<float>(FFT_SIZE - 1);
        const float frequency_hz = static_cast<float>(s_settings.frequency_offset_hz + s_pan_offset_hz)
            + (fraction - 0.5f) * shown_width_hz + mode_correction_hz;
        const size_t bin = fft_bin_for_display_frequency(frequency_hz, bin_width);
        const float re = spectrum[bin].re / normalization;
        const float im = spectrum[bin].im / normalization;
        const float instant_power = re * re + im * im;
        // Average LINEAR POWER, not dB values: persistent carriers are
        // retained while rapidly changing random noise is reduced.
        if (!s_averaging_initialized) {
            s_averaged_power[x] = instant_power;
        } else {
            s_averaged_power[x] += alpha * (instant_power - s_averaged_power[x]);
        }
        const float db = 10.0f * std::log10(
            std::max(s_averaged_power[x], 1.0e-12f));
        db_bins[x] = db;
    }
    // Update the LVGL chart through a public external-data-array API.
    // This avoids 512 individual chart setter calls (each of which may trigger
    // invalidation/overhead). We retain the strongest FFT bin per chart point
    // so narrow carriers remain visible.
    for (uint16_t point = 0; point < SPECTRUM_CHART_POINTS; ++point) {
        const size_t first_bin = static_cast<size_t>(point) * FFT_SIZE / SPECTRUM_CHART_POINTS;
        const size_t end_bin = static_cast<size_t>(point + 1) * FFT_SIZE / SPECTRUM_CHART_POINTS;
        float point_db = -100.0f;
        for (size_t bin = first_bin; bin < end_bin; ++bin) {
            if (db_bins[bin] > point_db) point_db = db_bins[bin];
        }
        s_spectrum_chart_values[point] = static_cast<lv_coord_t>(
            std::max(-100.0f, std::min(0.0f, point_db)));
    }
    s_averaging_initialized = true;
    if (s_frequency_transition_frames > 0) --s_frequency_transition_frames;

    // Calibrate both displays from the mean signal baseline in the central
    // 10 kHz. This runs once per audio-stream start.
    if (!s_auto_low_levels_initialized) {
        double pan_baseline_sum = 0.0;
        size_t pan_baseline_count = 0;
        double waterfall_baseline_sum = 0.0;
        size_t waterfall_baseline_count = 0;
        for (size_t x = 0; x < FFT_SIZE; ++x) {
            const float fraction = static_cast<float>(x) / static_cast<float>(FFT_SIZE - 1);
            const float offset_from_center_hz = (fraction - 0.5f) * shown_width_hz;
            if (std::fabs(offset_from_center_hz) <= 5000.0f) {
                pan_baseline_sum += db_bins[x];
                ++pan_baseline_count;
            }
        }
        if (s_waterfall_width > 0) {
            for (uint16_t x = 0; x < s_waterfall_width; ++x) {
                const float fraction = s_waterfall_width > 1
                    ? static_cast<float>(x) / static_cast<float>(s_waterfall_width - 1) : 0.5f;
                if (std::fabs((fraction - 0.5f) * shown_width_hz) <= 5000.0f) {
                    const size_t display_bin = static_cast<size_t>(std::lround(
                        fraction * static_cast<float>(FFT_SIZE - 1)));
                    waterfall_baseline_sum += db_bins[display_bin];
                    ++waterfall_baseline_count;
                }
            }
        }
        if (pan_baseline_count > 0) {
            const int pan_mean_db = static_cast<int>(std::lround(
                pan_baseline_sum / static_cast<double>(pan_baseline_count)));
            s_settings.pan_low_db = std::clamp(pan_mean_db, -120,
                std::min(-20, s_settings.pan_high_db - 5));
            if (waterfall_baseline_count > 0) {
                const int waterfall_mean_db = static_cast<int>(std::lround(
                    waterfall_baseline_sum / static_cast<double>(waterfall_baseline_count)));
                s_settings.waterfall_low_db = std::clamp(waterfall_mean_db, -120,
                    std::min(-20, s_settings.waterfall_high_db - 5));
            }
            for (auto &control : s_setting_controls) {
                if (!control.slider) continue;
                if (control.id == SettingId::PanLow)
                    lv_slider_set_value(control.slider, s_settings.pan_low_db, LV_ANIM_OFF);
                else if (control.id == SettingId::WaterfallLow)
                    lv_slider_set_value(control.slider, s_settings.waterfall_low_db, LV_ANIM_OFF);
            }
            update_setting_value_labels();
            s_auto_low_levels_initialized = true;
            if (s_reset_requested) {
                if (s_reset_button_label) lv_label_set_text(s_reset_button_label, "Reset");
                s_reset_requested = false;
            }
        }
    }

    // Render newest waterfall row(s) from display-ordered dB bins.
    if (waterfall_rows > 0 && s_waterfall_pixels && s_waterfall_width > 0 && s_waterfall_image) {
        static std::array<uint16_t, 2048> newest_waterfall_row{};
        if (s_waterfall_width <= newest_waterfall_row.size()) {
            for (uint16_t x = 0; x < s_waterfall_width; ++x) {
                const float fraction = s_waterfall_width > 1
                    ? static_cast<float>(x) / static_cast<float>(s_waterfall_width - 1) : 0.5f;
                const size_t display_bin = static_cast<size_t>(std::lround(
                    fraction * static_cast<float>(FFT_SIZE - 1)));
                const float span = static_cast<float>(std::max(1,
                    s_settings.waterfall_high_db - s_settings.waterfall_low_db));
                const float scaled = (db_bins[display_bin] - s_settings.waterfall_low_db) / span * 38.0f;
                newest_waterfall_row[x] = waterfall_color(scaled);
            }
            for (uint8_t row = 0; row < waterfall_rows; ++row) {
                const uint16_t physical_row = static_cast<uint16_t>(
                    (s_waterfall_head + row) % WATERFALL_HEIGHT);
                uint16_t *dst = s_waterfall_pixels + static_cast<size_t>(physical_row) * s_waterfall_width;
                uint16_t *mirror_dst = s_waterfall_pixels +
                    static_cast<size_t>(physical_row + WATERFALL_HEIGHT) * s_waterfall_width;
                std::memcpy(dst, newest_waterfall_row.data(), s_waterfall_width * sizeof(uint16_t));
                std::memcpy(mirror_dst, newest_waterfall_row.data(), s_waterfall_width * sizeof(uint16_t));
            }
            s_waterfall_dsc.data = reinterpret_cast<const uint8_t *>(
                s_waterfall_pixels + static_cast<size_t>(s_waterfall_head) * s_waterfall_width);
            lv_image_set_src(s_waterfall_image, &s_waterfall_dsc);
            lv_obj_invalidate(s_waterfall_image);
        }
    }

    render_spectrum_image(); // Throttled to 10 Hz; FFT data continues updating faster.

    update_frequency_ruler();

}

void update_ui_state(const std::string &host, const std::string &audio,
                     const std::string &format, const std::string &detail) {
    std::lock_guard<std::mutex> lock(s_ui_mutex);
    s_ui.host_status = host;
    s_ui.audio_status = audio;
    s_ui.format_status = format;
    s_ui.detail_status = detail;
}


// The FTDI VCP driver removes its two status bytes before invoking this
// callback. Keep the callback short: copy received bytes into a queue and
// let the CAT task assemble complete Elecraft protocol frames.
bool ftdi_cat_data_callback(const uint8_t *data, size_t data_len, void *user_arg) {
    (void)user_arg;
    if (!s_cat_rx_queue || (!data && data_len != 0)) return false;

    for (size_t i = 0; i < data_len; ++i) {
        if (xQueueSend(s_cat_rx_queue, &data[i], 0) != pdTRUE) {
            ESP_LOGW(TAG, "CAT RX queue full; dropping received data");
            return false;
        }
    }
    return true;
}

bool parse_kx3_frequency_frame(const std::string &frame, const char *prefix,
                               uint64_t &frequency_hz) {
    // Elecraft FA/FB response: prefix plus frequency in Hz, terminated by ';'.
    if (frame.size() < 3 || frame.compare(0, 2, prefix) != 0) return false;

    uint64_t value = 0;
    for (size_t i = 2; i < frame.size(); ++i) {
        const char ch = frame[i];
        if (ch < '0' || ch > '9') return false;
        value = value * 10u + static_cast<uint64_t>(ch - '0');
    }
    frequency_hz = value;
    return value > 0;
}


// The KX3 I/Q output has a mode-dependent audio-frequency displacement
// relative to the dial reference. With a 600 Hz CW sidetone, the measured
// raw FFT carrier is +600 Hz in CW and LSB, and -600 Hz in USB.
// IMPORTANT: fft_bin_for_display_frequency() reverses the signed frequency
// when mapping display coordinates to natural FFT bins. Therefore the
// correction applied here must have the opposite sign of the measured raw
// carrier displacement. This places a zero-beat CW carrier at the dial
// frequency instead of displaying it approximately 1.2 kHz low.
// Other modes remain uncorrected until their behavior is measured.
int kx3_iq_bin_correction_hz(char mode_code) {
    switch (mode_code) {
    case '1': // LSB: raw carrier +600 Hz, compensate -600 Hz
    case '3': // CW:  raw carrier +600 Hz, compensate -600 Hz
        return -600;
    case '2': // USB: raw carrier -600 Hz, compensate +600 Hz
        return 600;
    default:
        return 0;
    }
}

bool kx3_cat_request_frame(const char *command, const char *expected_prefix,
                           std::string &response, uint32_t &consecutive_timeouts) {
    // This task is the sole CAT transmitter/receiver, so discard any stale
    // bytes before each request and match the response to its command prefix.
    uint8_t rx_byte = 0;
    while (xQueueReceive(s_cat_rx_queue, &rx_byte, 0) == pdTRUE) {
    }

    const esp_err_t tx_err = cdc_acm_host_data_tx_blocking(
        s_ftdi_handle,
        reinterpret_cast<const uint8_t *>(command),
        std::strlen(command),
        1000);
    if (tx_err != ESP_OK) {
        ++consecutive_timeouts;
        if (consecutive_timeouts == 1 || (consecutive_timeouts % 10) == 0) {
            ESP_LOGW(TAG, "KX3 CAT transmit %s failed: %s",
                     command, esp_err_to_name(tx_err));
        }
        return false;
    }

    response.clear();
    const TickType_t wait_start = xTaskGetTickCount();
    const TickType_t response_timeout = pdMS_TO_TICKS(400);
    while ((xTaskGetTickCount() - wait_start) < response_timeout) {
        if (xQueueReceive(s_cat_rx_queue, &rx_byte, pdMS_TO_TICKS(40)) != pdTRUE) {
            continue;
        }

        const char ch = static_cast<char>(rx_byte);
        if (ch == ';') {
            if (response.compare(0, 2, expected_prefix) == 0) {
                consecutive_timeouts = 0;
                return true;
            }
            // Ignore unrelated/echo frames and keep waiting for this reply.
            response.clear();
        } else if (ch != '\r' && ch != '\n') {
            if (response.size() < 32) {
                response.push_back(ch);
            } else {
                response.clear();
            }
        }
    }

    ++consecutive_timeouts;
    if (consecutive_timeouts == 1 || (consecutive_timeouts % 10) == 0) {
        ESP_LOGW(TAG, "No valid KX3 %s response (%u consecutive timeouts)",
                 expected_prefix, static_cast<unsigned>(consecutive_timeouts));
    }
    return false;
}

void kx3_cat_poll_task(void *arg) {
    (void)arg;
    uint32_t fa_timeouts = 0;
    uint32_t md_timeouts = 0;
    ESP_LOGD(TAG, "Starting KX3 CAT polling: FA; MD; at 38400 baud");

    for (;;) {
        RadioControlRequest control{};
        if (s_radio_control_queue && xQueueReceive(s_radio_control_queue, &control, 0) == pdTRUE) {
            char command[8];
            if (control.kind == RadioControlKind::Band) {
                std::snprintf(command, sizeof(command), "BN%02u;", control.value);
                const esp_err_t err = cdc_acm_host_data_tx_blocking(s_ftdi_handle,
                    reinterpret_cast<const uint8_t *>(command), std::strlen(command), 1000);
                if (err == ESP_OK) {
                    vTaskDelay(pdMS_TO_TICKS(350)); // KX3 restores that band's last-used frequency/mode
                } else ESP_LOGE(TAG, "Band command transmission failed: %s", esp_err_to_name(err));
            } else {
                std::snprintf(command, sizeof(command), "MD%c;", control.value);
                const esp_err_t err = cdc_acm_host_data_tx_blocking(s_ftdi_handle,
                    reinterpret_cast<const uint8_t *>(command), std::strlen(command), 1000);
                if (err != ESP_OK) ESP_LOGE(TAG, "Mode command transmission failed: %s", esp_err_to_name(err));
            }
        }
        uint64_t requested_frequency_hz = 0;
        if (s_tune_queue && xQueueReceive(s_tune_queue, &requested_frequency_hz, 0) == pdTRUE) {
            char tune_command[20];
            std::snprintf(tune_command, sizeof(tune_command), "FA%011llu;",
                          static_cast<unsigned long long>(requested_frequency_hz));
            const esp_err_t tune_err = cdc_acm_host_data_tx_blocking(
                s_ftdi_handle, reinterpret_cast<const uint8_t *>(tune_command),
                std::strlen(tune_command), 1000);
            if (tune_err == ESP_OK) {
                ESP_LOGD(TAG, "Touch tune VFO A: %llu Hz",
                         static_cast<unsigned long long>(requested_frequency_hz));
            } else {
                ESP_LOGW(TAG, "Touch tune command failed: %s", esp_err_to_name(tune_err));
            }
        }
        std::string response;
        uint64_t frequency_hz = 0;

        if (kx3_cat_request_frame("FA;", "FA", response, fa_timeouts)) {
            if (parse_kx3_frequency_frame(response, "FA", frequency_hz)) {
                bool vfo_frequency_changed = false;
                {
                    std::lock_guard<std::mutex> cat_lock(s_cat_state_mutex);
                    vfo_frequency_changed = s_cat_state.vfo_a_valid &&
                                            s_cat_state.vfo_a_hz != frequency_hz;
                    s_cat_state.vfo_a_hz = frequency_hz;
                    s_cat_state.vfo_a_valid = true;
                }
                if (vfo_frequency_changed) {
                    // The CAT task must not call LVGL. Notify the UI task to
                    // clear old-frequency spectrum and waterfall data safely.
                    s_frequency_change_pending.store(true, std::memory_order_release);
                    ESP_LOGD(TAG, "VFO A changed; scheduling spectrum refresh");
                } else {
                }
                ESP_LOGD(TAG, "KX3 VFO A: %llu Hz (%.6f MHz)",
                         static_cast<unsigned long long>(frequency_hz),
                         static_cast<double>(frequency_hz) / 1000000.0);
            } else {
                ESP_LOGW(TAG, "Malformed KX3 FA response: %s;", response.c_str());
            }
        }

        if (kx3_cat_request_frame("MD;", "MD", response, md_timeouts)) {
            if (response.size() == 3 && response[2] >= '0' && response[2] <= '9') {
                {
                    std::lock_guard<std::mutex> cat_lock(s_cat_state_mutex);
                    s_cat_state.mode_code = response[2];
                    s_cat_state.mode_valid = true;
                }
            } else {
                ESP_LOGW(TAG, "Unexpected KX3 MD response: %s;", response.c_str());
            }
        }

        // Keep a modest update rate while allowing all three CAT requests to
        // complete serially on the same FTDI connection.
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void ftdi_new_device_callback(usb_device_handle_t usb_dev) {
    (void)usb_dev;
    if (!s_ftdi_event_queue) return;
    const uint8_t event = 1;
    if (xQueueSend(s_ftdi_event_queue, &event, 0) != pdTRUE) {
        ESP_LOGW(TAG, "FTDI device notification queue full; notification dropped");
    }
}

void ftdi_device_task(void *arg) {
    (void)arg;
    uint8_t event = 0;
    for (;;) {
        if (xQueueReceive(s_ftdi_event_queue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (s_ftdi_handle) {
            continue;
        }

        const cdc_acm_host_device_config_t device_config = {
            .connection_timeout_ms = 3000,
            .out_buffer_size = 256,
            .in_buffer_size = 0, // Required by the Espressif FTDI VCP driver.
            .event_cb = nullptr,
            .data_cb = ftdi_cat_data_callback,
            .user_arg = nullptr,
        };

        cdc_acm_dev_hdl_t handle = nullptr;
        const esp_err_t open_err = ftdi_vcp_open(
            FT232_PID, 0, &device_config, &handle);
        if (open_err != ESP_OK) {
            // Other USB devices (for example the Syba audio adapter) also
            // generate new-device notifications. They are not CAT interfaces.
            ESP_LOGD(TAG, "USB device notification: no FT232 KXUSB opened (%s)",
                     esp_err_to_name(open_err));
            continue;
        }

        const cdc_acm_line_coding_t line_coding = {
            .dwDTERate = 38400,
            .bCharFormat = 0, // 1 stop bit
            .bParityType = 0, // no parity
            .bDataBits = 8,
        };
        esp_err_t err = cdc_acm_host_line_coding_set(handle, &line_coding);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "FTDI opened but setting 38400 8N1 failed: %s",
                     esp_err_to_name(err));
            cdc_acm_host_close(handle);
            continue;
        }

        err = cdc_acm_host_set_control_line_state(handle, false, false);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "FTDI opened but disabling DTR/RTS failed: %s",
                     esp_err_to_name(err));
            cdc_acm_host_close(handle);
            continue;
        }

        s_ftdi_handle = handle;
        ESP_LOGD(TAG,
                 "Elecraft KXUSB opened (VID=0403 PID=6001); configured 38400 8N1, no flow control, DTR=0 RTS=0.");

        if (xTaskCreatePinnedToCore(kx3_cat_poll_task, "kx3_cat_poll_task",
                                    4096, nullptr, 5, nullptr, 1) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create KX3 CAT polling task");
            s_ftdi_handle = nullptr;
            cdc_acm_host_close(handle);
        }
    }
}

void uac_driver_callback(uint8_t addr, uint8_t iface_num,
                         const uac_host_driver_event_t event, void *arg) {
    (void)arg;
    if (!s_event_queue) return;

    UacEvent queued{};
    queued.address = addr;
    queued.interface_number = iface_num;
    queued.handle = nullptr;
    queued.handle_uac2 = nullptr;

    if (event == UAC_HOST_DRIVER_EVENT_RX_CONNECTED) {
        queued.kind = EventKind::RxConnected;
    } else if (event == UAC_HOST_DRIVER_EVENT_TX_CONNECTED) {
        queued.kind = EventKind::TxConnected;
    } else {
        return;
    }

    if (xQueueSend(s_event_queue, &queued, 0) != pdTRUE) {
        ESP_LOGW(TAG, "UAC event queue full; event dropped");
    }
}

void uac_device_callback(uac_host_device_handle_t handle,
                         const uac_host_device_event_t event, void *arg) {
    (void)arg;
    if (!s_event_queue) return;
    UacEvent queued{};
    queued.handle = handle;
    queued.handle_uac2 = nullptr;
    if (event == UAC_HOST_DRIVER_EVENT_DISCONNECTED) {
        queued.kind = EventKind::DeviceDisconnected;
    } else if (event == UAC_HOST_DEVICE_EVENT_RX_DONE) {
        queued.kind = EventKind::RxReady;
    } else if (event == UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR) {
        queued.kind = EventKind::TransferError;
    } else {
        return;
    }
    if (xQueueSend(s_event_queue, &queued, 0) != pdTRUE &&
        event != UAC_HOST_DEVICE_EVENT_RX_DONE) {
        ESP_LOGW(TAG, "UAC device event queue full");
    }
}

void uac2_driver_callback(uint8_t addr, uint8_t iface_num,
                          const uac2_host_driver_event_t event, void *arg) {
    (void)arg;
    if (!s_event_queue) return;
    UacEvent queued{};
    queued.address = addr;
    queued.interface_number = iface_num;
    queued.handle = nullptr;
    queued.handle_uac2 = nullptr;
    if (event == UAC2_HOST_DRIVER_EVENT_RX_CONNECTED) {
        queued.kind = EventKind::Uac2RxConnected;
    } else if (event == UAC2_HOST_DRIVER_EVENT_TX_CONNECTED) {
        queued.kind = EventKind::Uac2TxConnected;
    } else {
        return;
    }
    if (xQueueSend(s_event_queue, &queued, 0) != pdTRUE) {
        ESP_LOGW(TAG, "UAC2 event queue full; event dropped");
    }
}

void uac2_device_callback(uac2_host_device_handle_t handle,
                          const uac2_host_device_event_t event, void *arg) {
    (void)arg;
    if (!s_event_queue) return;
    UacEvent queued{};
    queued.handle = nullptr;
    queued.handle_uac2 = handle;
    if (event == UAC2_HOST_DEVICE_EVENT_DISCONNECTED) {
        queued.kind = EventKind::Uac2DeviceDisconnected;
    } else if (event == UAC2_HOST_DEVICE_EVENT_RX_DONE) {
        queued.kind = EventKind::Uac2RxReady;
    } else if (event == UAC2_HOST_DEVICE_EVENT_TRANSFER_ERROR) {
        queued.kind = EventKind::Uac2TransferError;
    } else if (event == UAC2_HOST_DEVICE_EVENT_STREAM_ERROR) {
        queued.kind = EventKind::Uac2StreamError;
    } else {
        return;
    }
    if (xQueueSend(s_event_queue, &queued, 0) != pdTRUE &&
        event != UAC2_HOST_DEVICE_EVENT_RX_DONE) {
        ESP_LOGW(TAG, "UAC2 device event queue full");
    }
}

// Stereo PCM frames are interleaved: I0,Q0,I1,Q1,... Both USB audio drivers
// feed this common processing function so the FFT and waterfall stay identical.
void process_pcm_bytes(const uint8_t *bytes, uint32_t got, bool is_uac2) {
    if (got < 4) return;


    const size_t frames = got / 4;
    const int gain_db = is_uac2 ? s_uac2_input_gain_db.load(std::memory_order_relaxed) : 0;
    const float sample_gain = is_uac2
        ? static_cast<float>(std::pow(10.0, static_cast<double>(gain_db) / 20.0))
        : 1.0f;
    static std::array<float, 1024> chunk_i{};
    static std::array<float, 1024> chunk_q{};

    for (size_t n = 0; n < frames; ++n) {
        const size_t k = 4 * n;
        const int16_t i = static_cast<int16_t>(
            uint16_t(bytes[k]) | (uint16_t(bytes[k + 1]) << 8));
        const int16_t q = static_cast<int16_t>(
            uint16_t(bytes[k + 2]) | (uint16_t(bytes[k + 3]) << 8));
        chunk_i[n] = (static_cast<float>(i) / 32768.0f) * sample_gain;
        chunk_q[n] = (static_cast<float>(q) / 32768.0f) * sample_gain;
    }

    {
        std::lock_guard<std::mutex> iq_lock(s_iq_mutex);
        for (size_t n = 0; n < frames; ++n) {
            s_i_ring[s_iq_write] = chunk_i[n];
            s_q_ring[s_iq_write] = chunk_q[n];
            s_iq_write = (s_iq_write + 1) % FFT_SIZE;
            if (s_iq_count < FFT_SIZE) ++s_iq_count;
        }
    }


}

void consume_pcm(uac_host_device_handle_t handle) {
    static uint8_t bytes[4096];
    uint32_t got = 0;
    const esp_err_t err = uac_host_device_read(handle, bytes, sizeof(bytes), &got, 0);
    if (err != ESP_OK) return;
    process_pcm_bytes(bytes, got, false);
}

void consume_pcm(uac2_host_device_handle_t handle) {
    static uint8_t bytes[4096];
    uint32_t got = 0;
    const esp_err_t err = uac2_host_device_read(handle, bytes, sizeof(bytes), &got, 0);
    if (err != ESP_OK) return;
    process_pcm_bytes(bytes, got, true);
}

std::string describe_alt(const uac_host_dev_alt_param_t &alt) {
    char text[220]{};
    int used = std::snprintf(text, sizeof(text), "%u channel(s), %u-bit PCM, %u-byte subframe; ",
                             alt.channels, alt.bit_resolution, alt.subframe_size);
    if (alt.sample_freq_type == 0) {
        std::snprintf(text + used, sizeof(text) - used, "rate range %lu-%lu Hz",
                      static_cast<unsigned long>(alt.sample_freq_lower),
                      static_cast<unsigned long>(alt.sample_freq_upper));
    } else {
        used += std::snprintf(text + used, sizeof(text) - used, "rates ");
        for (uint8_t i = 0; i < alt.sample_freq_type && i < UAC_FREQ_NUM_MAX; ++i) {
            used += std::snprintf(text + used, sizeof(text) - used, "%s%lu",
                                  i == 0 ? "" : ", ",
                                  static_cast<unsigned long>(alt.sample_freq[i]));
            if (used >= static_cast<int>(sizeof(text)) - 24) break;
        }
        std::snprintf(text + used, sizeof(text) - used, " Hz");
    }
    return text;
}

void uac_event_task(void *arg) {
    (void)arg;
    UacEvent event{};
    while (true) {
        if (xQueueReceive(s_event_queue, &event, portMAX_DELAY) != pdTRUE) continue;

        if (event.kind == EventKind::Uac2RxConnected) {
            ESP_LOGI(TAG, "UAC2 capture interface discovered: addr=%u iface=%u",
                     event.address, event.interface_number);
            if (s_uac2_rx_handle || s_rx_handle) {
                ESP_LOGW(TAG, "Audio capture already active/open; ignoring additional UAC2 RX interface");
                continue;
            }
            const uac2_host_device_config_t config = {
                .addr = event.address,
                .iface_num = event.interface_number,
                .buffer_size = 8192,
                .buffer_threshold = 2048,
                .callback = uac2_device_callback,
                .callback_arg = nullptr,
            };
            uac2_host_device_handle_t handle = nullptr;
            esp_err_t err = uac2_host_device_open(&config, &handle);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "UAC2 capture interface open failed: %s", esp_err_to_name(err));
                update_ui_state("USB host ready", "UAC2 device found, interface open failed",
                                "See serial log", "No UAC2 stream started");
                continue;
            }
            s_uac2_rx_handle = handle;
            uac2_device_info_t info{};
            if (uac2_host_device_get_info(handle, &info) == ESP_OK) {
                ESP_LOGI(TAG, "UAC2 device descriptors parsed successfully");
            }
            uac2_host_dev_alt_param_t alt{};
            if (uac2_host_get_device_alt_param(handle, 1, &alt) == ESP_OK) {
                ESP_LOGI(TAG, "UAC2 alt 1: %u ch, %u-bit, %u-byte subslot, EP=0x%02X, max packet=%u",
                         alt.channels, alt.bit_resolution, alt.sub_slot_size,
                         alt.ep_addr, alt.ep_max_packet_size);
            } else {
                ESP_LOGW(TAG, "UAC2 alternate-setting details unavailable; trying requested format anyway");
            }
            const uac2_host_stream_config_t stream = {
                .channels = 2,
                .bit_resolution = 16,
                .sample_freq = 48000,
                .flags = 0,
            };
            err = uac2_host_device_start(handle, &stream);
            if (err == ESP_OK) {
                s_active_rate = 48000;
                s_averaging_initialized = false;
                s_uac_transfer_error_count = 0;
                ESP_LOGI(TAG, "UAC2 stereo 16-bit capture started at 48000 Hz");
                update_ui_state("USB host ready", "UAC2 AUDIO INPUT - STREAMING",
                                "Streaming: stereo, 16-bit, 48000 Hz",
                                "Xonar UAC2 capture feeding the I/Q panadapter");
            } else {
                ESP_LOGE(TAG, "UAC2 48 kHz stereo capture start failed: %s", esp_err_to_name(err));
                uac2_host_device_close(handle);
                s_uac2_rx_handle = nullptr;
                s_active_rate = 0;
                update_ui_state("USB host ready", "UAC2 capture stream failed",
                                "48 kHz / stereo / 16-bit", "See serial log for driver error");
            }
        } else if (event.kind == EventKind::Uac2TxConnected) {
            ESP_LOGI(TAG, "UAC2 playback interface discovered (not used by panadapter): addr=%u iface=%u",
                     event.address, event.interface_number);
        } else if (event.kind == EventKind::Uac2RxReady) {
            if (event.handle_uac2 && event.handle_uac2 == s_uac2_rx_handle && s_active_rate) {
                consume_pcm(event.handle_uac2);
            }
        } else if (event.kind == EventKind::Uac2TransferError ||
                   event.kind == EventKind::Uac2StreamError) {
            ++s_uac_transfer_error_count;
            if (s_uac_transfer_error_count == 1 || (s_uac_transfer_error_count % 10) == 0) {
                ESP_LOGW(TAG, "UAC2 stream error #%lu (%s)",
                         static_cast<unsigned long>(s_uac_transfer_error_count),
                         event.kind == EventKind::Uac2StreamError ? "stream" : "transfer");
            }
        } else if (event.kind == EventKind::Uac2DeviceDisconnected) {
            if (event.handle_uac2 && event.handle_uac2 == s_uac2_rx_handle) {
                ESP_LOGW(TAG, "UAC2 capture device disconnected");
                uac2_host_device_close(s_uac2_rx_handle);
                s_uac2_rx_handle = nullptr;
                s_active_rate = 0;
                s_averaging_initialized = false;
                update_ui_state("USB host ready", "UAC2 audio device disconnected",
                                "Format: waiting for USB audio device",
                                "Reconnect the Xonar to continue");
            }
        } else if (event.kind == EventKind::RxConnected) {

            if (s_rx_handle || s_uac2_rx_handle) {
                ESP_LOGW(TAG, "An audio RX interface is already open; ignoring additional UAC1 RX interface");
                continue;
            }

            const uac_host_device_config_t config = {
                .addr = event.address,
                .iface_num = event.interface_number,
                .buffer_size = 8192,
                .buffer_threshold = 2048,
                .callback = uac_device_callback,
                .callback_arg = nullptr,
            };

            uac_host_device_handle_t handle = nullptr;
            esp_err_t err = uac_host_device_open(&config, &handle);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Could not open UAC capture interface: %s", esp_err_to_name(err));
                update_ui_state("USB host ready", "Audio input found, but interface open failed",
                                "See serial log for error", "No audio stream has been started");
                continue;
            }

            s_rx_handle = handle;
            uac_host_dev_info_t info{};
            uac_host_dev_alt_param_t alt{};
            std::string format = "Format details unavailable";
            if (uac_host_get_device_info(handle, &info) == ESP_OK) {
                ESP_LOGD(TAG, "UAC input device VID=%04X PID=%04X",
                         info.VID, info.PID);
            }
            if (uac_host_get_device_alt_param(handle, 1, &alt) == ESP_OK) {
                format = describe_alt(alt);
                ESP_LOGD(TAG, "First capture alternate setting: %s", format.c_str());
            } else {
                ESP_LOGW(TAG, "Could not read first capture alternate setting");
            }

            // The UAC component patch in this revision expands the discrete-rate
            // array to 8 entries so all seven Syba rates can be inspected.
            // Try the highest useful rates first, then fall back gracefully.
            const uint32_t candidates[] = {48000, 44100, 32000, 22050,
                                           16000, 11025, 8000};
            bool started = false;
            for (uint32_t rate : candidates) {
                const uac_host_stream_config_t stream = {
                    .channels = 2,
                    .bit_resolution = 16,
                    .sample_freq = rate,
                    .flags = 0,
                };
                ESP_LOGD(TAG, "Attempting stereo capture at %lu Hz", (unsigned long)rate);
                err = uac_host_device_start(handle, &stream);
                if (err == ESP_OK) {
                    s_active_rate = rate;
                    s_averaging_initialized = false;
                    started = true;
                    break;
                }
                ESP_LOGW(TAG, "Capture start at %lu Hz failed: %s",
                         (unsigned long)rate, esp_err_to_name(err));
            }
            if (started) {
                char fmt[128];
                std::snprintf(fmt, sizeof(fmt), "Streaming: stereo, 16-bit, %lu Hz",
                              (unsigned long)s_active_rate);
                update_ui_state("USB host ready", "USB AUDIO INPUT - STREAMING", fmt,
                                "Live I/Q PCM levels and FFT below (no KX3 connected)");
            } else {
                update_ui_state("USB host ready", "USB AUDIO INPUT - STREAM FAILED", format,
                                "See serial log for sample-rate errors");
            }
        } else if (event.kind == EventKind::TxConnected) {
            ESP_LOGD(TAG, "USB audio playback interface found: addr=%u iface=%u",
                     event.address, event.interface_number);
            update_ui_state("USB host ready",
                            "USB audio input/output interface detected",
                            "Playback interface also present",
                            "Panadapter uses the capture (RX) interface; playback is not used");
        } else if (event.kind == EventKind::RxReady) {
            if (event.handle && event.handle == s_rx_handle && s_active_rate) {
                consume_pcm(event.handle);
            }
        } else if (event.kind == EventKind::TransferError) {
            ++s_uac_transfer_error_count;
        } else if (event.kind == EventKind::DeviceDisconnected) {
            if (event.handle && event.handle == s_rx_handle) {
                // Driver has already reported disconnection; close the handle.
                uac_host_device_close(s_rx_handle);
                s_rx_handle = nullptr;
                s_active_rate = 0;
                s_averaging_initialized = false;
                update_ui_state("USB host ready", "USB audio device disconnected",
                                "Format: waiting for USB audio device",
                                "Reconnect the Syba adapter to continue");
            }
        }
    }
}

void clear_spectrum_for_frequency_change() {
    // Discard buffered I/Q samples acquired around the tuning transition, but
    // retain the displayed spectrum so it can decay smoothly toward the new
    // frequency's live data instead of blanking the screen.
    {
        std::lock_guard<std::mutex> lock(s_iq_mutex);
        s_iq_write = 0;
        s_iq_count = 0;
        s_i_ring.fill(0.0f);
        s_q_ring.fill(0.0f);
    }
    // Keep the existing average initialized. The FFT update uses a short
    // transition time constant for the next few frames, then returns to the
    // user's configured averaging time.
    s_frequency_transition_frames = FREQUENCY_TRANSITION_FRAMES;

    // Preserve the current spectrum and waterfall. The spectrum trace will
    // converge quickly to the new frequency. Waterfall history is not cleared,
    // faded, or recolored; new rows continue to scroll in normally.
    if (s_tuning_cursor) lv_obj_add_flag(s_tuning_cursor, LV_OBJ_FLAG_HIDDEN);
}

lv_obj_t *make_label(lv_obj_t *parent, const char *text, const lv_font_t *font,
                     uint32_t color) {
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_width(label, LV_PCT(100));
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
    return label;
}


void apply_display_settings() {
    // The spectrum is intentionally frozen while Settings is open. Defer its
    // chart-range redraw until the next live FFT after the panel closes.
    if (!s_settings_open.load(std::memory_order_acquire) && s_spectrum_chart && s_spectrum_image) {
        render_spectrum_image(true);
    }
    if (s_averaging_initialized) {
        // Re-seed the smoother after a major averaging-time change so the
        // waterfall does not retain an unexpectedly long prior history.
        // The next FFT naturally converges using the newly selected time.
    }
    update_frequency_ruler();
}

void update_setting_value_labels() {
    for (auto &control : s_setting_controls) {
        if (!control.value_label) continue;
        char value[40]{};
        switch (control.id) {
        case SettingId::FrequencyOffset:
            std::snprintf(value, sizeof(value), "%+d Hz", s_settings.frequency_offset_hz); break;
        case SettingId::PanLow:
            std::snprintf(value, sizeof(value), "%d dBFS", s_settings.pan_low_db); break;
        case SettingId::PanHigh:
            std::snprintf(value, sizeof(value), "%d dBFS", s_settings.pan_high_db); break;
        case SettingId::WaterfallLow:
            std::snprintf(value, sizeof(value), "%d dBFS", s_settings.waterfall_low_db); break;
        case SettingId::WaterfallHigh:
            std::snprintf(value, sizeof(value), "%d dBFS", s_settings.waterfall_high_db); break;
        case SettingId::Averaging:
            std::snprintf(value, sizeof(value), "%.1f s", s_settings.averaging_seconds); break;
        case SettingId::DisplayWidth:
            std::snprintf(value, sizeof(value), "%d kHz", s_settings.display_width_khz); break;
        case SettingId::WaterfallSpeed:
            std::snprintf(value, sizeof(value), "%d lines/s", s_settings.waterfall_speed_hz); break;
        case SettingId::Uac2InputGain:
            std::snprintf(value, sizeof(value), "%+d dB", s_settings.uac2_input_gain_db); break;
        }
        lv_label_set_text(control.value_label, value);
    }
}

void setting_slider_event(lv_event_t *event) {
    auto *control = static_cast<SettingControl *>(lv_event_get_user_data(event));
    if (!control) return;
    const int value = lv_slider_get_value(
        static_cast<lv_obj_t *>(lv_event_get_target(event)));
    switch (control->id) {
    case SettingId::FrequencyOffset: s_settings.frequency_offset_hz = value * 100; break;
    case SettingId::PanLow:
        s_settings.pan_low_db = std::min(value, s_settings.pan_high_db - 5); break;
    case SettingId::PanHigh:
        s_settings.pan_high_db = std::max(value, s_settings.pan_low_db + 5); break;
    case SettingId::WaterfallLow:
        s_settings.waterfall_low_db = std::min(value, s_settings.waterfall_high_db - 5); break;
    case SettingId::WaterfallHigh:
        s_settings.waterfall_high_db = std::max(value, s_settings.waterfall_low_db + 5); break;
    case SettingId::Averaging: s_settings.averaging_seconds = value / 10.0f; break;
    case SettingId::DisplayWidth: s_settings.display_width_khz = value; break;
    case SettingId::WaterfallSpeed:
        s_settings.waterfall_speed_hz = std::clamp(value, 30, 60);
        break;
    case SettingId::Uac2InputGain:
        s_settings.uac2_input_gain_db = std::clamp(value, 0, 24);
        s_uac2_input_gain_db.store(s_settings.uac2_input_gain_db, std::memory_order_relaxed);
        ESP_LOGI(TAG, "UAC2 input gain set to %+d dB (UAC1 unaffected)",
                 s_settings.uac2_input_gain_db);
        break;
    }
    update_setting_value_labels();
    apply_display_settings();
}

void add_setting_slider(lv_obj_t *panel, const char *title, SettingId id,
                        int min_value, int max_value, int initial_value,
                        size_t control_index) {
    // Each setting has a fixed-height group. This prevents flex expansion
    // from pushing the final slider into the Done button at the bottom.
    lv_obj_t *group = lv_obj_create(panel);
    lv_obj_remove_style_all(group);
    lv_obj_set_width(group, LV_PCT(100));
    lv_obj_set_height(group, 62);
    lv_obj_set_flex_flow(group, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(group, 0, 0);
    lv_obj_set_style_pad_row(group, 1, 0);
    lv_obj_clear_flag(group, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *row = lv_obj_create(group);
    lv_obj_remove_style_all(row);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_height(row, 27);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *heading = lv_label_create(row);
    lv_label_set_text(heading, title);
    lv_obj_set_style_text_color(heading, lv_color_hex(0xF3F7FF), 0);
    lv_obj_set_style_text_font(heading, APP_FONT_14, 0);
    lv_obj_set_style_text_letter_space(heading, 1, 0);
    lv_obj_set_flex_grow(heading, 1);

    lv_obj_t *value = lv_label_create(row);
    lv_obj_set_style_text_color(value, lv_color_hex(0x16DCEB), 0);
    lv_obj_set_style_text_font(value, APP_FONT_14, 0);
    lv_obj_set_style_text_align(value, LV_TEXT_ALIGN_RIGHT, 0);

    lv_obj_t *slider = lv_slider_create(group);
    // Keep the slider's real layout/touch height at 40 px. Styling the MAIN
    // part to 6 px also shrinks its layout box in this LVGL build; that clips
    // the 18 px knob and can prevent reliable dragging. Keep MAIN full-height
    // but transparent, and draw the thin colored indicator within that area.
    lv_obj_set_size(slider, LV_PCT(88), 28);
    lv_obj_set_style_align(slider, LV_ALIGN_CENTER, 0);
    lv_obj_set_ext_click_area(slider, 10);
    lv_obj_set_style_pad_ver(slider, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(slider, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(slider, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_height(slider, 6, LV_PART_INDICATOR);
    lv_obj_set_style_radius(slider, LV_RADIUS_CIRCLE, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider, lv_color_hex(0x13D4E5), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_width(slider, 12, LV_PART_KNOB);
    lv_obj_set_style_height(slider, 12, LV_PART_KNOB);
    lv_obj_set_style_radius(slider, LV_RADIUS_CIRCLE, LV_PART_KNOB);
    lv_obj_set_style_bg_color(slider, lv_color_hex(0xFFD400), LV_PART_KNOB);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_width(slider, 1, LV_PART_KNOB);
    lv_obj_set_style_border_color(slider, lv_color_hex(0xFFF2A6), LV_PART_KNOB);
    lv_obj_set_style_shadow_color(slider, lv_color_hex(0x000000), LV_PART_KNOB);
    lv_obj_set_style_shadow_width(slider, 4, LV_PART_KNOB);
    lv_obj_set_style_shadow_opa(slider, LV_OPA_40, LV_PART_KNOB);
    lv_obj_add_flag(slider, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(slider, LV_OBJ_FLAG_SCROLLABLE);
    lv_slider_set_range(slider, min_value, max_value);
    lv_slider_set_value(slider, initial_value, LV_ANIM_OFF);

    s_setting_controls[control_index] = {id, value, slider};
    lv_obj_add_event_cb(slider, setting_slider_event, LV_EVENT_VALUE_CHANGED,
                        &s_setting_controls[control_index]);
}

void close_settings_panel() {
    save_persistent_audio_settings();
    if (s_settings_panel) lv_obj_add_flag(s_settings_panel, LV_OBJ_FLAG_HIDDEN);
    if (s_settings_backdrop) lv_obj_add_flag(s_settings_backdrop, LV_OBJ_FLAG_HIDDEN);
    // Discard the paused display average so the first resumed FFT starts from
    // current I/Q samples instead of blending across the time Settings was open.
    s_averaging_initialized = false;
    s_settings_open.store(false, std::memory_order_release);
}

void open_settings_panel() {
    s_settings_open.store(true, std::memory_order_release);
    if (s_settings_backdrop) lv_obj_clear_flag(s_settings_backdrop, LV_OBJ_FLAG_HIDDEN);
    if (s_settings_panel) {
        lv_obj_clear_flag(s_settings_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(s_settings_panel);
    }
}

// Keep the pressed visual feedback on screen before performing an action
// that changes the UI. A short one-shot LVGL timer gives the display a chance
// to render the brighter/enlarged state, even for a quick touchscreen tap.
constexpr uint32_t BUTTON_FEEDBACK_DELAY_MS = 220;
constexpr int BUTTON_NORMAL_SCALE = 256;
constexpr int BUTTON_PRESSED_SCALE = 282;
constexpr uint32_t BUTTON_NORMAL_COLOR = 0x29445C;
constexpr uint32_t BUTTON_PRESSED_COLOR = 0x35C9F0;

void restore_button_visual(lv_obj_t *button) {
    if (!button) return;
    lv_obj_set_style_bg_color(button, lv_color_hex(BUTTON_NORMAL_COLOR), 0);
    lv_obj_set_style_transform_scale(button, BUTTON_NORMAL_SCALE, 0);
}

void reset_baseline_action_timer(lv_timer_t *timer) {
    auto *button = static_cast<lv_obj_t *>(lv_timer_get_user_data(timer));
    restore_button_visual(button);
    lv_timer_delete(timer);

    // Re-run the central-10-kHz panadapter baseline calculation.
    s_reset_requested = true;
    if (s_reset_button_label) {
        lv_label_set_text(s_reset_button_label, "Resetting");
    }
    s_auto_low_levels_initialized = false;
    ESP_LOGD(TAG, "Baseline recalculation requested from Reset button");
}

void reset_baseline_button_event(lv_event_t *event) {
    if (lv_event_get_code(event) != LV_EVENT_PRESSED) return;
    auto *button = static_cast<lv_obj_t *>(lv_event_get_target(event));
    // Explicit visual state starts immediately on touch-down and remains
    // visible for the feedback interval before Reset begins.
    lv_obj_set_style_bg_color(button, lv_color_hex(BUTTON_PRESSED_COLOR), 0);
    lv_obj_set_style_transform_scale(button, BUTTON_PRESSED_SCALE, 0);
    lv_timer_t *timer = lv_timer_create(reset_baseline_action_timer,
                                       BUTTON_FEEDBACK_DELAY_MS, button);
    lv_timer_set_repeat_count(timer, 1);
}

void settings_button_action_timer(lv_timer_t *timer) {
    auto *button = static_cast<lv_obj_t *>(lv_timer_get_user_data(timer));
    restore_button_visual(button);
    lv_timer_delete(timer);
    open_settings_panel();
}

void settings_button_event(lv_event_t *event) {
    if (lv_event_get_code(event) != LV_EVENT_PRESSED) return;
    auto *button = static_cast<lv_obj_t *>(lv_event_get_target(event));
    // Show touch feedback before opening the Settings overlay, which otherwise
    // hides the button before LVGL can render its pressed-state styling.
    lv_obj_set_style_bg_color(button, lv_color_hex(BUTTON_PRESSED_COLOR), 0);
    lv_obj_set_style_transform_scale(button, BUTTON_PRESSED_SCALE, 0);
    lv_timer_t *timer = lv_timer_create(settings_button_action_timer,
                                       BUTTON_FEEDBACK_DELAY_MS, button);
    lv_timer_set_repeat_count(timer, 1);
}

void settings_backdrop_tap_event(lv_event_t *event) {
    (void)event;
    close_settings_panel();
}

void settings_done_event(lv_event_t *event) {
    (void)event;
    close_settings_panel();
}

void create_settings_panel(lv_obj_t *screen) {
    // Transparent modal backdrop catches taps outside the panel. This lets a
    // tap anywhere on the underlying panadapter close Settings without
    // accidentally sending that tap to a chart or other screen control.
    s_settings_backdrop = lv_obj_create(screen);
    lv_obj_remove_style_all(s_settings_backdrop);
    lv_obj_add_flag(s_settings_backdrop, LV_OBJ_FLAG_FLOATING);
    lv_obj_add_flag(s_settings_backdrop, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_settings_backdrop, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(s_settings_backdrop, LV_PCT(100), LV_PCT(100));
    lv_obj_align(s_settings_backdrop, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(s_settings_backdrop, LV_OPA_TRANSP, 0);
    // Dismiss the panel on touch-down rather than waiting for finger release.
    lv_obj_add_event_cb(s_settings_backdrop, settings_backdrop_tap_event,
                        LV_EVENT_PRESSED, nullptr);
    lv_obj_add_flag(s_settings_backdrop, LV_OBJ_FLAG_HIDDEN);

    s_settings_panel = lv_obj_create(screen);
    // Floating and centered over the main display. 75% width leaves a useful
    // margin on both sides while providing substantially more slider space.
    lv_obj_add_flag(s_settings_panel, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_size(s_settings_panel, LV_PCT(75), LV_PCT(98));
    lv_obj_align(s_settings_panel, LV_ALIGN_CENTER, 0, 0);
    // Deep navy gradient, bright blue outline, and soft shadow to match the
    // approved settings mockup rather than the former flat dialog.
    lv_obj_set_style_bg_color(s_settings_panel, lv_color_hex(0x071A35), 0);
    lv_obj_set_style_bg_grad_color(s_settings_panel, lv_color_hex(0x142F52), 0);
    lv_obj_set_style_bg_grad_dir(s_settings_panel, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(s_settings_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(s_settings_panel, lv_color_hex(0x78BFFF), 0);
    lv_obj_set_style_border_width(s_settings_panel, 3, 0);
    lv_obj_set_style_radius(s_settings_panel, 20, 0);
    lv_obj_set_style_shadow_color(s_settings_panel, lv_color_hex(0x000814), 0);
    lv_obj_set_style_shadow_width(s_settings_panel, 28, 0);
    lv_obj_set_style_shadow_opa(s_settings_panel, LV_OPA_50, 0);
    lv_obj_set_style_shadow_ofs_y(s_settings_panel, 8, 0);
    lv_obj_set_style_pad_hor(s_settings_panel, 34, 0);
    lv_obj_set_style_pad_ver(s_settings_panel, 16, 0);
    lv_obj_set_style_pad_row(s_settings_panel, 4, 0);
    lv_obj_set_flex_flow(s_settings_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_settings_panel, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    // Fixed, compact control groups leave a dedicated footer for Done.
    // Keep the panel non-scrollable so slider gestures remain unambiguous.
    lv_obj_clear_flag(s_settings_panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(s_settings_panel);
    lv_label_set_text(title, "DISPLAY & AUDIO SETTINGS");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFD400), 0);
    lv_obj_set_style_text_font(title, APP_FONT_28, 0);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(title, LV_PCT(100));
    lv_obj_set_style_text_letter_space(title, 2, 0);
    lv_obj_set_style_pad_bottom(title, 0, 0);

    lv_obj_t *subtitle = lv_label_create(s_settings_panel);
    lv_label_set_text(subtitle, "SIGNAL LEVELS  |  DISPLAY  |  AUDIO INPUT");
    lv_obj_set_style_text_color(subtitle, lv_color_hex(0x91B6D8), 0);
    lv_obj_set_style_text_font(subtitle, APP_FONT_14, 0);
    lv_obj_set_style_text_letter_space(subtitle, 1, 0);
    lv_obj_set_width(subtitle, LV_PCT(100));
    lv_obj_set_style_text_align(subtitle, LV_TEXT_ALIGN_CENTER, 0);

    add_setting_slider(s_settings_panel, "PANADAPTER LOW LEVEL", SettingId::PanLow,
                       -120, -20, s_settings.pan_low_db, 0);
    add_setting_slider(s_settings_panel, "PANADAPTER HIGH LEVEL", SettingId::PanHigh,
                       -80, 0, s_settings.pan_high_db, 1);
    add_setting_slider(s_settings_panel, "WATERFALL LOW LEVEL", SettingId::WaterfallLow,
                       -120, -20, s_settings.waterfall_low_db, 2);
    add_setting_slider(s_settings_panel, "WATERFALL HIGH LEVEL", SettingId::WaterfallHigh,
                       -80, 0, s_settings.waterfall_high_db, 3);
    add_setting_slider(s_settings_panel, "AVERAGING TIME", SettingId::Averaging,
                       0, 30, 5, 4);
    add_setting_slider(s_settings_panel, "FREQUENCY DISPLAY WIDTH", SettingId::DisplayWidth,
                       2, 48, s_settings.display_width_khz, 5);
    add_setting_slider(s_settings_panel, "WATERFALL SPEED", SettingId::WaterfallSpeed,
                       30, 60, s_settings.waterfall_speed_hz, 6);
    add_setting_slider(s_settings_panel, "XONAR / UAC2 INPUT GAIN", SettingId::Uac2InputGain,
                       0, 24, s_settings.uac2_input_gain_db, 7);

    lv_obj_t *done = lv_button_create(s_settings_panel);
    lv_obj_set_width(done, LV_PCT(100));
    lv_obj_set_height(done, 52);
    lv_obj_set_style_margin_top(done, 5, 0);
    lv_obj_set_style_bg_color(done, lv_color_hex(0x0879E8), 0);
    lv_obj_set_style_bg_grad_color(done, lv_color_hex(0x0054C8), 0);
    lv_obj_set_style_bg_grad_dir(done, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_color(done, lv_color_hex(0x19A7FF), 0);
    lv_obj_set_style_border_width(done, 2, 0);
    lv_obj_set_style_radius(done, 16, 0);
    lv_obj_set_style_shadow_color(done, lv_color_hex(0x00152F), 0);
    lv_obj_set_style_shadow_width(done, 8, 0);
    lv_obj_set_style_shadow_opa(done, LV_OPA_50, 0);
    // Apply the Done action immediately when the user touches the button.
    lv_obj_add_event_cb(done, settings_done_event, LV_EVENT_PRESSED, nullptr);
    lv_obj_t *done_label = lv_label_create(done);
    lv_label_set_text(done_label, "DONE");
    lv_obj_set_style_text_color(done_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(done_label, APP_FONT_20, 0);
    lv_obj_center(done_label);

    update_setting_value_labels();
    apply_display_settings();
    lv_obj_add_flag(s_settings_panel, LV_OBJ_FLAG_HIDDEN);
}


void create_screen() {
    lv_obj_t *screen = lv_screen_active();
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(screen, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x101820), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(screen, 14, 0);
    lv_obj_set_style_pad_row(screen, 5, 0);
    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(screen, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);

    lv_obj_t *top_bar = lv_obj_create(screen);
    lv_obj_remove_style_all(top_bar);
    lv_obj_set_width(top_bar, LV_PCT(100));
    lv_obj_set_height(top_bar, 68);
    lv_obj_set_style_bg_color(top_bar, lv_color_hex(0x101820), 0);
    lv_obj_set_style_bg_opa(top_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_hor(top_bar, 8, 0);
    lv_obj_set_flex_flow(top_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(top_bar, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    s_top_band_label = lv_dropdown_create(top_bar);
    lv_dropdown_set_options(s_top_band_label, kBandOptions);
    lv_dropdown_set_selected(s_top_band_label, 5);
    lv_obj_set_width(s_top_band_label, 118);
    lv_obj_set_height(s_top_band_label, 52);
    lv_obj_set_style_text_font(s_top_band_label, APP_FONT_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_top_band_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_top_band_label, lv_color_hex(0x29445C), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_top_band_label, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(s_top_band_label, 6, LV_PART_MAIN);
    lv_obj_add_event_cb(s_top_band_label, radio_dropdown_event, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_add_event_cb(s_top_band_label, radio_dropdown_popup_style_event, LV_EVENT_CLICKED, nullptr);

    // Reset repeats the startup panadapter baseline calculation.
    lv_obj_t *reset_button = lv_button_create(top_bar);
    lv_obj_set_size(reset_button, 120, 52);
    // Enlarge the touch target around the existing visible button.
    lv_obj_set_ext_click_area(reset_button, 8);
    lv_obj_set_style_bg_color(reset_button, lv_color_hex(0x29445C), 0);
    // Press feedback is applied explicitly on LV_EVENT_PRESSED and restored
    // after the short visual-feedback interval before the action executes.
    lv_obj_set_style_bg_opa(reset_button, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(reset_button, 6, 0);
    // Run Reset on touch-down for a quicker-feeling response.
    lv_obj_add_event_cb(reset_button, reset_baseline_button_event,
                        LV_EVENT_PRESSED, nullptr);
    s_reset_button_label = lv_label_create(reset_button);
    lv_label_set_text(s_reset_button_label, "Reset");
    lv_obj_set_style_text_font(s_reset_button_label, APP_FONT_28, 0);
    lv_obj_set_style_text_color(s_reset_button_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(s_reset_button_label);

    s_top_mode_label = lv_dropdown_create(top_bar);
    lv_dropdown_set_options(s_top_mode_label, kModeOptions);
    lv_dropdown_set_selected(s_top_mode_label, 0);
    lv_obj_set_width(s_top_mode_label, 142);
    lv_obj_set_height(s_top_mode_label, 52);
    lv_obj_set_style_text_font(s_top_mode_label, APP_FONT_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_top_mode_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_top_mode_label, lv_color_hex(0x29445C), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_top_mode_label, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(s_top_mode_label, 6, LV_PART_MAIN);
    lv_obj_add_event_cb(s_top_mode_label, radio_dropdown_event, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_add_event_cb(s_top_mode_label, radio_dropdown_popup_style_event, LV_EVENT_CLICKED, nullptr);

    // Settings sits between Mode and Frequency; the old invisible corner
    // long-press shortcut has been removed.
    lv_obj_t *settings_button = lv_button_create(top_bar);
    lv_obj_set_size(settings_button, 140, 52);
    // Enlarge the touch target around the existing visible button.
    lv_obj_set_ext_click_area(settings_button, 8);
    lv_obj_set_style_bg_color(settings_button, lv_color_hex(0x29445C), 0);
    // Give immediate visual confirmation while the Settings button is touched.
    // Press feedback is applied explicitly on LV_EVENT_PRESSED and restored
    // after the short visual-feedback interval before Settings opens.
    lv_obj_set_style_bg_opa(settings_button, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(settings_button, 6, 0);
    // Open Settings on touch-down rather than waiting for finger release.
    lv_obj_add_event_cb(settings_button, settings_button_event,
                        LV_EVENT_PRESSED, nullptr);
    lv_obj_t *settings_button_label = lv_label_create(settings_button);
    lv_label_set_text(settings_button_label, "Settings");
    lv_obj_set_style_text_font(settings_button_label, APP_FONT_28, 0);
    lv_obj_set_style_text_color(settings_button_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(settings_button_label);

    // Frequency and battery status share a compact right-aligned vertical area.
    // The frequency precision is reduced from 10 Hz to 1 kHz to make room.
    lv_obj_t *frequency_status = lv_obj_create(top_bar);
    lv_obj_remove_style_all(frequency_status);
    lv_obj_set_size(frequency_status, 238, 60);
    lv_obj_set_style_pad_all(frequency_status, 0, 0);
    lv_obj_set_flex_flow(frequency_status, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(frequency_status, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(frequency_status, LV_OBJ_FLAG_SCROLLABLE);

    s_top_frequency_label = lv_label_create(frequency_status);
    lv_label_set_text(s_top_frequency_label, "--.--- MHz");
    lv_obj_set_style_text_font(s_top_frequency_label, APP_FONT_28, 0);
    lv_obj_set_style_text_color(s_top_frequency_label, lv_color_hex(0xFFD400), 0);
    lv_obj_set_style_text_align(s_top_frequency_label, LV_TEXT_ALIGN_RIGHT, 0);

    // Use a separately drawn bolt icon so charging indication does not depend
    // on any font's symbol-glyph coverage. It is explicitly shown/hidden each
    // battery refresh.
    lv_obj_t *battery_row = lv_obj_create(frequency_status);
    lv_obj_remove_style_all(battery_row);
    lv_obj_set_width(battery_row, LV_SIZE_CONTENT);
    lv_obj_set_height(battery_row, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(battery_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(battery_row, LV_FLEX_ALIGN_END,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(battery_row, 4, 0);
    lv_obj_clear_flag(battery_row, LV_OBJ_FLAG_SCROLLABLE);

    s_top_charging_icon = lv_obj_create(battery_row);
    lv_obj_remove_style_all(s_top_charging_icon);
    lv_obj_set_size(s_top_charging_icon, 18, 20);
    lv_obj_clear_flag(s_top_charging_icon, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_top_charging_icon, charging_bolt_draw_event,
                        LV_EVENT_DRAW_MAIN, nullptr);
    lv_obj_add_flag(s_top_charging_icon, LV_OBJ_FLAG_HIDDEN);

    s_top_battery_label = lv_label_create(battery_row);
    lv_label_set_text(s_top_battery_label, "BAT --.- V");
    lv_obj_set_style_text_font(s_top_battery_label, APP_FONT_20, 0);
    lv_obj_set_style_text_color(s_top_battery_label, lv_color_hex(0x7DDB8A), 0);
    lv_obj_set_style_text_align(s_top_battery_label, LV_TEXT_ALIGN_RIGHT, 0);

    s_spectrum_peak_label = nullptr;
    // Replace the LVGL chart widget with a lightweight generic container.
    // The spectrum trace is rasterized into a persistent RGB565 image buffer.
    s_spectrum_chart = lv_obj_create(screen);
    lv_obj_remove_style_all(s_spectrum_chart);
    lv_obj_set_width(s_spectrum_chart, LV_PCT(100));
    lv_obj_set_height(s_spectrum_chart, PANADAPTER_HEIGHT);
    lv_obj_set_style_bg_color(s_spectrum_chart, lv_color_hex(0x081018), 0);
    lv_obj_set_style_bg_opa(s_spectrum_chart, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_spectrum_chart, 1, 0);
    lv_obj_set_style_border_color(s_spectrum_chart, lv_color_hex(0x355064), 0);
    lv_obj_set_style_pad_all(s_spectrum_chart, 0, 0);
    lv_obj_clear_flag(s_spectrum_chart, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_spectrum_chart, LV_OBJ_FLAG_CLICKABLE);

    lv_display_t *spectrum_display = lv_display_get_default();
    const uint32_t spectrum_display_width = spectrum_display
        ? lv_display_get_horizontal_resolution(spectrum_display) : 1280;
    s_spectrum_width = static_cast<uint16_t>(
        spectrum_display_width > 28 ? spectrum_display_width - 28 : spectrum_display_width);
    const size_t spectrum_bytes = static_cast<size_t>(s_spectrum_width) * PANADAPTER_HEIGHT * sizeof(uint16_t);
    s_spectrum_pixels = static_cast<uint16_t *>(
        heap_caps_malloc(spectrum_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (s_spectrum_pixels) {
        std::fill(s_spectrum_pixels, s_spectrum_pixels + static_cast<size_t>(s_spectrum_width) * PANADAPTER_HEIGHT,
                  rgb565(8, 16, 24));
        s_spectrum_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
        s_spectrum_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
        s_spectrum_dsc.header.w = s_spectrum_width;
        s_spectrum_dsc.header.h = PANADAPTER_HEIGHT;
        s_spectrum_dsc.header.stride = s_spectrum_width * sizeof(uint16_t);
        s_spectrum_dsc.data_size = spectrum_bytes;
        s_spectrum_dsc.data = reinterpret_cast<const uint8_t *>(s_spectrum_pixels);
        s_spectrum_image = lv_image_create(s_spectrum_chart);
        lv_image_set_src(s_spectrum_image, &s_spectrum_dsc);
        lv_obj_set_size(s_spectrum_image, LV_PCT(100), PANADAPTER_HEIGHT);
        lv_obj_clear_flag(s_spectrum_image, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_scrollbar_mode(s_spectrum_image, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_pos(s_spectrum_image, 0, 0);
        lv_obj_add_flag(s_spectrum_image, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(s_spectrum_image, spectrum_touch_event, LV_EVENT_PRESSED, nullptr);
        lv_obj_add_event_cb(s_spectrum_image, spectrum_touch_event, LV_EVENT_PRESSING, nullptr);
        lv_obj_add_event_cb(s_spectrum_image, spectrum_touch_event, LV_EVENT_RELEASED, nullptr);
    } else {
        ESP_LOGE(TAG, "Could not allocate spectrum image buffer in PSRAM");
    }
    lv_obj_add_flag(s_spectrum_chart, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_spectrum_chart, spectrum_touch_event, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(s_spectrum_chart, spectrum_touch_event, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(s_spectrum_chart, spectrum_touch_event, LV_EVENT_RELEASED, nullptr);
    // Parent the tuning cursor to the screen so it is not clipped to the chart.
    s_tuning_cursor = lv_obj_create(screen);
    lv_obj_remove_style_all(s_tuning_cursor);
    lv_obj_set_size(s_tuning_cursor, 3, 1);
    lv_obj_set_style_bg_color(s_tuning_cursor, lv_color_hex(0xFF3030), 0);
    lv_obj_set_style_bg_opa(s_tuning_cursor, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_tuning_cursor, 0, 0);
    lv_obj_clear_flag(s_tuning_cursor, LV_OBJ_FLAG_CLICKABLE);
    // This is a visual overlay, not a flex-layout item.
    lv_obj_add_flag(s_tuning_cursor, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_add_flag(s_tuning_cursor, LV_OBJ_FLAG_HIDDEN);

    // Vertical reference line: 0 kHz is the I/Q center frequency.
    // The actual KX3 dial frequency is not yet available without CAT.
    s_center_marker = lv_obj_create(s_spectrum_chart);
    lv_obj_remove_style_all(s_center_marker);
    lv_obj_set_size(s_center_marker, 2, PANADAPTER_HEIGHT);
    lv_obj_align(s_center_marker, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(s_center_marker, lv_color_hex(0xFFD166), 0);
    // Keep the center reference visible but translucent so signals remain
    // easy to see beneath it.
    lv_obj_set_style_bg_opa(s_center_marker, LV_OPA_40, 0);
    lv_obj_set_style_border_width(s_center_marker, 0, 0);
    lv_obj_set_style_radius(s_center_marker, 0, 0);
    lv_obj_clear_flag(s_center_marker, LV_OBJ_FLAG_CLICKABLE);

    // Initialize spectrum image with the display floor.
    s_spectrum_chart_values.fill(static_cast<lv_coord_t>(s_settings.pan_low_db));
    render_spectrum_image(true);
    // A dedicated ruler sits immediately below the spectrum. Each numeric
    // label is positioned at its true horizontal frequency coordinate rather
    // than approximated with spaces in one long text string.
    s_spectrum_axis_label = lv_obj_create(screen);
    lv_obj_remove_style_all(s_spectrum_axis_label);
    lv_obj_set_width(s_spectrum_axis_label, LV_PCT(100));
    lv_obj_set_height(s_spectrum_axis_label, 24);
    lv_obj_clear_flag(s_spectrum_axis_label, LV_OBJ_FLAG_SCROLLABLE);
    for (size_t i = 0; i < FREQUENCY_TICK_COUNT; ++i) {
        s_frequency_labels[i] = lv_label_create(s_spectrum_axis_label);
        lv_label_set_text(s_frequency_labels[i], "");
        lv_obj_set_width(s_frequency_labels[i], 48);
        lv_obj_set_style_text_align(s_frequency_labels[i], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(s_frequency_labels[i], APP_FONT_14, 0);
        lv_obj_set_style_text_color(s_frequency_labels[i], lv_color_hex(0xAAB7C4), 0);
        lv_obj_set_height(s_frequency_labels[i], 20);
        s_frequency_grid_lines[i] = lv_obj_create(s_spectrum_chart);
        lv_obj_remove_style_all(s_frequency_grid_lines[i]);
        lv_obj_set_size(s_frequency_grid_lines[i], 1, PANADAPTER_HEIGHT);
        lv_obj_set_style_bg_color(s_frequency_grid_lines[i], lv_color_hex(0x355064), 0);
        lv_obj_set_style_bg_opa(s_frequency_grid_lines[i], LV_OPA_40, 0);
        lv_obj_clear_flag(s_frequency_grid_lines[i], LV_OBJ_FLAG_CLICKABLE);
    }
    update_frequency_ruler();

    // Waterfall image: mirrored RGB565 ring buffer keeps its visible history
    // contiguous and avoids moving the full history for each new row.
    lv_display_t *active_display = lv_display_get_default();
    const uint32_t display_width = active_display
        ? lv_display_get_horizontal_resolution(active_display) : 1280;
    s_waterfall_width = static_cast<uint16_t>(display_width > 28 ? display_width - 28 : display_width);
    const size_t waterfall_bytes = static_cast<size_t>(s_waterfall_width) * WATERFALL_HEIGHT * sizeof(uint16_t);
    s_waterfall_pixels = static_cast<uint16_t *>(heap_caps_malloc(
        waterfall_bytes * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (s_waterfall_pixels) {
        s_waterfall_head = 0;
        std::fill(s_waterfall_pixels, s_waterfall_pixels +
            static_cast<size_t>(s_waterfall_width) * WATERFALL_HEIGHT * 2, rgb565(0, 7, 28));
        s_waterfall_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
        s_waterfall_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
        s_waterfall_dsc.header.w = s_waterfall_width;
        s_waterfall_dsc.header.h = WATERFALL_HEIGHT;
        s_waterfall_dsc.header.stride = s_waterfall_width * sizeof(uint16_t);
        s_waterfall_dsc.data_size = waterfall_bytes;
        s_waterfall_dsc.data = reinterpret_cast<const uint8_t *>(s_waterfall_pixels);
        s_waterfall_image = lv_image_create(screen);
        lv_image_set_src(s_waterfall_image, &s_waterfall_dsc);
        lv_obj_clear_flag(s_waterfall_image, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_scrollbar_mode(s_waterfall_image, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_width(s_waterfall_image, LV_PCT(100));
        lv_obj_set_height(s_waterfall_image, WATERFALL_HEIGHT);
        lv_obj_add_flag(s_waterfall_image, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(s_waterfall_image, spectrum_touch_event, LV_EVENT_PRESSED, nullptr);
        lv_obj_add_event_cb(s_waterfall_image, spectrum_touch_event, LV_EVENT_PRESSING, nullptr);
        lv_obj_add_event_cb(s_waterfall_image, spectrum_touch_event, LV_EVENT_RELEASED, nullptr);
        lv_obj_set_style_border_width(s_waterfall_image, 1, 0);
        lv_obj_set_style_border_color(s_waterfall_image, lv_color_hex(0x355064), 0);
    } else {
        ESP_LOGE(TAG, "Could not allocate waterfall image buffer in PSRAM");
        make_label(screen, "Waterfall unavailable: PSRAM allocation failed", APP_FONT_14, 0xFF8080);
    }

    lv_obj_update_layout(screen);
    lv_area_t chart_area{};
    lv_area_t waterfall_area{};
    lv_obj_get_coords(s_spectrum_chart, &chart_area);
    if (s_waterfall_image) lv_obj_get_coords(s_waterfall_image, &waterfall_area);
    else waterfall_area = chart_area;
    const int cursor_height = std::max(1, static_cast<int>(waterfall_area.y2 - chart_area.y1 + 1));
    lv_obj_set_height(s_tuning_cursor, cursor_height);
    lv_obj_set_pos(s_tuning_cursor, lv_obj_get_x(s_spectrum_chart) - 1,
                   lv_obj_get_y(s_spectrum_chart));


    create_settings_panel(screen);
}


} // namespace

extern "C" void app_main(void) {
    ESP_LOGD(TAG, "Starting KX3 Tab5 Panadapter");

    load_persistent_audio_settings();

    auto &tab5 = espp::M5StackTab5::get();
    if (!tab5.initialize_io_expanders()) {
        ESP_LOGE(TAG, "Failed to initialize Tab5 IO expanders");
        return;
    }
    if (!tab5.initialize_lcd()) {
        ESP_LOGE(TAG, "Failed to initialize Tab5 LCD");
        return;
    }
    const size_t pixel_buffer_size = tab5.display_width() * tab5.display_height();
    if (!tab5.initialize_display(pixel_buffer_size)) {
        ESP_LOGE(TAG, "Failed to initialize Tab5 LVGL display");
        return;
    }

    // The Tab5 panel is natively 720x1280. Rotate LVGL by 90 degrees to use
    // the full 1280x720 landscape workspace. The full-frame draw buffer above
    // also gives LVGL enough room for its rotation path.
    lv_display_t *display = lv_display_get_default();
    if (display) {
        lv_display_set_rotation(display, LV_DISPLAY_ROTATION_90);
        ESP_LOGD(TAG, "Display set to landscape orientation (%u x %u)",
                 static_cast<unsigned>(tab5.rotated_display_width()),
                 static_cast<unsigned>(tab5.rotated_display_height()));
    } else {
        ESP_LOGE(TAG, "LVGL default display unavailable; cannot set landscape orientation");
    }

    if (!tab5.initialize_touch()) {
        ESP_LOGW(TAG, "Touch initialization failed; display remains available");
    }

    // Tab5's onboard INA226 monitors the battery rail on the internal I2C bus.
    // The BSP owns bus initialization and INA226 calibration.
    s_battery_monitor_initialized = tab5.initialize_battery_monitoring();
    if (!s_battery_monitor_initialized) {
        ESP_LOGW(TAG, "Battery monitor initialization failed; voltage display disabled");
    } else {
        // The Tab5 charger is disabled by default in the BSP. Explicitly enable
        // charging only when a battery is detected, to avoid the charger retry
        // behavior (and speaker pops) with an empty battery bay.
        const auto battery_at_startup = tab5.read_battery_status();
        if (battery_at_startup.is_present) {
            tab5.enable_battery_charging(true);
            ESP_LOGI(TAG, "Battery detected (%.2f V); battery charging enabled",
                     battery_at_startup.voltage_v);
        } else {
            tab5.enable_battery_charging(false);
            ESP_LOGW(TAG, "No battery detected; battery charging disabled");
        }
    }
    tab5.brightness(75.0f);

    // Match the working Tab5 HID example: finish I/O expander, LCD,
    // display, and touch initialization before changing USB-A VBUS.
    if (!tab5.set_usb_a_power(false)) {
        ESP_LOGW(TAG, "Could not switch USB-A power off before host startup");
    }
    create_screen();

    s_event_queue = xQueueCreate(32, sizeof(UacEvent));
    if (!s_event_queue) {
        ESP_LOGE(TAG, "Failed to create UAC event queue");
        return;
    }

    if (xTaskCreatePinnedToCore(uac_event_task, "uac_event_task", 6144, nullptr,
                                5, nullptr, 1) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UAC event task");
        return;
    }

    s_ftdi_event_queue = xQueueCreate(8, sizeof(uint8_t));
    if (!s_ftdi_event_queue) {
        ESP_LOGE(TAG, "Failed to create FTDI notification queue");
        return;
    }

    s_tune_queue = xQueueCreate(1, sizeof(uint64_t));
    if (!s_tune_queue) {
        ESP_LOGE(TAG, "Failed to create KX3 tune request queue");
        return;
    }

    s_radio_control_queue = xQueueCreate(8, sizeof(RadioControlRequest));
    if (!s_radio_control_queue) {
        ESP_LOGE(TAG, "Failed to create KX3 radio control queue");
        return;
    }

    s_cat_rx_queue = xQueueCreate(512, sizeof(uint8_t));
    if (!s_cat_rx_queue) {
        ESP_LOGE(TAG, "Failed to create KX3 CAT RX queue");
        return;
    }

    // Reuse the exact Tab5 USB host wrapper that works in the supplied HID
    // example. The USB-A 5 V rail remains off until the UAC driver is installed.
    espp::UsbHost host({
        .root_port_power_on_delay = std::chrono::milliseconds(CONFIG_KX3_USB_HOST_START_DELAY_MS),
        .dispatch_task_stack_size = 16 * 1024,
        .full_speed_only = true,
        .log_level = espp::Logger::Verbosity::INFO,
    });

    std::error_code ec;
    if (!host.initialize(ec)) {
        ESP_LOGE(TAG, "USB host initialization failed: %s", ec.message().c_str());
        update_ui_state("USB host initialization failed", "USB audio unavailable",
                        "See serial monitor", ec.message());
    } else {
        const uac_host_driver_config_t uac_config = {
            .create_background_task = true,
            .task_priority = 5,
            .stack_size = 6144,
            .core_id = 1,
            .callback = uac_driver_callback,
            .callback_arg = nullptr,
        };

        esp_err_t err = uac_host_install(&uac_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "UAC driver installation failed: %s", esp_err_to_name(err));
            update_ui_state("USB host ready", "UAC driver installation failed",
                            "See serial monitor", esp_err_to_name(err));
        } else {
            const uac2_host_driver_config_t uac2_config = {
                .create_background_task = true,
                .task_priority = 6,
                .stack_size = 8192,
                .core_id = 1,
                .callback = uac2_driver_callback,
                .callback_arg = nullptr,
            };
            err = uac2_host_install(&uac2_config);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "UAC2 driver installation failed: %s", esp_err_to_name(err));
                update_ui_state("USB host ready", "UAC1 installed; UAC2 driver failed",
                                "Syba remains supported", esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG, "UAC2 host driver installed; UAC1 and UAC2 discovery active");
            }
            const cdc_acm_host_driver_config_t cdc_config = {
                .driver_task_stack_size = 6144,
                .driver_task_priority = 5,
                .xCoreID = 1,
                .new_dev_cb = ftdi_new_device_callback,
            };
            err = cdc_acm_host_install(&cdc_config);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "CDC-ACM host installation for KXUSB failed: %s",
                         esp_err_to_name(err));
                update_ui_state("USB host ready", "UAC audio driver installed",
                                "KXUSB CAT driver installation failed",
                                esp_err_to_name(err));
            } else {
                if (xTaskCreatePinnedToCore(ftdi_device_task, "ftdi_device_task",
                                            6144, nullptr, 5, nullptr, 1) != pdPASS) {
                    ESP_LOGE(TAG, "Failed to create FTDI device task");
                } else {
                    ESP_LOGD(TAG, "FTDI VCP host installed; KXUSB detection enabled");
                }
            }

            vTaskDelay(pdMS_TO_TICKS(100));
            if (!tab5.set_usb_a_power(true)) {
                ESP_LOGE(TAG, "Failed to enable Tab5 USB-A 5 V");
                update_ui_state("USB host ready", "Could not enable USB-A power",
                                "Check Tab5 IO expander", "USB-A VBUS remains unavailable");
            } else {
                update_ui_state("USB host ready", "Waiting for USB audio input",
                                "Sample rate / channels will appear when detected",
                                "Connect the Syba adapter to the USB-A port");
            }
        }
    }

    UiState shown{};
    while (true) {
        UiState current;
        {
            std::lock_guard<std::mutex> lock(s_ui_mutex);
            current = s_ui;
        }
        CatDisplayState radio_state{};
        {
            const TickType_t now = xTaskGetTickCount();
            { std::lock_guard<std::mutex> lock(s_cat_state_mutex); radio_state = s_cat_state; }
            // Keep the local preview aligned while CAT is in flight. The KX3
            // can quantize the requested frequency (its 1 Hz digit is ignored
            // unless FINE tuning is enabled), so exact/5 Hz target matching can
            // leave the preview offset applied after the radio has moved. Treat
            // any confirmed VFO change from the pre-command frequency as the
            // acknowledgement, then use the radio's actual frequency as truth.
            // If the command produces no change or no response, discard the
            // temporary preview after a short timeout rather than leaving a
            // stale offset that can make the panadapter appear to tune twice.
            if (s_pan_tune_pending && radio_state.vfo_a_valid &&
                !s_touch_gesture_active) {
                const bool radio_moved = radio_state.vfo_a_hz != s_pan_tune_start_hz;
                const bool request_was_noop = s_pan_tune_target_hz == s_pan_tune_start_hz;
                const bool timed_out = static_cast<uint32_t>(lv_tick_get() - s_pan_tune_request_tick) >=
                                       PAN_TUNE_RECONCILE_TIMEOUT_MS;
                if (radio_moved || request_was_noop || timed_out) {
                    if (timed_out && !radio_moved && !request_was_noop) {
                        ESP_LOGW(TAG, "Touch tune reconciliation timed out; reverting preview to KX3 VFO");
                    }
                    s_pan_offset_hz = 0;
                    s_pan_tune_pending = false;
                    update_frequency_ruler();
                }
            }
            if (radio_state.vfo_a_valid) {
                char freq[48];
                std::snprintf(freq, sizeof(freq), "%.3f MHz",
                              static_cast<double>(radio_state.vfo_a_hz) / 1000000.0);
                if (s_top_frequency_label) lv_label_set_text(s_top_frequency_label, freq);

                const uint8_t actual_band_index = static_cast<uint8_t>(band_dropdown_index(radio_state.vfo_a_hz));
                if (s_band_selection_pending) {
                    if (actual_band_index == s_pending_band_index) {
                        s_band_selection_pending = false;
                    } else if ((now - s_band_selection_tick) > RADIO_SELECTION_CONFIRM_TIMEOUT) {
                        s_band_selection_pending = false;
                    }
                }
                if (!s_band_selection_pending && s_top_band_label) {
                    lv_dropdown_set_selected(s_top_band_label, actual_band_index);
                }
            }
            if (radio_state.mode_valid && s_top_mode_label) {
                const uint8_t actual_mode_index = static_cast<uint8_t>(mode_dropdown_index(radio_state.mode_code));
                if (s_mode_selection_pending) {
                    if (actual_mode_index == s_pending_mode_index) {
                        s_mode_selection_pending = false;
                    } else if ((now - s_mode_selection_tick) > RADIO_SELECTION_CONFIRM_TIMEOUT) {
                        s_mode_selection_pending = false;
                    }
                }
                if (!s_mode_selection_pending) {
                    lv_dropdown_set_selected(s_top_mode_label, actual_mode_index);
                }
            }
        }

        // Apply frequency-change resets only on the LVGL task. This also
        // covers a dial change made directly with the KX3's VFO knob. Keep a
        // pending reset queued while Settings is open; apply it on resume.
        if (!s_settings_open.load(std::memory_order_acquire)) {
            if (s_frequency_change_pending.exchange(false, std::memory_order_acq_rel)) {
                clear_spectrum_for_frequency_change();
            }

            const uint32_t now = xTaskGetTickCount();
            if (now - s_last_fft_tick >= pdMS_TO_TICKS(PANADAPTER_REFRESH_MS)) {
                s_last_fft_tick = now;
                static float waterfall_row_accumulator = 0.0f;
                waterfall_row_accumulator +=
                    static_cast<float>(std::clamp(s_settings.waterfall_speed_hz, 30, 60)) *
                    (static_cast<float>(PANADAPTER_REFRESH_MS) / 1000.0f);
                uint8_t waterfall_rows = static_cast<uint8_t>(waterfall_row_accumulator);
                waterfall_row_accumulator -= static_cast<float>(waterfall_rows);
                update_spectrum_chart(waterfall_rows);
            }
        }
        // Poll the INA226 at 2 Hz. This is independent of the spectrum update
        // rate and keeps I2C traffic negligible.
        const int64_t battery_now_us = esp_timer_get_time();
        if (s_battery_monitor_initialized &&
            (battery_now_us - s_last_battery_update_us) >= 2000000) {
            s_last_battery_update_us = battery_now_us;
            auto battery = tab5.read_battery_status();
            // The Tab5 charger exposes its actual CHG_STAT signal separately
            // from INA226 battery telemetry. Use that hardware status for the
            // icon; INA226 current direction is not a reliable charging flag.
            const bool charger_gpio = tab5.get_charging_status();
            // Diagnostic: compare the dedicated CHG_STAT GPIO with the INA226's
            // current-derived charging estimate. This helps determine whether
            // the charger is active and which status source is reliable.
            ESP_LOGI(TAG, "BATTERY DIAG: present=%d voltage=%.2f V current=%.0f mA "
                         "power=%.0f mW CHG_STAT=%d INA_charging=%d",
                     battery.is_present, battery.voltage_v, battery.current_ma,
                     battery.power_mw, charger_gpio, battery.is_charging);
            if (s_top_battery_label) {
                if (battery.is_present) {
                    char battery_text[32];
                    std::snprintf(battery_text, sizeof(battery_text),
                                  "BAT %.1f V", battery.voltage_v);
                    lv_label_set_text(s_top_battery_label, battery_text);
                    if (s_top_charging_icon) {
                        if (charger_gpio) {
                            lv_obj_clear_flag(s_top_charging_icon, LV_OBJ_FLAG_HIDDEN);
                        } else {
                            lv_obj_add_flag(s_top_charging_icon, LV_OBJ_FLAG_HIDDEN);
                        }
                    }
                    // NP-F550 pack: full is about 8.2-8.4 V; Tab5 shutdown
                    // threshold is approximately 6.0 V. These are display
                    // warning bands, not a precise state-of-charge estimate.
                    const uint32_t color = battery.voltage_v >= 7.2f ? 0x7DDB8A :
                                          battery.voltage_v >= 6.6f ? 0xFFD166 :
                                                                     0xFF5C5C;
                    lv_obj_set_style_text_color(s_top_battery_label,
                                               lv_color_hex(color), 0);
                } else {
                    lv_label_set_text(s_top_battery_label, "BAT --.- V");
                    lv_obj_set_style_text_color(s_top_battery_label,
                                               lv_color_hex(0x7DDB8A), 0);
                    if (s_top_charging_icon) {
                        lv_obj_add_flag(s_top_charging_icon, LV_OBJ_FLAG_HIDDEN);
                    }
                }
            }
        }

        lv_task_handler();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
