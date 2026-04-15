/**
 * main.c — LightMesh Protocol Usage Examples
 *
 * Demonstrates how to configure each device type and wire them
 * together using the binding table. Change DEVICE_ROLE via sdkconfig
 * or #define below to flash different firmware to each ESP32 board.
 *
 * Network topology example:
 *
 *   [Switch EP1] ──bind──► [Light  EP1]  (OnOff toggle)
 *   [Switch EP2] ──bind──► [Dimmer EP1]  (Level ramp)
 *   [Switch EP3] ──bind──► [Fan    EP1]  (Mode cycle)
 *
 * Physical GPIO (adjust per board):
 *   BUTTON_GPIO  = 9   (boot button on ESP32-H2 devkit)
 *   LED_GPIO     = 5
 *   PWM_GPIO     = 6   (dimmer → LEDC channel)
 *   FAN_PWM_GPIO = 7
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_openthread.h"
#include "esp_openthread_defaults.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "lm_protocol.h"
#include "lm_clusters.h"

static const char *TAG = "Main";

/* ─── Choose role via build config ──────────── */
/* #define ROLE_SWITCH  */
/* #define ROLE_LIGHT   */
/* #define ROLE_DIMMER  */
#define ROLE_FAN

/* ─── GPIO ───────────────────────────────────── */
#define BUTTON_GPIO  9
#define LED_GPIO     5
#define PWM_GPIO     6
#define FAN_PWM_GPIO 7

/* ─── Thread credentials (same across all nodes) */
static const char LM_THREAD_NETWORK_NAME[] = "LightMesh";
static const uint8_t LM_THREAD_PANID[]     = {0xAB, 0xCD};   /* PAN ID = 0xABCD */
static const uint8_t LM_THREAD_XPANID[]    = {
    0xDE, 0xAD, 0xBE, 0xEF,
    0xCA, 0xFE, 0xBA, 0xBE
};
static const uint8_t LM_THREAD_KEY[]       = {
    0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
    0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF
};
static const uint8_t LM_THREAD_CHANNEL     = 15;

/* ═══════════════════════════════════════════════
   DEVICE GLOBALS
   ═══════════════════════════════════════════════ */
static lm_device_t      g_device;
static lm_onoff_ctx_t   g_onoff;
static lm_level_ctx_t   g_level;
static lm_fan_ctx_t     g_fan;
static lm_switch_ctx_t  g_switch;

/* ═══════════════════════════════════════════════
   PWM INIT (for dimmer & fan)
   ═══════════════════════════════════════════════ */
static void pwm_init(int gpio, ledc_channel_t ch) {
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);
    ledc_channel_config_t channel = {
        .gpio_num   = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = ch,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&channel);
}

static void pwm_set_duty(ledc_channel_t ch, uint8_t level_0_254) {
    uint32_t duty = ((uint32_t)level_0_254 * 255) / 254;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

/* ═══════════════════════════════════════════════
   CALLBACKS
   ═══════════════════════════════════════════════ */
static void cb_light_changed(bool on, void *ud) {
    gpio_set_level(LED_GPIO, on ? 1 : 0);
    ESP_LOGI(TAG, "💡 Light %s", on ? "ON" : "OFF");
}

static void cb_dimmer_changed(uint8_t level, void *ud) {
    pwm_set_duty(LEDC_CHANNEL_0, level);
    ESP_LOGI(TAG, "🔆 Dimmer level=%d (%.1f%%)", level, level * 100.0 / 254);
}

static void cb_fan_changed(lm_fan_mode_t mode, uint8_t speed, void *ud) {
    /* Map 0-100% → 0-254 PWM duty */
    uint8_t level = (uint8_t)((uint32_t)speed * 254 / 100);
    pwm_set_duty(LEDC_CHANNEL_1, level);
    const char *modes[] = {"OFF","LOW","MEDIUM","HIGH","AUTO","BOOST"};
    ESP_LOGI(TAG, "🌀 Fan mode=%s speed=%d%%", modes[mode], speed);
}

/* ═══════════════════════════════════════════════
   CLUSTER DISPATCH TRAMPOLINES
   (register one handler per cluster, trampoline to typed handler)
   ═══════════════════════════════════════════════ */
static void dispatch_onoff(const lm_header_t *hdr,
                           const uint8_t *pl, uint16_t len) {
    lm_onoff_handler(hdr, pl, len, &g_onoff);
}
static void dispatch_level(const lm_header_t *hdr,
                           const uint8_t *pl, uint16_t len) {
    lm_level_handler(hdr, pl, len, &g_level);
}
static void dispatch_fan(const lm_header_t *hdr,
                         const uint8_t *pl, uint16_t len) {
    lm_fan_handler(hdr, pl, len, &g_fan);
}

/* ═══════════════════════════════════════════════
   BUTTON HANDLER (GPIO interrupt → debounce task)
   ═══════════════════════════════════════════════ */
static TaskHandle_t s_btn_task;

static void IRAM_ATTR btn_isr(void *arg) {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_btn_task, &woken);
    portYIELD_FROM_ISR(woken);
}

static void btn_task(void *arg) {
    uint32_t hold_ticks = 0;
    bool     held       = false;
    while (1) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) > 0) {
            /* Rising edge (press) */
            vTaskDelay(pdMS_TO_TICKS(20));  /* debounce */
            if (gpio_get_level(BUTTON_GPIO) == 0) {
                lm_switch_on_press(&g_switch);
                hold_ticks = 0;
                held       = false;
            }
        } else {
            /* Polling for hold */
            if (gpio_get_level(BUTTON_GPIO) == 0) {
                hold_ticks += 100;
                if (hold_ticks >= 600 && !held) {
                    held = true;
                    lm_switch_on_hold(&g_switch);
                }
            } else if (held || hold_ticks > 0) {
                lm_switch_on_release(&g_switch);
                hold_ticks = 0;
                held       = false;
            }
        }
    }
}

static void btn_init(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, btn_isr, NULL);
    xTaskCreate(btn_task, "btn", 2048, NULL, 5, &s_btn_task);
}

/* ═══════════════════════════════════════════════
   THREAD NETWORK SETUP
   ═══════════════════════════════════════════════ */
static void thread_init(void) {
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config  = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    esp_openthread_init(&config);

    otInstance *ot = esp_openthread_get_instance();

    /* Configure Thread dataset */
    otOperationalDataset ds;
    memset(&ds, 0, sizeof(ds));

    /* Network name */
    strncpy((char *)ds.mNetworkName.m8,
            LM_THREAD_NETWORK_NAME, OT_NETWORK_NAME_MAX_SIZE);
    ds.mComponents.mIsNetworkNamePresent = true;

    /* PAN ID */
    ds.mPanId = (LM_THREAD_PANID[0] << 8) | LM_THREAD_PANID[1];
    ds.mComponents.mIsPanIdPresent = true;

    /* Extended PAN ID */
    memcpy(ds.mExtendedPanId.m8, LM_THREAD_XPANID, 8);
    ds.mComponents.mIsExtendedPanIdPresent = true;

    /* Network Key */
    memcpy(ds.mNetworkKey.m8, LM_THREAD_KEY, 16);
    ds.mComponents.mIsNetworkKeyPresent = true;

    /* Channel */
    ds.mChannel = LM_THREAD_CHANNEL;
    ds.mComponents.mIsChannelPresent = true;

    otDatasetSetActive(ot, &ds);
    otThreadSetEnabled(ot, true);
    otIp6SetEnabled(ot, true);

    ESP_LOGI(TAG, "Thread started on channel %d", LM_THREAD_CHANNEL);
}

/* ═══════════════════════════════════════════════
   DEVICE SETUP HELPERS
   ═══════════════════════════════════════════════ */
static void setup_endpoint(uint8_t ep_id, lm_device_type_t type,
                            const char *name,
                            uint16_t cluster_id, void *cluster_ctx)
{
    lm_endpoint_t *ep = &g_device.endpoints[g_device.endpoint_count++];
    ep->endpoint_id   = ep_id;
    ep->device_type   = type;
    strncpy(ep->name, name, 15);
    ep->clusters[0].cluster_id  = cluster_id;
    ep->clusters[0].cluster_ctx = cluster_ctx;
    ep->cluster_count = 1;
}

/* ═══════════════════════════════════════════════
   MAIN
   ═══════════════════════════════════════════════ */
void app_main(void) {
    nvs_flash_init();

    /* ── Thread init (common to all roles) ─── */
    thread_init();

    /* ── Protocol init ─────────────────────── */
    memset(&g_device, 0, sizeof(g_device));
    lm_init(&g_device);

/* ════════════════════════════════════
   ROLE: LIGHT
   A simple on/off lamp. Bound by switch.
   ════════════════════════════════════ */
#ifdef ROLE_LIGHT
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    lm_onoff_init(&g_onoff, cb_light_changed, NULL);
    setup_endpoint(1, LM_DEVTYPE_LIGHT, "BedroomLight",
                   LM_CLUSTER_ONOFF, &g_onoff);

    lm_register_handler(LM_CLUSTER_ONOFF, dispatch_onoff);
    lm_start();

    ESP_LOGI(TAG, "Role: LIGHT  — waiting for commands");

/* ════════════════════════════════════
   ROLE: DIMMER
   A dimmable light. Level cluster.
   ════════════════════════════════════ */
#elif defined(ROLE_DIMMER)
    pwm_init(PWM_GPIO, LEDC_CHANNEL_0);

    lm_level_init(&g_level, cb_dimmer_changed, NULL);
    setup_endpoint(1, LM_DEVTYPE_DIMMER, "LivingDimmer",
                   LM_CLUSTER_LEVEL, &g_level);

    lm_register_handler(LM_CLUSTER_LEVEL, dispatch_level);
    lm_start();

    ESP_LOGI(TAG, "Role: DIMMER — waiting for commands");

/* ════════════════════════════════════
   ROLE: FAN
   Fan with speed + mode control.
   ════════════════════════════════════ */
#elif defined(ROLE_FAN)
    pwm_init(FAN_PWM_GPIO, LEDC_CHANNEL_1);

    lm_fan_init(&g_fan, cb_fan_changed, NULL);
    setup_endpoint(1, LM_DEVTYPE_FAN, "CeilingFan",
                   LM_CLUSTER_FAN, &g_fan);

    lm_register_handler(LM_CLUSTER_FAN, dispatch_fan);
    lm_start();

    ESP_LOGI(TAG, "Role: FAN — waiting for commands");

/* ════════════════════════════════════
   ROLE: SWITCH
   A 3-button switch (EP1/2/3).
   Bindings wired here to known node IDs.
   ════════════════════════════════════ */
#elif defined(ROLE_SWITCH)
    btn_init();

    /* Self endpoint for switch */
    uint8_t my_id[8];
    lm_get_node_id(my_id);
    setup_endpoint(1, LM_DEVTYPE_SWITCH, "WallSwitch1",
                   LM_CLUSTER_SWITCH, &g_switch);
    lm_switch_init(&g_switch, my_id, 1);

    lm_start();

    /*
     * ──────────────────────────────────────────
     * BINDING TABLE CONFIGURATION
     * Replace dst_id[] with the real EUI-64 of
     * each target device (read from its serial
     * port or via lm_discover()).
     * ──────────────────────────────────────────
     */

    /* Target: bedroom light */
    static const uint8_t LIGHT_ID[8]  = {
        0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x00,0x11};
    /* Target: living room dimmer */
    static const uint8_t DIMMER_ID[8] = {
        0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
    /* Target: ceiling fan */
    static const uint8_t FAN_ID[8]    = {
        0xDE,0xAD,0xBE,0xEF,0xCA,0xFE,0xBA,0xBE};

    /* EP1 button → Light toggle */
    lm_bind(my_id,    1, LM_CLUSTER_SWITCH,
            LIGHT_ID, 1, LM_CLUSTER_ONOFF);

    /* EP1 button → Dimmer toggle (same button controls both) */
    lm_bind(my_id,     1, LM_CLUSTER_SWITCH,
            DIMMER_ID, 1, LM_CLUSTER_LEVEL);

    /* EP1 button → Fan toggle */
    lm_bind(my_id,  1, LM_CLUSTER_SWITCH,
            FAN_ID, 1, LM_CLUSTER_FAN);

    ESP_LOGI(TAG, "Role: SWITCH — bindings configured");
    ESP_LOGI(TAG, "Press BOOT button to toggle all bound devices");

#endif /* ROLE */

    /* ── Main loop: OpenThread task ──────────── */
    while (1) {
        esp_openthread_task_switching_lock_release();
        vTaskDelay(pdMS_TO_TICKS(10));
        esp_openthread_task_switching_lock_acquire(portMAX_DELAY);
    }
}
