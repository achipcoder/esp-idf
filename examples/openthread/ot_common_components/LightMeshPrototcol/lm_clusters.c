/**
 * lm_clusters.c
 * Cluster state machines and handlers for:
 *   - OnOff  (0x0006)  → Light on/off
 *   - Level  (0x0008)  → Dimmer / brightness
 *   - Fan    (0x0202)  → Fan mode + speed
 *   - Switch (0x003B)  → Momentary switch, fires bindings
 */

#include "lm_clusters.h"
#include "lm_protocol.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include <string.h>

static const char *TAG = "LM_Clusters";

/* ══════════════════════════════════════════════════════
   ON/OFF CLUSTER   (cluster 0x0006)
   ══════════════════════════════════════════════════════ */

void lm_onoff_init(lm_onoff_ctx_t *ctx,
                   lm_onoff_changed_cb_t cb,
                   void *user_data)
{
    ctx->state     = false;
    ctx->on_change = cb;
    ctx->user_data = user_data;
}

static void onoff_apply(lm_onoff_ctx_t *ctx, bool new_state) {
    if (ctx->state != new_state) {
        ctx->state = new_state;
        ESP_LOGI(TAG, "OnOff → %s", new_state ? "ON" : "OFF");
        if (ctx->on_change)
            ctx->on_change(new_state, ctx->user_data);
    }
}

/* Called by the protocol layer when a command arrives for cluster 0x0006 */
void lm_onoff_handler(const lm_header_t *hdr,
                      const uint8_t *payload,
                      uint16_t payload_len,
                      lm_onoff_ctx_t *ctx)
{
    switch (hdr->cmd_attr_id) {
        case LM_CMD_ONOFF_OFF:
            onoff_apply(ctx, false);
            break;
        case LM_CMD_ONOFF_ON:
            onoff_apply(ctx, true);
            break;
        case LM_CMD_ONOFF_TOGGLE:
            onoff_apply(ctx, !ctx->state);
            break;
        default:
            ESP_LOGW(TAG, "OnOff: unknown cmd 0x%04X", hdr->cmd_attr_id);
    }
}

/* Utility: send OnOff command from this node to a target */
esp_err_t lm_onoff_send(const uint8_t *dst_id, uint8_t dst_ep, uint16_t cmd) {
    return lm_send_command(dst_id, dst_ep,
                           LM_CLUSTER_ONOFF, cmd, NULL, 0);
}

/* ══════════════════════════════════════════════════════
   LEVEL CLUSTER   (cluster 0x0008)   — Dimmer
   ══════════════════════════════════════════════════════ */

/* Dimming task: smoothly ramps level over transition time */
typedef struct {
    lm_level_ctx_t *ctx;
    uint8_t  target;
    uint32_t step_ms;
    int8_t   direction;   /* +1 or -1 */
    TimerHandle_t timer;
} lm_level_ramp_t;

static lm_level_ramp_t s_ramp;

static void level_ramp_cb(TimerHandle_t tmr) {
    lm_level_ramp_t *r = (lm_level_ramp_t *)pvTimerGetTimerID(tmr);
    lm_level_ctx_t  *ctx = r->ctx;

    if (r->direction > 0) {
        if (ctx->level >= r->target) {
            xTimerStop(tmr, 0);
            goto notify;
        }
        ctx->level = (ctx->level + 1 > r->target) ? r->target : ctx->level + 1;
    } else {
        if (ctx->level <= r->target) {
            xTimerStop(tmr, 0);
            goto notify;
        }
        ctx->level = (ctx->level == 0) ? 0 : ctx->level - 1;
        if (ctx->level < r->target) ctx->level = r->target;
    }
notify:
    if (ctx->on_change)
        ctx->on_change(ctx->level, ctx->user_data);
}

void lm_level_init(lm_level_ctx_t *ctx,
                   lm_level_changed_cb_t cb,
                   void *user_data)
{
    ctx->level     = 0;
    ctx->on_change = cb;
    ctx->user_data = user_data;
    s_ramp.timer = NULL;
}

void lm_level_handler(const lm_header_t *hdr,
                      const uint8_t *payload,
                      uint16_t payload_len,
                      lm_level_ctx_t *ctx)
{
    switch (hdr->cmd_attr_id) {
        case LM_CMD_LEVEL_MOVE_TO: {
            /* payload[0] = target level (0-254), payload[1..2] = ms (LE) */
            if (payload_len < 1) break;
            uint8_t target = payload[0];
            uint16_t trans_ms = (payload_len >= 3)
                                ? ((uint16_t)payload[2] << 8 | payload[1])
                                : 500;

            if (trans_ms == 0 || target == ctx->level) {
                ctx->level = target;
                if (ctx->on_change) ctx->on_change(ctx->level, ctx->user_data);
                break;
            }

            /* Ramp using FreeRTOS timer */
            uint32_t steps  = (target > ctx->level)
                               ? (target - ctx->level)
                               : (ctx->level - target);
            uint32_t step_ms = trans_ms / steps;
            if (step_ms < 1) step_ms = 1;

            s_ramp.ctx       = ctx;
            s_ramp.target    = target;
            s_ramp.direction = (target > ctx->level) ? 1 : -1;

            if (!s_ramp.timer) {
                s_ramp.timer = xTimerCreate("lvl_ramp", pdMS_TO_TICKS(step_ms),
                                            pdTRUE, &s_ramp, level_ramp_cb);
            } else {
                xTimerChangePeriod(s_ramp.timer, pdMS_TO_TICKS(step_ms), 0);
            }
            xTimerStart(s_ramp.timer, 0);

            ESP_LOGI(TAG, "Level MOVE_TO %d in %dms (%d steps @ %dms)",
                     target, trans_ms, steps, step_ms);
            break;
        }
        case LM_CMD_LEVEL_MOVE: {
            /* payload[0] = 0(up)/1(dn), payload[1] = rate (units/sec) */
            if (payload_len < 2) break;
            uint8_t mode = payload[0]; /* 0=up, 1=down */
            uint8_t rate = payload[1]; /* units/sec, 0=fastest */
            uint8_t target = (mode == 0) ? 254 : 0;
            uint16_t trans_ms = (rate == 0) ? 1000
                                : (uint16_t)(254 * 1000 / rate);
            /* Reuse MOVE_TO logic */
            uint8_t pl[3] = { target, (uint8_t)(trans_ms & 0xFF),
                                       (uint8_t)(trans_ms >> 8) };
            lm_header_t fake_hdr = *hdr;
            fake_hdr.cmd_attr_id = LM_CMD_LEVEL_MOVE_TO;
            lm_level_handler(&fake_hdr, pl, 3, ctx);
            break;
        }
        case LM_CMD_LEVEL_STOP:
            if (s_ramp.timer) xTimerStop(s_ramp.timer, 0);
            ESP_LOGI(TAG, "Level STOP at %d", ctx->level);
            break;

        default:
            ESP_LOGW(TAG, "Level: unknown cmd 0x%04X", hdr->cmd_attr_id);
    }
}

esp_err_t lm_level_send_move_to(const uint8_t *dst_id, uint8_t dst_ep,
                                  uint8_t level, uint16_t transition_ms)
{
    uint8_t payload[3] = {
        level,
        (uint8_t)(transition_ms & 0xFF),
        (uint8_t)(transition_ms >> 8)
    };
    return lm_send_command(dst_id, dst_ep,
                           LM_CLUSTER_LEVEL,
                           LM_CMD_LEVEL_MOVE_TO,
                           payload, 3);
}

/* ══════════════════════════════════════════════════════
   FAN CLUSTER   (cluster 0x0202)
   ══════════════════════════════════════════════════════ */

void lm_fan_init(lm_fan_ctx_t *ctx,
                 lm_fan_changed_cb_t cb,
                 void *user_data)
{
    ctx->mode      = LM_FAN_MODE_OFF;
    ctx->speed_pct = 0;
    ctx->on_change = cb;
    ctx->user_data = user_data;
}

void lm_fan_handler(const lm_header_t *hdr,
                    const uint8_t *payload,
                    uint16_t payload_len,
                    lm_fan_ctx_t *ctx)
{
    switch (hdr->cmd_attr_id) {
        case LM_CMD_FAN_SET_MODE:
            if (payload_len < 1) break;
            ctx->mode = (lm_fan_mode_t)payload[0];
            /* Auto-set speed based on mode */
            switch (ctx->mode) {
                case LM_FAN_MODE_OFF:    ctx->speed_pct = 0;   break;
                case LM_FAN_MODE_LOW:    ctx->speed_pct = 33;  break;
                case LM_FAN_MODE_MEDIUM: ctx->speed_pct = 66;  break;
                case LM_FAN_MODE_HIGH:   ctx->speed_pct = 100; break;
                case LM_FAN_MODE_BOOST:  ctx->speed_pct = 100; break;
                default: break;
            }
            ESP_LOGI(TAG, "Fan mode → %d  speed → %d%%",
                     ctx->mode, ctx->speed_pct);
            if (ctx->on_change)
                ctx->on_change(ctx->mode, ctx->speed_pct, ctx->user_data);
            break;

        case LM_CMD_FAN_SET_SPEED:
            if (payload_len < 1) break;
            ctx->speed_pct = payload[0];
            if (ctx->speed_pct > 100) ctx->speed_pct = 100;
            /* Derive mode from speed */
            if      (ctx->speed_pct == 0)   ctx->mode = LM_FAN_MODE_OFF;
            else if (ctx->speed_pct <= 40)  ctx->mode = LM_FAN_MODE_LOW;
            else if (ctx->speed_pct <= 70)  ctx->mode = LM_FAN_MODE_MEDIUM;
            else                             ctx->mode = LM_FAN_MODE_HIGH;
            ESP_LOGI(TAG, "Fan speed → %d%%  mode → %d",
                     ctx->speed_pct, ctx->mode);
            if (ctx->on_change)
                ctx->on_change(ctx->mode, ctx->speed_pct, ctx->user_data);
            break;

        default:
            ESP_LOGW(TAG, "Fan: unknown cmd 0x%04X", hdr->cmd_attr_id);
    }
}

esp_err_t lm_fan_send_mode(const uint8_t *dst_id, uint8_t dst_ep,
                            lm_fan_mode_t mode)
{
    uint8_t payload = (uint8_t)mode;
    return lm_send_command(dst_id, dst_ep,
                           LM_CLUSTER_FAN,
                           LM_CMD_FAN_SET_MODE,
                           &payload, 1);
}

esp_err_t lm_fan_send_speed(const uint8_t *dst_id, uint8_t dst_ep,
                              uint8_t speed_pct)
{
    if (speed_pct > 100) speed_pct = 100;
    return lm_send_command(dst_id, dst_ep,
                           LM_CLUSTER_FAN,
                           LM_CMD_FAN_SET_SPEED,
                           &speed_pct, 1);
}

/* ══════════════════════════════════════════════════════
   SWITCH CLUSTER   (cluster 0x003B)
   Acts as a PRODUCER — fires bindings when pressed.
   ══════════════════════════════════════════════════════ */

void lm_switch_init(lm_switch_ctx_t *ctx,
                    uint8_t node_id[8],
                    uint8_t endpoint)
{
    memcpy(ctx->node_id, node_id, 8);
    ctx->endpoint       = endpoint;
    ctx->last_cmd       = 0xFF;
}

/**
 * Call this when the physical button is pressed/released.
 * It fires all bindings attached to this switch endpoint.
 * The bound destination's cluster/cmd is defined in the binding.
 * Here we pass the ONOFF TOGGLE as the forwarded command,
 * but bindings can be set to any cluster/cmd.
 */
void lm_switch_on_press(lm_switch_ctx_t *ctx) {
    ctx->last_cmd = LM_CMD_SWITCH_PRESS;
    ESP_LOGI(TAG, "Switch PRESS on ep=%d → invoking bindings", ctx->endpoint);
    /* Fire bindings: translate switch press → OnOff Toggle */
    uint8_t noop = 0;
    lm_invoke_bindings(ctx->node_id, ctx->endpoint,
                       LM_CLUSTER_SWITCH,
                       LM_CMD_ONOFF_TOGGLE,   /* forwarded cmd to bound cluster */
                       &noop, 1);
}

void lm_switch_on_hold(lm_switch_ctx_t *ctx) {
    ctx->last_cmd = LM_CMD_SWITCH_HOLD;
    ESP_LOGI(TAG, "Switch HOLD on ep=%d", ctx->endpoint);
    /* Fire level dim-down while held */
    uint8_t payload[2] = { 1 /*down*/, 50 /*rate units/sec*/ };
    lm_invoke_bindings(ctx->node_id, ctx->endpoint,
                       LM_CLUSTER_SWITCH,
                       LM_CMD_LEVEL_MOVE,
                       payload, 2);
}

void lm_switch_on_release(lm_switch_ctx_t *ctx) {
    if (ctx->last_cmd == LM_CMD_SWITCH_HOLD) {
        /* Stop any ongoing move */
        uint8_t noop = 0;
        lm_invoke_bindings(ctx->node_id, ctx->endpoint,
                           LM_CLUSTER_SWITCH,
                           LM_CMD_LEVEL_STOP,
                           &noop, 1);
    }
    ctx->last_cmd = LM_CMD_SWITCH_RELEASE;
}
