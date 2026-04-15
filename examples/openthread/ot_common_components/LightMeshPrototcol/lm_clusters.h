#pragma once
/**
 * lm_clusters.h
 * Cluster state structures + function declarations
 */

#include "lm_protocol.h"

/* ──────────────────────────────────────────────
   ON/OFF CLUSTER
   ────────────────────────────────────────────── */
typedef void (*lm_onoff_changed_cb_t)(bool on, void *user_data);

typedef struct {
    bool                  state;
    lm_onoff_changed_cb_t on_change;
    void                 *user_data;
} lm_onoff_ctx_t;

void     lm_onoff_init(lm_onoff_ctx_t *ctx,
                       lm_onoff_changed_cb_t cb,
                       void *user_data);

void     lm_onoff_handler(const lm_header_t *hdr,
                          const uint8_t *payload,
                          uint16_t payload_len,
                          lm_onoff_ctx_t *ctx);

/* Convenience send helpers */
esp_err_t lm_onoff_send(const uint8_t *dst_id, uint8_t dst_ep, uint16_t cmd);

/* ──────────────────────────────────────────────
   LEVEL / DIMMER CLUSTER
   ────────────────────────────────────────────── */
typedef void (*lm_level_changed_cb_t)(uint8_t level, void *user_data);

typedef struct {
    uint8_t               level;   /* 0 = off, 254 = full */
    lm_level_changed_cb_t on_change;
    void                 *user_data;
} lm_level_ctx_t;

void      lm_level_init(lm_level_ctx_t *ctx,
                        lm_level_changed_cb_t cb,
                        void *user_data);

void      lm_level_handler(const lm_header_t *hdr,
                           const uint8_t *payload,
                           uint16_t payload_len,
                           lm_level_ctx_t *ctx);

esp_err_t lm_level_send_move_to(const uint8_t *dst_id, uint8_t dst_ep,
                                  uint8_t level, uint16_t transition_ms);

/* ──────────────────────────────────────────────
   FAN CLUSTER
   ────────────────────────────────────────────── */
typedef void (*lm_fan_changed_cb_t)(lm_fan_mode_t mode,
                                    uint8_t speed_pct,
                                    void *user_data);

typedef struct {
    lm_fan_mode_t       mode;
    uint8_t             speed_pct;  /* 0–100 */
    lm_fan_changed_cb_t on_change;
    void               *user_data;
} lm_fan_ctx_t;

void      lm_fan_init(lm_fan_ctx_t *ctx,
                      lm_fan_changed_cb_t cb,
                      void *user_data);

void      lm_fan_handler(const lm_header_t *hdr,
                         const uint8_t *payload,
                         uint16_t payload_len,
                         lm_fan_ctx_t *ctx);

esp_err_t lm_fan_send_mode(const uint8_t *dst_id, uint8_t dst_ep,
                            lm_fan_mode_t mode);
esp_err_t lm_fan_send_speed(const uint8_t *dst_id, uint8_t dst_ep,
                              uint8_t speed_pct);

/* ──────────────────────────────────────────────
   SWITCH CLUSTER  (produces commands, no state)
   ────────────────────────────────────────────── */
typedef struct {
    uint8_t node_id[8];
    uint8_t endpoint;
    uint8_t last_cmd;
} lm_switch_ctx_t;

void lm_switch_init(lm_switch_ctx_t *ctx,
                    uint8_t node_id[8],
                    uint8_t endpoint);

/* Call these from GPIO ISR / button driver */
void lm_switch_on_press(lm_switch_ctx_t *ctx);
void lm_switch_on_hold(lm_switch_ctx_t *ctx);
void lm_switch_on_release(lm_switch_ctx_t *ctx);
