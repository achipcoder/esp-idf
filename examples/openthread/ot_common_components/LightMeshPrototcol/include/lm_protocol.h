#pragma once
/**
 * LightMesh Protocol (LMP) - Custom Application Layer over ESP OpenThread
 * Similar to Matter but certificate-free, lightweight, binding-capable.
 *
 * Stack:
 *   LightMesh Protocol  <-- This layer
 *   CoAP / UDP
 *   IPv6 (Thread mesh)
 *   OpenThread (802.15.4)
 *   ESP32-H2 / ESP32-C6 radio
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_err.h"

/* ──────────────────────────────────────────────
   PROTOCOL CONSTANTS
   ────────────────────────────────────────────── */
#define LM_MAGIC_0          0xLM   /* replaced below */
#define LM_MAGIC_BYTE0      0x4C   /* 'L' */
#define LM_MAGIC_BYTE1      0x4D   /* 'M' */
#define LM_VERSION          0x01
#define LM_UDP_PORT         5700   /* LightMesh UDP port (Thread mesh-local) */
#define LM_MAX_PAYLOAD      128
#define LM_MAX_BINDINGS     16
#define LM_NODE_ID_LEN      8      /* 64-bit EUI-64 */
#define LM_BCAST_ADDR       "ff03::1"  /* Thread realm-local multicast */
#define LM_SHARED_KEY_LEN   16     /* Optional XOR obfuscation key (not crypto) */
#define LM_SEQ_WRAP         0xFFFF

/* ──────────────────────────────────────────────
   MESSAGE TYPES
   ────────────────────────────────────────────── */
typedef enum {
    LM_MSG_COMMAND      = 0x01,   /* Send a command to a cluster */
    LM_MSG_RESPONSE     = 0x02,   /* ACK / response to a command */
    LM_MSG_READ_REQ     = 0x03,   /* Read attribute request */
    LM_MSG_READ_RESP    = 0x04,   /* Read attribute response */
    LM_MSG_EVENT        = 0x05,   /* Unsolicited state change event */
    LM_MSG_BIND_REQ     = 0x10,   /* Request to bind two endpoints */
    LM_MSG_BIND_ACK     = 0x11,   /* Bind accepted */
    LM_MSG_UNBIND_REQ   = 0x12,   /* Remove a binding */
    LM_MSG_DISCOVER     = 0x20,   /* Device discovery broadcast */
    LM_MSG_DISCOVER_RESP= 0x21,   /* Discovery response */
    LM_MSG_HEARTBEAT    = 0x30,   /* Keepalive */
} lm_msg_type_t;

/* ──────────────────────────────────────────────
   CLUSTER IDs  (Matter-aligned numbering)
   ────────────────────────────────────────────── */
typedef enum {
    LM_CLUSTER_BASIC        = 0x0000,  /* Device info */
    LM_CLUSTER_ONOFF        = 0x0006,  /* On/Off (light, fan) */
    LM_CLUSTER_LEVEL        = 0x0008,  /* Level control (dimmer) */
    LM_CLUSTER_FAN          = 0x0202,  /* Fan control */
    LM_CLUSTER_SWITCH       = 0x003B,  /* Generic switch */
    LM_CLUSTER_BINDING      = 0xF000,  /* Binding management (custom) */
} lm_cluster_id_t;

/* ──────────────────────────────────────────────
   COMMAND IDs per CLUSTER
   ────────────────────────────────────────────── */
/* OnOff cluster (0x0006) */
#define LM_CMD_ONOFF_OFF            0x00
#define LM_CMD_ONOFF_ON             0x01
#define LM_CMD_ONOFF_TOGGLE         0x02

/* Level cluster (0x0008) */
#define LM_CMD_LEVEL_MOVE_TO        0x00  /* payload: uint8 level, uint16 transition_ms */
#define LM_CMD_LEVEL_MOVE           0x01  /* payload: uint8 mode(up/dn), uint8 rate */
#define LM_CMD_LEVEL_STEP           0x02  /* payload: uint8 mode, uint8 size */
#define LM_CMD_LEVEL_STOP           0x03

/* Fan cluster (0x0202) */
#define LM_CMD_FAN_SET_MODE         0x00  /* payload: uint8 mode */
#define LM_CMD_FAN_SET_SPEED        0x01  /* payload: uint8 percent 0-100 */

/* Switch cluster (0x003B) */
#define LM_CMD_SWITCH_PRESS         0x00  /* button press event */
#define LM_CMD_SWITCH_HOLD          0x01
#define LM_CMD_SWITCH_RELEASE       0x02

/* Attribute IDs (for READ_REQ / READ_RESP) */
#define LM_ATTR_ONOFF_STATE         0x0000  /* bool */
#define LM_ATTR_LEVEL_CURRENT       0x0000  /* uint8 0-254 */
#define LM_ATTR_FAN_MODE            0x0000  /* enum8 */
#define LM_ATTR_FAN_SPEED           0x0001  /* uint8 0-100 */
#define LM_ATTR_BASIC_DEVTYPE       0x0000  /* enum8 lm_device_type_t */
#define LM_ATTR_BASIC_NAME          0x0001  /* char[16] */

/* ──────────────────────────────────────────────
   DEVICE TYPES
   ────────────────────────────────────────────── */
typedef enum {
    LM_DEVTYPE_LIGHT        = 0x01,
    LM_DEVTYPE_DIMMER       = 0x02,
    LM_DEVTYPE_FAN          = 0x03,
    LM_DEVTYPE_SWITCH       = 0x04,
    LM_DEVTYPE_COMBO        = 0x05,  /* e.g. dimmer + fan */
} lm_device_type_t;

/* Fan modes */
typedef enum {
    LM_FAN_MODE_OFF     = 0x00,
    LM_FAN_MODE_LOW     = 0x01,
    LM_FAN_MODE_MEDIUM  = 0x02,
    LM_FAN_MODE_HIGH    = 0x03,
    LM_FAN_MODE_AUTO    = 0x04,
    LM_FAN_MODE_BOOST   = 0x05,
} lm_fan_mode_t;

/* ──────────────────────────────────────────────
   PACKET HEADER  (16 bytes, little-endian)
   ────────────────────────────────────────────── */
typedef struct __attribute__((packed)) {
    uint8_t  magic[2];        /* 0x4C 0x4D */
    uint8_t  version;         /* LM_VERSION */
    uint8_t  msg_type;        /* lm_msg_type_t */
    uint16_t seq;             /* Rolling sequence number */
    uint8_t  src_id[8];       /* EUI-64 of sender */
    uint8_t  dst_id[8];       /* EUI-64 of target, 0xFF..FF = broadcast */
    uint16_t cluster_id;      /* lm_cluster_id_t */
    uint16_t cmd_attr_id;     /* Command or attribute ID */
    uint8_t  endpoint;        /* Endpoint (supports multi-endpoint devices) */
    uint8_t  flags;           /* Bit0=ACK_required, Bit1=group, Bit2=bound */
    uint16_t payload_len;     /* Length of payload bytes following header */
    /* payload follows immediately after this header */
} lm_header_t;

#define LM_HEADER_SIZE  sizeof(lm_header_t)  /* == 28 bytes */
#define LM_MAX_FRAME    (LM_HEADER_SIZE + LM_MAX_PAYLOAD)

/* Flags bits */
#define LM_FLAG_ACK_REQ     (1 << 0)
#define LM_FLAG_GROUP       (1 << 1)
#define LM_FLAG_BOUND       (1 << 2)  /* message triggered by a binding */

/* Broadcast node ID */
static const uint8_t LM_BCAST_ID[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

/* ──────────────────────────────────────────────
   BINDING ENTRY
   ────────────────────────────────────────────── */
typedef struct {
    bool     active;
    uint8_t  src_id[8];         /* Source device EUI-64 */
    uint8_t  src_endpoint;
    uint16_t src_cluster;
    uint8_t  dst_id[8];         /* Destination device EUI-64 */
    uint8_t  dst_endpoint;
    uint16_t dst_cluster;
} lm_binding_entry_t;

/* ──────────────────────────────────────────────
   DEVICE DESCRIPTOR  (self-description)
   ────────────────────────────────────────────── */
#define LM_MAX_ENDPOINTS    4
#define LM_MAX_CLUSTERS     8

typedef struct {
    uint16_t cluster_id;
    void    *cluster_ctx;  /* points to cluster state struct */
} lm_cluster_entry_t;

typedef struct {
    uint8_t            endpoint_id;
    lm_device_type_t   device_type;
    char               name[16];
    lm_cluster_entry_t clusters[LM_MAX_CLUSTERS];
    uint8_t            cluster_count;
} lm_endpoint_t;

typedef struct {
    uint8_t       node_id[8];       /* This device's EUI-64 */
    lm_endpoint_t endpoints[LM_MAX_ENDPOINTS];
    uint8_t       endpoint_count;
} lm_device_t;

/* ──────────────────────────────────────────────
   CALLBACKS
   ────────────────────────────────────────────── */
typedef void (*lm_command_handler_t)(const lm_header_t *hdr,
                                     const uint8_t *payload,
                                     uint16_t payload_len);

typedef struct {
    uint16_t              cluster_id;
    lm_command_handler_t  handler;
} lm_cluster_handler_t;

/* ──────────────────────────────────────────────
   PUBLIC API
   ────────────────────────────────────────────── */
esp_err_t lm_init(lm_device_t *device);
esp_err_t lm_start(void);
void      lm_stop(void);

/* Send a command to a remote node */
esp_err_t lm_send_command(const uint8_t *dst_id,
                           uint8_t dst_endpoint,
                           uint16_t cluster_id,
                           uint16_t cmd_id,
                           const uint8_t *payload,
                           uint16_t payload_len);

/* Broadcast a command (e.g. switch press) */
esp_err_t lm_broadcast_command(uint16_t cluster_id,
                                uint16_t cmd_id,
                                const uint8_t *payload,
                                uint16_t payload_len);

/* Read a remote attribute */
esp_err_t lm_read_attribute(const uint8_t *dst_id,
                             uint8_t dst_endpoint,
                             uint16_t cluster_id,
                             uint16_t attr_id);

/* Register a cluster command handler */
esp_err_t lm_register_handler(uint16_t cluster_id,
                               lm_command_handler_t handler);

/* Binding API */
esp_err_t lm_bind(const uint8_t *src_id, uint8_t src_ep, uint16_t src_cluster,
                  const uint8_t *dst_id, uint8_t dst_ep, uint16_t dst_cluster);
esp_err_t lm_unbind(const uint8_t *src_id, uint8_t src_ep, uint16_t src_cluster,
                    const uint8_t *dst_id, uint8_t dst_ep, uint16_t dst_cluster);

/* Fire all bindings for a source endpoint (called internally on switch press) */
esp_err_t lm_invoke_bindings(const uint8_t *src_id, uint8_t src_ep,
                              uint16_t src_cluster,
                              uint16_t cmd_id,
                              const uint8_t *payload, uint16_t payload_len);

/* Discover devices on the Thread mesh */
esp_err_t lm_discover(void);

/* Get local node EUI-64 */
void lm_get_node_id(uint8_t out_id[8]);
