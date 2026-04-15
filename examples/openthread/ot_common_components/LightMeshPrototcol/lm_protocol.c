/**
 * lm_protocol.c
 * LightMesh Protocol core: UDP socket over OpenThread, frame encode/decode,
 * binding engine, command dispatch.
 */

#include "lm_protocol.h"
#include "esp_log.h"
#include "esp_openthread.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <openthread/udp.h>
#include <openthread/instance.h>
#include <openthread/thread.h>
#include <openthread/ip6.h>
#include <openthread/dataset.h>
#include <string.h>
#include <stdlib.h>

static const char *TAG = "LightMesh";

/* ──────────────────────────────────────────────
   INTERNAL STATE
   ────────────────────────────────────────────── */
static lm_device_t        *s_device      = NULL;
static otUdpSocket         s_socket;
static uint16_t            s_seq         = 0;
static SemaphoreHandle_t   s_mutex       = NULL;
static TaskHandle_t        s_rx_task     = NULL;

/* Handler registry */
#define LM_MAX_HANDLERS 16
static lm_cluster_handler_t s_handlers[LM_MAX_HANDLERS];
static uint8_t              s_handler_count = 0;

/* Binding table */
static lm_binding_entry_t s_bindings[LM_MAX_BINDINGS];

/* ──────────────────────────────────────────────
   HELPER: Get OT instance
   ────────────────────────────────────────────── */
static inline otInstance *get_ot(void) {
    return esp_openthread_get_instance();
}

/* ──────────────────────────────────────────────
   NODE ID: derive from EUI-64
   ────────────────────────────────────────────── */
void lm_get_node_id(uint8_t out_id[8]) {
    otExtAddress ext;
    otLinkGetExtendedAddress(get_ot(), &ext);
    memcpy(out_id, ext.m8, 8);
}

/* ──────────────────────────────────────────────
   FRAME BUILD
   ────────────────────────────────────────────── */
static uint16_t lm_build_frame(uint8_t *buf, size_t buf_size,
                                uint8_t msg_type,
                                const uint8_t *dst_id,
                                uint16_t cluster_id,
                                uint16_t cmd_attr_id,
                                uint8_t endpoint,
                                uint8_t flags,
                                const uint8_t *payload,
                                uint16_t payload_len)
{
    if (buf_size < LM_HEADER_SIZE + payload_len) return 0;

    lm_header_t *hdr = (lm_header_t *)buf;
    hdr->magic[0]    = LM_MAGIC_BYTE0;
    hdr->magic[1]    = LM_MAGIC_BYTE1;
    hdr->version     = LM_VERSION;
    hdr->msg_type    = msg_type;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    hdr->seq = s_seq++;
    xSemaphoreGive(s_mutex);

    lm_get_node_id(hdr->src_id);
    memcpy(hdr->dst_id, dst_id, 8);
    hdr->cluster_id  = cluster_id;
    hdr->cmd_attr_id = cmd_attr_id;
    hdr->endpoint    = endpoint;
    hdr->flags       = flags;
    hdr->payload_len = payload_len;

    if (payload && payload_len)
        memcpy(buf + LM_HEADER_SIZE, payload, payload_len);

    return LM_HEADER_SIZE + payload_len;
}

/* ──────────────────────────────────────────────
   FRAME VALIDATE
   ────────────────────────────────────────────── */
static bool lm_validate_frame(const uint8_t *buf, uint16_t len) {
    if (len < LM_HEADER_SIZE) return false;
    const lm_header_t *hdr = (const lm_header_t *)buf;
    if (hdr->magic[0] != LM_MAGIC_BYTE0 || hdr->magic[1] != LM_MAGIC_BYTE1) return false;
    if (hdr->version  != LM_VERSION) return false;
    if (len < LM_HEADER_SIZE + hdr->payload_len) return false;
    return true;
}

/* ──────────────────────────────────────────────
   UDP SEND (unicast & multicast)
   ────────────────────────────────────────────── */
static esp_err_t lm_udp_send(const otIp6Address *dst_ip,
                               const uint8_t *buf, uint16_t len)
{
    otMessageInfo msg_info;
    memset(&msg_info, 0, sizeof(msg_info));
    memcpy(&msg_info.mPeerAddr, dst_ip, sizeof(otIp6Address));
    msg_info.mPeerPort = LM_UDP_PORT;
    /* Use mesh-local source */
    otIp6AddressFromString("::", &msg_info.mSockAddr);
    msg_info.mSockPort = LM_UDP_PORT;

    otMessage *ot_msg = otUdpNewMessage(get_ot(), NULL);
    if (!ot_msg) {
        ESP_LOGE(TAG, "Failed to allocate OT message");
        return ESP_ERR_NO_MEM;
    }
    if (otMessageAppend(ot_msg, buf, len) != OT_ERROR_NONE) {
        otMessageFree(ot_msg);
        return ESP_FAIL;
    }
    otError err = otUdpSend(get_ot(), &s_socket, ot_msg, &msg_info);
    if (err != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "otUdpSend failed: %d", err);
        otMessageFree(ot_msg);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* Resolve node_id (EUI-64) to IPv6 using Thread mesh-local EID */
static bool lm_resolve_ip(const uint8_t *node_id, otIp6Address *out_ip) {
    /* Build mesh-local EID from EUI-64: fd00::/64 prefix + IID from EUI-64 */
    /* The IID is constructed per RFC 4291 from EUI-64 */
    const otMeshLocalPrefix *mlp = otThreadGetMeshLocalPrefix(get_ot());
    memcpy(out_ip->mFields.m8, mlp->m8, 8);
    /* IID from EUI-64 (flip universal/local bit) */
    out_ip->mFields.m8[8]  = node_id[0] ^ 0x02;
    out_ip->mFields.m8[9]  = node_id[1];
    out_ip->mFields.m8[10] = node_id[2];
    out_ip->mFields.m8[11] = 0xFF;
    out_ip->mFields.m8[12] = 0xFE;
    out_ip->mFields.m8[13] = node_id[5];
    out_ip->mFields.m8[14] = node_id[6];
    out_ip->mFields.m8[15] = node_id[7];
    return true;
}

/* ──────────────────────────────────────────────
   PUBLIC: SEND COMMAND (unicast)
   ────────────────────────────────────────────── */
esp_err_t lm_send_command(const uint8_t *dst_id,
                           uint8_t dst_endpoint,
                           uint16_t cluster_id,
                           uint16_t cmd_id,
                           const uint8_t *payload,
                           uint16_t payload_len)
{
    uint8_t buf[LM_MAX_FRAME];
    uint16_t frame_len = lm_build_frame(buf, sizeof(buf),
                                         LM_MSG_COMMAND,
                                         dst_id,
                                         cluster_id, cmd_id,
                                         dst_endpoint,
                                         LM_FLAG_ACK_REQ,
                                         payload, payload_len);
    if (!frame_len) return ESP_ERR_INVALID_SIZE;

    otIp6Address dst_ip;
    if (!lm_resolve_ip(dst_id, &dst_ip)) return ESP_ERR_NOT_FOUND;

    ESP_LOGD(TAG, "TX CMD cluster=0x%04X cmd=0x%04X ep=%d len=%d",
             cluster_id, cmd_id, dst_endpoint, payload_len);
    return lm_udp_send(&dst_ip, buf, frame_len);
}

/* ──────────────────────────────────────────────
   PUBLIC: BROADCAST COMMAND (multicast to all)
   ────────────────────────────────────────────── */
esp_err_t lm_broadcast_command(uint16_t cluster_id,
                                uint16_t cmd_id,
                                const uint8_t *payload,
                                uint16_t payload_len)
{
    uint8_t buf[LM_MAX_FRAME];
    uint16_t frame_len = lm_build_frame(buf, sizeof(buf),
                                         LM_MSG_COMMAND,
                                         LM_BCAST_ID,
                                         cluster_id, cmd_id,
                                         0xFF,   /* all endpoints */
                                         LM_FLAG_GROUP,
                                         payload, payload_len);
    if (!frame_len) return ESP_ERR_INVALID_SIZE;

    otIp6Address bcast_ip;
    otIp6AddressFromString(LM_BCAST_ADDR, &bcast_ip);
    return lm_udp_send(&bcast_ip, buf, frame_len);
}

/* ──────────────────────────────────────────────
   PUBLIC: READ ATTRIBUTE
   ────────────────────────────────────────────── */
esp_err_t lm_read_attribute(const uint8_t *dst_id,
                             uint8_t dst_endpoint,
                             uint16_t cluster_id,
                             uint16_t attr_id)
{
    uint8_t buf[LM_HEADER_SIZE];
    uint16_t frame_len = lm_build_frame(buf, sizeof(buf),
                                         LM_MSG_READ_REQ,
                                         dst_id,
                                         cluster_id, attr_id,
                                         dst_endpoint,
                                         LM_FLAG_ACK_REQ,
                                         NULL, 0);
    if (!frame_len) return ESP_ERR_INVALID_SIZE;
    otIp6Address dst_ip;
    lm_resolve_ip(dst_id, &dst_ip);
    return lm_udp_send(&dst_ip, buf, frame_len);
}

/* ──────────────────────────────────────────────
   BINDING ENGINE
   ────────────────────────────────────────────── */
esp_err_t lm_bind(const uint8_t *src_id, uint8_t src_ep, uint16_t src_cluster,
                  const uint8_t *dst_id, uint8_t dst_ep, uint16_t dst_cluster)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < LM_MAX_BINDINGS; i++) {
        if (!s_bindings[i].active) {
            s_bindings[i].active       = true;
            memcpy(s_bindings[i].src_id, src_id, 8);
            s_bindings[i].src_endpoint = src_ep;
            s_bindings[i].src_cluster  = src_cluster;
            memcpy(s_bindings[i].dst_id, dst_id, 8);
            s_bindings[i].dst_endpoint = dst_ep;
            s_bindings[i].dst_cluster  = dst_cluster;
            xSemaphoreGive(s_mutex);
            ESP_LOGI(TAG, "Bound: src_ep=%d -> dst_ep=%d cluster=0x%04X",
                     src_ep, dst_ep, src_cluster);
            return ESP_OK;
        }
    }
    xSemaphoreGive(s_mutex);
    return ESP_ERR_NO_MEM;  /* binding table full */
}

esp_err_t lm_unbind(const uint8_t *src_id, uint8_t src_ep, uint16_t src_cluster,
                    const uint8_t *dst_id, uint8_t dst_ep, uint16_t dst_cluster)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < LM_MAX_BINDINGS; i++) {
        if (s_bindings[i].active &&
            memcmp(s_bindings[i].src_id, src_id, 8) == 0 &&
            s_bindings[i].src_endpoint == src_ep &&
            s_bindings[i].src_cluster  == src_cluster &&
            memcmp(s_bindings[i].dst_id, dst_id, 8) == 0 &&
            s_bindings[i].dst_endpoint == dst_ep) {
            s_bindings[i].active = false;
            xSemaphoreGive(s_mutex);
            return ESP_OK;
        }
    }
    xSemaphoreGive(s_mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t lm_invoke_bindings(const uint8_t *src_id, uint8_t src_ep,
                              uint16_t src_cluster,
                              uint16_t cmd_id,
                              const uint8_t *payload, uint16_t payload_len)
{
    int fired = 0;
    for (int i = 0; i < LM_MAX_BINDINGS; i++) {
        lm_binding_entry_t *b = &s_bindings[i];
        if (!b->active) continue;
        if (memcmp(b->src_id, src_id, 8) != 0) continue;
        if (b->src_endpoint != src_ep)           continue;
        if (b->src_cluster  != src_cluster)      continue;

        uint8_t buf[LM_MAX_FRAME];
        uint8_t flags = LM_FLAG_BOUND | LM_FLAG_ACK_REQ;
        uint16_t frame_len = lm_build_frame(buf, sizeof(buf),
                                             LM_MSG_COMMAND,
                                             b->dst_id,
                                             b->dst_cluster, cmd_id,
                                             b->dst_endpoint,
                                             flags,
                                             payload, payload_len);
        otIp6Address dst_ip;
        lm_resolve_ip(b->dst_id, &dst_ip);
        lm_udp_send(&dst_ip, buf, frame_len);
        fired++;
    }
    ESP_LOGD(TAG, "Binding invocation fired %d targets", fired);
    return (fired > 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/* ──────────────────────────────────────────────
   HANDLER REGISTRY
   ────────────────────────────────────────────── */
esp_err_t lm_register_handler(uint16_t cluster_id,
                               lm_command_handler_t handler)
{
    if (s_handler_count >= LM_MAX_HANDLERS) return ESP_ERR_NO_MEM;
    s_handlers[s_handler_count].cluster_id = cluster_id;
    s_handlers[s_handler_count].handler    = handler;
    s_handler_count++;
    return ESP_OK;
}

static void lm_dispatch(const lm_header_t *hdr,
                        const uint8_t *payload,
                        uint16_t payload_len)
{
    for (int i = 0; i < s_handler_count; i++) {
        if (s_handlers[i].cluster_id == hdr->cluster_id) {
            s_handlers[i].handler(hdr, payload, payload_len);
            return;
        }
    }
    ESP_LOGW(TAG, "No handler for cluster 0x%04X", hdr->cluster_id);
}

/* ──────────────────────────────────────────────
   UDP RECEIVE CALLBACK (OT context)
   ────────────────────────────────────────────── */
static void lm_udp_recv_cb(void *ctx,
                            otMessage *msg,
                            const otMessageInfo *msg_info)
{
    uint16_t len = otMessageGetLength(msg) - otMessageGetOffset(msg);
    if (len < LM_HEADER_SIZE || len > LM_MAX_FRAME) {
        ESP_LOGW(TAG, "RX: bad length %d", len);
        return;
    }

    uint8_t buf[LM_MAX_FRAME];
    otMessageRead(msg, otMessageGetOffset(msg), buf, len);

    if (!lm_validate_frame(buf, len)) {
        ESP_LOGW(TAG, "RX: invalid frame (bad magic/version)");
        return;
    }

    const lm_header_t *hdr     = (const lm_header_t *)buf;
    const uint8_t     *payload = buf + LM_HEADER_SIZE;

    ESP_LOGD(TAG, "RX type=0x%02X cluster=0x%04X cmd=0x%04X seq=%d ep=%d",
             hdr->msg_type, hdr->cluster_id, hdr->cmd_attr_id,
             hdr->seq, hdr->endpoint);

    switch (hdr->msg_type) {
        case LM_MSG_COMMAND:
        case LM_MSG_EVENT:
            lm_dispatch(hdr, payload, hdr->payload_len);
            /* If ACK required, send response */
            if (hdr->flags & LM_FLAG_ACK_REQ) {
                uint8_t resp[LM_HEADER_SIZE];
                uint8_t resp_code = 0x00; /* success */
                uint16_t rlen = lm_build_frame(resp, sizeof(resp),
                                               LM_MSG_RESPONSE,
                                               hdr->src_id,
                                               hdr->cluster_id,
                                               hdr->cmd_attr_id,
                                               hdr->endpoint, 0,
                                               &resp_code, 1);
                otIp6Address src_ip;
                lm_resolve_ip(hdr->src_id, &src_ip);
                lm_udp_send(&src_ip, resp, rlen);
            }
            break;

        case LM_MSG_DISCOVER:
            /* Reply with our device descriptor */
            lm_discover();   /* re-use discover to send back our info */
            break;

        case LM_MSG_BIND_REQ: {
            /* Payload: [src_ep(1) src_cluster(2) dst_ep(1) dst_cluster(2)] */
            if (hdr->payload_len >= 6) {
                uint8_t  src_ep      = payload[0];
                uint16_t src_cluster = (payload[2] << 8) | payload[1];
                uint8_t  dst_ep      = payload[3];
                uint16_t dst_cluster = (payload[5] << 8) | payload[4];
                lm_bind(hdr->src_id, src_ep, src_cluster,
                        hdr->dst_id, dst_ep, dst_cluster);
                /* ACK */
                uint8_t ack[LM_HEADER_SIZE + 1];
                uint8_t ok = 0x00;
                uint16_t alen = lm_build_frame(ack, sizeof(ack),
                                               LM_MSG_BIND_ACK,
                                               hdr->src_id,
                                               LM_CLUSTER_BINDING, 0,
                                               0, 0, &ok, 1);
                otIp6Address src_ip;
                lm_resolve_ip(hdr->src_id, &src_ip);
                lm_udp_send(&src_ip, ack, alen);
            }
            break;
        }
        default:
            break;
    }
}

/* ──────────────────────────────────────────────
   DISCOVER
   ────────────────────────────────────────────── */
esp_err_t lm_discover(void)
{
    /* Build a compact device descriptor payload */
    uint8_t payload[32];
    uint8_t idx = 0;
    payload[idx++] = s_device->endpoint_count;
    for (int i = 0; i < s_device->endpoint_count && idx < 28; i++) {
        lm_endpoint_t *ep = &s_device->endpoints[i];
        payload[idx++] = ep->endpoint_id;
        payload[idx++] = (uint8_t)ep->device_type;
        memcpy(&payload[idx], ep->name, 8);
        idx += 8;
    }

    uint8_t buf[LM_MAX_FRAME];
    uint16_t frame_len = lm_build_frame(buf, sizeof(buf),
                                         LM_MSG_DISCOVER_RESP,
                                         LM_BCAST_ID,
                                         LM_CLUSTER_BASIC, 0,
                                         0, 0, payload, idx);
    otIp6Address bcast;
    otIp6AddressFromString(LM_BCAST_ADDR, &bcast);
    return lm_udp_send(&bcast, buf, frame_len);
}

/* ──────────────────────────────────────────────
   INIT & START
   ────────────────────────────────────────────── */
esp_err_t lm_init(lm_device_t *device)
{
    if (!device) return ESP_ERR_INVALID_ARG;
    s_device = device;
    memset(s_bindings, 0, sizeof(s_bindings));
    memset(s_handlers, 0, sizeof(s_handlers));
    s_handler_count = 0;
    s_seq = 0;
    s_mutex = xSemaphoreCreateMutex();
    lm_get_node_id(s_device->node_id);
    ESP_LOGI(TAG, "LightMesh init. Node: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
             device->node_id[0], device->node_id[1],
             device->node_id[2], device->node_id[3],
             device->node_id[4], device->node_id[5],
             device->node_id[6], device->node_id[7]);
    return ESP_OK;
}

esp_err_t lm_start(void)
{
    otSockAddr sock_addr;
    memset(&sock_addr, 0, sizeof(sock_addr));
    sock_addr.mPort = LM_UDP_PORT;

    otError err = otUdpOpen(get_ot(), &s_socket, lm_udp_recv_cb, NULL);
    if (err != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "otUdpOpen failed: %d", err);
        return ESP_FAIL;
    }
    err = otUdpBind(get_ot(), &s_socket, &sock_addr, OT_NETIF_THREAD);
    if (err != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "otUdpBind failed: %d", err);
        return ESP_FAIL;
    }

    /* Join realm-local multicast group */
    otIp6Address mcast_addr;
    otIp6AddressFromString(LM_BCAST_ADDR, &mcast_addr);
    otIp6SubscribeMulticastAddress(get_ot(), &mcast_addr);

    ESP_LOGI(TAG, "LightMesh started on port %d", LM_UDP_PORT);
    lm_discover();   /* Announce ourselves */
    return ESP_OK;
}

void lm_stop(void)
{
    otUdpClose(get_ot(), &s_socket);
    ESP_LOGI(TAG, "LightMesh stopped");
}
