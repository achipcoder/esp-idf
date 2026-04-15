# LightMesh Protocol (LMP) — Reference

## Architecture

```
┌───────────────────────────────────────────────────────┐
│            APPLICATION  (main.c — device role)        │
│   Light     Dimmer       Fan         Switch           │
├───────────────────────────────────────────────────────┤
│           CLUSTER LAYER  (lm_clusters.c)              │
│  OnOff(0006) Level(0008) Fan(0202) Switch(003B)       │
├───────────────────────────────────────────────────────┤
│           LightMesh Protocol  (lm_protocol.c)         │
│  Frame build/parse · Binding engine · Dispatch        │
├───────────────────────────────────────────────────────┤
│           UDP / IPv6  (OpenThread mesh-local)          │
│           Port 5700  ·  Multicast ff03::1             │
├───────────────────────────────────────────────────────┤
│           OpenThread 1.3  (802.15.4 radio)            │
│           ESP32-H2 / ESP32-C6                         │
└───────────────────────────────────────────────────────┘
```

---

## Packet Frame Layout (28-byte header)

```
Offset  Size  Field           Description
──────  ────  ─────────────   ──────────────────────────────────────
  0      2    magic           0x4C 0x4D  ('L' 'M')
  2      1    version         0x01
  3      1    msg_type        see Message Types below
  4      2    seq             rolling sequence (LE)
  6      8    src_id          EUI-64 of sender
 14      8    dst_id          EUI-64 of destination (0xFF×8 = bcast)
 22      2    cluster_id      cluster (LE)
 24      2    cmd_attr_id     command or attribute ID (LE)
 26      1    endpoint        endpoint number (0xFF = all)
 27      1    flags           bit0=ACK_req  bit1=group  bit2=bound
 26*     2    payload_len     bytes of payload that follow
 28+     n    payload         command-specific data
```

---

## Message Types

| Hex   | Name           | Direction      | Purpose                        |
|-------|----------------|----------------|--------------------------------|
| 0x01  | COMMAND        | unicast/bcast  | Send command to cluster        |
| 0x02  | RESPONSE       | unicast        | ACK to a command               |
| 0x03  | READ_REQ       | unicast        | Request an attribute value     |
| 0x04  | READ_RESP      | unicast        | Attribute value reply          |
| 0x05  | EVENT          | unicast/bcast  | Unsolicited state change       |
| 0x10  | BIND_REQ       | unicast        | Create a binding               |
| 0x11  | BIND_ACK       | unicast        | Binding confirmed              |
| 0x12  | UNBIND_REQ     | unicast        | Remove a binding               |
| 0x20  | DISCOVER       | broadcast      | Find all devices               |
| 0x21  | DISCOVER_RESP  | unicast        | Device self-description        |
| 0x30  | HEARTBEAT      | broadcast      | Keepalive                      |

---

## Clusters & Commands

### OnOff — 0x0006
| Cmd  | Name    | Payload  |
|------|---------|----------|
| 0x00 | OFF     | —        |
| 0x01 | ON      | —        |
| 0x02 | TOGGLE  | —        |

### Level (Dimmer) — 0x0008
| Cmd  | Name      | Payload                                    |
|------|-----------|--------------------------------------------|
| 0x00 | MOVE_TO   | `uint8 level` `uint16 transition_ms` (LE)  |
| 0x01 | MOVE      | `uint8 mode(0=up/1=dn)` `uint8 rate`       |
| 0x02 | STEP      | `uint8 mode` `uint8 step_size`             |
| 0x03 | STOP      | —                                          |

### Fan — 0x0202
| Cmd  | Name      | Payload                                    |
|------|-----------|--------------------------------------------|
| 0x00 | SET_MODE  | `uint8 mode` (0=off 1=low 2=med 3=hi 4=auto 5=boost) |
| 0x01 | SET_SPEED | `uint8 percent` (0–100)                    |

### Switch — 0x003B  (producer only)
| Cmd  | Name    | Binding forward target        |
|------|---------|-------------------------------|
| 0x00 | PRESS   | → OnOff TOGGLE                |
| 0x01 | HOLD    | → Level MOVE (dim down)       |
| 0x02 | RELEASE | → Level STOP                  |

---

## Binding Table

Each entry stores:

```
[ src_id(8) | src_ep(1) | src_cluster(2) | dst_id(8) | dst_ep(1) | dst_cluster(2) ]
```

When a switch press is detected, `lm_invoke_bindings()` scans the table,
finds all entries matching `(src_id, src_ep, src_cluster)`, and sends the
forwarded command to each bound destination over the Thread mesh.

Maximum 16 bindings per node (configurable via `LM_MAX_BINDINGS`).

---

## Device Commissioning Flow

```
1. Flash all nodes with same Thread credentials
   (network name, PAN-ID, XPANID, network key, channel)

2. Power on nodes → Thread mesh self-forms

3. Switch node: call lm_bind() with target EUI-64s
   (obtained via serial log or lm_discover())

4. lm_discover() broadcast → all nodes reply with
   device type + endpoint list

5. Bindings stored in RAM (persist to NVS with
   nvs_set_blob / nvs_get_blob — easy extension)
```

---

## Extending to More Device Types

Add a new cluster file following the pattern in `lm_clusters.c`:

```c
// 1. Define cluster ID in lm_protocol.h
#define LM_CLUSTER_THERMOSTAT  0x0201

// 2. Create state struct + handler in lm_clusters.h / .c
typedef struct { int16_t setpoint; ... } lm_thermo_ctx_t;
void lm_thermo_handler(const lm_header_t*, const uint8_t*, uint16_t, lm_thermo_ctx_t*);

// 3. Register in main.c
lm_register_handler(LM_CLUSTER_THERMOSTAT, dispatch_thermo);
```

---

## Security Note

LMP uses no certificates. The Thread network key provides link-layer
encryption (AES-CCM-128) which protects all frames on the 802.15.4 layer.
An optional application-layer XOR obfuscation key (`LM_SHARED_KEY_LEN`)
can be added for additional application privacy, but is not a substitute
for proper crypto in production deployments.
