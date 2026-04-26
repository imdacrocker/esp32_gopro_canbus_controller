# API Reference

This document covers all three API layers of the ESP32 GoPro CAN Bus Controller:

1. [HTTP REST API](#http-rest-api) — served by the built-in Wi-Fi management interface.
2. [CAN Bus Protocol](#can-bus-protocol) — the binary frame protocol used between the controller and RaceCapture.
3. [C Component APIs](#c-component-apis) — the public C headers for each firmware component.

### HTTP endpoint summary

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/` | Embedded web UI |
| `GET` | `/api/status` | Remembered and connected camera counts |
| `POST` | `/api/scan` | Start 30 s BLE discovery scan |
| `POST` | `/api/scan-cancel` | Cancel running discovery scan |
| `GET` | `/api/cameras` | Discovered BLE cameras (poll during scan) |
| `POST` | `/api/pair` | Pair a discovered BLE camera |
| `GET` | `/api/paired-cameras` | All camera slots with live status |
| `POST` | `/api/remove-camera` | Remove a paired BLE camera |
| `POST` | `/api/shutter` | Start or stop recording (all cameras or one slot) |
| `GET` | `/api/logging-state` | Current RaceCapture logging state |
| `GET` | `/api/utc` | Current GPS-derived UTC timestamp |
| `GET` | `/api/auto-control` | Read automatic CAN-driven recording flag |
| `POST` | `/api/auto-control` | Enable or disable automatic CAN-driven recording |
| `GET` | `/api/legacy/discovered` | Unmanaged Wi-Fi stations with DHCP addresses |
| `POST` | `/api/legacy/add` | Probe and add a Hero4 as a managed camera |
| `POST` | `/api/legacy/remove` | Remove a managed legacy camera |
| `POST` | `/api/factory-reset` | Erase all NVS data and reboot |

---

## HTTP REST API

The controller runs a Wi-Fi access point (SSID `HERO-RC-XXXXXX`, IP `10.71.79.1`) and an HTTP server on port 80. All API endpoints return `application/json`.

### Base URL

```
http://10.71.79.1
```

---

### `GET /`

Serves the embedded web management UI (`index.html`). Open this in a browser after connecting to the controller's Wi-Fi network.

**Response:** `text/html` — the single-page management application.

---

### `GET /api/status`

Returns the count of remembered (paired) and currently connected cameras.

**Response**

```json
{
  "remembered": 2,
  "connected": 1
}
```

| Field | Type | Description |
|-------|------|-------------|
| `remembered` | integer | Number of cameras stored in NVS (persist across reboots). |
| `connected` | integer | Number of cameras with an active BLE connection right now. |

---

### `POST /api/scan`

Starts a 30-second BLE discovery scan for nearby GoPro cameras. Only devices advertising the GoPro service UUID `0xFEA6` are added to the discovery list.

**Request body:** none

**Response**

```json
{ "status": "scanning" }
```

Poll `GET /api/cameras` every second after calling this endpoint to retrieve discovered cameras as they appear.

---

### `POST /api/scan-cancel`

Cancels a running discovery scan and resumes the passive background scan (if any configured camera is not yet connected), or leaves the radio idle (if all cameras are already connected).

Safe to call even when no scan is active — the call is a no-op in that case.

**Request body:** none

**Response**

```json
{ "status": "cancelled" }
```

---

### `GET /api/cameras`

Returns the list of GoPro cameras discovered during the most recent scan. This includes all discovered devices regardless of pairing status — the web UI filters out already-paired cameras client-side by cross-referencing `/api/paired-cameras` before rendering the list.

**Response** — array of discovered camera objects (may be empty):

```json
[
  {
    "name": "GoPro HERO11 Black",
    "addr": "AA:BB:CC:DD:EE:FF",
    "addr_type": 0,
    "rssi": -62
  }
]
```

| Field | Type | Description |
|-------|------|-------------|
| `name` | string | Advertised device name. Falls back to `GoPro XXYY` if no name is present. |
| `addr` | string | BLE MAC address in `XX:XX:XX:XX:XX:XX` format. |
| `addr_type` | integer | BLE address type: `0` = public, `1` = random. Required by `/api/pair`. |
| `rssi` | integer | Signal strength in dBm at time of discovery. |

---

### `POST /api/pair`

Initiates a BLE connection and pairing sequence with a specific camera. The camera must be advertising (powered on with Bluetooth enabled) when this call is made.

Calling this endpoint cancels any running discovery scan immediately and puts the BLE controller into initiating mode — the firmware connects as soon as the target camera's advertisement is seen, without waiting for further scan results.

On success, the camera is registered with `camera_manager`, bonded, and persisted to NVS. It will reconnect automatically on every subsequent boot.

**Request body** (`Content-Type: application/json`):

```json
{
  "addr": "AA:BB:CC:DD:EE:FF",
  "addr_type": 0
}
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `addr` | string | Yes | Camera BLE address from `/api/cameras`. |
| `addr_type` | integer | No | Address type from `/api/cameras`. Defaults to `0` (public) if omitted. |

**Response**

```json
{ "status": "pairing" }
```

The response is returned immediately; pairing happens asynchronously in the background. Watch the serial log or poll `/api/paired-cameras` for the result.

**Error responses**

| HTTP status | Body | Cause |
|-------------|------|-------|
| 400 | `Empty body` | Request body was missing. |
| 400 | `Missing addr` | JSON field `"addr"` was not found. |
| 400 | `Bad addr format` | MAC address could not be parsed. |

---

### `GET /api/paired-cameras`

Returns all configured camera slots with their current live status.

**Response** — array of configured camera slot objects. Unconfigured slots are omitted.

```json
[
  {
    "slot": 0,
    "index": 1,
    "name": "GoPro EEFF",
    "addr": "AA:BB:CC:DD:EE:FF",
    "status": "recording"
  },
  {
    "slot": 1,
    "index": 2,
    "name": "GoPro 5566",
    "addr": "11:22:33:44:55:66",
    "status": "disconnected"
  }
]
```

| Field | Type | Description |
|-------|------|-------------|
| `slot` | integer | 0-based slot index used for API calls such as `/api/remove-camera`. |
| `index` | integer | 1-based display number shown in the web UI. |
| `name` | string | Camera name (advertised name at pairing time, e.g. `"GoPro XXYY"`). |
| `addr` | string | BLE MAC address of the paired camera. |
| `status` | string | Live status string (see table below). |

**Status values**

| Value | Meaning |
|-------|---------|
| `"disconnected"` | Camera is paired but no active BLE connection. |
| `"not_recording"` | Connected and camera-ready, but not currently recording. |
| `"recording"` | Connected and actively recording. |

---

### `POST /api/remove-camera`

Removes a single paired camera. Clears the camera slot from NVS and RAM, terminates its active BLE connection (if any), and deletes its BLE bond from the NimBLE bond store. The camera will need to be re-paired via `/api/scan` and `/api/pair` to reconnect.

**Request body** (`Content-Type: application/json`):

```json
{ "slot": 0 }
```

| Field | Type | Description |
|-------|------|-------------|
| `slot` | integer | 0-based slot index from `/api/paired-cameras`. |

**Response**

```json
{ "status": "removed" }
```

**Error responses**

| HTTP status | Body | Cause |
|-------------|------|-------|
| 400 | `Empty body` | Request body was missing. |
| 400 | `Missing slot` | JSON field `"slot"` was not found. |
| 400 | `Invalid slot` | Slot index is out of range. |
| 400 | `Slot not configured` | Slot exists but has no camera assigned. |

---

### `POST /api/shutter`

Manually sends a start or stop recording command to all currently connected, camera-ready cameras.

This command **does** update the desired recording state tracked by `camera_manager`. If a camera is currently disconnected and reconnects later, the tick timer will automatically retry the command to bring it into the desired state. Sending `{"on": false}` clears the desired state, preventing the tick timer from re-enabling recording even if CAN previously set it.

> **Note:** If RaceCapture is actively sending `0x600` logging frames **and Automatic Control is enabled**, those will overwrite the desired state on the next frame received. The web UI and CAN path will "fight" each other — this is expected behaviour. The web UI shutter buttons are intended for diagnostics when CAN is disconnected, or when Automatic Control has been disabled via `/api/auto-control`.

**Request body** (`Content-Type: application/json`):

```json
{ "on": true }
```

```json
{ "slot": 0, "on": true }
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `on` | boolean | Yes | `true` to start recording; `false` to stop. |
| `slot` | integer | No | 0-based slot index from `/api/paired-cameras`. If present, the command targets only that camera slot and updates only that slot's desired recording state. If absent, the command targets all cameras and updates all slots' desired recording state (original behaviour). |

**Response**

```json
{ "dispatched": 2 }
```

| Field | Type | Description |
|-------|------|-------------|
| `dispatched` | integer | Number of cameras that received the command. `0` means no cameras were ready (the desired state is still updated and the tick timer will retry). |

**Error responses**

| HTTP status | Body | Cause |
|-------------|------|-------|
| 400 | `Empty body` | Request body was missing. |
| 400 | `Missing 'on' field` | JSON field `"on"` was not found. |
| 400 | `Invalid 'on' value` | `"on"` was not `true` or `false`. |

---

### `GET /api/logging-state`

Returns the current RaceCapture logging state as last reported on CAN ID `0x600`. The state reverts to `"unknown"` if no `0x600` frame has been received within the past 5 seconds (configurable via `CAN_MANAGER_LOGGING_TIMEOUT_MS`).

**Response**

```json
{ "state": "logging" }
```

| Value | Meaning |
|-------|---------|
| `"logging"` | RaceCapture is currently logging (`isLogging = 1`). |
| `"not_logging"` | RaceCapture is present but not logging (`isLogging = 0`). |
| `"unknown"` | No `0x600` frame received recently — RaceCapture may be absent or rebooting. |

---

### `GET /api/utc`

Returns the current best-estimate UTC derived from the RaceCapture GPS clock broadcast on CAN ID `0x602`. The ESP32 extrapolates forward from the last received frame using its monotonic timer (`esp_timer_get_time`), so sub-second accuracy is maintained between CAN frames.

**Response — GPS lock acquired:**

```json
{ "valid": true, "epoch_ms": 1744727527412 }
```

**Response — GPS lock not yet acquired:**

```json
{ "valid": false }
```

| Field | Type | Description |
|-------|------|-------------|
| `valid` | boolean | `true` if a valid `0x602` frame has been received (year > 2020). |
| `epoch_ms` | integer | Milliseconds since Unix epoch (only present when `valid` is `true`). |

---

### `GET /api/auto-control`

Returns the current state of the automatic camera control flag.

When `enabled` is `true` (the default on every boot), `isLogging` transitions received on the CAN bus automatically start and stop recording on all cameras. When `false`, CAN logging transitions are ignored — cameras hold their current state and can only be controlled manually via the web UI or `/api/shutter`.

The flag is never stored in NVS. It always resets to `true` when the controller reboots.

**Response**

```json
{ "enabled": true }
```

| Field | Type | Description |
|-------|------|-------------|
| `enabled` | boolean | `true` = automatic control active; `false` = manual override. |

---

### `POST /api/auto-control`

Enable or disable automatic camera control.

Changing this flag does **not** affect the current recording state of any camera. Cameras continue whatever they were doing. The flag only gates future `0x600` CAN logging transitions.

When re-enabling automatic control, the controller does **not** immediately resync to the current CAN logging state — it waits for the next `isLogging` transition from RaceCapture.

**Request body** (`Content-Type: application/json`):

```json
{ "enabled": false }
```

| Field | Type | Description |
|-------|------|-------------|
| `enabled` | boolean | `true` to enable automatic control; `false` to suppress CAN-driven recording. |

**Response** — reflects the state that was applied:

```json
{ "enabled": false }
```

**Error responses**

| HTTP status | Body | Cause |
|-------------|------|-------|
| 400 | `Empty body` | Request body was missing. |
| 400 | `Missing 'enabled' field` | JSON field `"enabled"` was not found. |
| 400 | `Invalid 'enabled' value` | `"enabled"` was not `true` or `false`. |

---

---

### `GET /api/legacy/discovered`

Returns the list of Wi-Fi stations currently connected to the controller's Soft-AP that are **not** already managed (i.e. not yet in `camera_manager`), have a DHCP-assigned IP address, and are not the requesting device itself.

Use this endpoint to find Hero4 cameras that have connected to the AP and are waiting to be added. Poll this endpoint after connecting a Hero4 in RC Mode — the device will typically appear within a few seconds of obtaining a DHCP lease.

**Response** — array of discovered station objects (may be empty):

```json
[
  {
    "addr": "D8:96:85:AA:BB:CC",
    "ip": "10.71.79.2"
  }
]
```

| Field | Type | Description |
|-------|------|-------------|
| `addr` | string | Station Wi-Fi MAC address in `XX:XX:XX:XX:XX:XX` format. |
| `ip` | string | DHCP-assigned IPv4 address in dotted-decimal format. |

Stations that have not yet completed DHCP negotiation are omitted; they appear automatically once their address is assigned.

---

### `POST /api/legacy/add`

Probes a connected station over HTTP and adds it as a managed legacy camera if it responds to the Hero4 `gpControl` API.

The operation is **asynchronous** — the controller posts a probe task to the `legacy_gopro` internal queue and responds immediately. The HTTP probe (up to 3 retries, 5 s timeout) runs in the background. Poll `GET /api/legacy/discovered` after approximately 15 seconds: if the MAC has disappeared, the camera was successfully promoted to managed status and will appear in `GET /api/paired-cameras`. If the MAC is still present, the probe failed.

On success, the camera's MAC and IP are saved to NVS. On subsequent reconnects, the controller auto-promotes the camera without requiring user action.

**Request body** (`Content-Type: application/json`):

```json
{
  "addr": "D8:96:85:AA:BB:CC",
  "ip": "10.71.79.2"
}
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `addr` | string | Yes | Wi-Fi MAC address from `GET /api/legacy/discovered`. |
| `ip` | string | No | Station IP from `GET /api/legacy/discovered`. If omitted or `0`, the probe attempts to use the IP stored in the station table. |

**Response**

```json
{ "status": "adding" }
```

**Error responses**

| HTTP status | Body | Cause |
|-------------|------|-------|
| 400 | `Empty body` | Request body was missing. |
| 400 | `Missing addr` | JSON field `"addr"` was not found. |
| 400 | `Bad addr format` | MAC address could not be parsed. |
| 500 | `Queue full` | Internal command queue is full — retry after a moment. |

---

### `POST /api/legacy/remove`

Removes a managed legacy camera. Unregisters the camera from `camera_manager`, frees its slot, and deletes its MAC and IP from NVS. The camera's Wi-Fi connection to the AP is not dropped — if still physically connected, it will reappear in `GET /api/legacy/discovered` after removal.

**Request body** (`Content-Type: application/json`):

```json
{ "addr": "D8:96:85:AA:BB:CC" }
```

| Field | Type | Description |
|-------|------|-------------|
| `addr` | string | Wi-Fi MAC address of the managed camera to remove. |

**Response**

```json
{ "status": "removed" }
```

**Error responses**

| HTTP status | Body | Cause |
|-------------|------|-------|
| 400 | `Empty body` | Request body was missing. |
| 400 | `Missing addr` | JSON field `"addr"` was not found. |
| 400 | `Bad addr format` | MAC address could not be parsed. |
| 500 | `Queue full` | Internal command queue is full — retry after a moment. |

---

### `POST /api/factory-reset`

Erases all NVS partitions (paired cameras, legacy camera records, all stored settings) and reboots the ESP32. **This is irreversible.** All paired BLE cameras and managed legacy cameras must be re-added after a factory reset.

The HTTP response is flushed before the erase begins so the caller receives a confirmation before the device goes offline.

**Request body:** none

**Response** (received before the device reboots):

```json
{ "status": "rebooting" }
```

The device will be unreachable for approximately 5–10 seconds while it restarts and the AP reinitialises.

---

## CAN Bus Protocol

The controller communicates with RaceCapture over a **1 Mbps CAN bus** using **standard 11-bit frame IDs** and classic (non-FD) 8-byte data frames.

### Physical layer

| Parameter | Value |
|-----------|-------|
| Bitrate | 1 Mbps (`CAN_MANAGER_BITRATE_BPS`) |
| Frame format | Standard 11-bit IDs |
| Data length | 8 bytes |
| Termination | Hardware jumper (120 Ω at each bus end) |

### Message IDs

| Direction | CAN ID (hex) | CAN ID (decimal) | Rate | Purpose |
|-----------|-------------|-----------------|------|---------|
| RaceCapture → ESP32 | `0x600` | 1536 | ~10 Hz | Logging control |
| ESP32 → RaceCapture | `0x601` | 1537 | 5 Hz (fixed) | Camera status |
| RaceCapture → ESP32 | `0x602` | 1538 | 25 Hz | UTC timestamp |

---

### `0x600` — RaceCapture → ESP32 (Command frame)

Sent by RaceCapture to command the controller's logging state. The controller fires its internal `on_logging_state_changed` callback only when `isLogging` changes value — repeated identical frames are silently ignored. If automatic camera control has been disabled via `/api/auto-control`, the callback still fires but returns immediately without acting on the cameras.

| Byte | Field | Type | Values |
|------|-------|------|--------|
| 0 | `isLogging` | uint8 | `0` = not logging, `1` = logging |
| 1–7 | Reserved | — | `0x00` |

---

### `0x601` — ESP32 → RaceCapture (Camera status frame)

Broadcast by the controller at 5 Hz regardless of whether the bus is active. Each byte reports the state of one camera slot.

> **Numbering note:** Cameras are numbered 1–4 in the HTTP API and web UI. In this CAN frame the byte *offset* is 0-based (byte 0 = Camera 1, byte 1 = Camera 2, etc.) — this is a CAN frame layout convention and matches the RaceCapture Direct CAN Mapping offsets.

| Byte | Field | Type | Values |
|------|-------|------|--------|
| 0 | Camera 1 state | uint8 | See `camera_state_t` below |
| 1 | Camera 2 state | uint8 | See `camera_state_t` below |
| 2 | Camera 3 state | uint8 | See `camera_state_t` below |
| 3 | Camera 4 state | uint8 | See `camera_state_t` below |
| 4–7 | Reserved | — | `0x00` |

---

### `0x602` — RaceCapture → ESP32 (UTC timestamp frame)

Broadcast by the RaceCapture Lua script at 25 Hz once GPS lock is acquired. Transmission is suppressed when GPS lock has not yet been established (the Lua script checks `getDateTime()` for year ≤ 1970).

| Bytes | Field | Type | Description |
|-------|-------|------|-------------|
| 0–7 | `epoch_ms` | uint64, little-endian | Milliseconds since Unix epoch (Jan 1 1970 00:00:00.000 UTC) |

The controller stores the received value alongside the ESP32 monotonic clock timestamp at the moment of receipt. `can_manager_get_utc_ms()` extrapolates forward from this pair so the caller always gets a current estimate between CAN frames.

On the first valid frame received, the controller logs a human-readable timestamp at INFO level and fires the registered `can_utc_acquired_cb_t` callback, e.g.:
```
UTC acquired — 2026-04-15 14:32:07.412 UTC  (1744727527412 ms epoch)
```

> **ID note:** `0x602` is a temporary assignment used until the RaceCapture developer ships native UTC broadcast support. When that firmware is available, remove the Lua script and update `CAN_ID_RC_UTC` in `can_manager.h` to avoid two nodes transmitting on the same ID.

---

### `camera_state_t` enumeration

```c
typedef enum {
    CAMERA_STATE_UNDEFINED    = 0,  // Slot not configured / no information yet
    CAMERA_STATE_DISCONNECTED = 1,  // Camera not found or connection lost
    CAMERA_STATE_IDLE         = 2,  // Connected, not recording
    CAMERA_STATE_RECORDING    = 3,  // Connected and actively recording
} camera_state_t;
```

### RaceCapture Direct CAN Mapping (0x601)

Configure one channel per camera slot in the RaceCapture **Direct CAN Mapping** page:

| Parameter | Value |
|-----------|-------|
| CAN ID | 1537 (decimal) |
| Source Type | Unsigned |
| Length | 1 byte |
| Offset | 0 (Cam1), 1 (Cam2), 2 (Cam3), 3 (Cam4) |
| Multiplier | 1 |
| Adder | 0 |
| Min | 0 |
| Max | 3 |

> **Lua index note:** RaceCapture Lua arrays are 1-indexed. `data[1]` corresponds to CAN byte offset 0. The Direct CAN Mapping page uses 0-indexed offsets directly.

---

## C Component APIs

All public headers use Doxygen-compatible `/** ... */` comment blocks. The sections below summarise each component's interface; refer to the header files for the authoritative parameter and return-value documentation.

---

### `ble_core`

**Header:** `components/ble_core/include/ble_core.h`

NimBLE stack abstraction. Manages scanning, connection lifecycle, encryption, and GATT writes. Camera-agnostic — all camera-specific logic is handled via registered callbacks.

#### Types

**`ble_core_callbacks_t`** — aggregated event callback table.

```c
typedef struct {
    ble_core_on_disc_cb_t                  on_disc;                  // advertisement seen during discovery
    ble_core_on_connected_cb_t             on_connected;             // connection established
    ble_core_on_encrypted_cb_t             on_encrypted;             // link encrypted — safe to use GATT
    ble_core_on_disconnected_cb_t          on_disconnected;          // connection dropped
    ble_core_on_notify_rx_cb_t             on_notify_rx;             // ATT notification received
    ble_core_is_known_addr_cb_t            is_known_addr;            // returns true for registered cameras
    ble_core_has_disconnected_cameras_cb_t has_disconnected_cameras; // returns true if any paired camera is not connected
} ble_core_callbacks_t;
```

All fields are optional; set unused callbacks to `NULL`.

The `has_disconnected_cameras` callback controls background scan suppression. When provided and it returns `false` (all known cameras are connected), background scans are skipped automatically. When `NULL`, background scans always restart as before.

#### Functions

```c
void ble_core_register_callbacks(const ble_core_callbacks_t *cbs);
```
Register the callback table. Must be called before `ble_core_init()`. Copies the struct by value.

---

```c
void ble_core_init(void);
```
Initialise the NimBLE stack, configure the security manager for bonding (no I/O / Just Works), and launch the host task. On `on_sync`, automatically attempts to reconnect all stored bonds. If bonded peers exist but some are not yet connected, a passive background scan is started. If no peers are paired, or all paired cameras are already connected, the background scan is suppressed (requires `has_disconnected_cameras` callback to be registered).

---

```c
void ble_core_start_discovery(void);
```
Start a 30-second passive scan that surfaces all advertisement packets to `on_disc`. Deduplication is disabled. Safe to call from any task.

---

```c
void ble_core_stop_discovery(void);
```
Cancel a running discovery scan and resume the background scan.

---

```c
void ble_core_connect_by_addr(const ble_addr_t *addr);
```
Cancel any running scan and immediately initiate a direct BLE connection to `addr`. Posts an event to the NimBLE host task that calls `ble_gap_disc_cancel()` then `ble_gap_connect()`, putting the controller into initiating mode. The controller connects as soon as the peer starts advertising — no advertisement needs to be seen before this call. If a connection attempt is already in progress the request is silently ignored. Safe to call from any task.

---

```c
void ble_core_purge_unknown_bonds(const ble_addr_t *keep, int keep_count);
```
Delete all NimBLE bonds except those in the `keep` array. Pass `NULL` / `0` to delete all bonds.

---

```c
void ble_core_remove_bond(const ble_addr_t *addr);
```
Remove the BLE bond for a single camera. Terminates the active connection to `addr` (if one exists) and deletes `addr`'s entry from the NimBLE peer-security store. The caller must remove the camera from `camera_manager` first so that `is_known_addr` returns false before the disconnect event fires, preventing an automatic reconnect. Safe to call from any task — the actual work is posted to the NimBLE host task.

---

```c
esp_err_t ble_core_gatt_write(uint16_t conn_handle, uint16_t attr_handle,
                               const uint8_t *data, uint16_t len);
```
Send an ATT Write Without Response command. Returns `ESP_OK` on success.

---

### `open_gopro_ble`

**Header:** `components/open_gopro_ble/include/open_gopro_ble.h`

OpenGoPro BLE driver implementing the [OpenGoPro BLE 2.0](https://gopro.github.io/OpenGoPro/ble_2_0) protocol. Registers a `camera_driver_t` vtable with `camera_manager` and handles all GATT service discovery, notification parsing, TLV command encoding, keep-alive, and recording-status polling.

#### Internal file layout

| File | Responsibility |
|------|---------------|
| `control.c` | Camera control commands (start/stop recording, SetDateTime), `start_cmd_pending` guard, status poll timer (5 s), keep-alive timer (3 s) |
| `driver.c` | `camera_driver_t` vtable, per-camera context allocation, discovery list, component init |
| `gatt.c` | GATT service/characteristic discovery, MTU negotiation, CCCD subscription |
| `pairing.c` | BLE lifecycle callbacks: connected, encrypted, disconnected |
| `notify.c` | ATT notification routing (recording status transitions, `start_cmd_pending` management, Preset Protobuf responses on GP-0077, command responses) |
| `presets.c` | Two-phase Video preset loading: `RequestGetPresetStatus` (Phase 1) + `NotifyPresetStatus` parse + `Load Preset` (Phase 2); GPBS reassembly for fragmented GP-0077 responses |
| `query.c` | On-demand query commands — `GetHardwareInfo` (0x3C) send + GPBS-aware response parsing; `Load Preset` (0x40) response handler |

#### Types

**`gopro_device_t`** — a camera found during a scan.

```c
typedef struct {
    char       name[32];   // Advertised device name
    ble_addr_t addr;       // BLE address (6 bytes + type)
    int8_t     rssi;       // Signal strength in dBm
} gopro_device_t;
```

**`gopro_gatt_handles_t`** — GATT attribute handles discovered per camera.

```c
typedef struct {
    uint16_t cmd_write;             // GP-0072: command write
    uint16_t cmd_resp_notify;       // GP-0073: command response notifications
    uint16_t settings_write;        // GP-0074: settings write (also used for Keep Alive)
    uint16_t settings_resp_notify;  // GP-0075: settings response notifications
    uint16_t query_write;           // GP-0076: query write
    uint16_t query_resp_notify;     // GP-0077: query response notifications
    uint16_t net_mgmt_cmd_write;    // GP-0091: network management command
    uint16_t net_mgmt_resp_notify;  // GP-0092: network management response
    uint16_t wifi_ssid_read;        // GP-0002: Wi-Fi AP SSID
    uint16_t wifi_pass_read;        // GP-0003: Wi-Fi AP password
    uint16_t wifi_power_write;      // GP-0001: Wi-Fi AP power
    uint16_t wifi_state_indicate;   // GP-0004: Wi-Fi AP state indications
} gopro_gatt_handles_t;
```

#### Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `GOPRO_MAX_DISCOVERED` | `10` | Maximum cameras held in the discovery list. |
| `STATUS_POLL_INTERVAL_MS` | `5000` | Interval for querying recording status from each connected camera. |
| `KEEP_ALIVE_INTERVAL_MS` | `3000` | Interval for sending the Keep Alive command to each connected camera. |

#### Functions

```c
void open_gopro_ble_init(void);
```
Initialise the OpenGoPro BLE component. Registers the `camera_driver_t` vtable with `camera_manager`, registers BLE event callbacks with `ble_core`, and starts the recording-status poll timer (5 s) and keep-alive timer (3 s). Must be called before `camera_manager_init()` and `ble_core_init()`.

---

```c
void open_gopro_ble_sync_time_all(void);
```
Send a SetDateTime command to every camera slot that is currently camera-ready. Called from `main.c`'s `can_utc_acquired_cb_t` handler to cover cameras that were already connected when GPS lock was first established. Slots that are not camera-ready are skipped silently. Safe to call from any task.

---

```c
void open_gopro_ble_start_discovery(void);
```
Clear the discovery list and start a 30-second BLE scan. Cameras are filtered by service UUID `0xFEA6` (GoPro). Safe to call from any task.

---

```c
void open_gopro_ble_stop_discovery(void);
```
Cancel a running discovery scan and resume the passive background scan (if any configured camera is not yet connected), or leave the radio idle if all cameras are already connected. No-op if no discovery scan is active. Safe to call from any task.

---

```c
int open_gopro_ble_get_discovered(gopro_device_t *out, int max_count);
```
Copy up to `max_count` discovered cameras into `out`. Returns the number of entries written.

---

```c
void open_gopro_ble_connect_by_addr(const ble_addr_t *addr);
```
Initiate connection and pairing with a specific camera address. Safe to call from any task.

---

```c
const camera_driver_t *open_gopro_ble_get_driver(void);
```
Return a pointer to the static GoPro `camera_driver_t` vtable.

---

```c
void *open_gopro_ble_create_driver_ctx(void);
```
Allocate and return a new, zeroed per-camera context. Returns `NULL` on OOM.

---

#### Keep-alive (`control.c`)

The OpenGoPro spec requires a Keep Alive packet to be sent every 3 seconds after a connection is established to prevent the camera from auto-sleeping. This is implemented as a global `esp_timer` in `control.c`.

- **Characteristic:** GP-0074 (`settings_write` handle)
- **Packet:** `{ 0x03, 0x5B, 0x01, 0x42 }` — TLV encoding: `[length=3][setting_id=0x5B][param_len=1][value=0x42]`
- **Fire-and-forget:** the camera's ACK response on GP-0075 is intentionally ignored. BLE disconnect handling covers connection loss.
- **Gating:** the timer fires every 3 seconds and iterates all camera slots. It only sends to slots where `camera_manager_is_camera_ready()` returns true — no explicit start/stop per connection is required.

#### Duplicate-send guard (`control.c` / `notify.c`)

The driver tracks a per-camera `start_cmd_pending` boolean in `gopro_ble_ctx_t` to prevent the `camera_manager` tick from hammering the camera with repeated start commands while it is still transitioning from idle to recording (a process that can take several seconds on GoPro hardware).

**State transitions:**

| Event | Effect on `start_cmd_pending` |
|-------|-------------------------------|
| `control_start_recording()` called | Set to `true` (command dispatched) |
| `control_start_recording()` called while already `true` | Returns `ESP_ERR_INVALID_STATE` immediately — no BLE write |
| Status poll confirms IDLE → RECORDING | Cleared to `false` (happy path) |
| Status poll confirms RECORDING → IDLE | Cleared to `false` (recovery — tick may now resend) |
| `control_stop_recording()` called | Cleared to `false` (explicit stop) |
| Camera disconnects | Cleared to `false` (context reset in `pairing.c`) |

**Recovery path:** when the camera transitions from RECORDING back to IDLE while `desired_recording` is still `true`, clearing `start_cmd_pending` lets the `camera_manager` tick dispatch a fresh start command on its next 2-second cycle. The gap between the status poll confirming IDLE and the tick firing is at most `STATUS_POLL_INTERVAL_MS` + 2000 ms.

**Lost-command caveat:** if the BLE write succeeds but the camera silently discards the command (e.g., RF collision) and never enters RECORDING, `start_cmd_pending` remains `true` indefinitely. The system will not retry. This is a deliberate trade-off — racing use cases prioritise getting all cameras started simultaneously over silent retry-until-success behaviour.

---

#### On-demand queries (`query.c`)

Hardware info and other query commands can be issued at any time after the slot is camera-ready. This is implemented in `query.c` and is not part of the public API.

**`SetDateTime` (cmd 0x0D)**

Sets the camera's clock to the current UTC. Sent via one of two paths depending on which is available first — the camera connection or the UTC:

- **Camera connects after UTC is available:** `readiness.c` calls `control_send_set_date_time()` directly when the readiness poll succeeds (status 0 returned from `GetHardwareInfo`), on every connection (first pairing and all reconnections).
- **UTC arrives while cameras are already connected:** `can_manager` fires the `can_utc_acquired_cb_t` callback exactly once. `main.c` responds by calling `open_gopro_ble_sync_time_all()`, which iterates all camera-ready slots and calls `control_send_set_date_time()` for each.

In both cases, `control_send_set_date_time()` reads the current UTC from `can_manager_get_utc_ms()`, converts it to calendar fields via `gmtime_r`, and writes the following TLV command to GP-0072 (`cmd_write`):

| Byte | Value | Description |
|------|-------|-------------|
| 0 | `0x09` | GPBS single-packet length (9 bytes follow) |
| 1 | `0x0D` | SetDateTime command ID |
| 2 | `0x07` | Parameter length (7 bytes) |
| 3–4 | year | `uint16`, big-endian |
| 5 | month | 1–12 |
| 6 | day | 1–31 |
| 7 | hour | 0–23 |
| 8 | minute | 0–59 |
| 9 | second | 0–59 |

The response arrives on GP-0073 (`cmd_resp_notify`) and is dispatched to `gopro_query_handle_cmd_response()`:

- Status `0x00` → logged at INFO: `SetDateTime accepted — camera clock updated`
- Any other status → logged at WARN with the rejection code

If UTC is not yet available when `control_send_set_date_time()` is called (e.g. GPS lock not yet acquired on the RaceCapture), the command is skipped with a warning and will not be retried for that connection.

**Video Preset Loading (two-phase flow)**

Initiated automatically by `readiness.c` when the readiness poll succeeds (status 0 from `GetHardwareInfo`), on every connection (first pairing and all reconnections), alongside `SetDateTime` and `RequestPairingFinish`.

*Phase 1 — `gopro_presets_request_video()` (`presets.c`)*

Sends `RequestGetPresetStatus` to GP-0076 (`query_write`):

| Byte | Value | Description |
|------|-------|-------------|
| 0 | `0x06` | GPBS single-packet length (6 bytes follow) |
| 1 | `0xF5` | Feature ID — Preset |
| 2 | `0x72` | Action ID — RequestGetPresetStatus |
| 3–5 | `0x08 0x01 0x00` | Protobuf: `register_preset_status: false` (query only) |

*Phase 2 — `NotifyPresetStatus` parse + `Load Preset`*

The camera responds asynchronously on GP-0077 (`query_resp_notify`) with a `NotifyPresetStatus` Protobuf payload. For cameras with many custom presets (e.g. HERO13 Black), this response is fragmented across multiple ATT notifications using GPBS application-layer framing. `presets.c` manages per-connection reassembly (mirrors the pattern in `query.c` for `GetHardwareInfo`).

`notify.c` routes GP-0077 notifications to `presets.c` based on the GPBS frame type and Feature ID (`0xF5`):
- **Continuation fragment** (frame_type ≥ 2): always routed to `gopro_presets_handle_query_fragment()`
- **Single-packet** (frame_type = 0, Feature ID = 0xF5, Action ID = 0xF2): routed to `gopro_presets_handle_notify_status()`
- **First fragment** (frame_type = 1, Feature ID = 0xF5): starts reassembly in `gopro_presets_handle_query_fragment()`

Once the complete payload is assembled, `presets.c` parses the Protobuf to find the first preset in the Video group (`EnumPresetGroup = 1000`) and sends a `Load Preset` command to GP-0072 (`cmd_write`):

| Byte | Value | Description |
|------|-------|-------------|
| 0 | `0x06` | GPBS single-packet length (6 bytes follow) |
| 1 | `0x40` | Load Preset command ID |
| 2 | `0x04` | Parameter length (4 bytes) |
| 3–6 | preset ID | Big-endian `uint32` preset ID from `NotifyPresetStatus` |

The camera's response arrives on GP-0073 (`cmd_resp_notify`) and is handled by the `0x40` case in `gopro_query_handle_cmd_response()` (`query.c`):
- Status `0x00` → logged at INFO: `Load Preset accepted — camera in Video mode`
- Any other status → logged at WARN with the rejection code

Disconnect cleanup: `gopro_presets_free()` is called from `gopro_on_disconnected_cb()` in `pairing.c` to release any in-progress reassembly context.

---

**`SetShutter` response (cmd 0x01)**

After every `control_start_recording()` or `control_stop_recording()` call, the camera sends a command response on `cmd_resp_notify` (GP-0073). `gopro_query_handle_cmd_response()` in `query.c` handles these:

- Status `0x00` → logged at INFO: `SetShutter command accepted by camera`
- Any other status → logged at WARN with the rejection code

This makes it possible to determine whether the camera actually accepted the recording command, rather than only seeing the outgoing write in the log.

**`GetHardwareInfo` (cmd 0x3C) — readiness poll**

`gopro_query_send_hw_info(conn_handle)` is called automatically by `readiness.c` as the core of the readiness poll loop. `gatt.c` hands off to `readiness.c` once all CCCD subscriptions are complete; `readiness.c` then sends `GetHardwareInfo` and waits up to 3 seconds for a response.

The response arrives on `cmd_resp_notify` (GP-0073). `notify.c` routes this characteristic's notifications before the camera-ready gate (so responses arrive even before `camera_manager_set_camera_ready()` has been called). `gopro_query_handle_cmd_response()` extracts the status byte from the response and forwards it to `gopro_readiness_handle_hw_info_status()` in `readiness.c`:

- **Status `0x00` (success):** camera is ready. `readiness.c` calls `gopro_on_camera_ready()`, which sets the camera-ready flag, sends `SetDateTime`, sends `RequestPairingFinish` (first pairing only), and requests the Video preset.
- **Status `0x02` (camera busy / not ready):** `readiness.c` increments the retry counter and re-sends after 3 seconds.
- **Any other non-zero status:** treated the same as `0x02` — retry.
- **Timeout (no response within 3 s):** the `esp_timer` one-shot fires `readiness_timeout_cb()`, which re-sends the command and rearms the timer.
- **Retry limit (10 retries, ~30 s total):** `ble_gap_terminate()` is called to drop the connection cleanly. The bond is preserved; BLE auto-reconnect will retry on the next advertisement.

On a status-0 response, `query.c` also falls through to parse the full response payload. The parsed fields (model number, model name, firmware version, serial number, AP SSID, AP MAC address) are logged at INFO level, and the **model name** is written into the camera slot via `camera_manager_set_model_name()`. It is then available through `camera_slot_info_t.model_name` for display in the web UI.

The model name is **not** persisted to NVS — it is RAM-only and repopulated on every connection.

**GPBS fragmentation:** `GetHardwareInfoRsp` is approximately 91 bytes. The camera sends this as multiple ATT notifications using GPBS (General Purpose Byte Stream) application-layer fragmentation, regardless of the negotiated ATT MTU. `query.c` handles GPBS reassembly transparently via a per-connection context buffer. During reassembly, the readiness poll waits for the final continuation fragment before inspecting the status byte.

**`camera_ready` timing:** `camera_manager_set_camera_ready(slot, true)` is called by `readiness.c` only after the camera returns status 0. This is later than the previous approach (which set the flag immediately on CCCD completion) and ensures that keep-alive, status poll, and recording commands are not sent to a camera that has not yet finished its internal initialisation. The keep-alive and status-poll timers begin sending to the slot on their next scheduled fire after the flag is set.

**ATT MTU:** `gatt.c` negotiates the maximum ATT MTU (`BLE_ATT_MTU_MAX` = 527 in ESP-IDF v6.0) immediately before GATT service discovery. The negotiated MTU is logged at INFO level. GPBS-level fragmentation still occurs because it is controlled by the camera firmware, not the ATT layer.

---

### `camera_manager`

**Header:** `components/camera_manager/include/camera_manager.h`

Camera slot state machine. Persists camera records to NVS. Runs a 2-second tick timer that retries recording commands and fires state-change callbacks.

#### Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `CAMERA_MAX_SLOTS` | `CONFIG_BT_NIMBLE_MAX_BONDS` | Maximum number of simultaneous paired cameras. |
| `CAMERA_NAME_LEN` | `32` | Maximum camera name length including null terminator. |
| `CAMERA_MODEL_NAME_LEN` | `32` | Maximum model name length including null terminator. |
| `CAMERA_STATUS_NOT_CONFIGURED` | `-1` | Slot is empty — no camera assigned. |
| `CAMERA_STATUS_DISCONNECTED` | `0` | Camera paired but not connected. |
| `CAMERA_STATUS_CONNECTED` | `1` | Connected; GATT ready or not yet ready. |
| `CAMERA_STATUS_RECORDING` | `2` | Connected and actively recording. |

#### Types

**`camera_slot_info_t`** — snapshot of a camera slot, returned by `camera_manager_get_slot_info()`.

```c
typedef struct {
    int        index;                                // Slot index (0-based internally; HTTP API adds 1 to produce the 1-based user-facing index)
    char       name[CAMERA_NAME_LEN];               // Advertised camera name
    char       model_name[CAMERA_MODEL_NAME_LEN];   // Model string from GetHardwareInfo, e.g. "HERO12 Black". Empty until hw_info is received after connection.
    ble_addr_t mac_address;                          // BLE MAC address
    bool       is_configured;                        // false if this slot is empty
    int        status;                               // One of the CAMERA_STATUS_* constants
} camera_slot_info_t;
```

**`camera_state_change_fn_t`** — callback fired on slot status changes.

```c
typedef void (*camera_state_change_fn_t)(int slot, int status, void *ctx);
```

`slot` is the 0-based camera slot index (Camera 1 = slot 0, Camera 4 = slot 3); `status` is one of the `CAMERA_STATUS_*` constants.

#### Functions

```c
void camera_manager_init(void);
```
Load all camera records from NVS into RAM and start the 2-second tick timer. Must be called after all drivers have been registered with `camera_manager_register_driver()`.

---

```c
void camera_manager_register_state_change_callback(camera_state_change_fn_t cb, void *ctx);
```
Register a callback invoked whenever a slot's derived status changes. Call before `camera_manager_init()` so no transitions are missed during the initial slot load. Only one callback is supported.

---

```c
void camera_manager_register_driver(camera_type_t type,
                                     const camera_driver_t *driver,
                                     void *(*create_ctx)(void));
```
Register a camera driver vtable and its factory function. The factory is called once per camera slot of the matching type when loading from NVS or registering a new camera. Up to 4 driver types may be registered.

---

```c
int camera_manager_register_new(const ble_addr_t *addr, const char *name,
                                  const camera_driver_t *driver, void *driver_ctx,
                                  camera_type_t type);
```
Register a newly paired camera. Returns the assigned slot index, or `-1` if all slots are full. Idempotent — returns the existing slot if `addr` is already registered.

---

```c
esp_err_t camera_manager_save_slot(int slot);
```
Persist the camera record for `slot` to NVS. Returns `ESP_OK` on success.

---

```c
esp_err_t camera_manager_remove_slot(int slot);
```
Clear a camera slot in RAM and erase its NVS record. Returns `ESP_OK` on success.

---

```c
camera_slot_info_t camera_manager_get_slot_info(int slot);
```
Return a snapshot of the given slot's current state. Returns a zeroed struct with `is_configured = false` for an empty or out-of-range slot.

---

```c
int  camera_manager_find_by_addr(const ble_addr_t *addr);
int  camera_manager_find_by_handle(uint16_t conn_handle);
int  camera_manager_find_free_slot(void);
```
Lookup helpers. Return the matching slot index, or `-1` if not found.

---

```c
int  camera_manager_remembered_count(void);
int  camera_manager_connected_count(void);
```
Return the number of configured slots and the number with an active BLE connection, respectively.

---

```c
void camera_manager_on_connected(int slot, uint16_t conn_handle);
void camera_manager_on_disconnected(uint16_t conn_handle);
void camera_manager_set_camera_ready(int slot, bool ready);
```
Called by `open_gopro_ble` to update slot state on BLE lifecycle events. Each fires the state-change callback immediately. `camera_manager_set_camera_ready()` is called by `readiness.c` after the `GetHardwareInfo` readiness poll confirms status 0.

---

```c
uint16_t camera_manager_get_handle(int slot);
bool     camera_manager_is_camera_ready(int slot);
void    *camera_manager_get_driver_ctx(int slot);
```
Slot accessors used by `open_gopro_ble` to route GATT operations to the correct per-camera context.

---

```c
void camera_manager_set_model_name(int slot, const char *model_name);
```
Store the camera's model name string (e.g. `"HERO12 Black"`) in the given slot. Called automatically by `query.c` when a `GetHardwareInfo` response is parsed after GATT setup. The value is held in RAM only — it is not persisted to NVS and will be re-populated on every reconnection. Accessible via `camera_slot_info_t.model_name` from `camera_manager_get_slot_info()`.

---

```c
void camera_manager_set_desired_recording(bool recording);
```
Set the target recording state for all slots. When `desired = true`, the 2-second tick timer calls `start_recording` on any connected, camera-ready camera that is not currently recording. Whether that call actually dispatches a BLE command depends on the driver — the GoPro BLE driver gates on an internal `start_cmd_pending` flag so that the command is only re-sent after the camera has previously confirmed a RECORDING state and then gone idle (recovery path), not while the initial command is still in flight.

---

```c
int camera_manager_start_recording_all(void);
int camera_manager_stop_recording_all(void);
```
Immediately dispatch a start/stop command to all connected, camera-ready cameras. Returns the number of cameras that received the command.

---

```c
int camera_manager_set_recording_slot(int slot, bool on);
```
Start or stop recording on a single camera slot. Sets `desired_recording` for that slot only (so the 2-second tick timer will retry if the slot is not immediately ready), then dispatches the command immediately if the slot is connected and camera-ready. Returns `1` if the command was dispatched, `0` if the slot was not ready (desired state is still updated).

Note: a subsequent call to `camera_manager_set_desired_recording()` or the `_all` variants will overwrite `desired_recording` for all slots, including any per-slot state set here.

---

```c
bool camera_manager_is_known_addr(const ble_addr_t *addr);
```
Returns `true` if `addr` matches any configured slot. Used by `ble_core` as the `is_known_addr` callback to decide whether to auto-reconnect an advertising device.

---

```c
void camera_manager_set_auto_control(bool enabled);
```
Enable or disable automatic camera control. When `enabled` is `true` (the default on every boot), `on_logging_state_changed` in `main.c` will start and stop cameras in response to RaceCapture `isLogging` transitions. When `false`, those transitions are silently ignored and cameras must be controlled manually. The flag is RAM-only — it is never written to NVS and resets to `true` on every boot.

---

```c
bool camera_manager_get_auto_control(void);
```
Returns the current automatic control state. Called by `on_logging_state_changed` in `main.c` before acting on any `0x600` logging transition, and by `wifi_manager` to serve `GET /api/auto-control`.

---

### `can_manager`

**Header:** `components/can_manager/include/can_manager.h`

ESP-IDF v6.0 TWAI (CAN) driver wrapper. Manages the on-chip TWAI node, a FreeRTOS processing task, ISR-driven RX, periodic 5 Hz TX, and automatic bus-off recovery.

> **ESP-IDF version note:** This component uses `esp_twai.h` / `esp_twai_onchip.h` from ESP-IDF v6.0. The legacy `driver/twai.h` API is not used and is deprecated in v6.0.

#### Configuration macros (in `can_manager.h`)

| Macro | Default | Description |
|-------|---------|-------------|
| `CAN_MANAGER_BITRATE_BPS` | `1000000` | Bus bitrate. |
| `CAN_MANAGER_TX_GPIO` | `GPIO_NUM_7` | CAN TX pin. |
| `CAN_MANAGER_RX_GPIO` | `GPIO_NUM_6` | CAN RX pin. |
| `CAN_MANAGER_RX_QUEUE_DEPTH` | `32` | Software RX queue depth. |
| `CAN_MANAGER_TX_QUEUE_DEPTH` | `8` | Hardware TX queue depth. |
| `CAN_MANAGER_TASK_PRIORITY` | `5` | FreeRTOS task priority. |
| `CAN_MANAGER_TX_INTERVAL_MS` | `200` | Camera status broadcast interval. |
| `CAN_MANAGER_MAX_CAMERAS` | `4` | Maximum camera slots in the `0x601` frame. |
| `CAN_ID_RC_COMMAND` | `0x600` | Frame ID for RaceCapture → ESP32 logging commands. |
| `CAN_ID_CAM_STATUS` | `0x601` | Frame ID for ESP32 → RaceCapture camera status broadcast. |
| `CAN_ID_RC_UTC`     | `0x602` | Frame ID for RaceCapture → ESP32 UTC timestamp. Temporary — see `0x602` frame spec above. |

#### Types

**`camera_state_t`** — camera state values encoded in `0x601` frames.

```c
typedef enum {
    CAMERA_STATE_UNDEFINED    = 0,
    CAMERA_STATE_DISCONNECTED = 1,
    CAMERA_STATE_IDLE         = 2,
    CAMERA_STATE_RECORDING    = 3,
} camera_state_t;
```

**`can_frame_t`** — self-contained received CAN frame delivered to the raw RX callback.

```c
typedef struct {
    uint32_t id;          // CAN identifier
    uint8_t  data[8];     // Frame payload
    uint8_t  data_len;    // Actual payload length (derived from DLC)
    uint8_t  dlc;         // Raw DLC value as received
    bool     is_extended; // true = 29-bit extended ID
    bool     is_rtr;      // true = remote transmission request
} can_frame_t;
```

**`can_rx_frame_cb_t`** — raw frame callback (all IDs, task context).

```c
typedef void (*can_rx_frame_cb_t)(const can_frame_t *frame, void *user_ctx);
```

**`can_logging_state_cb_t`** — callback fired when RaceCapture logging state changes.

```c
typedef void (*can_logging_state_cb_t)(bool is_logging, void *user_ctx);
```

**`can_utc_acquired_cb_t`** — callback fired exactly once when the first valid UTC timestamp is received (GPS lock acquired). Used to set the date/time on cameras that are already connected at that moment.

```c
typedef void (*can_utc_acquired_cb_t)(void *user_ctx);
```

#### Functions

```c
esp_err_t can_manager_init(void);
```
Initialise the TWAI node, register ISR callbacks, create the processing task, and begin the 5 Hz camera status broadcast. Returns `ESP_OK` on success. Non-fatal from `app_main` — the system continues without CAN if this fails.

---

```c
esp_err_t can_manager_deinit(void);
```
Disable the TWAI node, delete the processing task and RX queue, and release all resources. Returns `ESP_OK` on success.

---

```c
esp_err_t can_manager_register_rx_callback(can_rx_frame_cb_t cb, void *user_ctx);
```
Register a raw frame callback invoked for every received frame. Optional — primarily for development and bus sniffing. Returns `ESP_ERR_INVALID_ARG` if `cb` is `NULL`.

---

```c
esp_err_t can_manager_register_logging_callback(can_logging_state_cb_t cb, void *user_ctx);
```
Register a callback fired when the RaceCapture `isLogging` value in `0x600` frames changes. The callback is only invoked on state transitions, not on every frame. Returns `ESP_ERR_INVALID_ARG` if `cb` is `NULL`.

---

```c
esp_err_t can_manager_register_utc_acquired_callback(can_utc_acquired_cb_t cb, void *user_ctx);
```
Register a callback fired exactly once when the first valid UTC timestamp is received on `0x602`. Intended for setting the date/time on cameras that are already camera-ready at the moment GPS lock is first established. Register before `can_manager_init()` to avoid missing the event. Returns `ESP_ERR_INVALID_ARG` if `cb` is `NULL`.

---

```c
esp_err_t can_manager_set_camera_state(uint8_t camera_idx, camera_state_t state);
```
Update the recorded state for camera slot `camera_idx` (0-based internally; Camera 1 = index 0, Camera 4 = index 3). The new state is included in the next `0x601` broadcast within 200 ms. Thread-safe — may be called from any task. Returns `ESP_ERR_INVALID_ARG` if `camera_idx >= CAN_MANAGER_MAX_CAMERAS` or `state` is out of range.

---

```c
bool can_manager_get_utc_ms(uint64_t *epoch_ms_out);
```
Get the current best-estimate UTC as a millisecond Unix epoch. Uses the last received `0x602` timestamp plus elapsed time from the ESP32's monotonic clock (`esp_timer_get_time`) to extrapolate forward, so the result is current even between CAN frames. Thread-safe — may be called from any task.

Returns `true` and writes to `*epoch_ms_out` if a valid UTC has been received from the RaceCapture (GPS lock acquired). Returns `false` and leaves `*epoch_ms_out` undefined if no valid `0x602` frame has been received yet.

---

### `legacy_gopro`

**Header:** `components/legacy_gopro/include/legacy_gopro.h`

Legacy GoPro (Hero4) Wi-Fi driver. Manages camera discovery via the Soft-AP station table, HTTP probing, NVS persistence, and UDP-based shutter/keepalive control. Registers a `camera_driver_t` vtable with `camera_manager`.

#### Internal file layout

| File | Responsibility |
|------|---------------|
| `legacy_gopro.c` | FreeRTOS task state machine, NVS load/save, command queue processing, public API |
| `control.c` | UDP shutter broadcast, UDP keepalive/status-request, HTTP probe and date-time sync |
| `legacy_gopro_internal.h` | Shared `legacy_camera_t` struct and `s_cameras[]` / `s_udp_sock` declarations (internal only) |

#### Hero4 UDP RC-remote protocol

| Packet | Direction | Destination | Description |
|--------|-----------|-------------|-------------|
| `_GPHD_:0:0:2:0.000000\n` (22 bytes, ASCII) | ESP32 → Camera | Unicast per camera, port 8484 | Keepalive — must be sent every 2 s |
| `00 00 00 00 00 00 00 00 00 01 00 53 48 02` (14 bytes) | ESP32 → Camera | Broadcast `255.255.255.255:8484` | Shutter ON (start recording) |
| `00 00 00 00 00 00 00 00 00 01 00 53 48 00` (14 bytes) | ESP32 → Camera | Broadcast `255.255.255.255:8484` | Shutter OFF (stop recording) |
| `00 00 00 00 00 00 00 00 00 00 00 73 74` (13 bytes) | ESP32 → Camera | Unicast per camera, port 8484 | Status request |

The shutter broadcast reaches all cameras simultaneously regardless of how many are connected, providing the best recording-start synchronisation achievable over UDP.

#### Types

**`legacy_camera_t`** — per-camera runtime state (internal, defined in `legacy_gopro_internal.h`).

```c
typedef struct {
    bool     active;          // true while station is connected to the Soft-AP
    bool     managed;         // true after user adds the camera via /api/legacy/add
    bool     wifi_connected;  // true while camera is connected and keepalives are flowing
    uint32_t ip_addr;         // current IP, network byte order
    uint8_t  mac[6];
    int      slot;            // camera_manager slot; -1 until managed
    char     name[LEGACY_CAMERA_NAME_LEN];
    camera_recording_status_t recording_status;
    volatile TickType_t last_response_tick; // FreeRTOS tick of last UDP response; 0 = never
} legacy_camera_t;
```

> This struct is internal. The `driver_ctx` pointer stored in `camera_manager` points directly into the `s_cameras[]` array — entries must not be moved after registration.

#### Functions

```c
void legacy_gopro_init(void);
```
Load managed MACs from NVS, open the UDP socket (port 8383), and start the internal FreeRTOS task and UDP RX task. Must be called after `camera_manager_init()` and `wifi_manager_init()`.

---

```c
void legacy_gopro_on_station_wifi_associated(const uint8_t mac[6]);
```
Notify the component that a station has completed Wi-Fi L2 association on the Soft-AP. If the MAC is in NVS (previously managed camera), a `CMD_STATION_CONNECT` is posted with the saved IP, triggering an HTTP probe. Unknown MACs are ignored — they appear in `/api/legacy/discovered` once they have a DHCP address. Safe to call from any context (ISR-safe queue post). Called by `wifi_manager` from the `WIFI_EVENT_AP_STACONNECTED` handler.

---

```c
void legacy_gopro_on_station_connected(uint32_t ip, const uint8_t mac[6]);
```
Notify the component that a station has been assigned a DHCP IP. Only acts on MACs that are already in NVS (managed cameras) — posts `CMD_IP_UPDATE` to update the live IP and redirect keepalives if the address changed. Unknown MACs are ignored. Safe to call from any context. Called by `wifi_manager` from the `IP_EVENT_ASSIGNED_IP_TO_CLIENT` handler.

---

```c
void legacy_gopro_on_station_disconnected(const uint8_t mac[6]);
```
Notify the component that a station has disconnected from the Soft-AP. Managed cameras are kept in the table (slot remains visible as disconnected) and auto-promoted on reconnect. Safe to call from any context.

---

```c
bool legacy_gopro_add_camera(const uint8_t mac[6], uint32_t ip);
```
Probe a connected station and promote it to managed if it responds to the Hero4 HTTP API. Posts an asynchronous `CMD_ADD_CAMERA` to the internal task queue. Returns `true` if the command was queued, `false` if the queue is full. The HTTP probe (up to 3 retries) runs on the task. Called by `wifi_manager` from `POST /api/legacy/add`.

---

```c
bool legacy_gopro_remove_camera(const uint8_t mac[6]);
```
Remove a managed camera. Posts an asynchronous `CMD_REMOVE_CAMERA` that unregisters the camera, frees the `camera_manager` slot, and removes the MAC from NVS. Returns `true` if queued. Called by `wifi_manager` from `POST /api/legacy/remove`.

---

```c
bool legacy_gopro_is_managed_slot(int slot);
```
Returns `true` if the given `camera_manager` slot belongs to a managed legacy Wi-Fi camera. Used by `wifi_manager` to determine whether `POST /api/remove-camera` should call `ble_core_remove_bond()` (BLE camera) or `legacy_gopro_remove_camera()` (Wi-Fi camera).

---

```c
bool legacy_gopro_is_managed_mac(const uint8_t mac[6]);
```
Returns `true` if the MAC belongs to a currently managed legacy camera. Used by `wifi_manager` to filter managed cameras out of `/api/legacy/discovered`. Lock-free read; safe to call from any context.

---

```c
void legacy_gopro_sync_time_all(void);
```
Push the current UTC date/time to all managed and connected Wi-Fi cameras via the Hero4 HTTP date/time endpoint. Posts an asynchronous `CMD_SYNC_TIME` to the internal task queue. If UTC is not yet available (no GPS lock on the RaceCapture), the HTTP call is silently skipped per camera. Called from `main.c`'s `on_utc_acquired` callback alongside `open_gopro_ble_sync_time_all()`. Safe to call from any context.

---

### `wifi_manager`

**Header:** `components/wifi_manager/include/wifi_manager.h`

Wi-Fi soft-AP and HTTP server. Serves the embedded web UI and all `/api/*` endpoints, including the `/api/legacy/*` sub-tree. Owns the connected-station table and routes L2 and DHCP events to `legacy_gopro`.

#### Functions

```c
void wifi_manager_init(void);
```
Initialise the Wi-Fi AP interface, assign static IP `10.71.79.1`, configure the DHCP pool (`10.71.79.2–10.71.79.50`), bring up the AP (SSID `HERO-RC-XXXXXX`, open auth, channel 6), and register all HTTP URI handlers. Call once from `app_main()` after `camera_manager_init()` and before `legacy_gopro_init()`.

---

```c
void wifi_manager_wait_for_ap_ready(void);
```
Block until `WIFI_EVENT_AP_START` has fired (up to 5 seconds). Call this from `app_main()` between `wifi_manager_init()` and `ble_core_init()` to ensure the AP beacon is on air before BLE starts its radio-intensive connection procedures. The ESP32 shares one antenna between Wi-Fi and BLE; starting BLE before the AP is ready can starve the coexistence scheduler.

---

```c
uint32_t wifi_manager_get_station_ip(const uint8_t mac[6]);
```
Return the DHCP-assigned IP (network byte order) for a connected station by MAC address, or `0` if the station is not found or has no IP yet.

---

```c
int wifi_manager_get_connected_stations(wifi_mgr_sta_info_t *out, int max_count);
```
Copy up to `max_count` connected-station entries into `out`. Returns the number of entries written. Used by the `/api/legacy/discovered` handler to enumerate stations.

---

### `camera_driver` (interface)

**Header:** `components/camera_manager/include/camera_driver.h`

Driver vtable interface used by `camera_manager` to control cameras without coupling to any specific protocol.

#### Types

**`camera_recording_status_t`**

```c
typedef enum {
    CAMERA_RECORDING_UNKNOWN = 0,  // Status not yet known
    CAMERA_RECORDING_IDLE,         // Connected, not recording
    CAMERA_RECORDING_ACTIVE,       // Actively recording
} camera_recording_status_t;
```

**`camera_type_t`**

```c
typedef enum {
    CAMERA_TYPE_NONE        = 0,  // Unconfigured slot
    CAMERA_TYPE_GOPRO_BLE,        // GoPro via BLE (open_gopro_ble component)
    CAMERA_TYPE_LEGACY_WIFI,      // Legacy GoPro (Hero4) via Wi-Fi HTTP/UDP (legacy_gopro component)
} camera_type_t;
```

**`camera_driver_t`** — vtable struct.

```c
struct camera_driver {
    esp_err_t (*start_recording)(void *ctx);
    esp_err_t (*stop_recording)(void *ctx);
    camera_recording_status_t (*get_recording_status)(void *ctx);
};
```

All three function pointers receive the per-camera `ctx` allocated by the driver's factory function. `get_recording_status` must be non-blocking and return a cached value. `start_recording` and `stop_recording` dispatch commands asynchronously and return `ESP_OK` if the command was sent, or `ESP_ERR_INVALID_STATE` if the camera is not ready or a start command is already in flight (see `start_cmd_pending` in the `open_gopro_ble` section).
