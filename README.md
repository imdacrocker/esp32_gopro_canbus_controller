# ESP32 GoPro CAN Bus Controller

This project is currently in a working proof-of-concept phase. It has been tested against a GoPro Hero13 Black (OpenGoPro BLE) and a GoPro Hero4 (legacy Wi-Fi control).


An ESP32-S3 firmware that bridges GoPro cameras (controlled over BLE) with a [RaceCapture](https://autosportlabs.com/racecapture/) data logger (connected over CAN bus). When RaceCapture starts logging, all paired GoPro cameras start recording automatically. Camera connection status is broadcast back to RaceCapture in real time.

A companion Wi-Fi web interface lets you pair and manage cameras, monitor live recording status, and — when Automatic Control is disabled — manually trigger recording on all cameras at once or on individual cameras independently. No laptop or serial terminal required in the field.

---

## Table of Contents

- [TODO](#todo)
- [Hardware](#hardware)
- [Architecture](#architecture)
- [Component Overview](#component-overview)
- [Getting Started](#getting-started)
- [Building and Flashing](#building-and-flashing)
- [Configuration](#configuration)
- [First-Run: Pairing Cameras](#first-run-pairing-cameras)
- [First-Run: Adding Legacy Wi-Fi Cameras (Hero4)](#first-run-adding-legacy-wi-fi-cameras-hero4)
- [CAN Bus Wiring](#can-bus-wiring)
- [Troubleshooting](#troubleshooting)
- [Project Structure](#project-structure)
- [API Reference](#api-reference)

---

## TODO
Known bugs:
 - WiFi and BLE are fighting each other on the chip. This may need to be smoothed out

---

## Hardware

This firmware targets the **[ESP32-CAN-X2](https://autosportlabs.com/product/esp32-can-x2/) by AutoSport Labs**.

| Parameter | Value |
|-----------|-------|
| MCU | ESP32-S3-WROOM-1-N8R8 |
| Flash | 8 MB |
| CAN 1 (TWAI) TX | GPIO 7 |
| CAN 1 (TWAI) RX | GPIO 6 |
| CAN 2 (MCP2515, SPI) | CS=10, CLK=12, MISO=13, MOSI=11, IRQ=3 |
| GPIO voltage | 3.3 V — do not exceed |
| Supply voltage | 6–20 V nominal, 40 V max |

> **Note:** Only CAN 1 (the on-chip TWAI peripheral) is used by this firmware. CAN 2 (MCP2515) is not currently implemented.

The board includes 120 Ω termination resistors enabled by default via solder-jumper pads (TERM1/TERM2) on the back. Leave them intact if this board is at one end of the CAN bus. See [CAN Bus Wiring](#can-bus-wiring) for more detail.

---

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│                        app_main                          │
│  (wires camera state → CAN bridge, registers callbacks)  │
└────────┬───────────────────────────────┬─────────────────┘
         │                               │
         ▼                               ▼
┌─────────────────┐             ┌─────────────────┐
│  camera_manager │             │   can_manager   │
│  (slot state,   │◄───────────►│  (TWAI node,    │
│   NVS persist,  │  state CB   │   0x600 RX,     │
│   tick timer)   │             │   0x601 TX 5Hz, │
└──────┬──────────┘             │   0x602 RX UTC) │
       │                        └─────────────────┘
       │ driver vtable
       ├──────────────────────────────┐
       ▼                              ▼
┌─────────────────┐          ┌─────────────────┐
│ open_gopro_ble  │          │  legacy_gopro   │
│  (OpenGoPro     │          │  (Hero4 Wi-Fi   │
│   BLE protocol, │          │   HTTP/UDP,     │
│   GATT handles, │          │   keepalive,    │
│   status poll,  │          │   NVS persist)  │
│   keep-alive)   │          └────────┬────────┘
└────────┬────────┘                   │ station events
         │ BLE callbacks              ▼
         ▼                   ┌─────────────────┐
┌─────────────────┐          │  wifi_manager   │
│    ble_core     │          │  (Soft-AP,      │
│  (NimBLE stack, │          │   HTTP server,  │
│   scan, connect,│          │   embedded UI)  │
│   bond store)   │          └─────────────────┘
└─────────────────┘
```

**Data flow summary (CAN path — recording control):**

1. RaceCapture sends a `0x600` CAN frame with `isLogging = 1`.
2. `can_manager` detects the state change and calls the registered logging callback.
3. `app_main`'s callback sets the desired recording state on `camera_manager` and immediately dispatches a start command to all GATT-ready cameras.
4. `open_gopro_ble` sends the OpenGoPro `shutter start` TLV over BLE to each camera and logs the outgoing command.
5. The camera responds with an acknowledgement on `cmd_resp_notify` (GP-0073). `open_gopro_ble` logs whether the camera accepted or rejected the command.
6. `open_gopro_ble`'s status poll timer queries each camera for `encoding_active` every 5 seconds and updates the recording status in the driver context. A separate keep-alive command is sent to every connected camera every 3 seconds to prevent the camera from auto-sleeping.
7. `camera_manager`'s tick timer (2 s) reads the recording status from each driver and fires the state-change callback. If a camera that was previously confirmed as recording is now idle (e.g. it stopped unexpectedly), the tick dispatches a recovery start command. The start command is not retried while the initial command is still in flight — `open_gopro_ble` sets an internal `start_cmd_pending` flag when the command is sent and clears it only after the status poll confirms the transition (IDLE→RECORDING clears it on the happy path; RECORDING→IDLE clears it to allow the recovery send).
8. `app_main`'s state-change callback maps the internal status to a `camera_state_t` and calls `can_manager_set_camera_state()`.
9. `can_manager` broadcasts the updated `0x601` status frame to RaceCapture at 5 Hz.

**Data flow summary (CAN path — UTC time sync):**

1. A Lua script on the RaceCapture reads GPS-derived UTC from `getDateTime()` and broadcasts it at 25 Hz on CAN ID `0x602` once GPS lock is acquired. The payload is a 64-bit millisecond Unix epoch timestamp, little-endian, in all 8 bytes.
2. `can_manager` receives each `0x602` frame and records the epoch value alongside the ESP32 monotonic timestamp (`esp_timer_get_time()`) at the moment of receipt. The first valid frame (year > 2020) is logged at INFO level with a human-readable UTC string.
3. `can_manager_get_utc_ms()` extrapolates the stored epoch forward using elapsed monotonic time, so callers always get a current estimate regardless of when the last CAN frame arrived.
4. When GATT setup completes (all CCCD subscriptions done), `gatt.c` issues two commands in sequence:
   - `control_send_set_date_time()` — sets the camera's clock (see paths below).
   - `gopro_query_send_hw_info()` — requests the camera's model name, firmware version, and serial number. The response is parsed asynchronously and the model name is stored in the camera slot via `camera_manager_set_model_name()`, making it available through `camera_slot_info_t.model_name` for display in the web UI.

   SetDateTime specifically is sent via one of two paths depending on timing:
   - **Camera connects after UTC is available:** `gatt.c` calls `control_send_set_date_time()` directly when GATT setup completes.
   - **Camera is already connected when UTC first arrives:** `can_manager` fires a one-shot `can_utc_acquired_cb_t` callback. `main.c` handles it by calling `open_gopro_ble_sync_time_all()`, which iterates all GATT-ready slots and calls `control_send_set_date_time()` for each.
5. In both cases, `control_send_set_date_time()` fetches the current UTC from `can_manager_get_utc_ms()`, converts it to calendar fields, and writes a SetDateTime TLV command (ID `0x0D`) to GP-0072 (`cmd_write`).
6. The camera responds on GP-0073 (`cmd_resp_notify`). `query.c` logs whether the camera accepted or rejected the command.

**Data flow summary (web UI path):**

> The **Start/Stop All** and per-camera **Start/Stop** buttons are only shown when **Automatic Control** is disabled. When Automatic Control is on, recording is driven exclusively by the CAN path and manual shutter controls are hidden.

1. User disables Automatic Control via the toggle, then taps **▶ Start Recording** / **⏹ Stop Recording** (all cameras) or the per-camera toggle button on an individual camera row.
2. `wifi_manager` receives `POST /api/shutter`. For all-camera actions, `"on"` is the only field. For a per-camera action the request also includes `"slot"` with the 0-based camera slot index.
3. **All-camera path:** `camera_manager_set_desired_recording()` updates the desired state for every slot, then `camera_manager_start/stop_recording_all()` dispatches the command to every connected, GATT-ready camera immediately.
   **Per-camera path:** `camera_manager_set_recording_slot(slot, on)` updates `desired_recording` for that slot only and dispatches the command to that camera if it is connected and GATT-ready.
4. After a per-camera command is sent, the button is disabled immediately in the UI. It re-enables on the next `/api/paired-cameras` poll (every 3 seconds) once the camera reports its new state.
5. The tick timer continues retrying for any camera not yet in the desired state, and applies the command to cameras that reconnect later.

**Data flow summary (scan and pairing — web UI):**

1. User taps **+ Add / Manage Cameras** at the bottom of the screen. A bottom-sheet modal opens showing two sections: **Paired Cameras** (name + remove button) and **Add New Camera** (scan controls).
2. User taps **Scan for Cameras** inside the modal. The button label changes to **Cancel Scan**.
3. `wifi_manager` receives `POST /api/scan`. `open_gopro_ble` clears the discovery list and starts a 30-second BLE scan via `ble_core`.
4. Discovered GoPro cameras (service UUID `0xFEA6`) appear in the modal's results list. The UI polls `GET /api/cameras` once per second during the scan. Already-paired cameras are filtered out client-side by cross-referencing `GET /api/paired-cameras` — only new, unpaired cameras are shown.
5a. **Scan expires naturally (30 s):** the firmware fires `BLE_GAP_EVENT_DISC_COMPLETE`, the UI's 31-second timeout fires, polling stops, and the button reverts to **Scan for Cameras**.
5b. **User taps Cancel Scan:** `wifi_manager` receives `POST /api/scan-cancel`. `open_gopro_ble` calls `ble_core_stop_discovery()`, which cancels the scan and resumes the passive background scan if any paired camera is disconnected.
5c. **User closes the modal while scanning:** the UI calls `POST /api/scan-cancel` automatically before hiding the modal. This prevents an invisible background scan from continuing.
5d. **User taps Pair:** the UI stops polling and reverts the button immediately. `wifi_manager` receives `POST /api/pair`. The firmware cancels the scan and calls `ble_gap_connect()` directly — the controller enters initiating mode and connects as soon as the camera advertises. The UI shows a "Pairing initiated" message and the newly paired camera appears in the **Paired Cameras** list within a few seconds.

> If RaceCapture is actively sending `0x600` frames and **Automatic Control** is enabled, the CAN and web UI paths write to the same desired-state flag and can overwrite each other. This is intentional — the web UI is designed for diagnostics when CAN is disconnected.

**Automatic camera control:**

The web UI exposes an **Automatic Control** toggle (on by default, always resets to on at boot). When enabled, the CAN logging state drives camera recording as described above and all manual shutter controls are hidden — there is no way to accidentally override the CAN-driven state from the UI. When disabled, `0x600` transitions are ignored, cameras hold whatever state they were in when the toggle was switched off, and the following manual controls become visible:

- **Start Recording / Stop Recording** buttons (all cameras simultaneously).
- A per-camera **▶ Start** or **⏹ Stop** toggle button on each row of the Camera Status section. The button shows the action that will be taken (not the current state): **▶ Start** when the camera is idle, **⏹ Stop** when it is recording, and no button when the camera is disconnected. Tapping the button disables it immediately to prevent duplicate commands; it re-enables automatically on the next status poll once the camera confirms its new state.

The flag is RAM-only and never stored in NVS.

---

## Component Overview

| Component | Purpose |
|-----------|---------|
| `ble_core` | NimBLE stack wrapper. Owns scan, connect, encrypt, GATT write, and bond management. Camera-agnostic. |
| `open_gopro_ble` | OpenGoPro BLE driver. Implements the OpenGoPro BLE protocol (service UUID 0xFEA6, TLV command encoding). The camera is considered ready as soon as CCCD subscriptions complete — no polling loop is needed. `GetHardwareInfo` is sent automatically after every GATT setup to populate the camera's model name (e.g. `"HERO12 Black"`) in `camera_slot_info_t`. A two-phase preset flow runs on every connection to switch the camera to Video mode: Phase 1 sends `RequestGetPresetStatus` (Protobuf, GP-0076); Phase 2 parses the `NotifyPresetStatus` response on GP-0077 and sends `Load Preset` (TLV 0x40, GP-0072) for the first Video-group preset found. Sends a keep-alive packet every 3 seconds (per OpenGoPro spec) to prevent auto-sleep. Provides a `camera_driver_t` vtable to `camera_manager`. |
| `legacy_gopro` | Legacy GoPro (Hero4) Wi-Fi driver. Cameras connect to the controller's Soft-AP and are discovered via `/api/legacy/discovered`. When the user adds a camera, the component probes it over HTTP, registers it with `camera_manager`, and saves its MAC and IP to NVS for automatic reconnection on future boots. Shutter commands are sent as a single UDP broadcast to `255.255.255.255:8484` for simultaneous start across all cameras. Keepalives (ASCII RC-remote format) and recording-status polls run every 2 seconds per connected camera. Provides a `camera_driver_t` vtable to `camera_manager`. |
| `camera_manager` | Camera slot state machine. Persists camera records to NVS. Runs the 2-second tick timer that retries recording commands and publishes state changes. Owns the RAM-only `automatic_camera_control` flag that gates CAN-driven recording. Supports multiple driver types via the `camera_driver_t` vtable — both `open_gopro_ble` and `legacy_gopro` register themselves here. |
| `can_manager` | ESP-IDF v6.0 TWAI driver wrapper. Receives `0x600` (isLogging) and `0x602` (UTC timestamp) frames; broadcasts `0x601` camera status at 5 Hz. Exposes `can_manager_get_utc_ms()` for on-demand UTC retrieval with monotonic-clock extrapolation. Thread-safe. |
| `wifi_manager` | Soft-AP + HTTP server. Serves the embedded web UI and all `/api/*` endpoints, including the `/api/legacy/*` sub-tree for managing Hero4 cameras. Owns the connected-station table and notifies `legacy_gopro` of L2 association and DHCP events. |

---

## Getting Started


### Prerequisites

- [VS Code](https://code.visualstudio.com/download) installed 
- [ESP-IDF v6.0](https://docs.espressif.com/projects/esp-idf/en/v6.0/esp32s3/get-started/index.html) installed and sourced.
- Target chip: `esp32s3`
- A USB connection to the ESP32-CAN-X2 board (COM port).

### Clone the repository

```bash
git clone https://github.com/<your-username>/esp32_gopro_canbus_controller.git
cd esp32_gopro_canbus_controller
```

---

## Building and Flashing

From the ESP-IDF in VS Code
- Set the target to esp32s3
- Set the COM port (detect often works)
- Set flash method to UART
- Use the Build, Flash and Monitor to flash the ESP32

```bash
# Set the target (required once per checkout)
idf.py set-target esp32s3

# Build
idf.py build

# Flash and open the serial monitor (replace PORT with your COM port)
idf.py -p PORT flash monitor
```

To flash without opening the monitor:

```bash
idf.py -p PORT flash
```

To open the monitor on an already-flashed board:

```bash
idf.py -p PORT monitor
```

---

## Configuration

All user-configurable values are defined as compile-time constants in the relevant component headers. No `menuconfig` changes are required for the default setup.

---

### CAN bus (`components/can_manager/include/can_manager.h`)

| Macro | Default | Description |
|-------|---------|-------------|
| `CAN_MANAGER_BITRATE_BPS` | `1000000` | Bus bitrate in bps (1 Mbps). Must match RaceCapture. |
| `CAN_MANAGER_TX_GPIO` | `GPIO_NUM_7` | CAN TX pin (board-fixed). |
| `CAN_MANAGER_RX_GPIO` | `GPIO_NUM_6` | CAN RX pin (board-fixed). |
| `CAN_MANAGER_TX_INTERVAL_MS` | `200` | Camera status broadcast interval (5 Hz). |
| `CAN_MANAGER_RX_QUEUE_DEPTH` | `32` | Software RX queue depth. |
| `CAN_MANAGER_TX_QUEUE_DEPTH` | `8` | Hardware TX queue depth. |
| `CAN_MANAGER_TASK_PRIORITY` | `5` | FreeRTOS priority of the CAN processing task. |
| `CAN_ID_RC_COMMAND` | `0x600` | CAN ID for RaceCapture → ESP32 logging commands. |
| `CAN_ID_CAM_STATUS` | `0x601` | CAN ID for ESP32 → RaceCapture camera status broadcast. |
| `CAN_ID_RC_UTC`     | `0x602` | CAN ID for RaceCapture → ESP32 UTC timestamp. Temporary ID — will migrate to the RaceCapture developer's standard ID when native UTC broadcast is available in firmware. |

### OpenGoPro BLE (`components/open_gopro_ble/include/open_gopro_ble.h`)

| Macro | Default | Description |
|-------|---------|-------------|
| `GOPRO_MAX_DISCOVERED` | `10` | Maximum cameras held in the discovery list. |
| `STATUS_POLL_INTERVAL_MS` | `5000` | Interval for querying recording status from each connected camera. |
| `KEEP_ALIVE_INTERVAL_MS` | `3000` | Interval for sending the keep-alive command (ID `0x5B`) to each connected camera. |

### Camera slots (`components/camera_manager/include/camera_manager.h`)

| Macro | Source | Description |
|-------|--------|-------------|
| `CAMERA_MAX_SLOTS` | `CONFIG_BT_NIMBLE_MAX_BONDS` | Maximum paired cameras. Tied to NimBLE bond store limit. Increase by raising `BT_NIMBLE_MAX_BONDS` in `sdkconfig.defaults`. |

### Legacy GoPro Wi-Fi (`components/legacy_gopro/include/legacy_gopro_internal.h`)

| Macro | Default | Description |
|-------|---------|-------------|
| `LEGACY_MAX_CAMERAS` | `4` | Maximum Hero4 cameras managed simultaneously. |

UDP port numbers and keepalive/shutter packet formats are compile-time constants in `legacy_gopro_internal.h`. They match the Hero4 RC-remote protocol and should not need to be changed.

### Wi-Fi (`components/wifi_manager/wifi_manager.c`)

The AP IP address (`10.71.79.1`) and channel are compile-time constants in `wifi_manager.c`. The SSID is generated at runtime from the last 3 bytes of the AP MAC: `HERO-RC-XXXXXX`. The DHCP pool is `10.71.79.2–10.71.79.50`; cameras request their previously-assigned address via DHCP option 50, so addresses are naturally stable across reconnects.

---

## First-Run: Pairing Cameras

Cameras are paired via the built-in web interface. You do **not** need to use the GoPro app or pair via Bluetooth settings on your phone.

1. Power on the controller. The Wi-Fi AP will start in a few seconds.
2. On your phone or laptop, connect to the Wi-Fi network `HERO-RC-XXXXXX` (open, no password).
3. Open a browser and navigate to `http://10.71.79.1`.
4. Power on the GoPro camera(s) and ensure Bluetooth is enabled on the camera.
5. Tap **+ Add / Manage Cameras** at the bottom of the screen. The camera management modal will open.
6. Tap **Scan for Cameras**. A 30-second scan will begin. The button changes to **Cancel Scan** while the scan is running — tap it again to stop early. The scan also stops automatically when you tap **Pair**.
7. When your camera appears in the list, tap **Pair**. The controller cancels the scan and immediately initiates a BLE connection and pairing process. A "Pairing initiated" message will appear.
8. The camera will appear in the **Paired Cameras** list at the top of the modal within a few seconds. Close the modal — the camera now appears in the **Camera Status** section on the main screen. Its status will change from **Disconnected** to **Not recording** once the BLE link and GATT setup complete (typically a few seconds).

Paired cameras are stored in NVS and reconnect automatically every time the controller boots — you only need to pair once.

To remove a camera, tap **+ Add / Manage Cameras** to open the modal and tap the **✕** button next to the camera in the **Paired Cameras** list. The camera is immediately disconnected and its pairing is erased. It will need to be re-paired before it can reconnect.

---

## First-Run: Adding Legacy Wi-Fi Cameras (Hero4)

Older GoPros (e.g. Hero4) that do not support BLE connect to the controller's Soft-AP and are managed via the web UI's **Legacy Cameras** section. No BLE pairing is required.

1. Power on the controller and connect your phone or laptop to `HERO-RC-XXXXXX`.
2. Open `http://10.71.79.1` in a browser.
3. Power on the Hero4 and put it in **RC Mode** (Connections → RC). The camera broadcasts its own Wi-Fi AP with an SSID matching `HERO4...` — the controller's AP will take precedence once the camera associates.
4. Wait a few seconds for the camera to connect to the controller's Soft-AP and obtain a DHCP address.
5. In the web UI, scroll to the **Legacy Cameras** section and tap **Refresh**. The camera's MAC and IP address will appear in the **Discovered** list.
6. Tap **Add** next to the camera. The controller probes the device via HTTP to confirm it is a Hero4. This takes up to 15 seconds.
7. On success, the camera disappears from the Discovered list and appears in the **Camera Status** section with status **Not recording**. If it remains in the Discovered list after 15 seconds, the probe failed (wrong device or network issue).

The camera's MAC and last-known IP are saved to NVS. On subsequent boots, if the camera connects to the AP and the probe succeeds at the saved IP, it is automatically promoted to managed status — no user action required.

To remove a legacy camera, tap the **✕** button in the Camera Status section (or use `POST /api/legacy/remove`). If the camera remains physically connected to the AP, it will reappear in the Discovered list.

> **Note:** The Hero4 RC-remote protocol expects keepalive packets every 2 seconds. If the controller is powered off or reboots, the Hero4 may begin recording on its own — this is normal Hero4 behaviour and is not controllable from the firmware side.

---

## CAN Bus Wiring

The controller uses **CAN 1** (the ESP32-S3 on-chip TWAI peripheral) at **1 Mbps** with **standard 11-bit frame IDs**.

Connect the **CAN H** and **CAN L** lines between the ESP32-CAN-X2 board and the RaceCapture unit.

### Termination

The ESP32-CAN-X2 has 120 Ω termination resistors enabled by default.

- **End node** (controller at one end of the bus): leave the TERM1/TERM2 solder jumpers intact.
- **Middle node** (other devices on both sides): cut the TERM1/TERM2 trace on the back of the board to disable termination.

Incorrect termination causes bus errors and silent communication failures. Always verify the physical layer before debugging software issues.

### RaceCapture Configuration


#### Receiving camera status (0x601)

Add a **Direct CAN Mapping** channel for each camera slot:

| Field | Camera 1 | Camera 2 | Camera 3 | Camera 4 |
|-------|----------|----------|----------|----------|
| CAN ID | 1537 (0x601) | 1537 | 1537 | 1537 |
| Source Type | Unsigned | Unsigned | Unsigned | Unsigned |
| Offset (bytes) | 0 | 1 | 2 | 3 |
| Length | 1 byte | 1 byte | 1 byte | 1 byte |
| Multiplier | 1 | 1 | 1 | 1 |
| Adder | 0 | 0 | 0 | 0 |
| Min / Max | 0 / 3 | 0 / 3 | 0 / 3 | 0 / 3 |

Camera state values: `0` = undefined, `1` = disconnected, `2` = idle, `3` = recording.

A CAN preset will be available soon

#### Sending logging state (0x600)

To broadcast the logging status on the RaceCapture, you will need to use Lua.  You can insert this line anywhere in your onTick function:

```lua
txCAN(0, 0x600, 0, {isLogging(), 0, 0, 0, 0, 0, 0, 0})
```

#### Sending UTC timestamp (0x602)

The controller uses the RaceCapture's GPS-derived UTC to set the date and time on each camera at connection. Use the provided Lua script (`racecapture_utc_broadcast.lua`) as a standalone script on the RaceCapture.

The script broadcasts a 64-bit millisecond Unix epoch timestamp at 25 Hz on CAN ID `0x602`, little-endian, once GPS lock is acquired. Transmission is suppressed until a valid GPS fix is available.

> **Note:** `0x602` is a temporary ID used until the RaceCapture developer ships native UTC broadcast support in firmware. When that happens, remove this Lua script and update `CAN_ID_RC_UTC` in `can_manager.h` to the developer's assigned ID to avoid two nodes transmitting on the same ID.

---

## Troubleshooting

**Camera does not appear during scan**
- Ensure the GoPro's Bluetooth is on. On most GoPro models, enable **Connections → GoPro Connect** or simply leave Bluetooth enabled in the camera settings.
- Keep the camera within a few metres during pairing.

**Camera pairs but shows "Disconnected" after reboot**
- Check that the camera's Bluetooth is on before powering the controller.
- Review the serial log for bond store errors. If the NVS is corrupted, run `idf.py erase-flash` and re-pair.

**CAN bus errors in the serial log**
- Verify CAN H / CAN L are not swapped.
- Check termination — exactly two 120 Ω resistors must be present across the bus (one at each physical end).
- Confirm both devices are configured for 1 Mbps.

**Recording does not start when RaceCapture logs**
- Check that **Automatic Control** is enabled in the web UI. If it was toggled off, CAN logging transitions are intentionally ignored. The serial log will show `automatic_camera_control disabled — ignoring logging state change` from `main`.
- Confirm the `0x600` frame is being transmitted by RaceCapture (use the serial monitor to watch for `0x600 RX` log lines from `can_manager`).
- Confirm the command is reaching the camera: look for `conn=X cmd_write=0xXXXX: sending Start Recording` in the serial log from `open_gopro_ble`.
- Confirm the camera accepted the command: look for `SetShutter command accepted by camera` immediately after. A `SetShutter command rejected` warning with a non-zero status code means the camera is refusing the command (wrong mode, not in video mode, etc.).
- If no `sending Start Recording` line appears, check that cameras are GATT-ready: look for `gatt_ready` log lines after the camera connects.

**Recording does not start from the web UI**
- The web UI shutter button requires at least one camera in `gatt_ready` state. Check `/api/paired-cameras` — the status must be `not_recording` or `recording`, not `disconnected`.
- Look for `conn=X cmd_write=0xXXXX: sending Start Recording` in the log to confirm the command was dispatched.
- If the command is sent but the camera ignores it, check `SetShutter command accepted/rejected` in the log.

**Web UI is not reachable**
- Wait 5–10 seconds after power-on for the AP to initialise.
- Ensure your device is connected to `HERO-RC-XXXXXX`, not another network.
- Try `http://10.71.79.1` directly rather than using mDNS.

**Hero4 does not appear in Discovered list**
- Confirm the Hero4 is in RC Mode (Connections → RC) — this is the mode that makes it connect to an external AP as a station rather than hosting its own AP.
- Verify the camera is associated: check the serial log for `WIFI_EVENT_AP_STACONNECTED` events.
- The camera appears in Discovered only after obtaining a DHCP address. Check for `IP_EVENT_ASSIGNED_IP_TO_CLIENT` in the serial log. If no DHCP event fires, the camera may be using a static IP — refresh the list and check if a zero-IP entry is blocking it.
- The requester's own IP is excluded from the Discovered list, so the phone or laptop loading the UI will not appear even if it is connected to the same AP.

**Hero4 appears in Discovered but Add fails (camera stays in list after 15 s)**
- The HTTP probe (`GET /gp/gpControl/status`) timed out or received an unexpected response. Check that the IP shown in the Discovered list matches what the camera actually has (DHCP races can cause a brief mismatch).
- Look for `[legacy_gopro] probe failed` or `[legacy_gopro] HTTP error` in the serial log.
- Confirm the Hero4 firmware is up to date — very old firmware revisions may not respond to `gpControl` commands.

**Hero4 recording does not start / stop with RaceCapture**
- Confirm **Automatic Control** is enabled in the web UI.
- Look for `[legacy_gopro] UDP shutter ON/OFF` in the serial log to confirm the command was sent.
- The shutter command is a UDP broadcast to `255.255.255.255:8484`. If the camera is not responding, verify keepalives are working: look for `[legacy_gopro] keepalive →` log lines every 2 seconds. Keepalive failure usually means the camera is no longer connected to the AP.

---

## Project Structure

```
esp32_gopro_canbus_controller/
├── main/
│   └── main.c                  # Entry point; wires all components together
├── components/
│   ├── ble_core/               # NimBLE stack abstraction (camera-agnostic)
│   │   ├── include/ble_core.h
│   │   ├── ble_init.c          # Stack init, bond reconnect on boot
│   │   ├── ble_scan.c          # Scan, connect, scan event callback
│   │   ├── ble_connect.c       # Connection event handler, encryption
│   │   └── ble_gatt_write.c    # ATT write command helper
│   ├── open_gopro_ble/         # OpenGoPro BLE driver
│   │   ├── include/open_gopro_ble.h
│   │   ├── control.c           # Recording commands, status poll timer (5 s), keep-alive timer (3 s)
│   │   ├── driver.c            # camera_driver_t vtable, context alloc, discovery list, init
│   │   ├── gatt.c              # GATT service discovery, MTU negotiation, CCCD subscription
│   │   ├── notify.c            # GATT notification handler (recording status, preset responses, command responses)
│   │   ├── presets.c           # Two-phase Video preset loading (RequestGetPresetStatus → Load Preset 0x40)
│   │   ├── query.c             # On-demand query commands (GetHardwareInfo 0x3C, Load Preset 0x40 response, GPBS reassembly)
│   │   └── pairing.c           # Connected/encrypted/disconnected callbacks
│   ├── camera_manager/         # Camera slot state machine
│   │   ├── include/
│   │   │   ├── camera_manager.h
│   │   │   └── camera_driver.h # Driver vtable interface
│   │   └── camera_manager.c
│   ├── can_manager/            # CAN bus (TWAI) driver wrapper
│   │   ├── include/can_manager.h
│   │   └── can_manager.c
│   ├── legacy_gopro/           # Hero4 Wi-Fi HTTP/UDP control driver
│   │   ├── include/
│   │   │   ├── legacy_gopro.h          # Public API
│   │   │   └── legacy_gopro_internal.h # Shared state (not public)
│   │   ├── legacy_gopro.c      # State machine, NVS, FreeRTOS task, public API
│   │   └── control.c           # UDP shutter/keepalive/status, HTTP date-time
│   └── wifi_manager/           # Soft-AP, HTTP server, embedded web UI
│       ├── include/wifi_manager.h
│       ├── wifi_manager.c
│       └── www/
│           └── index.html      # Single-page management application (embedded in flash)
├── racecapture_utc_broadcast.lua   # RaceCapture Lua script — broadcasts GPS UTC on 0x602
├── README.md
└── API.md