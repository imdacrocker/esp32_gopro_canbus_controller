# ESP32 GoPro CAN Bus Controller

This project is currently in a working prrof-of-concept phase!  The project has been tested against a GoPro Hero13 black, but currently no more than that.


An ESP32-S3 firmware that bridges GoPro cameras (controlled over BLE) with a [RaceCapture](https://autosportlabs.com/racecapture/) data logger (connected over CAN bus). When RaceCapture starts logging, all paired GoPro cameras start recording automatically. Camera connection status is broadcast back to RaceCapture in real time.

A companion Wi-Fi web interface lets you pair cameras, check status, and manually trigger recording — no laptop or serial terminal required in the field.

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
- [CAN Bus Wiring](#can-bus-wiring)
- [Troubleshooting](#troubleshooting)
- [Project Structure](#project-structure)
- [API Reference](#api-reference)

---

## TODO
Known bugs:
 - Clearing the camera pairing from the web interface, is not working
 - Wifi and BLE are fighting each other on the chip.  This may need to be smoothed out

 Features:
 - Faster triggering of multiple cameras

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
│   tick timer)   │             │   0x601 TX 5Hz) │
└────────┬────────┘             └─────────────────┘
         │ driver vtable
         ▼
┌─────────────────┐
│ open_gopro_ble  │
│  (OpenGoPro     │
│   BLE protocol, │
│   GATT handles, │
│   status poll,  │
│   keep-alive)   │
└────────┬────────┘
         │ BLE callbacks
         ▼
┌─────────────────┐
│    ble_core     │
│  (NimBLE stack, │
│   scan, connect,│
│   bond store)   │
└─────────────────┘

┌─────────────────┐
│  wifi_manager   │
│  (Soft-AP,      │
│   HTTP server,  │
│   embedded UI)  │
└─────────────────┘
```

**Data flow summary:**

1. RaceCapture sends a `0x600` CAN frame with `isLogging = 1`.
2. `can_manager` detects the state change and calls the registered logging callback.
3. `app_main`'s callback sets the desired recording state on `camera_manager` and immediately dispatches a start command to all GATT-ready cameras.
4. `open_gopro_ble` sends the OpenGoPro `shutter start` TLV over BLE to each camera.
5. `open_gopro_ble`'s status poll timer queries each camera for `encoding_active` every 5 seconds and updates the recording status in the driver context. A separate keep-alive command is sent to every connected camera every 3 seconds to prevent the camera from auto-sleeping.
6. `camera_manager`'s tick timer (2 s) reads the recording status from each driver, fires the state-change callback, and retries the start command if a camera is still not recording.
7. `app_main`'s state-change callback maps the internal status to a `camera_state_t` and calls `can_manager_set_camera_state()`.
8. `can_manager` broadcasts the updated `0x601` status frame to RaceCapture at 5 Hz.

---

## Component Overview

| Component | Purpose |
|-----------|---------|
| `ble_core` | NimBLE stack wrapper. Owns scan, connect, encrypt, GATT write, and bond management. Camera-agnostic. |
| `open_gopro_ble` | OpenGoPro BLE driver. Implements the OpenGoPro BLE protocol (service UUID 0xFEA6, TLV command encoding). The camera is considered ready as soon as CCCD subscriptions complete — no polling loop is needed. `GetHardwareInfo` is available on demand via `gopro_query_send_hw_info()`. Sends a keep-alive packet every 3 seconds (per OpenGoPro spec) to prevent auto-sleep. Provides a `camera_driver_t` vtable to `camera_manager`. |
| `camera_manager` | Camera slot state machine. Persists camera records to NVS. Runs the 2-second tick timer that retries recording commands and publishes state changes. |
| `can_manager` | ESP-IDF v6.0 TWAI driver wrapper. Receives `0x600` frames, broadcasts `0x601` frames at 5 Hz. Thread-safe camera state updates. |
| `wifi_manager` | Soft-AP + HTTP server. Serves the embedded web UI and all `/api/*` endpoints. |

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

## Camera Setup

- Connect to the ESP32 via wifi.  The SSID will be HERO-RC-XXXXXX (with the last 6 being your ESP32's MAC address)
Note:  The Wifi connection seems to be colliding with the BLE at the moment.  You may have to refresh your settings and try multiple times to connect
- Open a web browser and go to 10.71.79.1
- Select Scan 
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
| `CAN_ID_RC_COMMAND` | `0x600` | CAN ID for RaceCapture → ESP32 commands. |
| `CAN_ID_CAM_STATUS` | `0x601` | CAN ID for ESP32 → RaceCapture status. |

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

### Wi-Fi (`components/wifi_manager/wifi_manager.c`)

The AP IP address (`10.71.79.1`) and channel (`1`) are compile-time constants in `wifi_manager.c`. The SSID is generated at runtime from the last 3 bytes of the AP MAC: `HERO-RC-XXXXXX`.

---

## First-Run: Pairing Cameras

Cameras are paired via the built-in web interface. You do **not** need to use the GoPro app or pair via Bluetooth settings on your phone.

1. Power on the controller. The Wi-Fi AP will start in a few seconds.
2. On your phone or laptop, connect to the Wi-Fi network `HERO-RC-XXXXXX` (open, no password).
3. Open a browser and navigate to `http://10.71.79.1`.
4. Power on the GoPro camera(s) and ensure Bluetooth is enabled on the camera.
5. On the web page, tap **Scan for Cameras**. A 30-second scan will begin.
6. When your camera appears in the list, tap **Pair**. The controller will initiate a BLE connection and complete the pairing process.
7. Once paired, the camera appears in the **Camera Status** section. The status will change to **Connected** once the BLE link and GATT setup complete (typically a few seconds).

Paired cameras are stored in NVS and reconnect automatically every time the controller boots — you only need to pair once.

To remove all camera pairings, tap **Reset Bonds** on the web page.

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

#### Sending logging state (0x600)

Configure a RaceCapture Lua script to transmit a CAN frame on ID `0x600` (decimal 1536) containing `isLogging` (0 or 1) in byte 0 at approximately 10 Hz.

> **Lua note:** RaceCapture Lua arrays are 1-indexed. `data[1]` corresponds to CAN byte offset 0.

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
- Confirm the `0x600` frame is being transmitted by RaceCapture (use the serial monitor to watch for `0x600 RX` log lines from `can_manager`).
- Confirm cameras show as `recording` in the web UI after a manual **Start Recording** press.
- Check that the desired recording state is being set: look for `Start recording dispatched to N camera(s)` in the serial log.

**Web UI is not reachable**
- Wait 5–10 seconds after power-on for the AP to initialise.
- Ensure your device is connected to `HERO-RC-XXXXXX`, not another network.
- Try `http://10.71.79.1` directly rather than using mDNS.

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
│   │   ├── notify.c            # GATT notification handler (recording status, command responses)
│   │   ├── query.c             # On-demand query commands (GetHardwareInfo 0x3C, GPBS reassembly)
│   │   └── pairing.c           # Connected/encrypted/disconnected callbacks
│   ├── camera_manager/         # Camera slot state machine
│   │   ├── include/
│   │   │   ├── camera_manager.h
│   │   │   └── camera_driver.h # Driver vtable interface
│   │   └── camera_manager.c
│   ├── can_manager/            # CAN bus (TWAI) driver wrapper
│   │   ├── include/can_manager.h
│   │   └── can_manager.c
│   └── wifi_manager/           # Soft-AP + HTTP server
│       ├── include/wifi_manager.h
│       ├── wifi_manager.c
│       └── www/
│           └── index.html      # Embedded web UI
├── CMakeLists.txt
├── partitions.csv
├── sdkconfig.defaults
└─�