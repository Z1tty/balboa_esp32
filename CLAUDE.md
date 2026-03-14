# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Flash

```bash
# Build
pio run

# Flash to ESP32
pio run --target upload

# Serial monitor (115200 baud)
pio device monitor

# Build + flash + monitor in one
pio run --target upload && pio device monitor
```

Port: `/dev/cu.usbserial-0001` (configured in `platformio.ini`).

## Configuration

Copy `include/config.h.example` → `include/config.h` and fill in:
- `WIFI_SSID` / `WIFI_PASSWORD`
- `MQTT_SERVER` / `MQTT_PORT`
- `PIN_DE_RE=17`, `PIN_RX2=16`, `PIN_TX2=4` (already set in example)

`config.h` is gitignored.

## Architecture

Single-file firmware (`src/main.cpp`). No RTOS tasks — everything runs in `loop()` with tight RS485 timing.

### RS485 frame flow

```
Serial2 (RX) → loop() byte accumulator → processFrame() → publishStatus() → MQTT
MQTT callback → pendingCmd queue → processFrame() CTS handler → rs485Send()
```

**Frame format:** `7E [ML] [src/dest] [class:BF/AF] [type] [payload...] [CRC] 7E`

`processFrame()` handles three message types:
- `FF AF 13` — status broadcast → `publishStatus()` → `balboa/state` JSON
- `xx BF 06` (CTS to `0x10`) → dequeues `pendingCmd` and fires toggle or set_temp
- `FE BF 00` — "any new clients?" → registration handshake (currently unused by spa)

### Command path (MQTT → spa)

Commands never send unsolicited. They set `pendingCmd` (mutex-protected) and wait for the next CTS-to-0x10 from the spa. The ESP32 must respond within ~0.8ms of CTS (before physical panel's NTS at ~1.4ms). All commands send from address `0x0A` (BWA WiFi module address).

Toggle bytes: JETS1=`0x04`, JETS2=`0x05`, BLOWER=`0x0C`, LIGHT=`0x11`, TEMP_RANGE=`0x50`, HEATING_MODE=`0x51`.

Set temp: raw value = `°C × 2`, range 10–40 °C.

### MQTT topics

| Topic | Direction | Notes |
|-------|-----------|-------|
| `balboa/state` | publish (retained) | JSON with all spa fields |
| `balboa/cmd/jets1` (jets2, blower, light, high_range, heating_mode) | subscribe | any payload triggers toggle |
| `balboa/cmd/set_temp` | subscribe | float °C as string |
| `balboa/cmd/dump` | subscribe | triggers 200-frame raw dump to serial |
| `balboa/cmd/passive` | subscribe | receive-only mode, 500-frame dump |

### Status JSON fields

`water_temp`, `set_temp` (°C float), `jets1`, `jets2`, `blower`, `heater`, `circulation`, `high_range`, `light` (0/1 int).

Source bytes: `buf[7]`=water, `buf[25]`=set, `buf[14]`=flags3, `buf[15]`=flags4, `buf[16]`=pp (pumps), `buf[18]`=flags5, `buf[19]`=lf (light/fan).

## Known hardware constraints

- ESP32 is on J35 (dedicated WiFi slot). Physical panel is on J34 — same RS485 bus.
- DE pin must go LOW **immediately after** `Serial2.flush()` — not after echo wait.
- Spa (BP2100G1) sends CTS exclusively to address `0x10`. Registration (FE BF 02) has never been observed to succeed on this bus.
- Physical panel (addr `0x10`) responds NTS at ~1.4ms after CTS. ESP32 toggle must arrive first (~0.8ms window).

## Debug tools

`capture_raw.py` — captures serial output to `./logs/`.

Log flags at top of `main.cpp`: `LOG_CTS`, `LOG_TX`, `LOG_STATUS`, `LOG_MQTT`, `LOG_REG` — toggle to reduce noise.

`g_rawDumpFrames` — set to N to print next N raw frames to serial (set automatically after commands and registration events).
