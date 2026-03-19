# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Flash

```bash
# Build
pio run

# Flash to ESP32 (USB)
pio run --target upload

# Flash over WiFi (OTA) — after initial USB flash
pio run --target upload --upload-port balboa-esp32.local

# Serial monitor (115200 baud)
pio device monitor

# Build + flash + monitor in one
pio run --target upload && pio device monitor
```

Port: `/dev/cu.usbserial-0001` (configured in `platformio.ini`). OTA hostname: `balboa-esp32` (mDNS).

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

Commands set `pendingCmd` (mutex-protected) and are sent **in the gap after the physical panel's NTS (BF 07)**, not during CTS. The physical panel (addr `0x10`) responds to CTS with NTS; the ESP32 then sends its command ~300µs later from address `0x0A` (BWA WiFi module address).

Toggle bytes: JETS1=`0x04`, JETS2=`0x05`, BLOWER=`0x0C`, LIGHT=`0x11`, TEMP_RANGE=`0x50`, HEATING_MODE=`0x51`, HOLD=`0x3C`.

Set temp: raw value = `°C × 2`, range 10–40 °C.
Set time (BF 21): payload `[hour | 0x80, min]` for 24h mode; `[hour, min]` for 12h.
Set filter (BF 23): payload 8 bytes — FC1 start h/m, FC1 duration h/m, FC2 start h/m (bit7=enabled), FC2 duration h/m.

### MQTT topics

| Topic | Direction | Notes |
|-------|-----------|-------|
| `balboa/state` | publish (retained) | JSON with all spa fields |
| `balboa/filter_config` | publish (retained) | JSON with filter cycle config |
| `balboa/cmd/jets1` (jets2, blower, light, high_range, heating_mode, hold) | subscribe | any payload triggers toggle |
| `balboa/cmd/set_temp` | subscribe | float °C as string |
| `balboa/cmd/set_time` | subscribe | `"HH:MM"` (24h default); append ` 12` for 12h mode |
| `balboa/cmd/set_filter` | subscribe | `"H1:M1 DH1:DM1 H2:M2 DH2:DM2"` (FC2 always enabled) |
| `balboa/cmd/config_req` | subscribe | sends BF 22 config request to spa |
| `balboa/cmd/dump` | subscribe | triggers 200-frame raw dump to serial |
| `balboa/cmd/passive` | subscribe | receive-only mode (clears addr), 500-frame dump |

### Status JSON fields

`water_temp`, `set_temp` (°C float), `time` (HH:MM string), `jets1`, `jets2`, `blower`, `heater` (0=off, 1=waiting/flashing, 2=heating/solid), `circulation`, `high_range`, `light`, `clock_24h`, `hold`, `hold_mins` (0/1 int unless noted), `heating_mode` ("ready" or "economy").

Source bytes: `buf[7]`=water, `buf[8]`=hour, `buf[9]`=min, `buf[10]`=heatMode, `buf[12]`=holdMins, `buf[14]`=flags3 (bit0=°C, bit1=24h), `buf[15]`=flags4 (bit2=highRange, bit4=heater-solid, bit5=heater-flash), `buf[16]`=pp (pumps), `buf[18]`=flags5 (bit1=circ, bits2-3=blower), `buf[19]`=lf (bits0-1=light), `buf[25]`=setRaw.

`processFrame()` also handles `BF 23` (filter cycle config) → `balboa/filter_config` JSON.

## Known hardware constraints

- ESP32 is on J35 (dedicated WiFi slot). Physical panel is on J34 — same RS485 bus.
- DE pin must go LOW **immediately after** `Serial2.flush()` — not after echo wait.
- Spa (BP2100G1) sends CTS exclusively to address `0x10`. Registration (FE BF 02) has never been observed to succeed on this bus.
- Physical panel (addr `0x10`) responds NTS at ~1.4ms after CTS. ESP32 does NOT intercept CTS — it sends its command ~300µs after the panel's NTS, in the gap before the next CTS.

## Debug tools

`capture_raw.py` — captures serial output to `./logs/`.

Log flags at top of `main.cpp`: `LOG_CTS`, `LOG_TX`, `LOG_STATUS`, `LOG_MQTT`, `LOG_REG` — toggle to reduce noise.

`g_rawDumpFrames` — set to N to print next N raw frames to serial (set automatically after commands and registration events).
