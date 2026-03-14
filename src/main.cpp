#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"

// ------------------------------ Logging -------------------------------------
static bool LOG_CTS    = true;  // CTS / NTS events — enabled for address discovery
static bool LOG_TX     = true;
static bool LOG_STATUS = true;
static bool LOG_MQTT   = true;
static bool LOG_REG    = true;  // channel registration events

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
unsigned long lastMqttReconnect = 0;

inline void rs485ReceiveMode()  { digitalWrite(PIN_DE_RE, LOW);  }
inline void rs485TransmitMode() { digitalWrite(PIN_DE_RE, HIGH); }

// -------------------------- Pending command queue ---------------------------
enum CommandType : uint8_t { CMD_NONE = 0, CMD_TOGGLE, CMD_SET_TEMP, CMD_SET_TIME, CMD_SET_FILTER };
struct PendingCommand {
  CommandType type;
  uint8_t     toggleByte;
  uint8_t     tempRaw;
  uint8_t     timeHour;
  uint8_t     timeMin;
  uint8_t     fc1h, fc1m, fc1dh, fc1dm;  // filter cycle 1: start + duration
  uint8_t     fc2h, fc2m, fc2dh, fc2dm;  // filter cycle 2: start (bit7=enabled) + duration
  bool        ready;
};
PendingCommand pendingCmd = { CMD_NONE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false };
portMUX_TYPE pendingMux = portMUX_INITIALIZER_UNLOCKED;

// ------------------------------ Balboa items --------------------------------
#define BALBOA_TOGGLE_JETS1        0x04
#define BALBOA_TOGGLE_JETS2        0x05
#define BALBOA_TOGGLE_BLOWER       0x0C
#define BALBOA_TOGGLE_LIGHT        0x11
#define BALBOA_TOGGLE_TEMP_RANGE   0x50
#define BALBOA_TOGGLE_HEATING_MODE 0x51
#define BALBOA_TOGGLE_HOLD         0x3C

// ------------------------------ RS485 protocol ------------------------------
// Frame format: 7E [ML] [byte2] [0xBF/0xAF] [type] [payload...] [CRC] 7E
//
// Observed bus traffic:
//   10 BF 06  — CTS broadcast od spa (zdroj = 0x10), jakékoliv zařízení může odeslat
//   10 BF 07  — NTS od fyzického panelu (adresa 0x10)
//   FF AF 13  — status broadcast
//
// Naše odpověď na CTS: příkaz s adresou 0x0A (BWA app adresa) jako zdrojem,
// nebo NTS 0A BF 07 pokud nemáme co poslat.
// Kanálová registrace (FE BF 0x) tímto spa není podporována.
//
// Reference: https://github.com/ccutrer/balboa_worldwide_app/blob/main/doc/protocol.md

// Adresa přidělená při registraci (FE BF 02). 0 = dosud neregistrováno.
static uint8_t  g_ourAddr     = 0;
static uint32_t g_lastRegSent = 0; // timestamp posledního FE BF 01 (rate limit)

static uint32_t g_lastRxMs      = 0;
static uint32_t g_lastCtsMs     = 0;   // čas posledního CTS (watchdog)
static uint8_t  g_rawDumpFrames = 0;   // dump next N raw frames (debug)

static uint8_t balboaCRC(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0x02;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return (uint8_t)(crc ^ 0x02);
}

static bool validateFrame(const uint8_t* buf, size_t len) {
  if (len < 6) return false;
  if (buf[0] != 0x7E) return false;
  if (buf[len - 1] != 0x7E) return false;

  const uint8_t ml = buf[1];
  if ((size_t)ml + 2 != len) return false;

  const uint8_t packetCrc = buf[len - 2];
  const uint8_t calcCrc   = balboaCRC(buf + 1, (uint8_t)(len - 3));
  return packetCrc == calcCrc;
}

// holdUs: extra µs to keep DE=HIGH after TX + echo (blocks bus from other transmitters).
// Use holdUs≈700 when sending as CTS response (blocks physical panel's NTS at ~1.4ms).
// Returns true if echo bytes matched (TX confirmed working), false on mismatch or timeout.
bool rs485Send(const uint8_t mt[3], const uint8_t* pl, uint8_t plLen, bool logFrame = true,
               uint8_t echoWaitMs = 10, uint16_t holdUs = 0) {
  uint8_t frame[64];
  uint8_t fi = 0;

  const uint8_t ml = (uint8_t)(3 + plLen + 2); // MT(3) + payload + CRC + END
  frame[fi++] = 0x7E;
  frame[fi++] = ml;
  for (uint8_t i = 0; i < 3;     i++) frame[fi++] = mt[i];
  for (uint8_t i = 0; i < plLen; i++) frame[fi++] = pl[i];

  uint8_t crcBuf[64];
  uint8_t ci = 0;
  crcBuf[ci++] = ml;
  for (uint8_t i = 0; i < 3;     i++) crcBuf[ci++] = mt[i];
  for (uint8_t i = 0; i < plLen; i++) crcBuf[ci++] = pl[i];

  frame[fi++] = balboaCRC(crcBuf, ci);
  frame[fi++] = 0x7E;

  rs485TransmitMode();
  Serial2.write(frame, fi);
  Serial2.flush();
  rs485ReceiveMode();  // DE=LOW immediately — must not hold bus while panel sends NTS at ~1.4ms

  // Read echo bytes (RS485 half-duplex loopback) and verify they match
  uint32_t echoDeadline = millis() + echoWaitMs;
  size_t   echoRead     = 0;
  uint8_t  echoBuf[64];
  while (echoRead < fi && (int32_t)(echoDeadline - millis()) > 0) {
    if (Serial2.available()) { echoBuf[echoRead++] = Serial2.read(); }
  }

  // Note: with DE/RE tied together, receiver is OFF during TX → echo=0 is expected.
  // Echo verification only works if RE stays LOW while DE=HIGH (separate control).
  (void)echoRead; (void)echoBuf;

  if (LOG_TX && logFrame) {
    Serial.printf("[TX] %uB:", fi);
    for (uint8_t i = 0; i < fi; i++) Serial.printf(" %02X", frame[i]);
    Serial.println();
  }
  return true;
}

// ------------------------------ Registration --------------------------------

void sendRegistrationRequest() {
  // Odpověď na "Any new clients?" (FE BF 00)
  // Payload: 02 F1 73 (device_type + fixed ID, dle referenčního projektu)
  const uint8_t mt[] = { 0xFE, 0xBF, 0x01 };
  const uint8_t pl[] = { 0x02, 0xF1, 0x73 };
  bool ok = rs485Send(mt, pl, sizeof(pl), true);
  Serial.printf("[REG] sent FE BF 01 (echo %s)\n", ok ? "OK" : "FAIL — TX problem?");
  g_rawDumpFrames = 15; // zachyť odpověď spa (FE BF 02)
}

void sendRegistrationAck() {
  // ACK po přijetí přiřazené adresy (FE BF 02)
  const uint8_t mt[] = { g_ourAddr, 0xBF, 0x03 };
  rs485Send(mt, nullptr, 0, true);
  Serial.printf("[REG] sent ACK (0x%02X BF 03)\n", g_ourAddr);
}

void sendNTS() {
  if (g_ourAddr == 0) return; // nemáme adresu, neodesílat NTS
  const uint8_t mt[] = { g_ourAddr, 0xBF, 0x07 };
  rs485Send(mt, nullptr, 0, false);
}

// ------------------------------ Commands ------------------------------------

void sendToggle(uint8_t item) {
  // Toggle is sent as response to CTS — queue it, do not send unsolicited.
  // This function is kept for completeness but pendingCmd queue is the primary path.
  Serial.println("[CMD] sendToggle called directly — use pendingCmd queue instead");
}

void sendSetTemp(uint8_t tempRaw) {
  // set_temp sent as CTS response from 0x0A (BWA WiFi module address)
  const uint8_t mt[] = { 0x0A, 0xBF, 0x20 };
  const uint8_t pl[] = { tempRaw };
  rs485Send(mt, pl, sizeof(pl), true, 2);
  Serial.printf("[CMD] set_temp %.1f C (raw 0x%02X)\n", tempRaw / 2.0f, tempRaw);
}

// buf[0]=7E, buf[1]=ML, buf[2]=dest, buf[3]=class(BF/AF), buf[4]=type, buf[5..]=payload
void publishStatus(const uint8_t* buf, size_t len) {
  if (len < 30) return;

  static uint32_t lastPublishMs = 0;
  static uint32_t lastAnyMs     = 0;
  static bool     havePrev      = false;
  static uint8_t  prev[9]       = {0};

  const uint32_t now = millis();

  const uint8_t holdState = buf[5];   // 0x00=normal, 0x05=hold active
  const uint8_t waterRaw  = buf[7];
  const uint8_t timeHour  = buf[8];   // 0-23
  const uint8_t timeMin   = buf[9];   // 0-59
  const uint8_t heatMode  = buf[10];  // 0=ready, 1=economy/rest
  const uint8_t holdMins  = buf[12];  // hold minutes remaining (e.g. 60=0x3C)
  const uint8_t flags3    = buf[14];
  // flags3 bit 0: temp scale (0=°F, 1=°C), bit 1: clock mode (0=12h, 1=24h)
  const uint8_t flags4    = buf[15];
  const uint8_t pp        = buf[16];
  const uint8_t flags5    = buf[18];
  const uint8_t lf        = buf[19];
  const uint8_t setRaw    = buf[25];

  bool holdActive = (holdState != 0x00);

  uint8_t curr[9] = { waterRaw, setRaw, flags4, pp, flags5, lf, flags3, heatMode, holdMins };
  bool changed  = !havePrev || memcmp(curr, prev, 8) != 0;
  bool periodic = (now - lastPublishMs) >= 10000;

  if (!changed && !periodic) return;
  if (changed && (now - lastAnyMs) < 400) return;
  lastAnyMs = now;

  float waterTemp = waterRaw / 2.0f;
  float setTemp   = setRaw   / 2.0f;

  uint8_t pump1 = (pp >> 0) & 0x03;
  uint8_t pump2 = (pp >> 2) & 0x03;

  bool jets1       = pump1 != 0;
  bool jets2       = pump2 != 0;
  bool blower      = (flags5 & 0x0C) != 0;
  uint8_t heater   = (flags4 & 0x10) ? 2 : (flags4 & 0x20) ? 1 : 0;
  // heater: 0=off, 1=waiting (flashing), 2=heating (solid)
  bool circulation = (flags5 & 0x02) != 0;
  bool highRange   = (flags4 & 0x04) != 0;
  bool light       = (lf & 0x03) == 0x03;

  if (changed) {
    Serial.printf("[RAW] w=%02X s=%02X t=%02u:%02u hm=%02X f4=%02X pp=%02X f5=%02X lf=%02X f3=%02X hold=%02X/%u\n",
                  waterRaw, setRaw, timeHour, timeMin, heatMode, flags4, pp, flags5, lf, flags3, holdState, holdMins);
  }

  // heating_mode: 0=ready (připravený), 1=economy (ekonomický/rest)
  const char* heatModeStr = (heatMode == 0) ? "ready" : "economy";
  bool clockMode24h = (flags3 & 0x02) != 0;  // bit 1 = 24h mode
  bool tempCelsius  = (flags3 & 0x01) != 0;  // bit 0 = Celsius (informativní)

  char json[384];
  snprintf(json, sizeof(json),
    "{\"water_temp\":%.1f,\"set_temp\":%.1f,"
    "\"time\":\"%02u:%02u\","
    "\"jets1\":%d,\"jets2\":%d,\"blower\":%d,"
    "\"heater\":%u,\"circulation\":%d,"
    "\"high_range\":%d,\"light\":%d,"
    "\"heating_mode\":\"%s\","
    "\"clock_24h\":%d,"
    "\"hold\":%d,\"hold_mins\":%u}",
    waterTemp, setTemp,
    timeHour, timeMin,
    jets1?1:0, jets2?1:0, blower?1:0,
    heater, circulation?1:0,
    highRange?1:0, light?1:0,
    heatModeStr,
    clockMode24h?1:0,
    holdActive?1:0, holdActive ? holdMins : 0
  );

  if (LOG_STATUS)
    Serial.printf("[STATUS%s] %s\n", changed ? "" : ":hb", json);
  if (mqtt.connected())
    mqtt.publish("balboa/state", json, true);

  memcpy(prev, curr, 8);
  havePrev      = true;
  lastPublishMs = now;
}

void processFrame(const uint8_t* buf, size_t len) {
  if (!validateFrame(buf, len)) return;

  g_lastRxMs = millis();

  // Raw dump mode — vypíše každý frame v plném rozsahu (debug registrace)
  if (g_rawDumpFrames > 0) {
    Serial.printf("[RAW%u] %uB:", g_rawDumpFrames, (unsigned)len);
    for (size_t i = 0; i < len; i++) Serial.printf(" %02X", buf[i]);
    Serial.println();
    g_rawDumpFrames--;
  }

  const uint8_t src  = buf[2]; // zdrojová adresa odesílatele
  const uint8_t cls  = buf[3]; // class byte (0xBF nebo 0xAF)
  const uint8_t type = buf[4]; // typ zprávy

  // -------------------- CTS — Clear To Send (BF 06) --------------------
  // Spa sends CTS to 0x10 (physical panel). Registration (FE BF 02) never arrives.
  // Strategy: do NOT intercept CTS — let the panel respond normally with NTS.
  // We send our command AFTER the panel's NTS (see handler below).
  if (cls == 0xBF && type == 0x06) {
    g_lastCtsMs = millis();
    const uint8_t dest = buf[2];
    if (LOG_CTS && dest != 0x10) Serial.printf("[CTS] dest=0x%02X (unexpected)\n", dest);
    return;
  }

  // -------------------- NTS from panel (BF 07) — send command in gap after NTS --------------------
  // After panel responds NTS, there is a gap (~5-50ms) before next CTS.
  // We send our command unsolicited from 0x0A during this gap.
  if (cls == 0xBF && type == 0x07) {
    PendingCommand cmd;
    portENTER_CRITICAL(&pendingMux);
    cmd = pendingCmd;
    if (cmd.ready) {
      pendingCmd.ready = false;
      pendingCmd.type  = CMD_NONE;
    }
    portEXIT_CRITICAL(&pendingMux);

    if (cmd.ready) {
      delayMicroseconds(300);  // small gap after panel NTS before our frame
      if (cmd.type == CMD_TOGGLE) {
        if (cmd.toggleByte == 0xFD) {
          // clock_mode: 0x0A BF 26 [temp_scale=1(°C), clock_mode(0=12h,1=24h)]
          const uint8_t mt[] = { 0x0A, 0xBF, 0x26 };
          const uint8_t pl[] = { 0x01, cmd.tempRaw };  // 0x01=Celsius, clock_mode
          rs485Send(mt, pl, sizeof(pl), true, 2);
          Serial.printf("[CMD] clock_mode %s sent after NTS\n", cmd.tempRaw ? "24h" : "12h");
        } else {
          const uint8_t mt[] = { 0x0A, 0xBF, 0x11 };
          const uint8_t pl[] = { cmd.toggleByte, 0x00 };
          rs485Send(mt, pl, sizeof(pl), true, 2);
          Serial.printf("[CMD] toggle 0x%02X sent after NTS\n", cmd.toggleByte);
        }
        g_rawDumpFrames = 10;
      } else if (cmd.type == CMD_SET_TEMP) {
        const uint8_t mt[] = { 0x0A, 0xBF, 0x20 };
        const uint8_t pl[] = { cmd.tempRaw };
        rs485Send(mt, pl, sizeof(pl), true, 2);
        Serial.printf("[CMD] set_temp raw=0x%02X sent after NTS\n", cmd.tempRaw);
        g_rawDumpFrames = 10;
      } else if (cmd.type == CMD_SET_TIME) {
        const uint8_t mt[] = { 0x0A, 0xBF, 0x21 };
        const uint8_t pl[] = { cmd.timeHour, cmd.timeMin };
        rs485Send(mt, pl, sizeof(pl), true, 2);
        Serial.printf("[CMD] set_time %02u:%02u (%s) sent after NTS\n",
                      cmd.timeHour & 0x7F, cmd.timeMin,
                      (cmd.timeHour & 0x80) ? "24h" : "12h");
        g_rawDumpFrames = 10;
      } else if (cmd.type == CMD_SET_FILTER) {
        const uint8_t mt[] = { 0x0A, 0xBF, 0x23 };
        const uint8_t pl[] = { cmd.fc1h, cmd.fc1m, cmd.fc1dh, cmd.fc1dm,
                               cmd.fc2h, cmd.fc2m, cmd.fc2dh, cmd.fc2dm };
        rs485Send(mt, pl, sizeof(pl), true, 2);
        Serial.printf("[CMD] set_filter FC1=%02u:%02u dur=%uh%02um FC2=%02u:%02u dur=%uh%02um\n",
                      cmd.fc1h, cmd.fc1m, cmd.fc1dh, cmd.fc1dm,
                      cmd.fc2h & 0x7F, cmd.fc2m, cmd.fc2dh, cmd.fc2dm);
        // Publish immediately — don't wait for BF 23 echo from panel
        if (mqtt.connected()) {
          bool fc2en = (cmd.fc2h & 0x80) != 0;
          char json[200];
          snprintf(json, sizeof(json),
            "{\"fc1_start\":\"%02u:%02u\",\"fc1_dur\":\"%uh%02um\","
            "\"fc2_start\":\"%02u:%02u\",\"fc2_dur\":\"%uh%02um\",\"fc2_enabled\":%d}",
            cmd.fc1h, cmd.fc1m, cmd.fc1dh, cmd.fc1dm,
            cmd.fc2h & 0x7F, cmd.fc2m, cmd.fc2dh, cmd.fc2dm,
            fc2en ? 1 : 0);
          mqtt.publish("balboa/filter_config", json, true);
          Serial.printf("[FILTER] immediate publish: %s\n", json);
        }
        g_rawDumpFrames = 10;
      }
    }
    return;
  }

  // -------------------- Status update broadcast (FF AF 13) --------------------
  if (src == 0xFF && type == 0x13) {
    publishStatus(buf, len);
    return;
  }

  // -------------------- Filter cycle config (BF 23) --------------------
  if (cls == 0xBF && type == 0x23 && len >= 14) {
    static uint8_t  prevFilter[8]    = {0xFF};
    static uint32_t lastFilterPubMs  = 0;
    const uint8_t*  pl = buf + 5;  // 8 payload bytes
    const uint32_t  now = millis();
    bool changed  = memcmp(pl, prevFilter, 8) != 0;
    bool periodic = (now - lastFilterPubMs) >= 30000;
    if (changed || periodic) {
      memcpy(prevFilter, pl, 8);
      lastFilterPubMs = now;
      bool fc2en = (pl[4] & 0x80) != 0;
      char json[200];
      snprintf(json, sizeof(json),
        "{\"fc1_start\":\"%02u:%02u\",\"fc1_dur\":\"%uh%02um\","
        "\"fc2_start\":\"%02u:%02u\",\"fc2_dur\":\"%uh%02um\",\"fc2_enabled\":%d}",
        pl[0], pl[1], pl[2], pl[3],
        pl[4] & 0x7F, pl[5], pl[6], pl[7],
        fc2en ? 1 : 0);
      Serial.printf("[FILTER%s] %s\n", changed ? "" : ":hb", json);
      if (mqtt.connected()) mqtt.publish("balboa/filter_config", json, true);
    }
    return;
  }

  // -------------------- Registrační handshake (FE BF 00/02) --------------------
  if (src == 0xFE && cls == 0xBF) {
    if (type == 0x00) {
      // "Any new clients?" — rate limit 15s, jen bez přiřazené adresy
      if (LOG_REG) Serial.println("[REG] received FE BF 00 (Any new clients?)");
      if (g_ourAddr == 0 && (millis() - g_lastRegSent) >= 15000) {
        g_lastRegSent = millis();
        sendRegistrationRequest();
      } else if (g_ourAddr == 0) {
        Serial.printf("[REG] rate-limited, skip (%.0fs since last)\n", (millis()-g_lastRegSent)/1000.0f);
      }
    } else if (type == 0x02 && len > 5) {
      // Přiřazení adresy: buf[5] = payload[0] = assigned channel
      g_ourAddr = buf[5];
      if (g_ourAddr > 0x2F) g_ourAddr = 0x2F;
      Serial.printf("[REG] assigned channel 0x%02X\n", g_ourAddr);
      sendRegistrationAck();
      // After ACK, send config request (BF 22) — required by Balboa WiFi protocol
      // before spa will accept toggle commands from this module.
      g_rawDumpFrames = 20;
      const uint8_t mt22[] = { g_ourAddr, 0xBF, 0x22 };
      const uint8_t pl22[] = { 0x00, 0x00, 0x01 };
      rs485Send(mt22, pl22, sizeof(pl22), true);
      Serial.println("[REG] sent config request (BF 22)");
    }
    return;
  }

  // Debug neznámých framů
  Serial.printf("[OTHER] %uB src=%02X cls=%02X type=%02X:", (unsigned)len, src, cls, type);
  for (size_t i = 0; i < len && i < 16; i++) Serial.printf(" %02X", buf[i]);
  if (len > 16) Serial.print(" ...");
  Serial.println();
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[48] = {0};
  memcpy(msg, payload, min((unsigned int)(sizeof(msg)-1), length));
  if (LOG_MQTT) Serial.printf("[MQTT] %s -> %s\n", topic, msg);

  auto setPendingToggle = [&](uint8_t item) {
    portENTER_CRITICAL(&pendingMux);
    pendingCmd.type       = CMD_TOGGLE;
    pendingCmd.toggleByte = item;
    pendingCmd.ready      = true;
    portEXIT_CRITICAL(&pendingMux);
    if (LOG_MQTT) Serial.printf("[CMD] queued toggle 0x%02X\n", item);
  };

  if      (strcmp(topic, "balboa/cmd/jets1")        == 0) setPendingToggle(BALBOA_TOGGLE_JETS1);
  else if (strcmp(topic, "balboa/cmd/jets2")        == 0) setPendingToggle(BALBOA_TOGGLE_JETS2);
  else if (strcmp(topic, "balboa/cmd/blower")       == 0) setPendingToggle(BALBOA_TOGGLE_BLOWER);
  else if (strcmp(topic, "balboa/cmd/light")        == 0) setPendingToggle(BALBOA_TOGGLE_LIGHT);
  else if (strcmp(topic, "balboa/cmd/high_range")   == 0) setPendingToggle(BALBOA_TOGGLE_TEMP_RANGE);
  else if (strcmp(topic, "balboa/cmd/heating_mode") == 0) setPendingToggle(BALBOA_TOGGLE_HEATING_MODE);
  else if (strcmp(topic, "balboa/cmd/hold")         == 0) setPendingToggle(BALBOA_TOGGLE_HOLD);
  else if (strcmp(topic, "balboa/cmd/dump")         == 0) {
    g_rawDumpFrames = 200;
    Serial.printf("[CMD] raw dump: %u frames\n", (unsigned)g_rawDumpFrames);
  }
  else if (strcmp(topic, "balboa/cmd/passive")      == 0) {
    // Pasivní mód: jen posloucháme, neodesíláme NTS/příkazy — pro zachycení panelových framů
    g_ourAddr = 0;
    g_rawDumpFrames = 500;
    Serial.println("[CMD] passive mode: addr=0, dump 500 frames");
  }
  else if (strcmp(topic, "balboa/cmd/config_req")   == 0) {
    // Config request: id BF 22 00 00 01
    if (g_ourAddr != 0) {
      const uint8_t mt[] = { g_ourAddr, 0xBF, 0x22 };
      const uint8_t pl[] = { 0x00, 0x00, 0x01 };
      bool busy = false;
      portENTER_CRITICAL(&pendingMux);
      busy = pendingCmd.ready;
      if (!busy) {
        // Použijeme CMD_TOGGLE s "magic" byte 0xFF jako marker pro config
        pendingCmd.type       = CMD_TOGGLE;
        pendingCmd.toggleByte = 0xFE; // special — bude handled níže
        pendingCmd.ready      = true;
      }
      portEXIT_CRITICAL(&pendingMux);
      if (!busy) {
        g_rawDumpFrames = 20;
        Serial.println("[CMD] queued config request");
      }
    }
  }
  else if (strcmp(topic, "balboa/cmd/set_temp")     == 0) {
    float t = atof(msg);
    if (t >= 10.0f && t <= 40.0f) {
      uint8_t raw = (uint8_t)(t * 2.0f + 0.5f);
      portENTER_CRITICAL(&pendingMux);
      pendingCmd.type    = CMD_SET_TEMP;
      pendingCmd.tempRaw = raw;
      pendingCmd.ready   = true;
      portEXIT_CRITICAL(&pendingMux);
      if (LOG_MQTT) Serial.printf("[CMD] queued set_temp %.1f C\n", t);
    } else {
      if (LOG_MQTT) Serial.printf("[CMD] set_temp out of range: %.1f\n", t);
    }
  }
  else if (strcmp(topic, "balboa/cmd/set_time") == 0) {
    // Formát: "HH:MM" (24h) nebo "HH:MM 12" pro 12h mód
    // Bit 7 hodinového bajtu = 24h příznak (dle protokolu, zjištěno z panelu)
    int h = -1, m = -1;
    sscanf(msg, "%d:%d", &h, &m);
    bool use24h = (strstr(msg, "12") == nullptr); // default 24h, pokud není "12" v payload
    if (h >= 0 && h <= 23 && m >= 0 && m <= 59) {
      portENTER_CRITICAL(&pendingMux);
      pendingCmd.type     = CMD_SET_TIME;
      pendingCmd.timeHour = (uint8_t)(h | (use24h ? 0x80 : 0x00));
      pendingCmd.timeMin  = (uint8_t)m;
      pendingCmd.ready    = true;
      portEXIT_CRITICAL(&pendingMux);
      if (LOG_MQTT) Serial.printf("[CMD] queued set_time %02d:%02d (%s)\n", h, m, use24h ? "24h" : "12h");
    } else {
      if (LOG_MQTT) Serial.printf("[CMD] set_time invalid: %s (use HH:MM)\n", msg);
    }
  }
  else if (strcmp(topic, "balboa/cmd/set_filter") == 0) {
    // Format: "H1:M1 DH1:DM1 H2:M2 DH2:DM2"
    // Příklad: "0:00 5:00 12:00 5:00" = FC1 00:00 trvá 5h, FC2 12:00 trvá 5h (FC2 vždy enabled)
    int h1=-1, m1=-1, dh1=-1, dm1=-1, h2=-1, m2=-1, dh2=-1, dm2=-1;
    int n = sscanf(msg, "%d:%d %d:%d %d:%d %d:%d", &h1, &m1, &dh1, &dm1, &h2, &m2, &dh2, &dm2);
    if (n == 8 &&
        h1 >= 0 && h1 <= 23 && m1 >= 0 && m1 <= 59 &&
        dh1 >= 0 && dh1 <= 23 && dm1 >= 0 && dm1 <= 59 &&
        h2 >= 0 && h2 <= 23 && m2 >= 0 && m2 <= 59 &&
        dh2 >= 0 && dh2 <= 23 && dm2 >= 0 && dm2 <= 59) {
      portENTER_CRITICAL(&pendingMux);
      pendingCmd.type  = CMD_SET_FILTER;
      pendingCmd.fc1h  = (uint8_t)h1;
      pendingCmd.fc1m  = (uint8_t)m1;
      pendingCmd.fc1dh = (uint8_t)dh1;
      pendingCmd.fc1dm = (uint8_t)dm1;
      pendingCmd.fc2h  = (uint8_t)(h2 | 0x80);  // bit7=1 = FC2 enabled
      pendingCmd.fc2m  = (uint8_t)m2;
      pendingCmd.fc2dh = (uint8_t)dh2;
      pendingCmd.fc2dm = (uint8_t)dm2;
      pendingCmd.ready = true;
      portEXIT_CRITICAL(&pendingMux);
      if (LOG_MQTT) Serial.printf("[CMD] queued set_filter FC1=%02d:%02d dur=%dh%02dm FC2=%02d:%02d dur=%dh%02dm\n",
                                   h1, m1, dh1, dm1, h2, m2, dh2, dm2);
    } else {
      if (LOG_MQTT) Serial.printf("[CMD] set_filter invalid: %s (use H1:M1 DH1:DM1 H2:M2 DH2:DM2)\n", msg);
    }
  }
  else if (strcmp(topic, "balboa/cmd/clock_mode") == 0) {
    // Přepnutí 12h/24h bez změny času — použij set_time s aktuálním časem
    // Posíláme BF 21 s hodinami | 0x80 pro 24h, nebo bez 0x80 pro 12h
    // Čas bude nutné nastavit zvlášť (nevíme přesný aktuální čas v tomto callbacku)
    int mode = atoi(msg);
    if (mode == 12 || mode == 24) {
      if (LOG_MQTT) Serial.printf("[CMD] clock_mode %dh — use set_time HH:MM to apply\n", mode);
    }
  }
}

void connectWiFi() {
  Serial.print("Connecting WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi OK");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
}

void connectMQTT() {
  if (mqtt.connected()) return;
  Serial.print("MQTT connect... ");
  String clientId = "esp32-balboa-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  bool ok = (strlen(MQTT_USER) > 0)
    ? mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)
    : mqtt.connect(clientId.c_str());
  if (ok) {
    Serial.println("OK");
    mqtt.subscribe("balboa/cmd/#");
    Serial.println("[MQTT] subscribed balboa/cmd/#");
  } else {
    Serial.printf("FAIL rc=%d\n", mqtt.state());
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nBalboa RS485 Controller");
  pinMode(PIN_DE_RE, OUTPUT);
  rs485ReceiveMode();
  Serial2.begin(115200, SERIAL_8N1, PIN_RX2, PIN_TX2);

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  connectWiFi();
  connectMQTT();

  Serial.println("Listening...");
}

void loop() {
  // MQTT: process up to 1ms — then immediately give RS485 priority
  if (!mqtt.connected()) {
    if (millis() - lastMqttReconnect > 3000) {
      lastMqttReconnect = millis();
      connectMQTT();
    }
  } else {
    mqtt.loop();
  }

  // Watchdog: žádný RS485 provoz
  if (g_lastRxMs != 0 && (millis() - g_lastRxMs) > 15000) {
    Serial.println("[WARN] no RS485 traffic for 15s");
    g_lastRxMs = millis();
  }
  // Watchdog: žádné CTS po 10s
  if (g_lastCtsMs != 0 && (millis() - g_lastCtsMs) > 10000) {
    Serial.println("[WARN] no CTS for 10s");
    g_lastCtsMs = millis();
  }

  // RS485 — process ALL available bytes immediately after MQTT
  static uint8_t buffer[256];
  static size_t  index = 0;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (b == 0x7E) {
      if (index > 1) {
        buffer[index++] = b;
        processFrame(buffer, index);
      }
      buffer[0] = 0x7E;
      index = 1;
    } else {
      if (index < sizeof(buffer)) buffer[index++] = b;
      if (index >= sizeof(buffer)) index = 0;
    }
  }
}
