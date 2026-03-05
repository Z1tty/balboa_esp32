#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"

// ------------------------------ Logging -------------------------------------
static bool LOG_CTS    = true;  // CTS / NTS events
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
enum CommandType : uint8_t { CMD_NONE = 0, CMD_TOGGLE, CMD_SET_TEMP };
struct PendingCommand {
  CommandType type;
  uint8_t     toggleByte;
  uint8_t     tempRaw;
  bool        ready;
};
PendingCommand pendingCmd = { CMD_NONE, 0, 0, false };
portMUX_TYPE pendingMux = portMUX_INITIALIZER_UNLOCKED;

// ------------------------------ Balboa items --------------------------------
#define BALBOA_TOGGLE_JETS1        0x04
#define BALBOA_TOGGLE_JETS2        0x05
#define BALBOA_TOGGLE_BLOWER       0x0C
#define BALBOA_TOGGLE_LIGHT        0x11
#define BALBOA_TOGGLE_TEMP_RANGE   0x50
#define BALBOA_TOGGLE_HEATING_MODE 0x51

// ------------------------------ RS485 protocol ------------------------------
// Frame format: 7E [ML] [byte2] [0xBF] [type] [payload...] [CRC] 7E
// byte2 = destination address (0xFE = broadcast unregistered, 0xFF = all,
//         our assigned id = addressed to us)
//
// Channel registration sequence:
//   Spa  -> 7E .. FE BF 00 ..  "Any new clients?"
//   Us   -> 7E .. FE BF 01 02 F1 73 ..  registration request
//   Spa  -> 7E .. FE BF 02 [id] ..  "Here is your address"
//   Us   -> 7E .. [id] BF 03 ..  ack
//
// Poll cycle (after registration):
//   Spa  -> 7E .. [id] BF 06 ..  Clear To Send (CTS)
//   Us   -> command  OR  7E .. [id] BF 07 ..  Nothing To Send (NTS)
//
// Reference: https://github.com/ccutrer/balboa_worldwide_app/blob/main/doc/protocol.md

static uint32_t g_lastRxMs      = 0;
static uint8_t  g_deviceId      = 0;   // assigned by spa (0 = unregistered)
static uint32_t g_lastCtsMs     = 0;   // last CTS received (for re-registration watchdog)
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

void rs485Send(const uint8_t mt[3], const uint8_t* pl, uint8_t plLen, bool logFrame = true) {
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
  delayMicroseconds(100);
  Serial2.write(frame, fi);
  Serial2.flush();

  // Read & discard echo (RS485 half-duplex loopback) — exactly fi bytes,
  // so we don't accidentally swallow the spa's immediate response.
  uint32_t echoDeadline = millis() + 10;
  size_t   echoRead     = 0;
  while (echoRead < fi && (int32_t)(echoDeadline - millis()) > 0) {
    if (Serial2.available()) { Serial2.read(); echoRead++; }
  }
  rs485ReceiveMode();

  if (LOG_TX && logFrame) {
    Serial.printf("[TX] %uB:", fi);
    for (uint8_t i = 0; i < fi; i++) Serial.printf(" %02X", frame[i]);
    Serial.println();
  }
}

// ------------------------------ Registration --------------------------------

void sendIDRequest() {
  // Registration request: FE BF 01, payload = device identifier bytes
  const uint8_t mt[] = { 0xFE, 0xBF, 0x01 };
  const uint8_t pl[] = { 0x02, 0xF1, 0x73 };
  rs485Send(mt, pl, sizeof(pl), true);
  if (LOG_REG) Serial.println("[REG] ID request sent");
  g_rawDumpFrames = 5; // dump next 5 raw frames to see spa's response
}

void sendIDAck() {
  // Acknowledge assigned address: [id] BF 03
  const uint8_t mt[] = { g_deviceId, 0xBF, 0x03 };
  rs485Send(mt, nullptr, 0, true);
  if (LOG_REG) Serial.printf("[REG] ID ack sent (id=0x%02X)\n", g_deviceId);
}

void sendNTS() {
  // Nothing To Send: [id] BF 07
  const uint8_t mt[] = { g_deviceId, 0xBF, 0x07 };
  rs485Send(mt, nullptr, 0, false);
}

// ------------------------------ Commands ------------------------------------

void sendToggle(uint8_t item) {
  // Toggle item: [id] BF 11, payload = [item, 0x00]
  const uint8_t mt[] = { g_deviceId, 0xBF, 0x11 };
  const uint8_t pl[] = { item, 0x00 };
  rs485Send(mt, pl, sizeof(pl), true);
}

void sendSetTemp(uint8_t tempRaw) {
  // Set temperature: [id] BF 20, payload = [tempRaw]
  const uint8_t mt[] = { g_deviceId, 0xBF, 0x20 };
  const uint8_t pl[] = { tempRaw };
  rs485Send(mt, pl, sizeof(pl), true);
  if (LOG_TX) Serial.printf("[TX] set_temp %.1f C (raw 0x%02X)\n", tempRaw / 2.0f, tempRaw);
}

// buf[0]=7E, buf[1]=ML, buf[2]=dest, buf[3]=class(BF/AF), buf[4]=type, buf[5..]=payload
void publishStatus(const uint8_t* buf, size_t len) {
  if (len < 30) return;

  static uint32_t lastPublishMs = 0;
  static uint32_t lastAnyMs     = 0;
  static bool     havePrev      = false;
  static uint8_t  prev[7]       = {0};

  const uint32_t now = millis();

  const uint8_t waterRaw = buf[7];
  const uint8_t setRaw   = buf[25];
  const uint8_t flags4   = buf[15];
  const uint8_t pp       = buf[16];
  const uint8_t flags5   = buf[18];
  const uint8_t lf       = buf[19];
  const uint8_t flags3   = buf[14];

  uint8_t curr[7] = { waterRaw, setRaw, flags4, pp, flags5, lf, flags3 };
  bool changed  = !havePrev || memcmp(curr, prev, 7) != 0;
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
  bool heater      = (flags4 & 0x30) != 0;
  bool circulation = (flags5 & 0x02) != 0;
  bool highRange   = (flags4 & 0x04) != 0;
  bool light       = (lf & 0x03) == 0x03;

  if (changed) {
    Serial.printf("[RAW] w=%02X s=%02X f4=%02X pp=%02X f5=%02X lf=%02X f3=%02X\n",
                  waterRaw, setRaw, flags4, pp, flags5, lf, flags3);
  }

  char json[320];
  snprintf(json, sizeof(json),
    "{\"water_temp\":%.1f,\"set_temp\":%.1f,"
    "\"jets1\":%d,\"jets2\":%d,\"blower\":%d,"
    "\"heater\":%d,\"circulation\":%d,"
    "\"high_range\":%d,\"light\":%d}",
    waterTemp, setTemp,
    jets1?1:0, jets2?1:0, blower?1:0,
    heater?1:0, circulation?1:0,
    highRange?1:0, light?1:0
  );

  if (LOG_STATUS)
    Serial.printf("[STATUS%s] %s\n", changed ? "" : ":hb", json);
  if (mqtt.connected())
    mqtt.publish("balboa/state", json, true);

  memcpy(prev, curr, 7);
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

  const uint8_t dest = buf[2]; // byte2 (source nebo dest podle směru)
  const uint8_t cls  = buf[3]; // class byte (0xBF nebo 0xAF)
  const uint8_t type = buf[4]; // typ zprávy

  // -------------------- Channel registration --------------------
  // "Any new clients?" FE BF 00
  if (dest == 0xFE && cls == 0xBF && type == 0x00) {
    if (g_deviceId == 0) {
      if (LOG_REG) Serial.println("[REG] Got 'Any new clients?' -> sending ID request");
      // Clear any stuck pending command so it doesn't block after registration
      portENTER_CRITICAL(&pendingMux);
      pendingCmd.ready = false;
      pendingCmd.type  = CMD_NONE;
      portEXIT_CRITICAL(&pendingMux);
      sendIDRequest();
    }
    return;
  }

  // "Here is your assigned ID" — typ 0x02, byte[2] může být 0xFE nebo 0x10 (adresa spa)
  if (cls == 0xBF && type == 0x02 && g_deviceId == 0) {
    if (len >= 7) {
      uint8_t newId = buf[5];
      if (newId > 0x2F) newId = 0x2F;
      g_deviceId = newId;
      if (LOG_REG) Serial.printf("[REG] Assigned ID: 0x%02X (from byte2=0x%02X)\n", g_deviceId, dest);
      sendIDAck();
    }
    return;
  }

  // Silently ignore CTS/NTS for other registered devices (reduces log noise)
  if (dest != 0xFE && dest != 0xFF && cls == 0xBF && (type == 0x06 || type == 0x07) && dest != g_deviceId)
    return;

  // -------------------- CTS (Clear To Send) addressed to us --------------------
  if (g_deviceId != 0 && dest == g_deviceId && cls == 0xBF && type == 0x06) {
    g_lastCtsMs = millis();

    PendingCommand cmd;
    portENTER_CRITICAL(&pendingMux);
    cmd              = pendingCmd;
    pendingCmd.ready = false;
    pendingCmd.type  = CMD_NONE;
    portEXIT_CRITICAL(&pendingMux);

    if (cmd.ready) {
      if (LOG_CTS) Serial.printf("[CTS] sending cmd type=%d\n", cmd.type);
      if (cmd.type == CMD_TOGGLE)   sendToggle(cmd.toggleByte);
      if (cmd.type == CMD_SET_TEMP) sendSetTemp(cmd.tempRaw);
    } else {
      sendNTS();
      if (LOG_CTS) Serial.println("[CTS] NTS");
    }
    return;
  }

  // -------------------- Status update (broadcast) --------------------
  // FF AF 13
  if (dest == 0xFF && type == 0x13) {
    publishStatus(buf, len);
    return;
  }

  // -------------------- Ignore known noise --------------------
  if (cls == 0xBF && type == 0x07) return; // NTS from other clients
  // Log unhandled FE BF frames to help diagnose registration
  if (dest == 0xFE && cls == 0xBF) {
    Serial.printf("[REG?] Unhandled FE BF type=%02X len=%u:", type, (unsigned)len);
    for (size_t i = 0; i < len; i++) Serial.printf(" %02X", buf[i]);
    Serial.println();
    return;
  }

  // Debug unknown frames (limited output)
  Serial.printf("[OTHER] %uB dest=%02X cls=%02X type=%02X:", (unsigned)len, dest, cls, type);
  for (size_t i = 0; i < len && i < 16; i++) Serial.printf(" %02X", buf[i]);
  if (len > 16) Serial.print(" ...");
  Serial.println();
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[48] = {0};
  memcpy(msg, payload, min((unsigned int)(sizeof(msg)-1), length));
  if (LOG_MQTT) Serial.printf("[MQTT] %s -> %s\n", topic, msg);

  auto setPendingToggle = [&](uint8_t item) {
    bool busy = false;
    portENTER_CRITICAL(&pendingMux);
    busy = pendingCmd.ready;
    if (!busy) {
      pendingCmd.type       = CMD_TOGGLE;
      pendingCmd.toggleByte = item;
      pendingCmd.ready      = true;
    }
    portEXIT_CRITICAL(&pendingMux);
    if (LOG_MQTT) Serial.printf("[CMD] %s toggle 0x%02X\n", busy ? "ignored (busy)" : "queued", item);
  };

  if      (strcmp(topic, "balboa/cmd/jets1")        == 0) setPendingToggle(BALBOA_TOGGLE_JETS1);
  else if (strcmp(topic, "balboa/cmd/jets2")        == 0) setPendingToggle(BALBOA_TOGGLE_JETS2);
  else if (strcmp(topic, "balboa/cmd/blower")       == 0) setPendingToggle(BALBOA_TOGGLE_BLOWER);
  else if (strcmp(topic, "balboa/cmd/light")        == 0) setPendingToggle(BALBOA_TOGGLE_LIGHT);
  else if (strcmp(topic, "balboa/cmd/high_range")   == 0) setPendingToggle(BALBOA_TOGGLE_TEMP_RANGE);
  else if (strcmp(topic, "balboa/cmd/heating_mode") == 0) setPendingToggle(BALBOA_TOGGLE_HEATING_MODE);
  else if (strcmp(topic, "balboa/cmd/set_temp")     == 0) {
    float t = atof(msg);
    if (t >= 10.0f && t <= 40.0f) {
      uint8_t raw = (uint8_t)(t * 2.0f + 0.5f);
      bool busy = false;
      portENTER_CRITICAL(&pendingMux);
      busy = pendingCmd.ready;
      if (!busy) {
        pendingCmd.type    = CMD_SET_TEMP;
        pendingCmd.tempRaw = raw;
        pendingCmd.ready   = true;
      }
      portEXIT_CRITICAL(&pendingMux);
      if (LOG_MQTT) Serial.printf("[CMD] %s set_temp %.1f C\n", busy ? "ignored (busy)" : "queued", t);
    } else {
      if (LOG_MQTT) Serial.printf("[CMD] set_temp out of range: %.1f\n", t);
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
  Serial.println("Waiting for registration (FE BF 00) and status updates (FF AF 13)...");
}

void loop() {
  if (!mqtt.connected()) {
    if (millis() - lastMqttReconnect > 3000) {
      lastMqttReconnect = millis();
      connectMQTT();
    }
  } else {
    mqtt.loop();
  }

  // No RS485 traffic watchdog — reset registration
  if (g_lastRxMs != 0 && (millis() - g_lastRxMs) > 15000) {
    Serial.println("[WARN] no RS485 traffic for 15s — resetting registration");
    g_lastRxMs  = millis();
    g_deviceId  = 0;
    g_lastCtsMs = 0;
  }

  // Registered but no CTS for 10s — spa dropped us, re-register
  if (g_deviceId != 0 && g_lastCtsMs != 0 && (millis() - g_lastCtsMs) > 10000) {
    Serial.printf("[WARN] no CTS for 10s (was id=0x%02X) — re-registering\n", g_deviceId);
    g_deviceId  = 0;
    g_lastCtsMs = 0;
  }

  static uint8_t buffer[256];
  static size_t  index = 0;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (index < sizeof(buffer)) buffer[index++] = b;

    // Frame boundary: two consecutive 0x7E bytes (end of frame + start of next)
    if (index >= 2 && buffer[index-2] == 0x7E && buffer[index-1] == 0x7E) {
      processFrame(buffer, index - 1);
      buffer[0] = 0x7E;
      index = 1;
    }

    if (index >= sizeof(buffer)) index = 0;
  }
}
