#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"

// ------------------------------ Logging -------------------------------------
// TIP: Keep LOG_TX = true for command debugging, but we intentionally do NOT
// print the frequent "Nothing-To-Send" keepalive frames to avoid log spam.
static bool LOG_CTS    = false; // set true only when debugging
static bool LOG_TX     = true;
static bool LOG_STATUS = true;
static bool LOG_MQTT   = true;

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
// This sketch follows the RS485 framing used by the Balboa Worldwide App:
// - The spa/controller sends a "Ready" frame (MT = 10 BF 06) when it's safe to
//   send EXACTLY ONE message on the bus.
// - We send commands with MT = 0A BF xx (toggle = 0A BF 11, set_temp = 0A BF 20).
// Reference: https://github.com/ccutrer/balboa_worldwide_app/blob/main/doc/protocol.md
static uint32_t g_lastRxMs = 0;

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
  if ((size_t)ml + 2 != len) return false; // total bytes must match ML

  const uint8_t packetCrc = buf[len - 2];
  const uint8_t calcCrc   = balboaCRC(buf + 1, (uint8_t)(len - 3)); // len + data
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

  // CRC is computed over [ML][MT..][payload..]
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

  // Read & discard echo (RS485 half-duplex loopback)
  delayMicroseconds(500);
  while (Serial2.available()) Serial2.read();

  delayMicroseconds(100);
  rs485ReceiveMode();

  if (LOG_TX && logFrame) {
    Serial.printf("[TX] %uB:", fi);
    for (uint8_t i = 0; i < fi; i++) Serial.printf(" %02X", frame[i]);
    Serial.println();
  }
}

// ------------------------ Ready window (CTS) --------------------------------
// We only transmit after the spa says it's "Ready" (MT = 10 BF 06).
// The handling is implemented in processFrame().

// ------------------------------ Commands ------------------------------------
void sendToggle(uint8_t item) {
  // Toggle item: MT = 0A BF 11, payload = [item, 0x00]
  const uint8_t mt[] = { 0x0A, 0xBF, 0x11 };
  const uint8_t pl[] = { item, 0x00 }; // 0x00 = toggle
  rs485Send(mt, pl, sizeof(pl), true);
}

void sendSetTemp(uint8_t tempRaw) {
  // Set temperature: MT = 0A BF 20, payload = [tempRaw]
  const uint8_t mt[] = { 0x0A, 0xBF, 0x20 };
  const uint8_t pl[] = { tempRaw };
  rs485Send(mt, pl, sizeof(pl), true);
  if (LOG_TX) Serial.printf("[TX] set_temp %.1f C (raw 0x%02X)\n", tempRaw / 2.0f, tempRaw);
}

// buf[0]=7E, buf[1]=ML, buf[2]=MT0, buf[3]=MT1, buf[4]=MT2, buf[5..]=data
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

  const uint8_t mt0 = buf[2];
  const uint8_t mt1 = buf[3];
  const uint8_t mt2 = buf[4];

  // -------------------- Ready / Clear-To-Send --------------------
  // Ready: 10 BF 06  (safe to send ONE message now)
  if (mt0 == 0x10 && mt1 == 0xBF && mt2 == 0x06) {
    PendingCommand cmd;
    portENTER_CRITICAL(&pendingMux);
    cmd              = pendingCmd;
    pendingCmd.ready = false;
    pendingCmd.type  = CMD_NONE;
    portEXIT_CRITICAL(&pendingMux);

    if (cmd.ready) {
      if (LOG_CTS) Serial.printf("[CTS] ready -> sending cmd type=%d\n", cmd.type);
      // No artificial delay here – the ready window can be short.
      if (cmd.type == CMD_TOGGLE)   sendToggle(cmd.toggleByte);
      if (cmd.type == CMD_SET_TEMP) sendSetTemp(cmd.tempRaw);
    } else {
      // Nothing queued; no need to transmit anything.
      if (LOG_CTS) Serial.println("[CTS] ready (no pending cmd)");
    }
    return;
  }

  // -------------------- Status --------------------
  // Status: FF AF 13
  if (mt0 == 0xFF && mt1 == 0xAF && mt2 == 0x13) {
    publishStatus(buf, len);
    return;
  }

  // -------------------- Noise we don't care about --------------------
  if (mt0 == 0xFE && mt1 == 0xBF) return; // misc polls/handshake frames
  if (mt1 == 0xBF && mt2 == 0x07) return; // other clients' "Nothing to Send"

  // Debug "other" frames (limited)
  Serial.printf("[OTHER] %uB mt=%02X %02X %02X:", (unsigned)len, mt0, mt1, mt2);
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
    if (LOG_MQTT) Serial.printf("[CMD] %s toggle 0x%02X\n", busy ? "ignored" : "queued", item);
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
      if (LOG_MQTT) Serial.printf("[CMD] %s set_temp %.1f C\n", busy ? "ignored" : "queued", t);
    } else {
      if (LOG_MQTT) Serial.printf("[CMD] set_temp mimo rozsah: %.1f\n", t);
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
  Serial.println("Waiting for READY frames (10 BF 06) and status updates (FF AF 13)...");
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

  // Optional: if bus goes silent for long, print a warning (and avoid spamming).
  if (g_lastRxMs != 0 && (millis() - g_lastRxMs) > 15000) {
    Serial.println("[WARN] no RS485 traffic for 15s");
    g_lastRxMs = millis();
  }

  static uint8_t buffer[256];
  static size_t  index = 0;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (index < sizeof(buffer)) buffer[index++] = b;

    // Frame boundary is always "... 7E 7E ..." (end + next start)
    if (index >= 2 && buffer[index-2] == 0x7E && buffer[index-1] == 0x7E) {
      processFrame(buffer, index - 1);
      buffer[0] = 0x7E;
      index = 1;
    }

    if (index >= sizeof(buffer)) index = 0;
  }
}
