/*
TODO: 
  - Handle WiFi disconnects/new IP address
  - Add support for restart button
  - Add support for reset button to clear WiFi config
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <esp_dmx.h>
#include <ArtnetWiFi.h>
#include <Wire.h>
#include <U8g2lib.h>

#define DEBUG 1
#if DEBUG
  #define DBG_PRINT(x) Serial.print(x)
  #define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG_PRINT(x)
  #define DBG_PRINTF(...)
#endif

String apSSID;
const char* apPass = "dmxpass123";

ArtnetWiFiReceiver artnet;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
unsigned long lastDisplayUpdate = 0;
String ipDisplayText;
String wifiStatusText;

// === Pin Config ===
int txPin = 17;
int rxPin = 16;
int enablePin = 21;
#define LED_PIN 2
#define BTN_RESTART 26
#define BTN_RESET_WIFI 27

// === DMX Config ===
dmx_port_t dmxPort = 1;
dmx_config_t config = DMX_CONFIG_DEFAULT;
dmx_personality_t personalities[] = {};
int personality_count = 0;
byte data[DMX_PACKET_SIZE];
unsigned long lastUpdate = millis();
unsigned long lastLedBlink = 0;
bool ledOn = false;
const unsigned long LED_BLINK_DURATION = 100; // ms
const unsigned long LED_MIN_INTERVAL = 200; // ms between flashes

unsigned long frameCount = 0;
unsigned long lastFpsCalc = 0;
int fps = 0;
uint16_t currentUniverse = 0;

// === Art-Net style DMX frame handler (can be called by self-test or Art-Net) ===
// Signature chosen to match common Art-Net libraries (universe, length, sequence, data)
void onDmxFrame(uint16_t universe, uint16_t numberOfChannels, uint8_t sequence, const uint8_t* dmxData) {
  // Clamp to DMX payload (start code is data[0], channels start at 1)
  uint16_t len = numberOfChannels;
  if (len > 512) len = 512;

  // Ensure start code is zero for lighting data
  data[0] = 0;

  // Copy channel data into DMX buffer starting at slot 1
  for (uint16_t i = 0; i < len; i++) {
    data[1 + i] = dmxData[i];
  }

  // Transmit the packet
  dmx_write(dmxPort, data, 1 + len);     // write start code + channels present
  dmx_send_num(dmxPort, 1 + len);
  dmx_wait_sent(dmxPort, DMX_TIMEOUT_TICK);

  unsigned long now = millis();
  if (now - lastLedBlink > LED_MIN_INTERVAL) {
    digitalWrite(LED_PIN, HIGH);
    ledOn = true;
    lastLedBlink = now;
  }

  frameCount++;
  currentUniverse = universe;

  // Optional concise log
  // DBG_PRINTF("[ArtNetCB] Uni %u Len %u Seq %u | CH1-8:", universe, len, sequence);
  // for (int i = 0; i < 8 && i < len; i++) DBG_PRINTF(" %3u", data[1 + i]);
  // DBG_PRINT("\n");
}

void onDmxFrame(const uint8_t *frameData, uint16_t size, const ArtDmxMetadata &meta, const ArtNetRemoteInfo &remote) {
  uint16_t len = size;
  if (len > 512) len = 512;

  data[0] = 0; // Start code zero for lighting data

  for (uint16_t i = 0; i < len; i++) {
    data[1 + i] = frameData[i];
  }

  dmx_write(dmxPort, data, 1 + len);
  dmx_send_num(dmxPort, 1 + len);
  dmx_wait_sent(dmxPort, DMX_TIMEOUT_TICK);

  unsigned long now = millis();
  if (now - lastLedBlink > LED_MIN_INTERVAL) {
    digitalWrite(LED_PIN, HIGH);
    ledOn = true;
    lastLedBlink = now;
  }

  // DBG_PRINTF("[ArtNetCB] Uni %u Len %u | CH1-8:", meta.universe, len);
  // for (int i = 0; i < 8 && i < len; i++) DBG_PRINTF(" %3u", data[1 + i]);
  // DBG_PRINT("\n");
}

// Adapter: converts ArtNet callback signature to legacy onDmxFrame() signature
void onArtnetFrameAdapter(const uint8_t *data, uint16_t size,
                          const ArtDmxMetadata &meta, const ArtNetRemoteInfo &remote) {
  // The ArtNet callback gives us data, size, metadata (which includes universe & sequence)
  uint16_t universe = meta.universe;
  uint8_t sequence = meta.sequence;
  onDmxFrame(universe, size, sequence, data);
}

void handleWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      ipDisplayText = WiFi.localIP().toString();
      wifiStatusText = "Connected";
      DBG_PRINTF("WiFi GOT IP: %s\n", ipDisplayText.c_str());
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      wifiStatusText = "Disconnected";
      ipDisplayText = "";
      DBG_PRINT("WiFi disconnected\n");
      break;
    case SYSTEM_EVENT_AP_START:
      wifiStatusText = "Running AP...";
      ipDisplayText = WiFi.softAPIP().toString();
      DBG_PRINTF("AP IP: %s\n", ipDisplayText.c_str());
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(300);

  pinMode(BTN_RESTART, INPUT_PULLUP);
  pinMode(BTN_RESET_WIFI, INPUT_PULLUP);

  DBG_PRINT("\n=== ESP32 DMX Self-Test ===");

  // === OLED Display Init ===
  u8g2.begin();
  u8g2.setFont(u8g2_font_8x13_tr);
  u8g2.clearBuffer();
  u8g2.drawStr(0, 14, "Checking for");
  u8g2.drawStr(0, 36, "stored WiFi...");
  u8g2.sendBuffer();

  // --- Wi-Fi ---
  String macID = WiFi.macAddress();
  macID.replace(":", "");
  apSSID = "DMX" + macID.substring(6, 12); // e.g. DMX3DC770

  WiFi.setHostname(apSSID.c_str());

  WiFiManager wm;
  wm.setAPCallback([](WiFiManager *mgr) {
    String apSSID = mgr->getConfigPortalSSID();
    String apPass = "dmxpass123";
    DBG_PRINT("Config portal started!\n");
    DBG_PRINTF("SSID: %s\n", apSSID.c_str());
    DBG_PRINTF("Password: %s\n", apPass.c_str());
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_8x13_tr);
    u8g2.drawStr(0, 14, "Configure WiFi");
    u8g2.drawStr(0, 36, ("SSID:" + apSSID).c_str());
    u8g2.drawStr(0, 58, ("PW:" + apPass).c_str());
    u8g2.sendBuffer();
  });

  if (!wm.autoConnect(apSSID.c_str(), apPass)) {
    DBG_PRINT("Wi-Fi connection failed or timed out.\n");
    wifiStatusText = "Running AP...";
  }

  // Detect if no saved Wi-Fi credentials and display Configure WiFi message
  if (!wm.getWiFiIsSaved()) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_8x13_tr);
    u8g2.drawStr(0, 14, "Configure WiFi");
    u8g2.drawStr(0, 36, ("SSID: " + apSSID).c_str());
    u8g2.drawStr(0, 58, ("PW: " + String(apPass)).c_str());
    u8g2.sendBuffer();
  }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_8x13_tr);
  u8g2.drawStr(0, 14, "Connecting...");
  u8g2.drawStr(0, 36, ("SSID: " + WiFi.SSID()).c_str());
  u8g2.sendBuffer();
  delay(1000);

  WiFi.setSleep(false);

  // register event handler to track IP/status changes
  WiFi.onEvent(handleWiFiEvent);

  if (WiFi.status() == WL_CONNECTED) {
    ipDisplayText = WiFi.localIP().toString();
    wifiStatusText = "Connected";
    DBG_PRINTF("Wi-Fi connected: %s\n", WiFi.localIP().toString().c_str());
  } else {
    DBG_PRINT("Running in AP mode, waiting for configuration...\n");
    wifiStatusText = "Running AP...";
    // Optionally set ipDisplayText to the AP IP
    ipDisplayText = WiFi.softAPIP().toString();
  } 

  // Now Wi-Fi is connected and IP is assigned, start Art-Net
  artnet.begin();
  artnet.subscribeArtDmxUniverse(0, onArtnetFrameAdapter);
  DBG_PRINT("✅ Art-Net listener ready on universe 0\n");

  // --- DMX Driver Install ---
  dmx_driver_install(dmxPort, &config, personalities, personality_count);
  dmx_set_pin(dmxPort, txPin, rxPin, enablePin);
  DBG_PRINT("✅ DMX driver initialised successfully\n");
}

unsigned long btnRestartPressedAt = 0;
unsigned long btnResetPressedAt = 0;
const unsigned long LONG_PRESS_MS = 2000;
const unsigned long DEBOUNCE_MS = 50;

void loop() {
  unsigned long now = millis();

  // non-blocking restart button handling
  bool restartState = (digitalRead(BTN_RESTART) == LOW);
  if (restartState && btnRestartPressedAt == 0) {
    btnRestartPressedAt = now;
  } else if (!restartState && btnRestartPressedAt != 0) {
    unsigned long held = now - btnRestartPressedAt;
    btnRestartPressedAt = 0;
    if (held >= DEBOUNCE_MS && held < LONG_PRESS_MS) {
      DBG_PRINT("Restart (short) pressed\n");
      u8g2.clearBuffer(); u8g2.drawStr(0,14,"Restarting..."); u8g2.sendBuffer();
      delay(200); // short feedback
      ESP.restart();
    }
  }

  // non-blocking reset-wifi long-press handling
  bool resetState = (digitalRead(BTN_RESET_WIFI) == LOW);
  if (resetState && btnResetPressedAt == 0) {
    btnResetPressedAt = now;
  } else if (!resetState && btnResetPressedAt != 0) {
    unsigned long held = now - btnResetPressedAt;
    btnResetPressedAt = 0;
    if (held >= LONG_PRESS_MS) {
      DBG_PRINT("Reset Wi-Fi (long press) detected\n");
      u8g2.clearBuffer(); u8g2.drawStr(0,14,"Resetting WiFi..."); u8g2.sendBuffer();
      delay(200);
      WiFiManager wm;
      wm.resetSettings();
      delay(200);
      ESP.restart();
    }
  }
  
  artnet.parse();

  if (ledOn && millis() - lastLedBlink > LED_BLINK_DURATION) {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  }
  
  if (millis() - lastFpsCalc > 1000) {
      fps = frameCount;
      frameCount = 0;
      lastFpsCalc = millis();
  }
  
  // === OLED Display Update ===
  if (millis() - lastDisplayUpdate > 1000) {
    lastDisplayUpdate = millis();
    u8g2.clearBuffer();

    if (wifiStatusText == "Connected") {
      char buf[48];
      snprintf(buf, sizeof(buf), "%s", ipDisplayText.c_str());
      u8g2.drawStr(0, 14, buf);
      snprintf(buf, sizeof(buf), "Uni:%u  FPS:%d", currentUniverse, fps);
      u8g2.drawStr(0, 36, buf);
    } else if (WiFi.getMode() & WIFI_AP) {
      char buf[48];
      u8g2.drawStr(0, 14, "Running AP...");
      snprintf(buf, sizeof(buf), "SSID:%s", apSSID.c_str());
      u8g2.drawStr(0, 36, buf);
      snprintf(buf, sizeof(buf), "Pass:%s", apPass);
      u8g2.drawStr(0, 58, buf);
    } else {
      u8g2.drawStr(0, 14, "Connecting...");
    }

    u8g2.sendBuffer();
  }

}