#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <esp_dmx.h>
#include <ArtnetWiFi.h>

ArtnetWiFiReceiver artnet;

// === Pin Config ===
int txPin = 17;
int rxPin = 16;
int enablePin = 21;
#define LED_PIN 2

// === DMX Config ===
dmx_port_t dmxPort = 1;
dmx_config_t config = DMX_CONFIG_DEFAULT;
dmx_personality_t personalities[] = {};
int personality_count = 0;
byte data[DMX_PACKET_SIZE];
unsigned long lastUpdate = millis();

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

  // Optional concise log
  // Serial.printf("[ArtNetCB] Uni %u Len %u Seq %u | CH1-8:", universe, len, sequence);
  // for (int i = 0; i < 8 && i < len; i++) Serial.printf(" %3u", data[1 + i]);
  // Serial.println();
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

  // Serial.printf("[ArtNetCB] Uni %u Len %u | CH1-8:", meta.universe, len);
  // for (int i = 0; i < 8 && i < len; i++) Serial.printf(" %3u", data[1 + i]);
  // Serial.println();
}

// Adapter: converts ArtNet callback signature to legacy onDmxFrame() signature
void onArtnetFrameAdapter(const uint8_t *data, uint16_t size,
                          const ArtDmxMetadata &meta, const ArtNetRemoteInfo &remote) {
  // The ArtNet callback gives us data, size, metadata (which includes universe & sequence)
  uint16_t universe = meta.universe;
  uint8_t sequence = meta.sequence;
  onDmxFrame(universe, size, sequence, data);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(300);

  Serial.println("\n=== ESP32 DMX Self-Test ===");

  // --- Wi-Fi ---
  WiFiManager wm;
  if (!wm.autoConnect("ESP32-DMX", "dmxpass123")) {
    Serial.println("Wi-Fi failed, restarting...");
    ESP.restart();
  }
  WiFi.setSleep(false);
  Serial.printf("Wi-Fi connected: %s\n", WiFi.localIP().toString().c_str());

  artnet.begin();
  artnet.subscribeArtDmxUniverse(0, onArtnetFrameAdapter);
  Serial.println("✅ Art-Net listener ready on universe 0");

  // --- DMX Driver Install ---
  dmx_driver_install(dmxPort, &config, personalities, personality_count);
  dmx_set_pin(dmxPort, txPin, rxPin, enablePin);
  Serial.println("✅ DMX driver initialised successfully");
}

void loop() {
  artnet.parse();
  
  // unsigned long now = millis();
  // static bool high = false;

  // if (now - lastUpdate >= 1000) {
  //   uint8_t level = high ? 50 : 0;

  //   // Build a temporary 8-channel payload (CH1–8)
  //   uint8_t testPayload[8];
  //   for (int i = 0; i < 4; i++) testPayload[i] = level;

  //   // Use the Art-Net-style handler so the path is identical to real Art-Net
  //   onDmxFrame(/*universe*/0, /*numberOfChannels*/8, /*sequence*/0, testPayload);

  //   Serial.printf("[SelfTest] Called onDmxFrame with CH1–8=%u\n", level);
  //   digitalWrite(LED_PIN, high ? HIGH : LOW);

  //   high = !high;
  //   lastUpdate = now;
  // }
}