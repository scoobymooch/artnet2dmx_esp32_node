# ESP32 ArtNet2DMX Node

**ESP32 ArtNet2DMX Node** is a robust, Wi-Fi–enabled Art‑Net to DMX512 bridge designed for low-latency lighting control using the ESP32 platform.  
It receives **Art‑Net DMX** data over Wi‑Fi and outputs standard DMX512 via a UART-connected RS‑485 transceiver.

---

## ✨ Features

- Receives **Art‑Net DMX** (unicast or broadcast)
- Compatible with **Qlab**, **Pro DMX**, **xLights**, and similar controllers
- Configurable **DMX TX pin**, **universe**, and **baud rate**
- Integrated **WiFiManager** for easy wireless setup and reconfiguration
- OLED display for live status (IP, FPS, Wi‑Fi state, and button feedback)
- **Restart** and **Wi‑Fi Reset** buttons with on‑screen feedback
- Compact 8‑character unique SSID (e.g. `DMX3DC770`)
- Built‑in **DEBUG** flag to toggle serial logging
- Runs on most **ESP32 DevKit**, **WROOM**, or compatible boards

---

## ⚙️ Hardware Requirements

| Component | Notes |
|------------|-------|
| **ESP32 board** | Standard DevKit or WROOM module |
| **RS‑485 transceiver** | MAX485, SN75176, or equivalent |
| **OLED display** | 128×64 I²C display (e.g. SSD1306) |
| **Restart button** | GPIO26 → GND |
| **Wi‑Fi Reset button** | GPIO27 → GND |
| **Optional LED** | Status indicator (GPIO2 default) |

### Example Wiring

| Signal | ESP32 Pin | RS‑485 Pin |
|--------|------------|------------|
| TX (DMX) | GPIO17 | DI |
| GND | GND | GND |
| 5V | VIN | VCC |

Buttons connect between their GPIO pin and GND.  
The transceiver’s RO/RE pins may be tied high if not used for receive.

---

## 🧰 Folder Structure

```
artnet2dmx_esp32_node/
├── src/
│   └── main.cpp
├── include/
├── lib/
├── platformio.ini
└── README.md
```

---

## 🔧 Configuration

Edit relevant values in `main.cpp` if desired:

```cpp
#define DMX_TX_PIN 17
#define DMX_BAUD   250000
#define UNIVERSE   0
```

Wi‑Fi is automatically configured through **WiFiManager**.  
If no saved network exists, the ESP32 starts its own AP mode and displays:

```
Configure WiFi
SSID: DMXxxxxxx
PW: dmxpass123
```

After connecting and entering credentials, it will reboot and join your network.

---

## 🖲️ Button Behaviour

| Button | GPIO | Function |
|--------|------|-----------|
| **Restart** | 26 | Reboots the device; OLED shows “Restarting...” |
| **Wi‑Fi Reset** | 27 | Clears saved Wi‑Fi credentials, shows “Resetting...”, and restarts into AP mode |

---

## 🪛 Building & Uploading

### 1. Install PlatformIO
```bash
pip install platformio
```

### 2. Build and Upload
Connect your ESP32 by USB:
```bash
pio run --target upload
```

### 3. Monitor Serial Output
```bash
pio device monitor
```

---

## 🧪 Testing

Use an Art‑Net sender such as:
- **Qlab**
- **ArtNetominator**
- **xLights**
- **ArtNet Showrunner**

Send DMX frames to the ESP32’s IP on **port 6454**.  
Connected DMX fixtures should respond immediately.

---

## ⚙️ Debugging

A compile‑time flag controls serial logging:

```cpp
#define DEBUG 1
```

Set to `0` for production to disable all `Serial` output.  
When enabled, serial prints are wrapped with `DBG_PRINT()` and `DBG_PRINTF()` macros.

---

## 🧭 Troubleshooting

| Issue | Possible Cause | Solution |
|--------|----------------|-----------|
| Device shows "Configure WiFi" repeatedly | Invalid credentials or poor Wi‑Fi signal | Reconnect and re‑enter credentials |
| DMX not outputting | Wrong TX pin or wiring | Check pin mapping and RS‑485 orientation |
| Flickering output | Wi‑Fi interference or grounding issue | Shorter cable, better PSU grounding |
| OLED frozen on "Connecting..." | Network unreachable | Reset Wi‑Fi to reconfigure |

---

## 📜 License

MIT License © 2025 Matt Barr
