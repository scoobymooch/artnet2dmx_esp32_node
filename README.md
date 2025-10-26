# artnet2dmx_esp32_node

**artnet2dmx_esp32_node** is a lightweight ESP32 firmware that receives **Art-Net** (DMX over IP) data via Wi-Fi and outputs standard DMX512 through a UART pin.  
It’s designed for simple, low-latency lighting control and works seamlessly with tools like **Qlab**, **Pro DMX**, or your own Art-Net sender.

---

## ✨ Features

- Receives **Art-Net DMX** (unicast or broadcast)
- Compatible with most Art-Net controllers and visualisers
- Configurable **DMX TX pin**, **universe**, and **baud rate**
- **Wi-Fi Manager** for easy network setup
- Runs on **ESP32 DevKit**, **WROOM**, and similar boards
- Ideal companion for the [ArtNet Showrunner](https://github.com/yourusername/artnet_showrunner) project

---

## ⚙️ Hardware Requirements

| Component | Notes |
|------------|-------|
| **ESP32 board** | Any standard DevKit or WROOM variant |
| **RS-485 transceiver** | e.g. MAX485 or SN75176 |
| **DMX output** | Pin connected to transceiver DI (TX) |
| **Optional LED** | Status indicator |

**Example wiring:**
| Signal | ESP32 Pin | RS-485 Pin |
|--------|------------|------------|
| TX (DMX) | GPIO17 | DI |
| GND | GND | GND |
| 5V | VIN | VCC |

---

## 🔧 Configuration

Edit values in `main.cpp`:

```cpp
#define DMX_TX_PIN   17
#define DMX_BAUD     250000
#define UNIVERSE     0
```

Wi-Fi credentials are handled automatically via WiFiManager.

---

## 🚀 Building and Uploading

### 1. Install PlatformIO
From VS Code or CLI:
```bash
pip install platformio
```

### 2. Build and upload
Connect the ESP32 by USB, then:
```bash
pio run --target upload
```

### 3. Monitor serial output
```bash
pio device monitor
```

---

## 🧪 Testing

Use an Art-Net sender (e.g. **Qlab**, **ArtNetominator**, or **ArtNet Showrunner**)  
Send DMX frames to the ESP32’s IP on **port 6454**.  
Connected DMX fixtures should respond instantly.

---

## 🧰 Folder Structure

```
artnet2dmx_esp32_node/
├── src/
│   └── main.cpp
├── include/
├── lib/
├── test/
├── platformio.ini
└── README.md
```

---

## 📜 License
MIT License © 2025 Matt Barr
