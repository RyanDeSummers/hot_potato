# Hot Potato Game - ESP32 IR Multiplayer

A multiplayer "Hot Potato" game for ESP32 devices using IR communication and ESP-NOW for coordination. Players pass an invisible "potato" via IR signals, with the holder having red LEDs and runners trying to claim it.

## 🎮 Game Modes

### Mode A (Downlink)
- **IR Direction**: HOLDER transmits IR beacons → RUNNERS receive IR
- **ESP-NOW**: RUNNERS send `PASS_REQ` → HOLDER grants pass
- **Architecture**: Robust implementation with test harness integration

### Mode B (Uplink) 
- **IR Direction**: RUNNERS transmit IR uplinks → HOLDER receives IR
- **ESP-NOW**: HOLDER broadcasts IR windows → RUNNERS respond with IR
- **Architecture**: Original implementation with proven stability

## 🛠️ Hardware Requirements

- **ESP32 device** (M5Stack FIRE recommended)
- **IR LED** (connected to GPIO 26 for transmission)
- **IR Receiver** (connected to GPIO 36 for reception, demodulated to UART)
- **Side LEDs** (WS2812/SK6812 on GPIO 15 for visual feedback)
- **USB cable** for programming and power

## 📋 Prerequisites

1. **ESP-IDF v5.3.3** installed and configured
2. **Python 3.7+** (comes with ESP-IDF)
3. **Git** for cloning the repository

## 🚀 Quick Start

### 1. Clone the Repository
```bash
git clone <your-repo-url>
cd hot_potato
```

### 2. Install Dependencies

#### Option A: Using Setup Script (Recommended)
```bash
# Windows
setup.bat
```

#### Option B: Manual Setup
```bash
idf.py reconfigure
```

### 3. Build and Flash

#### Mode A (Downlink) - Recommended
```bash
# HOST device (opens join window, coordinates countdown)
idf.py -DCMAKE_CXX_FLAGS="-DZT_IS_HOST=1" build flash

# PEER device (joins existing network)
idf.py -DCMAKE_CXX_FLAGS="-DZT_IS_HOST=0" build flash
```

#### Mode B (Uplink)
```bash
# HOST device
idf.py -DCMAKE_CXX_FLAGS="-DHP_TEST_UPLINK=1 -DZT_IS_HOST=1" build flash

# PEER device
idf.py -DCMAKE_CXX_FLAGS="-DHP_TEST_UPLINK=1 -DZT_IS_HOST=0" build flash
```

### 4. Monitor Output
```bash
idf.py monitor
```

## 🎯 How to Play

1. **Setup**: Connect your ESP32 devices and power them on
2. **Discovery**: HOST device opens a join window, PEER devices connect
3. **Countdown**: All devices show synchronized 3-2-1 countdown
4. **Role Assignment**: One random device becomes HOLDER (red LEDs), others become RUNNERS
5. **Gameplay**: 
   - HOLDER has the "potato" (red LEDs on)
   - RUNNERS try to claim it by pointing their IR receiver at the HOLDER
   - When IR is detected, a pass request is sent via ESP-NOW
   - First valid request wins and roles switch
6. **Cooldown**: 1000ms cooldown prevents immediate passback

## 🔧 Build Configuration

### Mode Selection
- **Mode A (Default)**: `ir_downlink_test_main()` - HOLDER TX IR, RUNNERS RX IR
- **Mode B**: `ir_uplink_test_main()` - RUNNERS TX IR, HOLDER RX IR

### Host/Peer Configuration
- **HOST**: `-DZT_IS_HOST=1` - Opens join window, coordinates countdown
- **PEER**: `-DZT_IS_HOST=0` - Joins existing network

### Complete Build Examples
```bash
# Mode A - HOST
idf.py -DCMAKE_CXX_FLAGS="-DZT_IS_HOST=1" build flash monitor

# Mode A - PEER  
idf.py -DCMAKE_CXX_FLAGS="-DZT_IS_HOST=0" build flash monitor

# Mode B - HOST
idf.py -DCMAKE_CXX_FLAGS="-DHP_TEST_UPLINK=1 -DZT_IS_HOST=1" build flash monitor

# Mode B - PEER
idf.py -DCMAKE_CXX_FLAGS="-DHP_TEST_UPLINK=1 -DZT_IS_HOST=0" build flash monitor
```

## 📁 Project Structure

```
hot_potato/
├── main/
│   ├── main.cpp                 # Entry point (mode selection)
│   ├── ir_downlink_test.cpp     # Mode A implementation
│   ├── ir_uplink_test.cpp       # Mode B implementation
│   ├── zt_test_mode.cpp         # Test harness (discovery/countdown)
│   ├── ir_simple_test.h         # Common constants and macros
│   ├── game_logic.cpp           # Game state management
│   ├── SoundManager.cpp         # Audio feedback
│   └── CMakeLists.txt           # Build configuration
├── components/                  # Custom components (if any)
├── CMakeLists.txt              # Project configuration
├── sdkconfig                   # ESP-IDF configuration
├── setup.bat                   # Setup script (Windows)
└── README.md                   # This file
```

## 🔍 Key Features

- **Test Harness**: Automatic device discovery and synchronized countdown
- **Cooldown System**: Prevents rapid back-and-forth passing
- **Robust IR Parsing**: Ring buffer handles split UART reads
- **ESP-NOW Handshake**: Reliable pass acknowledgment
- **LED Feedback**: Visual indication of current role
- **UI Consistency**: Same countdown and role displays across modes
- **Error Recovery**: Handles network disconnections and reconnections

## 🐛 Troubleshooting

### Common Issues

1. **Build Errors**
   - Ensure ESP-IDF v5.3.3 is installed and sourced
   - Run `idf.py reconfigure` to install dependencies
   - Check that all source files are included in `main/CMakeLists.txt`

2. **No IR Communication**
   - Verify IR LED is connected to GPIO 26
   - Verify IR receiver is connected to GPIO 36
   - Check IR LED polarity and power supply
   - Ensure devices are within line-of-sight

3. **ESP-NOW Issues**
   - Ensure devices are on the same WiFi channel (default: channel 1)
   - Check that MAC addresses are being read correctly
   - Verify broadcast peer is added successfully

4. **LED Issues**
   - Verify WS2812/SK6812 LEDs are connected to GPIO 15
   - Check power supply (5V for most LED strips)
   - Ensure correct number of LEDs in `HP_LED_COUNT`

### Debug Output
Monitor the serial output for detailed logs:
```bash
idf.py monitor
```

Look for:
- `IR beacon sent (SEQ=X)` - IR transmission working
- `BEACON ok: seq=X from XX:XX:XX:XX:XX:XX` - IR reception working
- `GRANT -> XX:XX:XX:XX:XX:XX seq=X` - Pass requests working
- `ROLE -> HOLDER/RUNNER` - Role transitions working

## 📝 Configuration

### IR Settings (`ir_simple_test.h`)
- `BEACON_PERIOD_MS`: IR beacon transmission interval (default: 300ms)
- `PASS_COOLDOWN_MS`: Cooldown between passes (default: 1000ms)
- `IR_BAUD`: UART baud rate for IR reception (default: 2400)
- `ESPNOW_CHANNEL`: WiFi channel for ESP-NOW (default: 1)

### LED Settings
- `HP_LED_GPIO`: GPIO pin for LED strip (default: 15)
- `HP_LED_COUNT`: Number of LEDs in strip (default: 10)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly with multiple devices
5. Submit a pull request

## 📄 License

[Add your license information here]

## 🙏 Acknowledgments

- ESP-IDF team for the excellent development framework
- M5Stack for the FIRE board design
- ESP-NOW and RMT peripheral documentation

---

**Happy gaming! 🥔✨**
