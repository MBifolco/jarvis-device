# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
This is an ESP32-based voice assistant device called "jarvis-device" that implements:
- Wake word detection using ESP-SR
- Audio recording with VAD (Voice Activity Detection) 
- Bluetooth Low Energy communication for data transmission
- Audio playback capabilities
- Real-time audio processing pipeline

### System Integration
This firmware works in conjunction with a Flutter mobile app located at `../jarvis-app/`. The ESP32 device:
- Acts as a BLE peripheral that the Flutter app connects to
- Streams compressed or uncompressed audio data to the mobile app via BLE characteristics
- Receives configuration commands from the mobile app
- Provides real-time status updates (recording state, battery, etc.)

**Cross-Component Context**: See the parent `../CLAUDE.md` for system-wide integration details and communication protocols.

## Build System
This project uses ESP-IDF (Espressif IoT Development Framework) with CMake:

### Essential Commands
- **Build project**: `idf.py build`
- **Flash to device**: `idf.py flash`
- **Monitor logs**: `idf.py monitor` 
- **Full clean build**: `idf.py fullclean && idf.py build`
- **Set target**: `idf.py set-target esp32s3` (or appropriate ESP32 variant)

### Configuration
- **Configure project**: `idf.py menuconfig`
- **Save configuration**: Configuration is stored in `sdkconfig`

## Architecture

### Core Components Structure
- **main/main.c**: Main application entry point with dual AFE instances (wake vs hold)
- **main/src/**: Core implementation modules
  - `audio_pipeline.c`: Central audio processing and buffering logic
  - `audio_rx.c` & `audio_tx.c`: Audio reception and transmission
  - `audio_tone.c`: Audio tone generation
  - `wakeword_handler.c`: Wake word detection handling
  - `app_ble.c` & `bluetooth.c`: Bluetooth communication
  - `afe_setup.c`: Audio Front End configuration
  - `config.c`: Device configuration management
- **main/include/**: Header files defining interfaces

### Audio Processing Pipeline
The system uses two separate AFE (Audio Front End) instances:
1. **Wake instance**: Optimized for wake word detection (AEC disabled)
2. **Hold instance**: Post-wake recording with AEC enabled for better quality

Key flow:
1. Continuous audio capture via I2S from microphone
2. Feed audio through current AFE instance (wake/hold)
3. Wake word detection triggers switch to hold instance
4. VAD controls recording start/stop during hold phase
5. Audio data compressed (if config calls for it) and transmitted via BLE

### Hardware Configuration
- **Microphone I2S**: Port I2S_NUM_0 (BCK=GPIO8, WS=GPIO10, DATA=GPIO9)  
- **Speaker I2S**: Port I2S_NUM_1 (BCK=GPIO6, WS=GPIO5, DATA=GPIO7)
- **Sample Rate outgoing**: 16kHz
- **Sample Rate incoming**: 24kHz
- **Recording Limit**: 30 seconds max per session

### Key Constants (main/include/config.h)
- `SAMPLE_RATE OUTGOING`: 16000 Hz
- `SAMPLE_RATE INCOMING`: 24000 Hz
- `POST_WAKE_SECONDS`: 30 (max recording length)
- `KEEP_ALIVE_MS`: 20000 (keep-alive timeout)
- `MIN_RECORD_SAMPLES`: 16000 (minimum 1-second recording)

### Task Architecture
- **feed_task**: Reads I2S audio data and feeds to AFE
- **fetch_task**: Processes AFE results for wake word and VAD events
- **Keep-alive timer**: Manages timeout and AFE instance switching

## Dependencies
The project uses ESP-IDF managed components:
- `espressif__esp-sr`: Speech recognition library for wake word and VAD
- `espressif__esp-dsp`: Digital signal processing utilities  
- `espressif__esp_audio_codec`: Audio encoding/decoding

## Development Tips
- Monitor heap usage carefully due to audio buffer allocation
- Both AFE instances must be properly initialized before use
- Audio playback (`g_playing_back` flag) pauses audio processing
- Use `ESP_LOGI` with appropriate tags for debugging different modules
- The dual-AFE architecture prevents "AEC engine destroyed" errors during instance switching