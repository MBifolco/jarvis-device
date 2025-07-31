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
- Streams compressed audio data from device to app (ADPCM encoded)
- Receives uncompressed PCM audio from app for playback (24kHz, 16-bit mono)
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
- **Sample Rate outgoing**: 16kHz (recording from microphone)
- **Sample Rate incoming**: 24kHz (playback from app, matching OpenAI TTS)
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

## Audio Reception Architecture (audio_rx.c)

### Overview
The audio reception system implements a robust streaming architecture for receiving PCM audio from the Flutter app via BLE:

1. **Direct BLE to Chunk Pool**: No intermediate buffering - BLE data is written directly to pre-allocated audio chunks
2. **Sync Pattern Protocol**: Uses 0xAA 0x55 sync bytes to reliably detect packet boundaries
3. **State Machine**: Three states handle sync detection, header parsing, and audio reception
4. **48KB Chunks**: Each chunk holds exactly 1 second of 24kHz audio (48,000 bytes)

### Key Implementation Details

#### Sync Pattern Protocol
- Each audio packet starts with sync bytes: `0xAA 0x55`
- Followed by 4-byte little-endian length header
- Then raw PCM audio data (24kHz, 16-bit mono)
- State machine recovers from corruption by searching for next sync pattern

#### Chunk Pool Architecture
- Pre-allocated pool of 12 chunks (48KB each) in PSRAM
- Eliminates memory fragmentation from dynamic allocation
- Chunks are reused via simple allocation/free mechanism
- Playback task runs at high priority (7) for uninterrupted audio

#### BLE Reception Flow
```
BLE Write → State Machine → Header Validation → Direct Copy to Chunk → Queue to Playback
```

### Critical Configuration

#### Watchdog Settings (sdkconfig)
- IDLE task watchdog monitoring is DISABLED to prevent audio interruptions
- `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0` = not set
- `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1` = not set
- Only critical tasks subscribe to watchdog

#### Task Priorities
- `audio_play` task: Priority 7 (highest) - ensures smooth playback
- `feed` task: Priority 5 - audio capture
- `fetch` task: Priority 5 - AFE processing
- NimBLE Host: Priority 5 - BLE communication

## Development Tips
- Monitor heap usage carefully due to audio buffer allocation
- Both AFE instances must be properly initialized before use
- Audio playback (`g_playing_back` flag) pauses audio processing
- Use `ESP_LOGI` with appropriate tags for debugging different modules
- The dual-AFE architecture prevents "AEC engine destroyed" errors during instance switching
- If audio skips occur, check watchdog settings - IDLE tasks should not be monitored
- The sync pattern (0xAA 0x55) is critical for reliable streaming - do not remove