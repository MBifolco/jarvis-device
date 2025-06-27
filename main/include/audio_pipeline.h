#ifndef AUDIO_PIPELINE_H
#define AUDIO_PIPELINE_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_afe_sr_iface.h"


// Process VAD/fetch events (post-wake logic) and manage keep-alive
void audio_pipeline_handle_vad(afe_fetch_result_t *res, TimerHandle_t keepAliveTimer);

// Initialize the audio state (must be called once in app_main)
void audio_pipeline_init(size_t buf_capacity);

// Recording flag
bool   audio_pipeline_is_recording(void);
void   audio_pipeline_set_recording(bool state);

// Stop‐record flag (from VAD)
bool   audio_pipeline_get_stop_record(void);
void   audio_pipeline_set_stop_record(bool state);

// Seen‐speech flag (to gate end‐of‐speech)
bool   audio_pipeline_get_seen_speech(void);
void   audio_pipeline_set_seen_speech(bool state);

// PCM buffer management
size_t  audio_pipeline_get_buf_capacity(void);
void    audio_pipeline_set_buf_capacity(size_t cap);
size_t  audio_pipeline_get_buf_filled(void);
void    audio_pipeline_set_buf_filled(size_t filled);
int16_t *audio_pipeline_get_mono_buf(void);
int16_t *audio_pipeline_alloc_mono_buf(size_t capacity);
void    audio_pipeline_free_mono_buf(void);

// Feed one I2S chunk into AFE, buffer PCM if armed/keep-alive, and send when ready.
void audio_pipeline_process_chunk(esp_afe_sr_iface_t *iface,
                                  esp_afe_sr_data_t  *data,
                                  int16_t           *i2s_buf,
                                  size_t             read_bytes,
                                  int                channels,
                                  TimerHandle_t      keepAliveTimer);

#endif // AUDIO_PIPELINE_H
