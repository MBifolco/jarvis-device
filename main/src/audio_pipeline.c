#include "audio_pipeline.h"
#include "keepalive.h"
#include "app_ble.h"
#include "esp_log.h"
#include "config.h"

static const char *TAG = "audio_pipeline";

static volatile bool  s_recording   = false;
static volatile bool  s_stop_record = false;
static volatile bool  s_seen_speech = false;
static size_t         s_buf_capacity= 0;
static size_t         s_buf_filled  = 0;
static int16_t       *s_mono_buf    = NULL;

void audio_pipeline_init(size_t buf_capacity) {
    s_buf_capacity = buf_capacity;
    s_buf_filled   = 0;
    s_recording    = false;
    s_stop_record  = false;
    s_seen_speech  = false;
    s_mono_buf     = NULL;
}

bool   audio_pipeline_is_recording(void)               { return s_recording; }
void   audio_pipeline_set_recording(bool state)        { s_recording = state; }

bool   audio_pipeline_get_stop_record(void)            { return s_stop_record; }
void   audio_pipeline_set_stop_record(bool state)      { s_stop_record = state; }

bool   audio_pipeline_get_seen_speech(void)            { return s_seen_speech; }
void   audio_pipeline_set_seen_speech(bool state)      { s_seen_speech = state; }

size_t audio_pipeline_get_buf_capacity(void)           { return s_buf_capacity; }
void   audio_pipeline_set_buf_capacity(size_t cap)     { s_buf_capacity = cap; }

size_t audio_pipeline_get_buf_filled(void)             { return s_buf_filled; }
void   audio_pipeline_set_buf_filled(size_t filled)    { s_buf_filled = filled; }

int16_t *audio_pipeline_get_mono_buf(void)             { return s_mono_buf; }

int16_t *audio_pipeline_alloc_mono_buf(size_t capacity)
{
    if (s_mono_buf) {
        heap_caps_free(s_mono_buf);
    }
    s_mono_buf = heap_caps_malloc(
        sizeof(int16_t) * capacity,
        MALLOC_CAP_SPIRAM
    );
    assert(s_mono_buf);
    return s_mono_buf;
}

void    audio_pipeline_free_mono_buf(void){
    if (s_mono_buf) {
        free(s_mono_buf);
        s_mono_buf = NULL;
    }
}

void audio_pipeline_handle_vad(afe_fetch_result_t *res, TimerHandle_t keepAliveTimer)
{
    // Only when already recording or in keep-alive, and not a new wake event
    if ((audio_pipeline_is_recording() || keepalive_is_enabled())
        && res->wakeup_state != WAKENET_DETECTED)
    {
        if (res->vad_state) {
            audio_pipeline_set_seen_speech(true);
            if (keepalive_is_enabled() && !audio_pipeline_is_recording()) {
                xTimerStop(keepAliveTimer, 0);
                keepalive_disable();
                audio_pipeline_alloc_mono_buf(audio_pipeline_get_buf_capacity());
                audio_pipeline_set_buf_filled(0);
                audio_pipeline_set_stop_record(false);
                audio_pipeline_set_recording(true);
                audio_pipeline_set_seen_speech(true);
                ESP_LOGI(TAG, "Re-armed recording during keep-alive");
            }
        } else {
            if (audio_pipeline_is_recording()
                && audio_pipeline_get_seen_speech()
                && audio_pipeline_get_buf_filled() > MIN_RECORD_SAMPLES)
            {
                ESP_LOGI(TAG, "VAD silence → stopping (samples=%u)",
                            (unsigned)audio_pipeline_get_buf_filled());
                audio_pipeline_set_stop_record(true);
            }
        }
    }
    
}

void audio_pipeline_process_chunk(esp_afe_sr_iface_t *iface,
                                  esp_afe_sr_data_t  *data,
                                  int16_t           *i2s_buf,
                                  size_t             read_bytes,
                                  int                channels,
                                  TimerHandle_t      keepAliveTimer)
{
    // 1) Feed AFE/VAD
    iface->feed(data, i2s_buf);

    // 2) Capture PCM when recording or in keep-alive
    if ((audio_pipeline_is_recording() || keepalive_is_enabled())
        && !audio_pipeline_get_stop_record()
        && audio_pipeline_get_mono_buf())
    {
        size_t frames = read_bytes / (channels * sizeof(int16_t));
        int16_t *buf  = audio_pipeline_get_mono_buf();
        for (size_t i = 0;
             i < frames && audio_pipeline_get_buf_filled() < audio_pipeline_get_buf_capacity();
             i++)
        {
            size_t idx = audio_pipeline_get_buf_filled();
            buf[idx]   = i2s_buf[i * channels];
            audio_pipeline_set_buf_filled(idx + 1);
        }
    }

    // 3) If done (VAD stop or buffer full) → send & reset
    if (audio_pipeline_is_recording()
        && (audio_pipeline_get_stop_record() ||
            audio_pipeline_get_buf_filled() >= audio_pipeline_get_buf_capacity()))
    {
        ESP_LOGI(TAG, "Sending %u samples (%.2f s)…",
                 (unsigned)audio_pipeline_get_buf_filled(),
                 audio_pipeline_get_buf_filled() / (float)SAMPLE_RATE);

        app_ble_send_audio(
            (const uint8_t *)audio_pipeline_get_mono_buf(),
            audio_pipeline_get_buf_filled() * sizeof(int16_t)
        );

        audio_pipeline_free_mono_buf();
        audio_pipeline_set_recording(false);
        audio_pipeline_set_seen_speech(false);
        audio_pipeline_set_buf_filled(0);

        // start keep-alive
        xTimerReset(keepAliveTimer, 0);
        keepalive_enable();
    }
}