// audio_tx.c

#include "audio_tx.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "os/os_mbuf.h"
#include "gatt_svc.h"
#include "esp_audio_enc_reg.h"
#include "esp_audio_enc.h"
#include "freertos/task.h"
// For ADPCM
#include "esp_adpcm_enc.h"
// For Opus
#include "esp_opus_enc.h"

#include <string.h>
#include <stdlib.h>

#ifndef MIN
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#define TAG              "audio_tx"
#define BLE_CHUNK_SIZE   497
#define WAV_HEADER_SIZE  46

static audio_codec_t   s_codec        = AUDIO_CODEC_ADPCM;
static esp_audio_enc_handle_t enc_handle = NULL;
static int in_frame_size  = 0;
static int out_frame_size = 0;

// Internal: pick fourcc based on codec
static uint16_t get_wav_audio_format(void) {
    return (s_codec == AUDIO_CODEC_ADPCM) ? 0x11 /*IMA ADPCM*/ : 0xFF /*custom*/;
}

/** Public: switch codec */
void audio_tx_set_codec(audio_codec_t codec) {
    s_codec = codec;
}

/** Initialize the selected encoder */
esp_err_t audio_tx_compression_init(void)
{
    esp_err_t err;
    esp_audio_enc_config_t cfg = {0};

    if (s_codec == AUDIO_CODEC_ADPCM) {
        err = esp_adpcm_enc_register();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ADPCM register failed: %s", esp_err_to_name(err));
            return err;
        }
        esp_adpcm_enc_config_t adpcm_cfg = ESP_ADPCM_ENC_CONFIG_DEFAULT();
        adpcm_cfg.sample_rate = 16000;
        adpcm_cfg.channel     = 1;
        adpcm_cfg.bits_per_sample = 16;
        cfg.type   = ESP_AUDIO_TYPE_ADPCM;
        cfg.cfg    = &adpcm_cfg;
        cfg.cfg_sz = sizeof(adpcm_cfg);

    } else { // AUDIO_CODEC_OPUS
        err = esp_opus_enc_register();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Opus register failed: %s", esp_err_to_name(err));
            return err;
        }
        esp_opus_enc_config_t opus_cfg = ESP_OPUS_ENC_CONFIG_DEFAULT();
        opus_cfg.sample_rate = 16000;
        opus_cfg.channel     = 1;
        opus_cfg.bitrate     = 16000; // tune as you like
        cfg.type   = ESP_AUDIO_TYPE_OPUS;
        cfg.cfg    = &opus_cfg;
        cfg.cfg_sz = sizeof(opus_cfg);
    }

    err = esp_audio_enc_open(&cfg, &enc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Open encoder failed: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_audio_enc_get_frame_size(enc_handle, &in_frame_size, &out_frame_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Get frame size failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "%s encoder in=%d out=%d",
             s_codec==AUDIO_CODEC_ADPCM?"ADPCM":"Opus",
             in_frame_size, out_frame_size);
    return ESP_OK;
}

// Build a tiny WAV header only for ADPCM
static void build_adpcm_wav_header(uint8_t *h, size_t payload_bytes)
{
    uint32_t sample_rate = 16000;
    uint16_t channels    = 1;
    uint16_t bits_per_sample = 4;   // ADPCM nibble size
    uint16_t block_align     = 256; // ADPCM block align
    uint16_t extra_size      = 2;
    uint32_t byte_rate       = (sample_rate * block_align) / 505;
    uint32_t fmt_chunk_size  = 20;
    uint32_t wav_size        = 36 + payload_bytes;

    memcpy(h, "RIFF", 4);
    memcpy(h+4, &wav_size, 4);
    memcpy(h+8, "WAVEfmt ", 8);
    memcpy(h+16, &fmt_chunk_size, 4);
    uint16_t fmt = get_wav_audio_format();
    memcpy(h+20, &fmt, 2);
    memcpy(h+22, &channels, 2);
    memcpy(h+24, &sample_rate, 4);
    memcpy(h+28, &byte_rate, 4);
    memcpy(h+32, &block_align, 2);
    memcpy(h+34, &bits_per_sample, 2);
    memcpy(h+36, &extra_size, 2);
    memcpy(h+38, "data", 4);
    memcpy(h+42, &payload_bytes, 4);
}

// based on your current audio_tx.c :contentReference[oaicite:2]{index=2}
// lib/src/audio_tx.c
// audio_tx.c

esp_err_t audio_tx_send(const int16_t *pcm_samples, size_t sample_count)
{
    if (!chr_conn_handle || !audio_notify_handle) {
        ESP_LOGW(TAG, "No BLE conn or char");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "audio_tx_send: %u samples", (unsigned)sample_count);
    // Compute sizes
    size_t pcm_bytes   = sample_count * sizeof(int16_t);
    size_t frame_bytes = (size_t)in_frame_size;  // in bytes
    size_t out_cap     = (s_codec==AUDIO_CODEC_OPUS)
                        ? (size_t)out_frame_size * 2
                        : (size_t)out_frame_size;

    // Allocate temp buffers
    uint8_t *inbuf  = malloc(frame_bytes);
    uint8_t *outbuf = malloc(out_cap);
    if (!inbuf || !outbuf) {
        ESP_LOGE(TAG, "OOM encoding buffers");
        goto cleanup;
    }

    // PASS #1: measure total size (including 2-byte prefixes)
    size_t total_payload = 0;
    const uint8_t *p = (const uint8_t*)pcm_samples;
    size_t rem = pcm_bytes;
    while (rem >= frame_bytes) {
        memcpy(inbuf, p, frame_bytes);
        p   += frame_bytes;
        rem -= frame_bytes;
        esp_audio_enc_in_frame_t  inf  = { .buffer=inbuf,  .len=frame_bytes };
        esp_audio_enc_out_frame_t outf = { .buffer=outbuf, .len=out_cap     };
        if (esp_audio_enc_process(enc_handle, &inf, &outf) != ESP_OK) break;
        total_payload += outf.len + 2;  // +2 for our length prefix
    }

    ESP_LOGI(TAG, "Measured total_payload = %u bytes", total_payload);

    // Send WAV header with total_payload
    {
        uint8_t hdr[WAV_HEADER_SIZE] = {0};
        // Reuse your existing builder; get_wav_audio_format() yields 0x11 or 0xFF
        build_adpcm_wav_header(hdr, total_payload);
        struct os_mbuf *omh = ble_hs_mbuf_from_flat(hdr, WAV_HEADER_SIZE);
        ble_gatts_notify_custom(chr_conn_handle, audio_notify_handle, omh);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // PASS #2: actually encode + stream each frame with 2-byte length
    p   = (const uint8_t*)pcm_samples;
    rem = pcm_bytes;
    while (rem >= frame_bytes) {
        memcpy(inbuf, p, frame_bytes);
        p   += frame_bytes;
        rem -= frame_bytes;

        esp_audio_enc_in_frame_t  inf  = { .buffer=inbuf,  .len=frame_bytes };
        esp_audio_enc_out_frame_t outf = { .buffer=outbuf, .len=out_cap     };
        if (esp_audio_enc_process(enc_handle, &inf, &outf) != ESP_OK) {
            ESP_LOGE(TAG, "encode error");
            break;
        }

        // Build a 2-byte little-endian length prefix + payload
        uint16_t len16 = outf.len;
        uint8_t tmp[2];
        tmp[0] = len16 & 0xFF;
        tmp[1] = (len16 >> 8) & 0xFF;

        // Send prefix
        {
          struct os_mbuf *om = ble_hs_mbuf_from_flat(tmp, 2);
          ble_gatts_notify_custom(chr_conn_handle, audio_notify_handle, om);
          vTaskDelay(pdMS_TO_TICKS(5));
        }
        // Send frame data (in one go, since outf.len < BLE_CHUNK_SIZE)
        {
          struct os_mbuf *om = ble_hs_mbuf_from_flat(outbuf, outf.len);
          ble_gatts_notify_custom(chr_conn_handle, audio_notify_handle, om);
          vTaskDelay(pdMS_TO_TICKS( (s_codec==AUDIO_CODEC_ADPCM)?60:20 ));
        }
    }

cleanup:
    free(inbuf);
    free(outbuf);
    return ESP_OK;
}
