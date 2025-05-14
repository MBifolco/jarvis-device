// audio_tx.h

#ifndef AUDIO_TX_H
#define AUDIO_TX_H

#include "esp_err.h"
#include "esp_audio_enc_default.h"

/** Which codec to use when compressing outgoing audio */
typedef enum {
    AUDIO_CODEC_ADPCM = 0,
    AUDIO_CODEC_OPUS,
} audio_codec_t;

/** Select the codec before init/send */
void audio_tx_set_codec(audio_codec_t codec);

/** Initialize the selected codec (must call after set_codec) */
esp_err_t audio_tx_compression_init(void);

/** Send one big PCM buffer; will compress & chunk by BLE_CHUNK_SIZE */
esp_err_t audio_tx_send(const int16_t *pcm_samples, size_t sample_count);

#endif // AUDIO_TX_H
