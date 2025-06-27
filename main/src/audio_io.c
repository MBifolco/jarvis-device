#include "audio_io.h"
#include "audio_rx.h"
#include "audio_tx.h"            // for audio_tx_compression_init()
#include "esp_log.h"

static const char *TAG = "audio_io";

void audio_io_init(void)
{
    ESP_LOGI(TAG, "Initializing audio RX");
    audio_rx_init();

    ESP_LOGI(TAG, "Initializing audio TX compression");
    audio_tx_compression_init();
}
