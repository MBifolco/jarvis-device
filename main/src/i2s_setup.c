#include "i2s_setup.h"
#include "driver/gpio.h"  // for your GPIO_NUM_x pins
#include "esp_log.h"
#include "config.h"      // for SAMPLE_RATE, I2S_MIC_PORT, I2S_SPK_PORT, etc.

static const char *TAG = "i2s_setup";


// I2S RX init (unchanged)
void i2s_mic_init(void)
{
    i2s_config_t cfg = {
        .mode                = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate         = SAMPLE_RATE,
        .bits_per_sample     = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format      = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format= I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags    = 0,
        .dma_buf_count       = 4,
        .dma_buf_len         = 512,
        .use_apll            = false,
        .tx_desc_auto_clear  = false,
        .fixed_mclk          = 0
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_MIC_PORT, &cfg, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_MIC_PORT, &(i2s_pin_config_t){
        .bck_io_num   = MIC_BCK_IO,
        .ws_io_num    = MIC_WS_IO,
        .data_in_num  = MIC_DATA_IO,
        .data_out_num = I2S_PIN_NO_CHANGE
    }));
    ESP_LOGI(TAG, "I2S RX inited");
}

// I2S TX init (unchanged)
void i2s_play_init(void)
{
    i2s_config_t cfg = {
        .mode                = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate         = SAMPLE_RATE,
        .bits_per_sample     = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format      = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format= I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags    = 0,
        .dma_buf_count       = 4,
        .dma_buf_len         = 512,
        .use_apll            = false,
        .tx_desc_auto_clear  = true,
        .fixed_mclk          = 0
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_SPK_PORT, &cfg, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_SPK_PORT, &(i2s_pin_config_t){
        .bck_io_num   = SPK_BCK_IO,
        .ws_io_num    = SPK_WS_IO,
        .data_out_num = SPK_DATA_IO,
        .data_in_num  = I2S_PIN_NO_CHANGE
    }));
    ESP_LOGI(TAG, "I2S TX inited");
}
