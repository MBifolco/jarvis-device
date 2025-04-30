idf_component_register(
    SRCS
        "wake_word_engine.c" "bluetooth.c"
    INCLUDE_DIRS
        "."      # local headers
    REQUIRES
        driver          # I2S & GPIO
        esp_common      # esp_err.h, esp_log.h
        freertos        # FreeRTOS tasks
        bt 
        nvs_flash
)
