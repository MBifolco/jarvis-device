file(GLOB_RECURSE srcs "main.c" "bluetooth.c" "audio_tone.c" "src/*.c")

idf_component_register(
    SRCS 
        "${srcs}"
    INCLUDE_DIRS 
        "./include"
    REQUIRES
        driver          # I2S & GPIO
        esp_common      # esp_err.h, esp_log.h
        freertos        # FreeRTOS tasks
        bt 
        nvs_flash
)


#set_source_files_properties(
#  src/audio_tx.c PROPERTIES COMPILE_FLAGS "-fstack-usage"
#)