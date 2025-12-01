idf_component_register(
    SRCS "main.c" 

    # Adicionando os drivers que seu código C utiliza:
    PRIV_REQUIRES 
        freertos     # Para Tasks e vTaskDelay
        driver       # <--- SOLUÇÃO para "driver/gpio.h" e "driver/ledc.h"
        esp_timer    # <--- SOLUÇÃO para "esp_timer.h"
        esp_system   
        log          # Para ESP_LOGI
)
