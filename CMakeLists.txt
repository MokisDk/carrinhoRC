idf_component_register(
    SRCS "main.c" 

    #para adicionar os drives necessarios
    PRIV_REQUIRES 
        freertos    
        driver       
        esp_timer    
        esp_system   
        log          
)

