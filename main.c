#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rom/ets_sys.h" // Para as funções de desabilitar/reabilitar interrupções
#include "driver/gpio.h" // <--- ESSENCIAL PARA GPIO_NUM_XX e GPIO_MODE_XX
#include "driver/ledc.h" // <--- ESSENCIAL PARA LEDC_XXX
#include "esp_timer.h"
#include "sdkconfig.h"
// --- TAG de Log ---
static const char *TAG = "CARRO_RC";

// Pinos de Entrada (Receptor RC)
#define RC_THROTTLE_PIN     GPIO_NUM_14 
#define RC_STEERING_PIN     GPIO_NUM_12 

// Pinos de Saída (Motor DC e Servo)
#define MOTOR_IN1_PIN       GPIO_NUM_25 
#define MOTOR_IN2_PIN       GPIO_NUM_26 
#define MOTOR_ENABLE_PIN    GPIO_NUM_27 
#define SERVO_PIN           GPIO_NUM_13

// Configuração LEDC/PWM
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_TIMER_MOTOR    LEDC_TIMER_0
#define LEDC_CHANNEL_MOTOR  LEDC_CHANNEL_0
#define LEDC_FREQUENCY      5000 
#define LEDC_RESOLUTION     LEDC_TIMER_8_BIT 
#define LEDC_TIMER_SERVO    LEDC_TIMER_1
#define LEDC_CHANNEL_SERVO  LEDC_CHANNEL_1
#define LEDC_FREQUENCY_SERVO 50 
#define LEDC_RESOLUTION_SERVO LEDC_TIMER_16_BIT 

// Variáveis Globais Voláteis (Substituindo std::atomic)
// O 'volatile' avisa ao compilador que esta variável pode mudar a qualquer momento.
volatile uint32_t g_throttle_width_us = 1500; 
volatile uint32_t g_steering_width_us = 1500; 
volatile uint64_t g_throttle_edge_time = 0;
volatile uint64_t g_steering_edge_time = 0;

// --- Protótipo da Função ISR (para que o compilador saiba que existe) ---
static void IRAM_ATTR rc_isr_handler(void* arg);


// --- 2. Funções de Controle e Inicialização ---

void set_motor_dc(int duty, int direction) {
    if (duty < 0) duty = 0;
    if (duty > 255) duty = 255; 

    if (direction == 1) { // Frente
        gpio_set_level(MOTOR_IN1_PIN, 1);
        gpio_set_level(MOTOR_IN2_PIN, 0);
    } else if (direction == -1) { // Trás
        gpio_set_level(MOTOR_IN1_PIN, 0);
        gpio_set_level(MOTOR_IN2_PIN, 1);
    } else { // Parar
        gpio_set_level(MOTOR_IN1_PIN, 0);
        gpio_set_level(MOTOR_IN2_PIN, 0);
    }
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR);
}

void set_servo_angle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    uint32_t pulse_width_us = 500 + (angle * 2000) / 180;
    
    // Duty = (pulse_width_us * 2^16) / 20000us
    uint32_t duty = (pulse_width_us * 65536) / 20000;
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_SERVO, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_SERVO);
}

void init_ledc_pwm() {
    // Configuração dos Timers e Canais LEDC (idêntica à versão C++)
    ledc_timer_config_t ledc_timer_motor = { .speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER_MOTOR, .duty_resolution = LEDC_RESOLUTION, .freq_hz = LEDC_FREQUENCY, .clk_cfg = LEDC_AUTO_CLK };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_motor));
    ledc_channel_config_t ledc_channel_motor = { .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_MOTOR, .timer_sel = LEDC_TIMER_MOTOR, .gpio_num = MOTOR_ENABLE_PIN, .duty = 0 };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_motor));
    
    ledc_timer_config_t ledc_timer_servo = { .speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER_SERVO, .duty_resolution = LEDC_RESOLUTION_SERVO, .freq_hz = LEDC_FREQUENCY_SERVO, .clk_cfg = LEDC_AUTO_CLK };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_servo));
    ledc_channel_config_t ledc_channel_servo = { .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_SERVO, .timer_sel = LEDC_TIMER_SERVO, .gpio_num = SERVO_PIN, .duty = 0 };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_servo));
}

void init_rc_input() {
    // Configuração dos pinos de Saída e Entrada
    gpio_set_direction(MOTOR_IN1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_IN2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_IN1_PIN, 0);
    gpio_set_level(MOTOR_IN2_PIN, 0);
    
    gpio_install_isr_service(0); 
    gpio_set_direction(RC_THROTTLE_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(RC_STEERING_PIN, GPIO_MODE_INPUT);

    // Configura interrupções em AMBAS as bordas (Subida e Descida)
    gpio_set_intr_type(RC_THROTTLE_PIN, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(RC_STEERING_PIN, GPIO_INTR_ANYEDGE);

    // Adiciona o handler (a função ISR) para cada pino
    gpio_isr_handler_add(RC_THROTTLE_PIN, rc_isr_handler, (void*)RC_THROTTLE_PIN);
    gpio_isr_handler_add(RC_STEERING_PIN, rc_isr_handler, (void*)RC_STEERING_PIN);
}

// --- 3. Função de Serviço de Interrupção (ISR) ---
static void IRAM_ATTR rc_isr_handler(void* arg) {
    gpio_num_t pin = (gpio_num_t)arg;
    uint64_t current_time = esp_timer_get_time();

    if (pin == RC_THROTTLE_PIN) {
        if (gpio_get_level(pin) == 1) { // Borda de Subida (Início)
            g_throttle_edge_time = current_time; 
        } else { // Borda de Descida (Fim)
            uint32_t pulse_duration = (uint32_t)(current_time - g_throttle_edge_time);
            if (pulse_duration >= 900 && pulse_duration <= 2100) {
                g_throttle_width_us = pulse_duration;
            }
        }
    } else if (pin == RC_STEERING_PIN) {
        if (gpio_get_level(pin) == 1) { // Borda de Subida (Início)
            g_steering_edge_time = current_time; 
        } else { // Borda de Descida (Fim)
            uint32_t pulse_duration = (uint32_t)(current_time - g_steering_edge_time);
            if (pulse_duration >= 900 && pulse_duration <= 2100) {
                g_steering_width_us = pulse_duration;
            }
        }
    }
}


// --- 4. Tarefa Principal e Ponto de Entrada ---

void rc_control_task(void *arg) {
    const int NEUTRO_RC = 1500;
    const int DEADZONE = 50; 
    uint32_t throttle_rc, steering_rc; // Variáveis locais para leitura segura

    set_servo_angle(90); 

    while (1) {
        // --- Leitura Segura das Variáveis Voláteis ---
        // Desabilita as interrupções para garantir que a leitura seja atômica
        // (necessário porque a leitura de uint32_t pode não ser atômica na arquitetura)
        ets_intr_lock();
        throttle_rc = g_throttle_width_us;
        steering_rc = g_steering_width_us;
        ets_intr_unlock();
        
        // --- Lógica de Controle (Idêntica à versão C++) ---
        
        // Direção (Servo)
        if (steering_rc >= 1000 && steering_rc <= 2000) {
            int servo_angle = (steering_rc - 1000) * 180 / 1000; 
            set_servo_angle(servo_angle);
        }

        // Velocidade (Motor DC)
        if (throttle_rc >= 1000 && throttle_rc <= 2000) {
            
            if (throttle_rc > (NEUTRO_RC - DEADZONE) && throttle_rc < (NEUTRO_RC + DEADZONE)) {
                set_motor_dc(0, 0); 
            
            } else if (throttle_rc >= (NEUTRO_RC + DEADZONE)) {
                int speed = (throttle_rc - (NEUTRO_RC + DEADZONE)) * 255 / (2000 - (NEUTRO_RC + DEADZONE));
                set_motor_dc(speed, 1);
                
            } else { 
                int speed = ((NEUTRO_RC - DEADZONE) - throttle_rc) * 255 / ((NEUTRO_RC - DEADZONE) - 1000);
                set_motor_dc(speed, -1);
            }
        }
        
        ESP_LOGI(TAG, "Throttle: %u us | Steering: %u us", throttle_rc, steering_rc);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
    
// Ponto de Entrada (Main em C)
void app_main(void) {
    ESP_LOGI(TAG, "Inicializando sistema de controle RC (C Puro)...");
    
    init_ledc_pwm();
    init_rc_input();
    
    xTaskCreate(rc_control_task, "RC_Control", 4096, NULL, 5, NULL);
}
