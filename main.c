

// esp log... = informação, saber oq ocorre no codigo 
// lembrando : gpio = entrada e saida de modo geral
/* tipos
-INPUT = le e escuta o sinal
-OUTPUT = envia ou escreve um sinal
obs : o set-direction MODE OUTPUT = define o pino de saida
      o set-direction MODE INPUT = define o pino de entrada
      o set-level = define a saida high ou low */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rom/ets_sys.h" // Para as funções de desabilitar/reabilitar interrupções
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "sdkconfig.h"
// --- TAG de Log ---
static const char *TAG = "CARRO_RC";

// NUMEROS ILUSTRATIVOS DOS PINOS
//  Pinos de Entrada (Receptor RC)
#define ACELERACAO_PIN GPIO_NUM_14
#define DIRECAO_PIN GPIO_NUM_12
// Pinos de Saída (Motor e Servo)
#define MOTOR_IN1_PIN GPIO_NUM_25
#define MOTOR_IN2_PIN GPIO_NUM_26
#define HABILITAR_MOTOR_PIN GPIO_NUM_27
#define SERVO_PIN GPIO_NUM_13

// Configuração LEDC/PWM -> controlador do pulso

#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER_MOTOR LEDC_TIMER_0
#define LEDC_CANAL_MOTOR LEDC_CHANNEL_0
#define LEDC_FREQUENCIA 5000
#define LEDC_RESOLUTION LEDC_TIMER_8_BIT
#define LEDC_TIMER_SERVO LEDC_TIMER_1
#define LEDC_CANAL_SERVO LEDC_CHANNEL_1
#define LEDC_FREQUENCIA_SERVO 50
#define LEDC_RESOLUCAO_SERVO LEDC_TIMER_16_BIT

/* Variáveis Globais Voláteis
IMPORTANTE: indica que a variavel vai ficar mudando */
volatile uint32_t lar_aceleracao_pulso = 1500;
volatile uint32_t lar_direcao_pulso = 1500;
volatile uint64_t time_aceleracao_borda = 0;
volatile uint64_t time_direcao_borda = 0;


static void IRAM_ATTR controle_pulso(void *arg);

/*direcao inicializar
duty *se refere a ciclo de trabalho / fator de carga*/
void definir_MOTOR_dc(int duty, int direcao)
{
    if (duty < 0)
    {
        duty = 0;
    }
    if (duty > 255)
    {
        duty = 255;
    }

    if (direcao == 1)
    { // Frente
        gpio_set_level(MOTOR_IN1_PIN, 1);
        gpio_set_level(MOTOR_IN2_PIN, 0);
    }
    else if (direcao == -1)
    { // Trás
        gpio_set_level(MOTOR_IN1_PIN, 0);
        gpio_set_level(MOTOR_IN2_PIN, 1);
    }
    else
    { // Parar
        gpio_set_level(MOTOR_IN1_PIN, 0);
        gpio_set_level(MOTOR_IN2_PIN, 0);
    }

    ledc_set_duty(LEDC_MODE, LEDC_CANAL_MOTOR, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CANAL_MOTOR);
}

void definir_angulo_SERVO (int angulo)
{
    if (angulo < 0)
    {
        angulo = 0;
    }
    if (angulo > 180)
    {
        angulo = 180;
    }
    uint32_t largura_pulso = 500 + (angulo * 2000) / 180;

    // Duty = ( lergura do pulso * 2^16) / 20000us
    uint32_t duty = (largura_pulso * 65536) / 20000;

    ledc_set_duty(LEDC_MODE, LEDC_CANAL_SERVO, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CANAL_SERVO);
}

// pwm sinal do motor \o/

void inicializar_pwm()
{
    // Configuração dos Timers e Canais LEDC 
    //lembrar IMPORTANTE
    ledc_timer_config_t ledc_timer_motor = {.speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER_MOTOR, .duty_resolution = LEDC_RESOLUTION, .freq_hz = LEDC_FREQUENCIA, .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_motor));
    ledc_channel_config_t canal_motor_conf = {.speed_mode = LEDC_MODE, .channel = LEDC_CANAL_MOTOR, .timer_sel = LEDC_TIMER_MOTOR, .gpio_num = HABILITAR_MOTOR_PIN, .duty = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&canal_motor_conf));

    ledc_timer_config_t ledc_timer_servo = {.speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER_SERVO, .duty_resolution = LEDC_RESOLUCAO_SERVO, .freq_hz = LEDC_FREQUENCIA_SERVO, .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_servo));
    ledc_channel_config_t canal_servo_conf = {.speed_mode = LEDC_MODE, .channel = LEDC_CANAL_SERVO, .timer_sel = LEDC_TIMER_SERVO, .gpio_num = SERVO_PIN, .duty = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&canal_servo_conf));
}

void inicializar_saida_entre()
{
    // Configuração dos pinos de Saída e Entrada
    gpio_set_direction(MOTOR_IN1_PIN, GPIO_MODE_OUTPUT); //direçao dos pinos
    gpio_set_direction(MOTOR_IN2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_IN1_PIN, 0); //nivel voltagem 
    gpio_set_level(MOTOR_IN2_PIN, 0);

    gpio_install_isr_service(0); //ativo interrupção
    gpio_set_direction(ACELERACAO_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(DIRECAO_PIN, GPIO_MODE_INPUT);

    // Configura interrupções em AMBAS as bordas up ou down
    gpio_set_intr_type(ACELERACAO_PIN, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(DIRECAO_PIN, GPIO_INTR_ANYEDGE);

      //um mediador a mudanças e mediçoes do controle pro esp****
    gpio_isr_handler_add(ACELERACAO_PIN, controle_pulso, (void *)ACELERACAO_PIN);
    gpio_isr_handler_add(DIRECAO_PIN, controle_pulso, (void *)DIRECAO_PIN);
}

//Serviço de Interrupção
static void IRAM_ATTR controle_pulso(void *arg)
{
    gpio_num_t pin = (gpio_num_t)arg;
    uint64_t time_atual = esp_timer_get_time();

    if (pin == ACELERACAO_PIN)
    {
        if (gpio_get_level(pin) == 1)
        { // Borda de Subida (Início)
            time_aceleracao_borda = time_atual;
        }
        else
        { // Borda de Descida (Fim)
            uint32_t duracao_Pulse = (uint32_t)(time_atual - time_aceleracao_borda);
            if (duracao_Pulse >= 900 && duracao_Pulse <= 2100)
            {
                lar_aceleracao_pulso = duracao_Pulse;
            }
        }
    }
    else if (pin == DIRECAO_PIN)
    {
        if (gpio_get_level(pin) == 1)
        { // Borda de Subida (Início)
            time_direcao_borda = time_atual;
        }
        else
        { // Borda de Descida (Fim)
            uint32_t duracaoPulso = (uint32_t)(time_atual - time_direcao_borda);
            if (duracaoPulso >= 900 && duracaoPulso <= 2100)
            {
                lar_direcao_pulso = duracaoPulso;
            }
        }
    }
}


//parte principal
void carrinho_controle(void *arg)
{
    const int NEUTRO = 1500;
    const int DEADZONE = 50;
    uint32_t aceleracao, direcao;

    definir_angulo_SERVO(90);

    while (1)
    {
        // VARIAVESI volateis**
        /*Desabilita as interrupções para garantir que a leitura seja atômica
        IMPORTANTE necessário porque a leitura de uint32_t pode não ser atômica na arquitetura)*/
        ets_intr_lock();
        aceleracao = lar_aceleracao_pulso;
        direcao = lar_direcao_pulso;
        ets_intr_unlock();

        // Direção (Servo)
        if (direcao >= 1000 && direcao <= 2000)
        {
            int servo_angulo = (direcao - 1000) * 180 / 1000;
            definir_angulo_SERVO(servo_angulo);
        }

        // Velocidade (Motor)
        if (aceleracao >= 1000 && aceleracao <= 2000)
        {

            if (aceleracao > (NEUTRO - DEADZONE) && aceleracao < (NEUTRO + DEADZONE))
            {
                definir_MOTOR_dc(0, 0);
            }
            else if (aceleracao >= (NEUTRO + DEADZONE))
            {
                int speed = (aceleracao - (NEUTRO + DEADZONE)) * 255 / (2000 - (NEUTRO + DEADZONE));
                definir_MOTOR_dc(speed, 1);
            }
            else
            {
                int speed = ((NEUTRO - DEADZONE) - aceleracao) * 255 / ((NEUTRO - DEADZONE) - 1000);
                definir_MOTOR_dc(speed, -1);
            }
        }

        ESP_LOGI(TAG, "Throttle: %u us | Steering: %u us", aceleracao, direcao);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    //ESP_LOGI(TAG, "Inicializando sistema de controle");

    inicializar_pwm();
    inicializar_saida_entre();

    xTaskCreate(carrinho_controle, "RC_Control", 4096, NULL, 5, NULL);
}

