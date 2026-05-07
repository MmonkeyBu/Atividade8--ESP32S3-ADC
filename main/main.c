#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// Definições de Hardware para ESP32-S3
#define BUTTON_PIN         GPIO_NUM_4
#define LED_PIN            GPIO_NUM_18
#define ADC_UNIT           ADC_UNIT_1
#define ADC_CHANNEL        ADC_CHANNEL_0      // GPIO 1 na S3
#define ADC_ATTEN          ADC_ATTEN_DB_12    // Atenuação para ler até 3.3V
#define ADC_BITWIDTH       ADC_BITWIDTH_12    // Resolução de 12 bits (0-4095)

// Configurações de Software
#define ADC_SAMPLES         16                // Média de 16 amostras para maior estabilidade
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_10_BIT // Aumentado para 10 bits (0-1023) para suavidade
#define LEDC_FREQUENCY      5000              // 5 kHz

// Variáveis de Controle
volatile bool is_hold_mode = false;
volatile uint32_t current_adc_raw = 0;
volatile uint32_t hold_adc_raw = 0;

// Handles
adc_oneshot_unit_handle_t adc_handle;
adc_cali_handle_t adc_cali_handle = NULL;

// ============================================================================
// Tratamento do Botão (Interrupção com Debounce)
// ============================================================================
static void IRAM_ATTR button_isr_handler(void* arg) {
    static uint64_t last_time = 0;
    uint64_t now = esp_timer_get_time();

    // Debounce de 250ms
    if (now - last_time > 250000) {
        if (!is_hold_mode) {
            hold_adc_raw = current_adc_raw; // Trava o valor atual
            is_hold_mode = true;
        } else {
            is_hold_mode = false;
        }
        last_time = now;
    }
}

// ============================================================================
// Tarefa de Controle (ADC -> PWM)
// ============================================================================
static void adc_pwm_task(void *pvParameters) {
    int raw_val = 0;
    
    while (1) {
        if (!is_hold_mode) {
            uint32_t sum = 0;
            for (int i = 0; i < ADC_SAMPLES; i++) {
                adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw_val);
                sum += raw_val;
            }
            current_adc_raw = sum / ADC_SAMPLES;
        }

        // Valor a ser aplicado (atual ou travado)
        uint32_t val_to_apply = is_hold_mode ? hold_adc_raw : current_adc_raw;

        // Mapeamento: ADC (12 bits -> 4095) para PWM (10 bits -> 1023)
        // Cálculo: (valor * 1023) / 4095
        uint32_t duty = (val_to_apply * 1023) / 4095;

        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

        vTaskDelay(pdMS_TO_TICKS(20)); // Atualização a 50Hz
    }
}

// ============================================================================
// Monitoramento Serial (500ms)
// ============================================================================
static void monitoring_callback(TimerHandle_t xTimer) {
    uint32_t display_raw = is_hold_mode ? hold_adc_raw : current_adc_raw;
    int voltage_mv = 0;

    // Converte valor bruto para mV usando a calibração da fábrica
    if (adc_cali_handle) {
        adc_cali_raw_to_voltage(adc_cali_handle, (int)display_raw, &voltage_mv);
    }

    printf("[%s] Bruto: %4lu | Tensão: %4d mV | LED: %3lu%%\n",
           is_hold_mode ? "HOLD" : "LIVE",
           display_raw, 
           voltage_mv,
           (display_raw * 100) / 4095);
}

// ============================================================================
// Inicializações
// ============================================================================
void init_hw(void) {
    // 1. ADC OneShot + Calibração
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT };
    adc_oneshot_new_unit(&init_config1, &adc_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH,
        .atten = ADC_ATTEN,
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config);

    // Esquema de calibração para ESP32-S3
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT,
        .chan = ADC_CHANNEL,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);

    // 2. LEDC (PWM)
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LED_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

    // 3. Botão com Pull-up
    gpio_config_t btn_conf = {
        .intr_type    = GPIO_INTR_NEGEDGE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&btn_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);
}

void app_main(void) {
    init_hw();

    // Task para processamento de sinal
    xTaskCreate(adc_pwm_task, "adc_pwm_task", 4096, NULL, 5, NULL);

    // Timer para monitoramento (500ms)
    TimerHandle_t timer = xTimerCreate("monitor", pdMS_TO_TICKS(500), pdTRUE, NULL, monitoring_callback);
    xTimerStart(timer, 0);

    printf("Sistema iniciado. Pressione o botão no GPIO4 para alternar HOLD/LIVE.\n");
}