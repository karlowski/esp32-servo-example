#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "driver/ledc.h"
#include <math.h>

#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL ADC_CHANNEL_4 // GPIO #5
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_BITWIDTH ADC_BITWIDTH_12

#define LEDC_SERVO_FREQUENCY 50
#define LEDC_SERVO_TIMER LEDC_TIMER_1
#define LEDC_SERVO_CHANNEL LEDC_CHANNEL_1
#define LEDC_SERVO_CHANNEL_PIN GPIO_NUM_4
#define LEDC_SERVO_DUTY_RESOLUTION LEDC_TIMER_14_BIT

#define SERVO_MAX_DUTY (1u<<14)
#define SERVO_PERIOD_US 20000u
#define SERVO_MIN_US 420
#define SERVO_MAX_US 2400
#define SERVO_MIN_ANGLE 0.0f
#define SERVO_MAX_ANGLE 180.0f

#define POTEN_MIN 0
#define POTEN_MAX 3600
#define POTEN_MAX_ANGLE 300.0f
#define ADC_MAX_RAW 4095.0f

typedef struct
{
    int raw;
    int value_mv;
    adc_unit_t adc_unit;
    adc_channel_t channel;
    adc_atten_t atten;
    adc_bitwidth_t bitwidth;
    adc_oneshot_unit_handle_t oneshot_handle;
    adc_cali_handle_t cali_handle;
} adc_component_config_t;

uint64_t last_operation_at = 0;

void adc_config(adc_component_config_t* adc)
{
    adc_oneshot_unit_init_cfg_t oneshot_init_cfg = {
        .unit_id = adc->adc_unit,
    };
    adc_oneshot_chan_cfg_t oneshot_chan_cfg = {
        .bitwidth = adc->bitwidth,
        .atten = adc->atten,
    };
    adc_cali_curve_fitting_config_t cali_curve_fitting_cfg = {
        .unit_id = adc->adc_unit,
        .atten = adc->atten,
        .bitwidth = adc->bitwidth,
        .chan = adc->channel,
    };

    adc_oneshot_new_unit(&oneshot_init_cfg, &adc->oneshot_handle);
    adc_oneshot_config_channel(adc->oneshot_handle, adc->channel, &oneshot_chan_cfg);
    adc_cali_create_scheme_curve_fitting(&cali_curve_fitting_cfg, &adc->cali_handle);
}

static void servo_set_us(uint32_t us)
{
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;

    uint32_t duty = (uint32_t)(us * SERVO_MAX_DUTY / SERVO_PERIOD_US);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_SERVO_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_SERVO_CHANNEL);
}

static void servo_set_angle(float deg)
{
    uint32_t us = SERVO_MIN_US + (uint32_t)((deg/180.0f) * (SERVO_MAX_US - SERVO_MIN_US));

    servo_set_us(us);
}

void app_main() 
{
    static adc_component_config_t potentiometer = {
        .raw = 0,
        .value_mv = 0,
        .adc_unit = ADC_UNIT,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
        .channel = ADC_CHANNEL,
    };
    adc_config(&potentiometer);

    static ledc_timer_config_t servo_timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_SERVO_DUTY_RESOLUTION,
        .timer_num = LEDC_SERVO_TIMER,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&servo_timer_config);

    static ledc_channel_config_t servo_channel_config = {
        .gpio_num = LEDC_SERVO_CHANNEL_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_SERVO_CHANNEL,
        .timer_sel = LEDC_SERVO_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&servo_channel_config);

    while(1)
    {
        adc_oneshot_read(potentiometer.oneshot_handle, potentiometer.channel, &potentiometer.raw);

        float deg_raw = (float)potentiometer.raw * POTEN_MAX_ANGLE / ADC_MAX_RAW;
        float deg = deg_raw;
        if (deg > SERVO_MAX_ANGLE) deg = SERVO_MAX_ANGLE;
        if (deg < SERVO_MIN_ANGLE) deg = SERVO_MIN_ANGLE;

        servo_set_angle(deg);

        if (deg_raw > SERVO_MAX_ANGLE)
        {
            ESP_LOGI("SAMPLE", "angle (absolute) = %.1f deg; angle (diff) = %.1f; raw = %d", deg, deg_raw - SERVO_MAX_ANGLE, potentiometer.raw);
        } else 
        {
            ESP_LOGI("SAMPLE", "angle (absolute) = %.1f deg; angle (diff) = NULL; raw = %d", deg, potentiometer.raw);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
