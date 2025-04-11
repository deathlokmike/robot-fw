#include "XClk.h"

#include "driver/ledc.h"

bool ClockEnable(int pin, int hz) {
    ESP_LOGD(cameraLogTag, "Started clock enable");
    periph_module_enable(PERIPH_LEDC_MODULE);

    ledc_timer_config_t timer_conf;
    timer_conf.duty_resolution = LEDC_TIMER_1_BIT;
    timer_conf.freq_hz = hz;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf.timer_num = LEDC_TIMER_3;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    esp_err_t err = ledc_timer_config(&timer_conf);

    if (err != ESP_OK) {
        ESP_LOGE(cameraLogTag, "Failed to configure LEDC timer: %s",
                 esp_err_to_name(err));
        return false;
    }

    ledc_channel_config_t ch_conf;
    ch_conf.channel = LEDC_CHANNEL_3;
    ch_conf.timer_sel = LEDC_TIMER_3;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.duty = 1;
    ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    ch_conf.gpio_num = pin;
    ch_conf.hpoint = 0;
    err = ledc_channel_config(&ch_conf);

    if (err != ESP_OK) {
        ESP_LOGE(cameraLogTag, "Failed to configure LEDC channel: %s",
                 esp_err_to_name(err));
        return false;
    }
    ESP_LOGD(cameraLogTag, "Clock done");
    return true;
}

void ClockDisable() { periph_module_disable(PERIPH_LEDC_MODULE); }
