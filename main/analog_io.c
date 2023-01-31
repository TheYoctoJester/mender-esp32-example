/**
 * @file      led.c
 * @brief     Code to operate the RGB-LEDs on the Mender-ESP32 demo board
 *
 * MIT License
 *
 * Copyright (c) 2023 Josef Holzmayr, based on ESP-IDF example code
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "analog_io.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

/**
 * @brief Tag used for logging
 */
static const char *TAG = "analog_io";

//ADC Channels
#define ADC1_EXAMPLE_CHAN0          ADC1_CHANNEL_0

/* ADC Attenuation */

#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_11

/**
 * @brief ADC reading task
 */
static void analog_io_task(void* arg)
{
    int last = 0;
    while (1) {
		/* read current ADC value, and print if delta is large enough */
        int current = adc1_get_raw(ADC1_EXAMPLE_CHAN0);
        if (abs(last - current) > 100) {
            ESP_LOGI(TAG, "raw adc read: %d", current);
        }
        last = current;
		/* this task is rather fast! */
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void run_analog_io(void)
{
    //ADC1 config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));

    /* start analog_io task */
    ESP_LOGI(TAG, "starting analog io task");
    xTaskCreate(analog_io_task, "analog_io_task", 200, NULL, 10, NULL);
}