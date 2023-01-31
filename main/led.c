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

#include "led.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

/**
 * @brief Tag used for logging
 */
static const char *TAG = "led";

/**
 * @brief RMT channel used for LED 
 */
#define CONFIG_BLINK_LED_RMT_CHANNEL 0

/**
 * @brief GPIO used for LED strip
 */
#define BLINK_LED_GPIO 17

/**
 * @brief Number of LEDs in addressed chain
 */
#define BLINK_LED_COUNT 3

static uint8_t s_led_state = 0;
static led_strip_t *pStrip_a;

/**
 * @brief LED blinking task
 */
static void LED_strip_task(void* arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

		/* toggle */
		s_led_state = (s_led_state + 1) % 2;
		/* If the addressable LED is enabled */
		if (s_led_state) {
			/* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
			pStrip_a->set_pixel(pStrip_a, 0, 128, 0, 0);
			pStrip_a->set_pixel(pStrip_a, 1, 0, 128, 0);
			pStrip_a->set_pixel(pStrip_a, 2, 0, 0, 128);
			/* Refresh the strip to send data */
			pStrip_a->refresh(pStrip_a, 100);
		} else {
			/* Set all LED off to clear all pixels */
			pStrip_a->clear(pStrip_a, 50);
		}
    }
}

void run_led(void)
{
    /* LED strip initialization with the GPIO and pixels number*/
    pStrip_a = led_strip_init(CONFIG_BLINK_LED_RMT_CHANNEL, BLINK_LED_GPIO, BLINK_LED_COUNT);
    /* Set all LED off to clear all pixels */
    pStrip_a->clear(pStrip_a, 50);

    /* start LED blinking task */
    ESP_LOGI(TAG, "starting LED blink task");
    xTaskCreate(LED_strip_task, "LED_strip_task", 200, NULL, 10, NULL);
}