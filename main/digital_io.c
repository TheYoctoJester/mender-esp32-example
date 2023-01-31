/**
 * @file      digital_io.c
 * @brief     Code to read rotary encoder and set relay on the Mender-ESP32 demo board
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

#include "digital_io.h"

#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

/**
 * @brief Tag used for logging
 */
static const char *TAG = "digital_io";

/**
 * Brief:
 * This test code shows how to configure gpio and how to use gpio interrupt.
 *
 * GPIO status:
 * GPIO18: output
 * GPIO19: output
 * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
 * GPIO5:  input, pulled up, interrupt from rising edge.
 */

#define GPIO_INPUT_IO_0     19
#define GPIO_INPUT_IO_1     20
#define GPIO_INPUT_IO_2     21
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2))
#define ESP_INTR_FLAG_DEFAULT 0

#define GPIO_OUTPUT_IO_0    18
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val 19/20/21: %d/%d/%dS\n",
                io_num,
                gpio_get_level(GPIO_INPUT_IO_0),
                gpio_get_level(GPIO_INPUT_IO_1),
                gpio_get_level(GPIO_INPUT_IO_2)
            );
        }
    }
}

static void relay_task(void* arg)
{
	int relay_state = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2500));

		/* toggle */
		relay_state = (relay_state + 1) % 2;
		/* set relay*/
        gpio_set_level(GPIO_OUTPUT_IO_0, relay_state);
    }
}

void run_digital_io(void)
{
	//zero-initialize the config structure.
    gpio_config_t o_conf = {};
    //disable interrupt
    o_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    o_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    o_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    o_conf.pull_down_en = 0;
    //disable pull-up mode
    o_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&o_conf);
    
    //zero-initialize the config structure.
    gpio_config_t i_conf = {};
    // pin 19, rot A - both edges
    //interrupt of rising edge
    i_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO19/20/21 here
    i_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    i_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    i_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&i_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_2, GPIO_INTR_NEGEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    ESP_LOGI(TAG, "starting digital input task");
    xTaskCreate(gpio_task_example, "gpio_task_example", 200, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void*) GPIO_INPUT_IO_2);

    /* start analog_io task */
    ESP_LOGI(TAG, "starting digital output task");
    xTaskCreate(relay_task, "relay_task", 200, NULL, 10, NULL);
}