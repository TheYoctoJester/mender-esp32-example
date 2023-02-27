/**
 * @file      demo.c
 * @brief     Demonstration application on the Mender-ESP32 demo board
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

#include "demo.h"

#include "esp_adc_cal.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "led_strip.h"
#include "ssd1306.h"

#include "defines.h"

static
ssd1306_handle_t ssd1306_dev = NULL;

static
xQueueHandle encoder_evt_queue = NULL;

static
led_strip_t *pStrip_a;

static
uint32_t neopixel_speed = NEOPIXEL_SPEED_MAX;
static
uint32_t neopixel_map = 0x00;

static
uint32_t relay_state = 0;

static
void init_IO()
{
	/* analog IO - ADC1 config */
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));

	/* digital IO - GPIO config */
	/* GPIO 18: relay */
    gpio_config_t o_conf = {};
    o_conf.intr_type = GPIO_INTR_DISABLE;
    o_conf.mode = GPIO_MODE_OUTPUT;
    o_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    o_conf.pull_down_en = 0;
    o_conf.pull_up_en = 0;
    gpio_config(&o_conf);
    
	/* GPIO 19/20/21: rotary encoder plus button */
    gpio_config_t i_conf = {};
    i_conf.intr_type = GPIO_INTR_ANYEDGE;
    i_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    i_conf.mode = GPIO_MODE_INPUT;
    i_conf.pull_up_en = 1;
    gpio_config(&i_conf);

    /* change gpio intrrupt type for push button */
    gpio_set_intr_type(GPIO_INPUT_PUSH, GPIO_INTR_NEGEDGE);

	/* SSD1306 - display */
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}


static
void IRAM_ATTR encoder_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;

	int rot_a = gpio_get_level(GPIO_INPUT_ROT_A);
	int rot_b = gpio_get_level(GPIO_INPUT_ROT_B);
	//int push = gpio_get_level(GPIO_INPUT_PUSH);

	uint32_t e = ENCODER_EVENT_NONE;
	if (gpio_num == GPIO_INPUT_PUSH) {
		e = ENCODER_EVENT_PUSH;
	}
	else if (gpio_num == GPIO_INPUT_ROT_A) {
		if (rot_a == rot_b) {
			e = ENCODER_EVENT_LEFT;
		}
		else {
			e = ENCODER_EVENT_RIGHT;
		}
	}

	if (e != ENCODER_EVENT_NONE) {
    	xQueueSendFromISR(encoder_evt_queue, &e, NULL);
	}
}

static
void init_Encoder()
{
    //create a queue to handle gpio event from isr
    encoder_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_ROT_A, encoder_isr_handler, (void*) GPIO_INPUT_ROT_A);
    //hook isr handler for specific gpio pin
    //gpio_isr_handler_add(GPIO_INPUT_ROT_B, encoder_isr_handler, (void*) GPIO_INPUT_ROT_B);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_PUSH, encoder_isr_handler, (void*) GPIO_INPUT_PUSH);
}

static
void init_Display()
{
    ssd1306_dev = ssd1306_create(I2C_MASTER_NUM, SSD1306_I2C_ADDRESS);
    ssd1306_refresh_gram(ssd1306_dev);
    ssd1306_clear_screen(ssd1306_dev, 0x00);
}

static
void init_NeoPixel()
{
    /* LED strip initialization with the GPIO and pixels number*/
    pStrip_a = led_strip_init(NEOPIXEL_RMT_CHANNEL, NEOPIXEL_GPIO, NEOPIXEL_COUNT);
    /* Set all LED off to clear all pixels */
    pStrip_a->clear(pStrip_a, 50);
}

static
void toggle_Relay()
{
	relay_state = (relay_state + 1) % 2;
}

static
void update_Display(char const * s)
{
    ssd1306_draw_string(ssd1306_dev, 32, 16, (uint8_t const *)"Mender-", 12, 1);
#if DEMO_VERSION == 1
    ssd1306_draw_string(ssd1306_dev, 32, 28, (uint8_t const *)"ESP32 V1.0", 12, 1);
#elif DEMO_VERSION == 2
    ssd1306_draw_string(ssd1306_dev, 32, 28, (uint8_t const *)"ESP32 V2.0", 12, 1);
#else
#error "Meh!"
#endif
    ssd1306_draw_string(ssd1306_dev, 32, 40, (uint8_t const *)s, 12, 1);
    ssd1306_refresh_gram(ssd1306_dev);
}

static inline
uint32_t proc_to_hex(uint32_t const p)
{
	return p * 255 / 100;
}
static
void update_NeoPixel(uint32_t const brightness)
{
	uint32_t b = proc_to_hex(brightness);

	/* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
	pStrip_a->set_pixel(pStrip_a, 0, (neopixel_map & 0b00000001) ? b : 0, 0, 0);
	pStrip_a->set_pixel(pStrip_a, 1, 0, (neopixel_map & 0b00000010) ? b : 0, 0);
	pStrip_a->set_pixel(pStrip_a, 2, 0, 0, (neopixel_map & 0b00000100) ? b : 0);
	/* Refresh the strip to send data */
	pStrip_a->refresh(pStrip_a, 100);
}

static
void color_task()
{
	uint32_t state = 0;

    while (1) {
		switch (state)
		{
#if DEMO_VERSION == 1
			case 1:
				neopixel_map = 0b00000001;
				break;
			case 2:
				neopixel_map = 0b00000011;
				break;
			case 3:
				neopixel_map = 0b00000111;
				break;
			case 4:
				neopixel_map = 0b00000110;
				break;
			case 5:
				neopixel_map = 0b00000100;
				break;
#elif DEMO_VERSION == 2
			case 1:
				neopixel_map = 0b00000001;
				break;
			case 2:
				neopixel_map = 0b00000010;
				break;
			case 3:
				neopixel_map = 0b00000100;
				break;
			case 4:
				neopixel_map = 0b00000010;
				break;
			case 5:
				neopixel_map = 0b00000001;
				toggle_Relay();
				break;
#else
#error "Meh!"
#endif
			default:
				neopixel_map = 0b00000000;
				state = 0;
				break;
		}
		++state;
        vTaskDelay(pdMS_TO_TICKS(neopixel_speed));
	}
}

static
void demo_task()
{
	uint32_t neopixel_brightness = 50;
	char s[16];

    while (1) {
		while(uxQueueMessagesWaiting(encoder_evt_queue)) {
    		uint32_t encoder_event;
        	if(xQueueReceive(encoder_evt_queue, &encoder_event, portMAX_DELAY)) {
				switch (encoder_event)
				{
					case ENCODER_EVENT_PUSH:
						toggle_Relay();
						break;

					case ENCODER_EVENT_LEFT:
						if (neopixel_brightness > 0) {
							--neopixel_brightness;
						}
						break;
					case ENCODER_EVENT_RIGHT:
						if (neopixel_brightness < 100) {
							++neopixel_brightness;
						}
						break;
					default:
						break;
				}
			}
        }

		sprintf(s, "%03d %s", neopixel_brightness, relay_state ? "On " : "Off");
		update_Display(s);
		update_NeoPixel(neopixel_brightness);

		/* "min": 6870, "max": 230 */
		int a = adc1_get_raw(ADC1_EXAMPLE_CHAN0);
		if (a > ADC_MAX) {
			a = ADC_MAX;
		}
		else if (a < ADC_MIN) {
			a = ADC_MIN;
		}
		neopixel_speed = NEOPIXEL_SPEED_MIN + ((a - ADC_MIN) * NEOPIXEL_SPEED_DELTA) / ADC_DELTA;

		/* set relay*/
        gpio_set_level(GPIO_OUTPUT_RELAY, relay_state);
    }
}

/**
 * @brief Initialize stuff and run the demo task
 * @return nothing
 */
void run_demo()
{
	init_IO();
	init_Encoder();
	init_Display();
	init_NeoPixel();

    xTaskCreate(color_task, "color_task", 2048, NULL, 10, NULL);
    xTaskCreate(demo_task, "demo_task", 4096, NULL, 10, NULL);
}