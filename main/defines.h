/**
 * @file      defines.h
 * @brief     Definitions for the Mender demo application
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

#ifndef __DEFINES_H_
#define __DEFINES_H_


/* ADC Channels */
#define ADC1_EXAMPLE_CHAN0          ADC1_CHANNEL_0
#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_11
#define ADC_MAX						6870	/* from one demo board, no real deep meaning */
#define ADC_MIN						232
#define ADC_DELTA					(ADC_MAX - ADC_MIN)

/* GPIOs */
#define GPIO_INPUT_ROT_A		19
#define GPIO_INPUT_ROT_B		20
#define GPIO_INPUT_PUSH			21
#define GPIO_INPUT_PIN_SEL		((1ULL << GPIO_INPUT_ROT_A) | (1ULL << GPIO_INPUT_ROT_B) | (1ULL << GPIO_INPUT_PUSH))
#define ESP_INTR_FLAG_DEFAULT	0

#define GPIO_OUTPUT_RELAY		18
#define GPIO_OUTPUT_PIN_SEL		(1ULL << GPIO_OUTPUT_RELAY)

/* SSD1306 display */
#define I2C_MASTER_SCL_IO 33        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 34        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1    /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */

/* NeoPixel LEDs */
#define NEOPIXEL_RMT_CHANNEL 0
#define NEOPIXEL_GPIO 17
#define NEOPIXEL_COUNT 3
#define NEOPIXEL_SPEED_MAX 5000
#define NEOPIXEL_SPEED_MIN 500
#define NEOPIXEL_SPEED_DELTA (NEOPIXEL_SPEED_MAX - NEOPIXEL_SPEED_MIN)

/* Encoder events */
#define ENCODER_EVENT_NONE 0
#define ENCODER_EVENT_PUSH 1
#define ENCODER_EVENT_LEFT 2
#define ENCODER_EVENT_RIGHT 3

#endif /* __DEFINES_H_ */
