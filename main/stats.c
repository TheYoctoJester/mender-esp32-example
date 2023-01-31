/**
 * @file      stats.c
 * @brief     Example glue code to use Mender in an ESP32 project
 *
 * MIT License
 *
 * Copyright (c) 2022-2023 joelguittet and mender-mcu-client contributors
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

#include "stats.h"

#include <stdio.h>

#if configUSE_TRACE_FACILITY == 1

/**
 * @brief Tag used for logging
 */
static const char *TAG = "stats";

/**
 * @brief Print FreeRTOS stats
 */
void
print_stats(void) {

    /* Take a snapshot of the number of tasks in case it changes while this function is executing */
    volatile UBaseType_t uxArraySize = uxTaskGetNumberOfTasks();

    /* Allocate a TaskStatus_t structure for each task */
    TaskStatus_t *pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));
    if (NULL != pxTaskStatusArray) {

        /* Generate raw status information about each task */
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, NULL);

        /* For each populated position in the pxTaskStatusArray array, format the raw data as human readable ASCII data */
        printf("--------------------------------------------------------\n");
        printf("Task Name       | Stack High Water Mark\n");
        printf("--------------------------------------------------------\n");
        for (UBaseType_t index = 0; index < uxArraySize; index++) {
            printf("%15s | %u bytes\n", pxTaskStatusArray[index].pcTaskName, pxTaskStatusArray[index].usStackHighWaterMark * sizeof(configSTACK_DEPTH_TYPE));
        }

        /* Release memory */
        vPortFree(pxTaskStatusArray);
    }

    /* Print usage of the heap */
    printf("--------------------------------------------------------\n");
    printf("Free Heap Size: %u bytes\n", xPortGetFreeHeapSize());
    printf("Minimum Ever Free Heap Size: %u bytes\n", xPortGetMinimumEverFreeHeapSize());
    printf("--------------------------------------------------------\n");
}

#endif