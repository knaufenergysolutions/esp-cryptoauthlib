/*
 *  Key generation application
 *  Espressif MIT License
 *  Copyright 2021 Espressif Systems (Shanghai) CO LTD
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *  http://www.apache.org/licenses/LICENSE-2.0
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  Mbedtls License
 *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 */
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include "esp_log.h"
#include <esp_console.h>
#include <driver/uart.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "driver/usb_serial_jtag.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_vfs_dev.h"

#include "commands.h"

#define CMD_BUFFER_SIZE 1024

static const char *TAG = "CMD Handler";
QueueHandle_t uart_queue;

static void setup_usb_serial() 
{

    // /* Disable buffering on stdin */
    // setvbuf(stdin, NULL, _IONBF, 0);

    // /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    // esp_vfs_dev_usb_serial_jtag_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    // /* Move the caret to the beginning of the next line on '\n' */
    // esp_vfs_dev_usb_serial_jtag_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    // /* Enable non-blocking mode on stdin and stdout */
    // fcntl(fileno(stdout), F_SETFL, 0);
    // fcntl(fileno(stdin), F_SETFL, 0);

    usb_serial_jtag_driver_config_t usb_serial_jtag_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_config.rx_buffer_size = 1024;
    usb_serial_jtag_config.tx_buffer_size = 1024;

    esp_err_t ret = ESP_OK;
    /* Install USB-SERIAL-JTAG driver for interrupt-driven reads and writes */
    ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    if (ret != ESP_OK) {
        printf("Failed to install USB-SERIAL-JTAG driver\n");
        return;
    }

    // /* Tell vfs to use usb-serial-jtag driver */
    // esp_vfs_usb_serial_jtag_use_driver();
}

static void scli_loop()
{
    int uart_num = 0, i, cmd_ret;
    uint8_t linebuf[CMD_BUFFER_SIZE];
    esp_err_t ret;
    uart_event_t event;

    uart_driver_install(uart_num, CMD_BUFFER_SIZE, 0, 32, &uart_queue, 0);
    printf("Initialising Command line: >>");
    fflush(stdout);
    while (true) {
        uart_write_bytes(uart_num, "\n>>\n", 4);
        bzero(linebuf, sizeof(linebuf));
        i = 0;
        do {
            ret = xQueueReceive(uart_queue, (void * )&event, (TickType_t)portMAX_DELAY);
            if (ret != pdPASS) {
                continue;
            }

            if (event.type == UART_DATA) {
                while (uart_read_bytes(uart_num, (uint8_t *) &linebuf[i], 1, 0)) {
                    if (linebuf[i] == '\r') {
                        uart_write_bytes(uart_num, "\r\n", 2);
                    } else {
                        uart_write_bytes(uart_num, (char *) &linebuf[i], 1);
                    }
                    i++;
                }
            }
        } while ((i < CMD_BUFFER_SIZE - 1) && linebuf[i - 1] != '\r');

        /* Remove the truncating \r\n */
        linebuf[strlen((char *)linebuf) - 1] = '\0';
        printf("\n");

        esp_console_run((char *) linebuf, &cmd_ret);
    }
}

static void usb_cli_loop() {
    int i, cmd_ret;
    uint8_t linebuf[CMD_BUFFER_SIZE];
    esp_err_t ret;
    char ch = 0xFF;

    setup_usb_serial();
    printf("Initialising Command line: >>");
    // fflush(stdout);
    while (true) {
        printf("\n>>\n");
        bzero(linebuf, sizeof(linebuf));
        i = 0;
        // printf(".");
        do {
            while (ch == 0xFF) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
                usb_serial_jtag_read_bytes(&ch, 1, 0);
                // usb_serial_jtag_write_bytes(".", 1, 0);
            }
            linebuf[i] = ch;
            ch = 0xFF;
            if (linebuf[i] == '\r') {
                // printf("\r\n");
                // fflush(stdout);
                usb_serial_jtag_write_bytes("\r\n", 2, 0);
            } else {
                // printf((char *) &linebuf[i]);
                // fflush(stdout);
                // usb_serial_jtag_write_bytes((char *) &linebuf[i], 1, 0);
            }
            i++;
        } while ((i < CMD_BUFFER_SIZE - 1) && linebuf[i - 1] != '\r');

        /* Remove the truncating \r\n */
        linebuf[strlen((char *)linebuf) - 1] = '\0';
        printf("%s",(char*)linebuf);
        printf("\n");

        esp_console_run((char *) linebuf, &cmd_ret);
    }

}

static void scli_task(void *arg)
{
    esp_console_config_t console_config = {
        .max_cmdline_args = 8,
        .max_cmdline_length = CMD_BUFFER_SIZE,
    };
    esp_console_init(&console_config);
    esp_console_register_help_command();

    if (register_command_handler() == ESP_OK) {
        // scli_loop();
        usb_cli_loop();
    } else {
        ESP_LOGE(TAG, "Failed to register all commands");
    }

    ESP_LOGI(TAG, "Stopping the CLI");
    vTaskDelete(NULL);
}

void app_main()
{
    BaseType_t cli_task = xTaskCreate(scli_task, "scli_task", 8 * 1024, NULL, configMAX_PRIORITIES - 5, NULL);
    if (cli_task != pdPASS) {
        ESP_LOGE(TAG, "Couldn't create scli thread");
    }
}
