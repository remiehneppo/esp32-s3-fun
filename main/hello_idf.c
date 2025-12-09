#include <stdio.h>
#include <stdlib.h>
#include "esp_http_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "portmacro.h"

#define led 2
#define button 21

void task_1(void *pvParams) {
    while (1) {
        gpio_set_level(led, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(led, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
}

void checkButton(void *pvParams) {
    while (1) {
        if (gpio_get_level(button) == 0) {
            vTaskDelay(40 / portTICK_PERIOD_MS);
            if(gpio_get_level(button) == 0) {
                printf("Button has been press\n");
            }
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    esp_http_client_config_t http_client_cfg = {

    };
    printf("hello world");
    gpio_reset_pin(led);
    gpio_reset_pin(button);
    gpio_set_direction(led, GPIO_MODE_OUTPUT);
    gpio_set_direction(button, GPIO_MODE_INPUT);
    gpio_set_pull_mode(button, GPIO_PULLUP_ONLY);

    xTaskCreate(
        &task_1,
        "blink",
        2048,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        &checkButton, 
        "check button",
        2048, 
        NULL, 
        2, 
        NULL
    );



}
