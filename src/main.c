#include "esp_common.h"
#include "freertos/task.h"
#include "gpio.h"
#include <stdbool.h>

#define LED_ON  0  // nível lógico para LED ligado no GPIO16
#define LED_OFF 1  // nível lógico para LED desligado no GPIO16

#define LED_GPIO_NUM 2
#define LED_GPIO_BIT (1 << LED_GPIO_NUM)  // BIT2 = (1 << 2)

volatile bool led_state = false;  // true = aceso, false = apagado

uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;
    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;
        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;
        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;
        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }
    return rf_cal_sec;
}

// Tarefa que pisca o LED
void task_blink(void* ignore)
{
    gpio16_output_conf();  // configura GPIO16 como saída

    while (true) {
        gpio16_output_set(LED_ON);
        led_state = true;
        vTaskDelay(1000 / portTICK_RATE_MS);

        gpio16_output_set(LED_OFF);
        led_state = false;
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

// Tarefa que imprime o estado do LED na serial
void task_status(void* ignore)
{
    while (true) {
        if (led_state) {
            printf("LED ACESO\n");
        } else {
            printf("LED APAGADO\n");
        }
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

void task_blink2(void* ignore)
{
    // Configura GPIO2 como saída
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
    gpio_output_set(0, 0, LED_GPIO_BIT, 0);  // Habilita GPIO2 como saída

    while (true) {
        gpio_output_set(0, LED_GPIO_BIT, 0, 0);  // Nível LOW = liga o LED
        vTaskDelay(500 / portTICK_RATE_MS);
        gpio_output_set(LED_GPIO_BIT, 0, 0, 0);  // Nível HIGH = desliga o LED
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

// Função principal do usuário
void user_init(void)
{
    uart_div_modify(0, UART_CLK_FREQ / 115200);  // configura baudrate
    printf("Inicializando tarefas...\n");

    xTaskCreate(&task_blink, "blink", 2048, NULL, 1, NULL);
    xTaskCreate(&task_status, "status", 2048, NULL, 1, NULL);
    xTaskCreate(&task_blink2, "blink", 2048, NULL, 1, NULL);
}
