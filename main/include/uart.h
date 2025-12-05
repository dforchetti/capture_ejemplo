#ifndef UART_H
#define UART_H
#include <stdio.h>
#include <ctype.h>
#include "driver/uart.h"

extern const char *TAG;

void uart_exeption(uart_event_t event, uint8_t *dtmp);

static QueueHandle_t uart0_queue;

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM (3) /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

typedef bool (*func_varargs_t)(int n, char *str, ...);

#define NCODIGOS 11

bool fun1(int n, char *str, ...);
bool fun2(int n, char *str, ...);
bool fun3(int n, char *str, ...);
bool fun4(int n, char *str, ...);
bool fun5(int n, char *str, ...);
bool fun6(int n, char *str, ...);
bool fun7(int n, char *str, ...);
bool fun8(int n, char *str, ...);
bool fun9(int n, char *str, ...);
bool fun10(int n, char *str, ...);
bool fun11(int n, char *str, ...);

const char *COD[NCODIGOS] = {"START" /*1*/,
                             "STOP" /*2*/,
                             "RESTART" /*3*/,
                             "RESET" /*4*/,
                             "CALIBRA" /*5*/,
                             "SAVE" /*6*/,
                             "CONFIG" /*7*/,
                             "BEGIN" /*8*/,
                             "LAUNCH" /*9*/,
                             "STATUS" /*10*/,
                             "CANAL" /*11*/};

func_varargs_t funciones[NCODIGOS] = {fun1, fun2, fun3, fun4, fun5, fun6, fun7, fun8, fun9, fun10, fun11};

// Definir tipo para puntero a función con argumentos variables

// Funciones de ejemplo con argumentos variables
bool fun1(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);
    return true;
}

bool fun2(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);
    return true;
}

bool fun3(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);
    for (int i = 3; i >= 0; i--)
    {
        printf("reseteando %d\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    esp_restart();
    return true;
}
bool fun4(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);
    return true;
}
bool fun5(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);
    return true;
}
bool fun6(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);
    return true;
}
bool fun7(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);
    return true;
}
bool fun8(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);
    return true;
}
bool fun9(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);
    return true;
}
bool fun10(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);
    return true;
}
// CANAL
bool fun11(int n, char *str, ...)
{
    char *ptr;
    printf("Codigo de activacion recibido: %s\n", COD[n]);
    ptr = strstr();

    return true;
}

void configura_uart(void)
{
    uart_config_t uart_config = {};
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    // Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);
    // Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
    // Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 20);
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t *dtmp = (uint8_t *)malloc(RD_BUF_SIZE);
    bool comando_reconocido = false;
    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            bzero(dtmp, RD_BUF_SIZE);
            // ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);

            if (event.type == UART_DATA)
            {
                uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);

                // pasa a mayusculas

                char *s = (char *)dtmp;
                while (*s)
                {
                    *s = toupper((unsigned char)*s);
                    s++;
                }

                char *ptr;

                for (int i = 0; i < NCODIGOS; i++)
                {
                    ptr = strstr((const char *)dtmp, COD[i]);
                    // ptr

                    if (ptr != NULL)
                    {
                        ESP_LOGI(TAG, "Comando reconocido: %s", ptr);

                        comando_reconocido = funciones[i](i, (char *)dtmp);
                    }
                }
                if (!comando_reconocido)
                {
                    printf("Comando no reconocido: %s\n", dtmp);
                }
            }
            else
            {
                uart_exeption(event, dtmp);
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void uart_exeption(uart_event_t event, uint8_t *dtmp)
{

    size_t buffered_size;
    if (event.type == UART_FIFO_OVF)
    {

        ESP_LOGI(TAG, "hw fifo overflow");
        // If fifo overflow happened, you should consider adding flow control for your application.
        // The ISR has already reset the rx FIFO,
        // As an example, we directly flush the rx buffer here in order to read more data.
        uart_flush_input(EX_UART_NUM);
        xQueueReset(uart0_queue);
    }
    else if (event.type == UART_BUFFER_FULL)
    {
        ESP_LOGI(TAG, "ring buffer full");
        // If buffer full happened, you should consider increasing your buffer size
        // As an example, we directly flush the rx buffer here in order to read more data.
        uart_flush_input(EX_UART_NUM);
        xQueueReset(uart0_queue);
        // Event of UART RX break detected
    }
    else if (event.type == UART_BREAK)
    {
        ESP_LOGI(TAG, "uart rx break");

        // Event of UART parity check error
    }
    else if (event.type == UART_PARITY_ERR)
    {
        ESP_LOGI(TAG, "uart parity error");
        // Event of UART frame error
    }
    else if (event.type == UART_FRAME_ERR)
    {
        ESP_LOGI(TAG, "uart frame error");
        // UART_PATTERN_DET
    }
    else if (event.type == UART_PATTERN_DET)
    {
        uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
        int pos = uart_pattern_pop_pos(EX_UART_NUM);
        ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
        if (pos == -1)
        {
            // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
            // record the position. We should set a larger queue size.
            // As an example, we directly flush the rx buffer here.
            uart_flush_input(EX_UART_NUM);
        }
        else
        {
            uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
            uint8_t pat[PATTERN_CHR_NUM + 1];
            memset(pat, 0, sizeof(pat));
            uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "read data: %s", dtmp);
            ESP_LOGI(TAG, "read pat : %s", pat);
        }
        // Others
    }
    else
    {
        ESP_LOGI(TAG, "uart event type: %d", event.type);
    }
}

#endif // UART_H