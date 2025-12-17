#ifndef UART_H
#define UART_H
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "driver/uart.h"

const char *separa_argumentos = "--";
const char *separa_comandos = ",";

extern const char *TAG;
extern data CANAL[];
extern mcpwm_cap_channel_handle_t cap_chan[];
extern bool _simula_;
extern mcpwm_timer_handle_t h_timer;
extern mcpwm_cmpr_handle_t h_comparator;
extern mcpwm_timer_config_t timer_config;
extern int32_t duty;
extern uint32_t frec_reloj_filtro;
extern const int PWM_RESOLUTION_HZ;
extern bool fcalibra;

void uart_exeption(uart_event_t event, uint8_t *dtmp);

static QueueHandle_t uart0_queue;

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM (3) /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

typedef bool (*func_varargs_t)(int n, char *str, ...);

#define NCODIGOS 14

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
bool fun12(int n, char *str, ...);
bool fun13(int n, char *str, ...);
bool fun14(int n, char *str, ...);

const char *COD[NCODIGOS] = {"INICIA" /*1*/,
                             "PARA" /*2*/,
                             "RESTART" /*3*/,
                             "RESET" /*4*/,
                             "CALIBRA" /*5*/,
                             "SAVE" /*6*/,
                             "CONFIG" /*7*/,
                             "SIMULA" /*8*/,
                             "LAUNCH" /*9*/,
                             "STATUS" /*10*/,
                             "CANAL" /*11*/,
                             "DUTY" /*12*/,
                             "FREC" /*13*/,
                             "SENSIBILIDAD" /*14*/};

func_varargs_t funciones[NCODIGOS] = {fun1, fun2, fun3, fun4, fun5, fun6, fun7, fun8, fun9, fun10, fun11, fun12, fun13, fun14};

// Definir tipo para puntero a función con argumentos variables

// Funciones de ejemplo con argumentos variables

//"START"
//--------------------------------------------------------------------------------
bool fun1(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);

    for (int i = 0; i < 4; i++)
    {
        if (CANAL[i].enable)
        {
            // estos valores son para reiniciar las cuentas de cada canal
            CANAL[i].f_error = false;
            CANAL[i].cont_errores = 0;
            CANAL[i].dir_flanco_actual = 0;
            CANAL[i].dir_flanco_anterior = 0;
            CANAL[i].t_anterior = 0;
            CANAL[i].reinicia_variables();

            // esto es para que vuelva a ingresar a la interrupcio
            printf("activando canal %d\n", i);
            mcpwm_capture_channel_enable(cap_chan[i]);
        }
        else
        {
            mcpwm_capture_channel_disable(cap_chan[i]);
        }
    }
    return true;
}

//"STOP"
//--------------------------------------------------------------------------------
bool fun2(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);

    for (int i = 0; i < 4; i++)
    {
        // estos valores son para reiniciar las cuentas de cada canal
        CANAL[i].f_error = false;
        CANAL[i].cont_errores = 0;
        CANAL[i].dir_flanco_actual = 0;
        CANAL[i].dir_flanco_anterior = 0;
        CANAL[i].t_anterior = 0;
        CANAL[i].reinicia_variables();

        // desactiva la captura de todos los canales
        mcpwm_capture_channel_disable(cap_chan[i]);
    }
    return true;
}

//"RESTART"
//--------------------------------------------------------------------------------
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

//"RESET"
//--------------------------------------------------------------------------------
bool fun4(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);
    return true;
}

//"CALIBRA"
//--------------------------------------------------------------------------------
bool fun5(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);

    fcalibra = true;

    for (int i = 0; i < 4; i++)
    {
        CANAL[i].f_calibra[0] = true;
        CANAL[i].f_calibra[1] = true;
        CANAL[i].contador_calibra[0] = 0;
        CANAL[i].contador_calibra[1] = 0;
        CANAL[i].ui_mean_sum[0] = 0;
        CANAL[i].ui_mean_sum[1] = 0;
    }

    return true;
}

//"SAVE"
//--------------------------------------------------------------------------------
bool fun6(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);
    return true;
}

//"CONFIG"
//--------------------------------------------------------------------------------
bool fun7(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);
    return true;
}

//"SIMULA"
//--------------------------------------------------------------------------------
bool fun8(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);

    _simula_ = !_simula_;

    if (_simula_)
    {
        mcpwm_comparator_set_compare_value(h_comparator, (timer_config.period_ticks * duty) / 100);
        printf("Modo SIMULACION ACTIVADO\n");
    }
    else
    {
        mcpwm_comparator_set_compare_value(h_comparator, 0);
        printf("Modo SIMULACION DESACTIVADO\n");
    }

    return true;
}

//"LAUNCH"
//--------------------------------------------------------------------------------
bool fun9(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);
    return true;
}

//"STATUS"
//--------------------------------------------------------------------------------
bool fun10(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);
    return true;
}

// CANAL
//--------------------------------------------------------------------------------

bool fun11(int n, char *str, ...)
{

    char *token;
    // descarta el primer espacio si lo hay
    // strtok(str, "-");
    // get the first token
    token = strtok(str, separa_argumentos); // descarta el primer espacio si lo hay
    token = strtok(NULL, separa_argumentos);
    int valor = 0;
    bool dato_valido = false;
    // walk through other tokens
    while (token != NULL)
    {
        valor = atoi(token);

        printf(" numero %d\n", valor);

        token = strtok(NULL, separa_argumentos); // busca el siguiente token

        dato_valido = true;

        if (valor >= 1 && valor <= 4)
        {
            int i = valor - 1;

            CANAL[i].enable = CANAL[i].enable ? false : true; // hace toggle del enable

            printf("CANAL:%d\tE:%s\tgpio:%d\tID:%d\tmedU:%u\tmedD:%u\n", valor,
                   (CANAL[i].enable) ? "E" : "D",
                   CANAL[i].gpio_num,
                   CANAL[i].ID,
                   (unsigned int)CANAL[i].ui_mean[0],
                   (unsigned int)CANAL[i].ui_mean[1]);

            if (CANAL[i].enable)
            {
                // estos valores son para reiniciar las cuentas de cada canal
                CANAL[i].f_error = false;
                CANAL[i].cont_errores = 0;
                CANAL[i].dir_flanco_actual = 0;
                CANAL[i].dir_flanco_anterior = 0;
                CANAL[i].t_anterior = 0;
                CANAL[i].reinicia_variables();

                // esto es para que vuelva a ingresar a la interrupcio
                mcpwm_capture_channel_enable(cap_chan[i]);
            }
            else
            {
                mcpwm_capture_channel_disable(cap_chan[i]);
            }
        }
    }

    if (!dato_valido)
    {
        printf("\n----canales status----\n");
        for (int i = 0; i < 4; i++)
        {
            printf("CANAL:%d\tE:%s\tgpio:%d\tID:%d\tmedU:%u\tmedD:%u\n", i + 1,
                   (CANAL[i].enable) ? "E" : "D",
                   CANAL[i].gpio_num,
                   CANAL[i].ID,
                   (unsigned int)CANAL[i].ui_mean[0],
                   (unsigned int)CANAL[i].ui_mean[1]);
        }
        printf("---------------------\n");
    }

    //    printf("Codigo de activacion recibido: %s\n", COD[n]);
    // ptr = strstr();

    return true;
}

bool fun12(int n, char *str, ...)
{
    bool dato_valido = false;

    printf("%s\n", str);

    if (strstr(str, "+") != NULL)
    {
        duty = duty + 10;

        if (duty >= 100)
        {
            duty = 100;
        }

        dato_valido = true;
    }
    else if (strstr(str, "-") != NULL)
    {
        duty = duty - 10;

        if (duty <= 0)
        {
            duty = 0;
        }

        dato_valido = true;
    }

    if (dato_valido)
    {

        if (duty == 100)
        {
            duty = 100;
        }
        else if (duty == 0)
        {
            duty = 0;
        }

        mcpwm_comparator_set_compare_value(h_comparator, timer_config.period_ticks * duty / 100);

        printf("duty actual: %ld %%\n", duty);
    }
    else
    {
        printf("no cambia duty \n");
    }

    return true;
}

bool fun13(int n, char *str, ...)
{
    bool dato_valido = false;
    uint32_t frec = 0;

    printf("%s\n", str);

    frec = atoi(str);

    if (frec >= 50 && frec <= 300) // en khz
    {
        frec_reloj_filtro = frec * 1000;
        timer_config.period_ticks = PWM_RESOLUTION_HZ / frec_reloj_filtro; // periodo en ticks
        mcpwm_timer_set_period(h_timer, timer_config.period_ticks);
        mcpwm_comparator_set_compare_value(h_comparator, (timer_config.period_ticks * duty) / 100);

        dato_valido = true;
    }

    if (dato_valido)
    {
        printf("frecuencia de reloj del filtro actual: %ld kHz\n", frec_reloj_filtro / 1000);
    }
    else
    {
        printf("frecuencia de reloj del filtro fuera de rango (50-300) kHz \n");
    }

    return true;
}

//"SENSIBILIDAD"
//--------------------------------------------------------------------------------
bool fun14(int n, char *str, ...)
{
    printf("Codigo de activacion recibido: %s\n", COD[n]);

    char *token;
    bool dato_valido = false;
    int canal = 0;
    int valor = 0;
    bool cambiar_up = false;
    bool cambiar_down = false;

    // Parsear el string de entrada
    token = strtok(str, separa_argumentos);  // descarta el primer espacio si lo hay
    token = strtok(NULL, separa_argumentos); // canal o comando directo

    if (token != NULL)
    {
        // Verificar si es un canal específico con doble guión (--1, --2, --3, --4)
        bool es_canal_especifico = false;
        if (token[0] == '-' && token[1] == '-' && strlen(token) == 3)
        {
            canal = atoi(&token[2]); // extraer número después de los guiones
            if (canal >= 1 && canal <= 4)
            {
                es_canal_especifico = true;
            }
        }

        if (es_canal_especifico)
        {
            // Comando para canal específico
            int i = canal - 1; // índice del array (0-3)

            token = strtok(NULL, separa_argumentos); // siguiente token
            if (token != NULL)
            {
                // Verificar si es un flag U o D (siempre en mayúsculas)
                if (strcmp(token, "U") == 0)
                {
                    cambiar_up = true;
                    token = strtok(NULL, separa_argumentos); // obtener el valor
                    if (token != NULL)
                    {
                        valor = atoi(token);
                        if (valor > 0)
                        {
                            CANAL[i].delta_max[0] = valor; // UP
                            printf("Canal %d: Delta UP = %d\n", canal, valor);
                            dato_valido = true;
                        }
                        else
                        {
                            printf("Error: El valor debe ser positivo\n");
                        }
                    }
                    else
                    {
                        printf("Error: Falta especificar el valor para U\n");
                    }
                }
                else if (strcmp(token, "D") == 0)
                {
                    cambiar_down = true;
                    token = strtok(NULL, separa_argumentos); // obtener el valor
                    if (token != NULL)
                    {
                        valor = atoi(token);
                        if (valor > 0)
                        {
                            CANAL[i].delta_max[1] = valor; // DOWN
                            printf("Canal %d: Delta DOWN = %d\n", canal, valor);
                            dato_valido = true;
                        }
                        else
                        {
                            printf("Error: El valor debe ser positivo\n");
                        }
                    }
                    else
                    {
                        printf("Error: Falta especificar el valor para D\n");
                    }
                }
                else
                {
                    // Formato original: valor UP y opcionalmente DOWN
                    int delta_up = atoi(token);
                    int delta_down = delta_up; // por defecto mismo valor

                    token = strtok(NULL, separa_argumentos); // valor delta DOWN (opcional)
                    if (token != NULL)
                    {
                        delta_down = atoi(token);
                    }

                    // Validar que los valores sean positivos
                    if (delta_up > 0 && delta_down > 0)
                    {
                        // Actualizar los valores delta del canal
                        CANAL[i].delta_max[0] = delta_up;   // UP
                        CANAL[i].delta_max[1] = delta_down; // DOWN

                        printf("Canal %d: Delta UP = %d, Delta DOWN = %d\n", canal, delta_up, delta_down);
                        dato_valido = true;
                    }
                    else
                    {
                        printf("Error: Los valores delta deben ser positivos\n");
                    }
                }
            }
            else
            {
                printf("Error: Falta especificar parámetros\n");
            }
        }
        else
        {
            // Comando para TODOS los canales (no se especificó canal válido)
            if (strcmp(token, "U") == 0)
            {
                cambiar_up = true;
                token = strtok(NULL, separa_argumentos); // obtener el valor
                if (token != NULL)
                {
                    valor = atoi(token);
                    if (valor > 0)
                    {
                        for (int i = 0; i < 4; i++)
                        {
                            CANAL[i].delta_max[0] = valor; // UP
                        }
                        printf("Todos los canales: Delta UP = %d\n", valor);
                        dato_valido = true;
                    }
                    else
                    {
                        printf("Error: El valor debe ser positivo\n");
                    }
                }
                else
                {
                    printf("Error: Falta especificar el valor para U\n");
                }
            }
            else if (strcmp(token, "D") == 0)
            {
                cambiar_down = true;
                token = strtok(NULL, separa_argumentos); // obtener el valor
                if (token != NULL)
                {
                    valor = atoi(token);
                    if (valor > 0)
                    {
                        for (int i = 0; i < 4; i++)
                        {
                            CANAL[i].delta_max[1] = valor; // DOWN
                        }
                        printf("Todos los canales: Delta DOWN = %d\n", valor);
                        dato_valido = true;
                    }
                    else
                    {
                        printf("Error: El valor debe ser positivo\n");
                    }
                }
                else
                {
                    printf("Error: Falta especificar el valor para D\n");
                }
            }
            else
            {
                // Formato numérico: aplicar a todos los canales
                int delta_up = atoi(token);
                int delta_down = delta_up; // por defecto mismo valor

                token = strtok(NULL, separa_argumentos); // valor delta DOWN (opcional)
                if (token != NULL)
                {
                    delta_down = atoi(token);
                }

                // Validar que los valores sean positivos
                if (delta_up > 0 && delta_down > 0)
                {
                    // Actualizar los valores delta de TODOS los canales
                    for (int i = 0; i < 4; i++)
                    {
                        CANAL[i].delta_max[0] = delta_up;   // UP
                        CANAL[i].delta_max[1] = delta_down; // DOWN
                    }
                    printf("Todos los canales: Delta UP = %d, Delta DOWN = %d\n", delta_up, delta_down);
                    dato_valido = true;
                }
                else
                {
                    printf("Error: Los valores delta deben ser positivos\n");
                }
            }
        }
    }
    else
    {
        // Sin parámetros, mostrar estado actual de todos los canales
        printf("\n----Sensibilidad de canales----\n");
        for (int i = 0; i < 4; i++)
        {
            printf("Canal %d: Delta UP = %u, Delta DOWN = %u\n",
                   i + 1,
                   (unsigned int)CANAL[i].delta_max[0],
                   (unsigned int)CANAL[i].delta_max[1]);
        }
        printf("------------------------------\n");
        dato_valido = true;
    }

    if (!dato_valido)
    {
        printf("Uso:\n");
        printf("  SENSIBILIDAD--1--U--[valor]          (cambiar solo UP del canal 1)\n");
        printf("  SENSIBILIDAD--2--D--[valor]          (cambiar solo DOWN del canal 2)\n");
        printf("  SENSIBILIDAD--3--[up]--[down]        (cambiar ambos del canal 3)\n");
        printf("  SENSIBILIDAD--4--[valor]             (cambiar ambos del canal 4 al mismo valor)\n");
        printf("  SENSIBILIDAD--U--[valor]             (cambiar solo UP de TODOS los canales)\n");
        printf("  SENSIBILIDAD--D--[valor]             (cambiar solo DOWN de TODOS los canales)\n");
        printf("  SENSIBILIDAD--[up]--[down]           (cambiar ambos de TODOS los canales)\n");
        printf("  SENSIBILIDAD--[valor]                (cambiar ambos de TODOS los canales al mismo valor)\n");
        printf("Ejemplos:\n");
        printf("  SENSIBILIDAD--1--U--100     (solo UP del canal 1 = 100)\n");
        printf("  SENSIBILIDAD--2--D--150     (solo DOWN del canal 2 = 150)\n");
        printf("  SENSIBILIDAD--3--100--200   (canal 3: UP=100, DOWN=200)\n");
        printf("  SENSIBILIDAD--U--250        (UP de todos los canales = 250)\n");
        printf("  SENSIBILIDAD--D--300        (DOWN de todos los canales = 300)\n");
        printf("  SENSIBILIDAD--100--150      (todos: UP=100, DOWN=150)\n");
        printf("  SENSIBILIDAD--200           (todos: UP=200, DOWN=200)\n");
    }

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
    // size_t buffered_size;
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

                char *token;
                char *ptr;
                // get the first token
                token = strtok((char *)dtmp, separa_comandos);
                // walk through other tokens
                while (token != NULL)
                {
                    printf(" %s\n", token);

                    for (int i = 0; i < NCODIGOS; i++)
                    {
                        ptr = strstr(token, COD[i]);

                        if (ptr != NULL)
                        {
                            ESP_LOGI(TAG, "Comando reconocido: %s", ptr);
                            ptr = ptr + strlen(COD[i]);
                            // ESP_LOGI(TAG, "parametros: %s", ptr);
                            comando_reconocido = funciones[i](i, ptr);
                        }
                    }
                    token = strtok(NULL, separa_comandos); // busca el siguiente token
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