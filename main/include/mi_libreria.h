#ifndef INCLUDE_MI_LIBRERIA_H_
#define INCLUDE_MI_LIBRERIA_H_

#include "math.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

using namespace std;
// Configuración del UART
#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024
#define PATTERN_CHR_NUM 3

// Palabra específica a buscar
extern const char *TAG;
static const char *TARGET_WORD = "activar"; // Cambia esto por tu palabra
extern mcpwm_cap_timer_handle_t cap_timer[];
extern bool f_arraque;

class data
{

private:
  // const int TRIGG_MAX = 0;
  // const int TRIGG_MIN =0;
  const int N_disparos_MAX = 100;
  const int N_disparos_MIN = 100;

public:
  //-----------------------------------------------------------------------------------
  // config
  //-----------------------------------------------------------------------------------
  int gpio_num;             // el GPIO asociado a la captura
  int dpin;                 // el pin asociado al debug del GPIO
  int cap_timer;            // el timer asociado a la medicion de tiempo 0/1
  int ID;                   // un numero identificatorio 1,2,3,..etc
  int code;                 // un codigo identificatorio cualqueira 111,222, etc
  int dir_flanco_actual;    // UP/DOWN
  bool dir_flanco_anterior; // UP/DOWN
  int cont_errores;         // contador de errores en el procesamiento de los datos
  bool f_error;             // flag de error true/false desde la ultima vez que se controló
  //-----------------------------------------------------------------------------------
  // trigger el indice 2 obedece a estadoUP, estado DOWN
  //-----------------------------------------------------------------------------------
  // uint32_t delta_trigger_max[2];     // umbral maximo de disparo
  // uint32_t delta_trigger_min[2];     // umbral maximo de disparo
  uint32_t t_anterior;
  uint32_t delta_max[2]; // error maximo de tiempo

  int contador_disparos_max[2];   // contador histórico de disparos consecutivos
  int contador_disparos_min[2];   // contador histórico de disparos consecutivos
  int contador_error_disparos[2]; // si no se logra alcanzar el valor del
                                  // contador de disparos up, se resetea la
                                  // cuenta y se incrementa este contador
  int32_t max[2];                 // valor maximo desde el ultimo control
  int32_t min[2];                 // valor minimo desde el ultimo control
  int n_max_disparos[2];          // numero maximo de disparos para conseguir un evento
  bool flag_evento[2];

  //-----------------------------------------------------------------------------------
  // stats
  //-----------------------------------------------------------------------------------

  uint32_t ui_mean[2]; // para poder comparar en la isr
  uint64_t M2[2];
  uint32_t ui_mean_sum[2];
  int count[2];

  //-----------------------------------------------------------------------------------
  // funciones
  //-----------------------------------------------------------------------------------
  // constructor
  data();
  // destructor
  ~data();

  void reset();
};

data::data()
{
  this->reset();
}
data::~data()
{
  this->reset();
}
void data::reset(void)
{
  this->gpio_num = 0;
  this->cap_timer = 0;
  this->ID = 0;
  this->code = 0;
  this->dir_flanco_actual = 0;
  this->dir_flanco_anterior = 0;
  this->cont_errores = 0;
  this->f_error = 0;

  for (int i = 0; i < 2; i++)
  {
    this->count[i] = 0;
    this->M2[i] = 0;
    this->ui_mean[i] = 0;
    this->ui_mean_sum[i] = 0;
    // this->delta_trigger_max[i] = TRIGG_MAX;
    // this->delta_trigger_min[i] = TRIGG_MIN;
    this->contador_disparos_max[i] = N_disparos_MAX;
    this->contador_disparos_min[i] = N_disparos_MIN;
    this->contador_error_disparos[i] = 0;
    this->max[i] = 0;
    this->min[i] = 0;
    this->n_max_disparos[i] = 0;
    this->flag_evento[i] = false;
  }
}

// Función para inicializar UART
static void uart_init_config(void)
{
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  // Configurar parámetros del UART
  ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

  // Configurar pines (UART0 por defecto usa GPIO1 TX, GPIO3 RX)
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM, GPIO_NUM_1, GPIO_NUM_3,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  // Instalar driver UART
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
}

// Función para buscar palabra en el texto
static bool contains_word(const char *text, const char *word)
{
  if (text == NULL || word == NULL)
    return false;

  char *found = strstr(text, word);
  return (found != NULL);
}

// Función para limpiar buffer
static void clean_buffer(char *buffer, int length)
{
  for (int i = 0; i < length; i++)
  {
    buffer[i] = 0;
  }
}

// Tarea principal para leer y procesar datos seriales
static void serial_monitor_task(void *arg)
{
  uint8_t data[BUF_SIZE];
  char received_text[BUF_SIZE];
  int text_index = 0;

  ESP_LOGI(TAG, "Serial monitor iniciado");
  ESP_LOGI(TAG, "Esperando datos... Palabra objetivo: '%s'", TARGET_WORD);
  // printf("Envía texto por serial. Buscando la palabra: %s\n", TARGET_WORD);

  while (f_arraque == NULL)
  {
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

  while (1)
  {
    // Leer datos del UART
    int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);

    if (len > 0)
    {
      ESP_LOGI(TAG, "ACA");
      data[len] = '\0'; // Terminar string

      // Procesar cada carácter recibido
      for (int i = 0; i < len; i++)
      {
        char c = data[i];

        // Si es nueva línea o retorno de carro, procesar el texto completo
        if (c == '\n' || c == '\r')
        {
          if (text_index > 0)
          {
            received_text[text_index] = '\0';

            // Mostrar texto recibido
            ESP_LOGI(TAG, "Texto recibido: %s", received_text);

            // Buscar palabra específica
            if (contains_word(received_text, TARGET_WORD))
            {
              ESP_LOGI(TAG, "¡Palabra '%s' encontrada!", TARGET_WORD);
              printf(">>> ¡COINCIDENCIA! La palabra '%s' fue detectada.\n", TARGET_WORD);

              // Aquí puedes agregar acciones cuando se encuentra la palabra
              // Ejemplo: encender un LED, enviar respuesta, etc.
              // gpio_set_level(LED_PIN, 1);
            }
            else
            {
              ESP_LOGI(TAG, "Palabra '%s' NO encontrada", TARGET_WORD);
            }

            // Reiniciar buffer
            text_index = 0;
            clean_buffer(received_text, BUF_SIZE);
          }
        }
        // Si es carácter imprimible, agregar al buffer
        else if (c >= 32 && c <= 126 && text_index < BUF_SIZE - 1)
        {
          received_text[text_index++] = c;
        }
      }
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

#endif /* INCLUDE_MI_LIBRERIA_H_ */
