//  en el cmd
// python -m esptool --chip esp32-s3 --port COM10 erase_flash
#include <stdio.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"
#include "include/mi_libreria.h"
#include "portmacro.h"
#include "soc/gpio_num.h"
#include "driver/uart.h"
#include "include/uart.h"
#include <nvs_flash.h>
#include <nvs.h>

// #define CONFIG_LOG_DEFAULT_LEVEL 0 // 1 #0 = NONE, 1 = ERROR, 2 = WARN, 3 = INFO, 4 = DEBUG, 5 = VERBOSE
//  Variables globales para NVS
//  nvs_handle_t nvs_handle;

#define GUARDAR_CONFIGURACION_NVS false

//------------------------------------------------------------------------------------------
// tipos de datos
//------------------------------------------------------------------------------------------
enum modo
{
  APAGADO = 0,
  ENCENDIDO = 1,
  PAUSA = 2,
  CALIBRA = 3,
  TEST = 4,
  ERROR = 5,
};

typedef struct
{
  gpio_num_t gpio_num;
  gpio_num_t dpin;
  gpio_num_t en_pin;
  gpio_num_t OC_pin;
  int cap_timer;
  int ID;
  int code;
  bool enable;
  int prescaler;
  uint32_t ui_mean;
  uint32_t delta_max;
} canal_config;

typedef struct
{
  canal_config CANAL[4];
  modo modo_inicial;
  int frec_emula;
  int contador;
} mi_config_t;

// estructura para el intercambio de datos con la interrupcion
struct CaptureEvent
{
  bool edge;
  int cont;
  int n_muestras[2];
  uint32_t max[2];
  uint32_t min[2];
  float mean[2];
  uint32_t ui_mean_sum[2];
  gpio_num_t gpio_num;
};

//------------------------------------------------------------------------------------------
// variable globales
//------------------------------------------------------------------------------------------
#define EXPERIMENTAL true

#if (EXPERIMENTAL)
#define PIN_DE_SALIDA_SENAL GPIO_NUM_22
#define PIN_DE_DEBUG GPIO_NUM_19

#define DEBUG_PIN_1 GPIO_NUM_18
#define DEBUG_PIN_2 GPIO_NUM_5
#define DEBUG_PIN_3 GPIO_NUM_17
#define DEBUG_PIN_4 GPIO_NUM_27

#define IN_PIN_1 GPIO_NUM_21
#define IN_PIN_2 GPIO_NUM_19
#define IN_PIN_3 GPIO_NUM_21
#define IN_PIN_4 GPIO_NUM_26

#define EN_PIN_1 GPIO_NUM_23
#define EN_PIN_2 GPIO_NUM_23
#define EN_PIN_3 GPIO_NUM_23
#define EN_PIN_4 GPIO_NUM_23

#define OC_PIN_1 GPIO_NUM_21
#define OC_PIN_2 GPIO_NUM_21
#define OC_PIN_3 GPIO_NUM_21
#define OC_PIN_4 GPIO_NUM_21

#else

#define PIN_DE_SALIDA_SENAL GPIO_NUM_1
#define PIN_DE_DEBUG GPIO_NUM_3

#define DEBUG_PIN_1 GPIO_NUM_10
#define DEBUG_PIN_2 GPIO_NUM_11
#define DEBUG_PIN_3 GPIO_NUM_12
#define DEBUG_PIN_4 GPIO_NUM_13

#define IN_PIN_1 GPIO_NUM_2
#define IN_PIN_2 GPIO_NUM_2
#define IN_PIN_3 GPIO_NUM_2
#define IN_PIN_4 GPIO_NUM_2

#define EN_PIN_1 GPIO_NUM_4
#define EN_PIN_2 GPIO_NUM_4
#define EN_PIN_3 GPIO_NUM_4
#define EN_PIN_4 GPIO_NUM_4

#define OC_PIN_1 GPIO_NUM_15
#define OC_PIN_2 GPIO_NUM_15
#define OC_PIN_3 GPIO_NUM_15
#define OC_PIN_4 GPIO_NUM_15

#endif

#define N_CANALES 4
#define CAPTURE_PRESCALER 1

const int PWM_RESOLUTION_HZ = 80000000; // 80 MHz
#define DECIMACION 10000                // frec_reloj_filtro / CAPTURE_PRESCALER
#define ACTUALIZA 100                   // frec_reloj_filtro / CAPTURE_PRESCALER

uint32_t frec_reloj_filtro = 150000; // frecuencia de reloj del filtro en Hz
int32_t duty = 50;                   // duty cycle en %
bool fcalibra = false;

modo estado_actual = ENCENDIDO;

const char *TAG = "capture";
const char *DATOS = "DATA";

mcpwm_cap_channel_handle_t cap_chan[4] = {NULL, NULL, NULL, NULL};
bool _simula_ = true;
bool f_calibra = false;

data CANAL[N_CANALES];

bool led_state = false;
volatile bool f_envioExitoso = true;
bool f_arraque = false;
volatile int cont_default = 0;

mi_config_t config_default = {
    .CANAL = {
        {.gpio_num = IN_PIN_1, .dpin = DEBUG_PIN_1, .en_pin = EN_PIN_1, .OC_pin = OC_PIN_1, .cap_timer = 0, .ID = 0, .code = 111, .enable = 1, .prescaler = CAPTURE_PRESCALER, .ui_mean = 40000000, .delta_max = 100000},
        {.gpio_num = IN_PIN_2, .dpin = DEBUG_PIN_2, .en_pin = EN_PIN_2, .OC_pin = OC_PIN_2, .cap_timer = 0, .ID = 1, .code = 222, .enable = 1, .prescaler = CAPTURE_PRESCALER, .ui_mean = 40000000, .delta_max = 100000},
        {.gpio_num = IN_PIN_3, .dpin = DEBUG_PIN_3, .en_pin = EN_PIN_3, .OC_pin = OC_PIN_3, .cap_timer = 1, .ID = 2, .code = 333, .enable = 1, .prescaler = CAPTURE_PRESCALER, .ui_mean = 40000000, .delta_max = 100000},
        {.gpio_num = IN_PIN_4, .dpin = DEBUG_PIN_4, .en_pin = EN_PIN_4, .OC_pin = OC_PIN_4, .cap_timer = 1, .ID = 3, .code = 444, .enable = 1, .prescaler = CAPTURE_PRESCALER, .ui_mean = 40000000, .delta_max = 100000}},
    .modo_inicial = ENCENDIDO,
    .frec_emula = 150000,
    .contador = 0};

struct CaptureEvent dato;

// Crear mutex
portMUX_TYPE spinlock_isr = portMUX_INITIALIZER_UNLOCKED;
QueueHandle_t xQueue = NULL;

//------------------------------------------------------------------------------------------
// funciones
//------------------------------------------------------------------------------------------
void debug(void);
void config_GPIO(void);
void config_capture(void);
void config_mcpwm(void);
void config_timer(void);
void start_capture_timer(void);
void stop_capture_timer(void);
void init_nvs();
esp_err_t guardar_completo_nvs(mi_config_t *config);
esp_err_t leer_completo_nvs(mi_config_t *config);
void inicia_variables_globales(mi_config_t *config);
void muestra_configuracion_nvs(mi_config_t *config);

static void timer_callback(void *arg);
static bool capture_callback(mcpwm_cap_channel_handle_t cap_chan,
                             const mcpwm_capture_event_data_t *edata,
                             void *user_data);
//------------------------------------------------------------------------------------------

//
void task1(void *parameter)
{
  long tactual;
  long tanterior = esp_timer_get_time();
  bool estado = false;

  while (1)
  {

    tactual = esp_timer_get_time();

    if (tactual - tanterior >= (1000000) * (duty * estado + (100 - duty) * (!estado)) / 100)
    {

      tanterior = tactual;
      estado = !estado;
      // gpio_set_level(PIN_DE_SALIDA_SENAL, estado);
      // printf("TASK1...%d\n", estado);
    }
  }
}

extern "C" void app_main(void)
{

  esp_task_wdt_deinit(); // funciona para deshabilitar el WDT del freertos

  init_nvs();
  /*
    if (GUARDAR_CONFIGURACION_NVS)
    {
      ESP_ERROR_CHECK(guardar_completo_nvs(&config_default));
      printf("Guardado!\n");
      inicia_variables_globales(&config_default);
    }
    else
    {
      mi_config_t config_leida;
      ESP_ERROR_CHECK(leer_completo_nvs(&config_leida));
      muestra_configuracion_nvs(&config_leida);
      inicia_variables_globales(&config_leida);
    }
    */
  inicia_variables_globales(&config_default);

  xQueue = xQueueCreate(200, sizeof(struct CaptureEvent)); // cola para hasta 10 entero

  configura_uart();

  xTaskCreate(uart_event_task, "uart_event_task", 10000, NULL, 12, NULL);
  xTaskCreate(task1, "Task simula", 10000, NULL, 1, NULL);

  // config GPIO
  //------------------------------------------------------------------------------------------
  config_GPIO();

  // Config TIMER
  //------------------------------------------------------------------------------------------
  // config_timer();

  // config PWM
  //------------------------------------------------------------------------------------------
  config_mcpwm();

  // capture channels
  //------------------------------------------------------------------------------------------
  config_capture();

  // Bucle principal
  //------------------------------------------------------------------------------------------

  ESP_LOGI(TAG, "Arrancanding CORE (%d)", xPortGetCoreID());

  bool estado = false;
  long tanterior = esp_timer_get_time();
  long tactual;

  while (1) //{vTaskDelay(10);}
  {
    if (xQueueReceiveFromISR(xQueue, &dato, 0))
    {

      dato.mean[0] = (float)(dato.ui_mean_sum[0] / dato.n_muestras[0] / 80.0); // 80 MHz de reloj
      dato.mean[1] = (float)(dato.ui_mean_sum[1] / dato.n_muestras[1] / 80.0); // 80 MHz de reloj

      // ESP_LOGI(TAG, "<%i>, GPIO:%i DU(%6.2fu) MAX(%6.2f%%) MIN(%6.2f%%) N(%d), DD(%6.2fu) MAX(%6.2f%%) MIN(%6.2f%%) N(%d)", dato.cont, dato.gpio_num, dato.mean[0], 100.0 * (dato.max[0] / 80.0 - dato.mean[0]) / dato.mean[0], 100.0 * (dato.min[0] / 80.0 - dato.mean[0]) / dato.mean[0], dato.n_muestras[0], dato.mean[1], 100.0 * (dato.max[1] / 80.0 - dato.mean[1]) / dato.mean[1], 100.0 * (dato.min[1] / 80.0 - dato.mean[1]) / dato.mean[1], dato.n_muestras[1]);
      ESP_LOGI(DATOS, "<%i>, GPIO:%i M:%2.6f NU %d, M:%2.6f ND %d", dato.cont, dato.gpio_num, dato.mean[0], dato.n_muestras[0], dato.mean[0], dato.n_muestras[1]);
    }

    tactual = esp_timer_get_time();
    if (tactual - tanterior >= 1000000)
    {
      tanterior = tactual;
      estado = !estado;

      if (!fcalibra)
      {

        //"\033[0;" COLOR "m"
        int i = 0;

        printf(">");
        for (i = 0; i < N_CANALES; i++)
        {
          if (CANAL[i].enable)
          {
            printf(" (%d/%d)", CANAL[i].count[0], CANAL[i].count[1]);
          }
        }
        printf("\n>");

        printf(">     ");
        for (i = 0; i < N_CANALES; i++)
        {
          if (CANAL[i].enable)
          {
            printf(" (%ld/%ld)", CANAL[i].delta[0], CANAL[i].delta[1]);
          }
        }
        printf("\n>");

        printf("> mean ");
        for (i = 0; i < N_CANALES; i++)
        {
          if (CANAL[i].enable)
          {
            printf(" (%ld/%ld)", CANAL[i].ui_mean[0], CANAL[i].ui_mean[1]);
          }
        }
        printf("\n>");

        printf("> delta _medida");
        for (i = 0; i < N_CANALES; i++)
        {
          if (CANAL[i].enable)
          {
            printf(" (%ld/%ld)", (long)(CANAL[i].delta[0] - CANAL[i].ui_mean[0]), (long)(CANAL[i].delta[1] - CANAL[i].ui_mean[1]));
          }
        }
        printf("\n>");

        printf("> MAX");
        for (i = 0; i < N_CANALES; i++)
        {
          if (CANAL[i].enable)
          {
            printf(" (%d/%d)", CANAL[i].contador_disparos_max[0], CANAL[i].contador_disparos_max[1]);
          }
        }
        printf("\n>");

        printf("> MIN");
        for (i = 0; i < N_CANALES; i++)
        {
          if (CANAL[i].enable)
          {
            printf(" (%d/%d)", CANAL[i].contador_disparos_min[0], CANAL[i].contador_disparos_min[1]);
          }
        }
        printf("\n>");

        printf("-----------------------------------------------------------------\n");
      }

      else
      {

        printf("Calibrando...\n");

        // printf("Canal %d Edge %d Calibrado: Nueva media = %lu\n", dato->code, edge, dato->ui_mean[edge]);
      }
    }

    /*
        estado = !estado;
        //   gpio_set_level(PIN_DE_SALIDA_SENAL, estado);
        contador++;
        printf("%d (%ld).\n", contador, (tactual - tanterior) / 1000);
        vTaskDelay(500 / portTICK_PERIOD_MS);*/
  }
  vTaskDelay(10);
  //------------------------------------------------------------------------------------------
}

//------------------------------------------------------------------------------------------
static void timer_callback(void *arg)
{
  led_state = !led_state;
  // gpio_set_level(PIN_DE_SALIDA_SENAL, led_state);
  gpio_set_level(DEBUG_PIN_1, 1);
  gpio_set_level(DEBUG_PIN_1, 0);
}

//------------------------------------------------------------------------------------------

volatile bool flag_evento = false;
uint32_t tinicial;
int edge;
uint32_t value;
// int32_t delta;
int CH;

//------------------------------------------------------------------------------------------
static bool capture_callback(mcpwm_cap_channel_handle_t cap_chan,
                             const mcpwm_capture_event_data_t *edata,
                             void *user_data)
{

  static int cont = 0;
  static CaptureEvent event;

  cont_default++;

  data *dato = (data *)user_data; // dato apunta para cada interrupción a su estructura de datos CANAL[x]

  gpio_set_level((gpio_num_t)dato->dpin, 1);

  // es un segundo control por las dudas
  if (!dato->enable)
  {
    gpio_set_level((gpio_num_t)dato->dpin, 0);
    return true;
  }

  // entrando a zona crítica
  portENTER_CRITICAL_ISR(&spinlock_isr);
  edge = edata->cap_edge;
  value = edata->cap_value;
  portEXIT_CRITICAL_ISR(&spinlock_isr);

  dato->dir_flanco_actual = edge;

  // para saltear las cuentas si es la primera vez
  if (dato->count[edge] == 0)
  {
    dato->count[edge]++; // inicializo el contador de este flanco

    if (dato->count[!edge] == 0)
    {
      dato->t_anterior = value;
      dato->dir_flanco_anterior = edge;
      // gpio_set_level((gpio_num_t)dato->dpin, 0);
      // gpio_set_level((gpio_num_t)dato->dpin, 1);
      gpio_set_level((gpio_num_t)dato->dpin, 0);
      gpio_set_level((gpio_num_t)dato->dpin, 1);
      gpio_set_level((gpio_num_t)dato->dpin, 0);
      // gpio_set_level(GPIO_NUM_10, 0);
      return true;
    }
  }

  // contabiliza si hubo algun error en el patron de conmutaciones
  if (dato->dir_flanco_anterior == dato->dir_flanco_actual)
  {
    dato->cont_errores++;
    dato->f_error = true;
    // hace un doble parpadeo
    gpio_set_level((gpio_num_t)dato->dpin, 0);
    gpio_set_level((gpio_num_t)dato->dpin, 1);
    gpio_set_level((gpio_num_t)dato->dpin, 0);
    gpio_set_level((gpio_num_t)dato->dpin, 1);
    gpio_set_level((gpio_num_t)dato->dpin, 0);
    return true;
  }

  dato->dir_flanco_anterior = dato->dir_flanco_actual;

  dato->delta[edge] = value - dato->t_anterior; //
  dato->t_anterior = value;

  dato->count[edge]++; // son dos contadores independientes para cada flanco

  // Almacena los valores maximos y minimos del periodo
  if (dato->delta[edge] > dato->max[edge])
  {
    dato->max[edge] = dato->delta[edge];
  }
  if (dato->delta[edge] < dato->min[edge])
  {
    dato->min[edge] = dato->delta[edge];
  }

  if (dato->f_calibra[edge] == true)
  {
    dato->contador_calibra[edge]++;
    // dato->ui_mean_sum[edge] = dato->ui_mean_sum[edge] + 40000000; // dato->delta[edge];
    dato->ui_mean_sum[edge] = dato->ui_mean_sum[edge] + dato->delta[edge];

    if (dato->contador_calibra[edge] >= dato->ciclos_de_calibracion)
    {
      dato->ui_mean[edge] = (uint32_t)(dato->ui_mean_sum[edge] / dato->contador_calibra[edge]);
      dato->ui_mean_sum[edge] = 0;
      dato->contador_calibra[edge] = 0;
      dato->f_calibra[edge] = false;
      // printf("Canal %d Edge %d Calibrado: Nueva media = %lu\n", dato->code, edge, dato->ui_mean[edge]);
      fcalibra = false;
    }
  }
  else
  {

    if (dato->delta[edge] >= (dato->ui_mean[edge] + dato->delta_max[edge])) // si DELTA > (MEDIA + UMBRAL) -> trigger MAXIMO++
    {
      dato->contador_disparos_max[edge]++;

      if (dato->contador_disparos_max[edge] >= dato->n_max_disparos[edge]) // si trigger MAXIMO alcanza el numero de disparos maximos
      {
        dato->flag_evento_max[edge] = true;
        dato->contador_disparos_max[edge] = dato->n_max_disparos[edge]; // para que no siga incrementando
      }
    }
    else if (dato->delta[edge] <= (dato->ui_mean[edge] - dato->delta_max[edge])) // si DELTA < (MEDIA - UMBRAL) -> trigger MINIMO++
    {
      dato->contador_disparos_min[edge]++;

      if (dato->contador_disparos_min[edge] >= dato->n_max_disparos[edge]) // si trigger MINIMO alcanza el numero de disparos maximos
      {
        dato->flag_evento_min[edge] = true;
        dato->contador_disparos_min[edge] = dato->n_max_disparos[edge]; // para que no siga incrementando
      }
    }
    else // si no se supera ningun límite se implementa una estrategia tipo anti-reset_wind-up
    {
      if (dato->contador_disparos_min[edge] > 0)
      {
        dato->contador_disparos_min[edge]--;
        if (dato->contador_disparos_min[edge] < 0)
        {
          dato->contador_disparos_min[edge] = 0;
          dato->flag_evento_min[edge] = false;
        }
      }
      if (dato->contador_disparos_max[edge] > 0)
      {
        dato->contador_disparos_max[edge]--;

        if (dato->contador_disparos_max[edge] < 0)
        {
          dato->contador_disparos_max[edge] = 0;
          dato->flag_evento_max[edge] = false;
        }
      }
    }
  }
  cont++;

  if (cont == DECIMACION) // cada cierto numero de muestras envio un resumen
  {
    //  if (cont == 100) {

    event.cont++;
    event.edge = edata->cap_edge;
    event.gpio_num = (gpio_num_t)dato->code;

    event.n_muestras[0] = dato->count[0];
    event.n_muestras[1] = dato->count[1];
    event.ui_mean_sum[0] = dato->ui_mean_sum[0];
    event.ui_mean_sum[1] = dato->ui_mean_sum[1];
    event.max[0] = dato->max[0];
    event.max[1] = dato->max[1];
    event.min[0] = dato->min[0];
    event.min[1] = dato->min[1];

    dato->count[0] = 0;
    dato->count[1] = 0;
    dato->max[0] = 0;
    dato->max[1] = 0;
    dato->min[0] = 0x7FFFFFFF;
    dato->min[1] = 0x7FFFFFFF;

    cont = 0;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    f_envioExitoso = xQueueSendFromISR(xQueue, &event, &xHigherPriorityTaskWoken); // dato faltante == true si no pudo enviar el dato en la cola
  }
  /*
    if (dato->count[edge] == ACTUALIZA)
    {
      dato->ui_mean[edge] = dato->ui_mean_sum[edge] / dato->count[edge];
      dato->ui_mean_sum[edge] = 0;
      dato->count[edge] = 0;
    }
  */
  gpio_set_level((gpio_num_t)dato->dpin, 0);
  // gpio_set_level(GPIO_NUM_10, 0);
  return true;
}
//------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------
void debug(void)
{

  for (int i = 0; i < N_CANALES; i++)
  {
    CANAL[i].reset();

    //  ESP_LOGI(TAG, "%d, %f %i %f\n\r", i, CANAL[i].stats.mean,
    //           CANAL[i].stats.count, CANAL[i].stats.M2);
  }

  // voy a inventar unas pruebas
  // Vector de 30 enteros con media ≈5 y desviación estándar ≈2
  /*float datos[30] = {3, 4, 5, 6, 7, 2, 3, 4, 5, 6, 4, 5, 6, 7, 8, 1, 2, 3, 4,
  5, 6, 7, 8, 9, 3, 4, 5, 6, 7, 4};

  for(int i= 0; i<30;i++){
          CANAL[0].update_d(&CANAL[0].stats,datos[i]);
  }
  int i = 0;

  ESP_LOGI(TAG, "CH:%d, MEDIA:%f  #datos:%i
  s:%f\n\r",i,CANAL[i].stats.mean,CANAL[i].stats.count,CANAL[i].get_s(&CANAL[i].stats)
  );

  while(1){
          vTaskDelay(100);
  }*/
}
//------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------
void config_GPIO(void)
{

  ESP_LOGI(TAG, "Configurando PIN DE DEBUG");

  gpio_config_t io_conf = {};

  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.intr_type = GPIO_INTR_DISABLE;

  for (int i = 0; i < N_CANALES; i++)
  {
    io_conf.pin_bit_mask = 1ULL << CANAL[i].gpio_num;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(CANAL[i].gpio_num, 0));
  }

  io_conf.pin_bit_mask = 1ULL << PIN_DE_SALIDA_SENAL;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  ESP_ERROR_CHECK(gpio_set_level(PIN_DE_SALIDA_SENAL, 0));

  io_conf.pin_bit_mask = 1ULL << PIN_DE_DEBUG;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  ESP_ERROR_CHECK(gpio_set_level(PIN_DE_DEBUG, 0));

  // habria que ver si puedo sacar todo este codigo repetido

  io_conf.pin_bit_mask = 1ULL << DEBUG_PIN_1;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  ESP_ERROR_CHECK(gpio_set_level(DEBUG_PIN_1, 0));

  io_conf.pin_bit_mask = 1ULL << DEBUG_PIN_2;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  ESP_ERROR_CHECK(gpio_set_level(DEBUG_PIN_2, 0));

  io_conf.pin_bit_mask = 1ULL << DEBUG_PIN_3;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  ESP_ERROR_CHECK(gpio_set_level(DEBUG_PIN_3, 0));

  io_conf.pin_bit_mask = 1ULL << DEBUG_PIN_4;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  ESP_ERROR_CHECK(gpio_set_level(DEBUG_PIN_4, 0));
}
//------------------------------------------------------------------------------------------

mcpwm_cap_timer_handle_t cap_timer[2] = {NULL, NULL};

//------------------------------------------------------------------------------------------
void config_capture(void)
{
  ESP_LOGI(TAG, "Install capture timers");

  // mcpwm_cap_timer_handle_t cap_timer[2] = {NULL, NULL};

  mcpwm_capture_timer_config_t cap_conf[2];

  // cap_conf[0].clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
  cap_conf[0].clk_src = MCPWM_CAPTURE_CLK_SRC_APB;
  cap_conf[0].group_id = 0;
  //  cap_conf[1].clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
  cap_conf[1].clk_src = MCPWM_CAPTURE_CLK_SRC_APB;
  cap_conf[1].group_id = 1;

  ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf[0], &cap_timer[0]));
  ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf[1], &cap_timer[1]));

  mcpwm_capture_channel_config_t cap_ch_conf;

  cap_ch_conf.gpio_num = 0;
  // capture on both edge
  cap_ch_conf.flags.neg_edge = true;
  cap_ch_conf.flags.pos_edge = true;

  cap_ch_conf.prescale = CAPTURE_PRESCALER;

  // pull up internally
  // cap_ch_conf.flags.pull_up = true;
  cap_ch_conf.intr_priority = 0;

  mcpwm_capture_event_callbacks_t cbs;

  cbs.on_cap = capture_callback;

  for (int ch = 0; ch < 4; ch++)
  {

    ESP_LOGI(TAG, "Install capture channel %i", ch);
    cap_ch_conf.gpio_num = CANAL[ch].gpio_num;

    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer[CANAL[ch].cap_timer],
                                              &cap_ch_conf, &cap_chan[ch]));

    ESP_LOGI(TAG, "Register capture callback %i", ch);

    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(
        cap_chan[ch], &cbs, &CANAL[ch]));

    ESP_LOGI(TAG, "Enable capture channel %i", ch);
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan[ch]));
  }
  //----------------------------------------------------------------------------------------------

  ESP_LOGI(TAG, "Enable capture timer");
  ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer[0]));
  ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer[1]));

  ESP_LOGI(TAG, "start capture timer");

  if (estado_actual == ENCENDIDO)
  {
    start_capture_timer();
    ESP_LOGI(TAG, "CAPTURA encendido");
  }
  else
  {
    stop_capture_timer();
    ESP_LOGI(TAG, "CAPTURA apagada");
  }
}
//------------------------------------------------------------------------------------------

// start capture timer
//------------------------------------------------------------------------------------------
void start_capture_timer(void)
{
  ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer[0]));
  ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer[1]));

  for (int i = 0; i < 4; i++)
  {
    CANAL[i].count[0] = 0;
    CANAL[i].count[1] = 0;
    CANAL[i].dir_flanco_anterior = 0;
    CANAL[i].dir_flanco_actual = 0;
    CANAL[i].cont_errores = 0;
    CANAL[i].f_error = false;
    CANAL[i].t_anterior = 0;
    CANAL[i].max[0] = 0;
    CANAL[i].max[1] = 0;
    CANAL[i].min[0] = 0x7FFFFFFF;
    CANAL[i].min[1] = 0x7FFFFFFF;
  }
}
//------------------------------------------------------------------------------------------

// stop capture timer
//------------------------------------------------------------------------------------------
void stop_capture_timer(void)
{
  ESP_ERROR_CHECK(mcpwm_capture_timer_stop(cap_timer[0]));
  ESP_ERROR_CHECK(mcpwm_capture_timer_stop(cap_timer[1]));
}
//------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------
void config_timer(void)
{

  // Create periodic timer with 1000ms (1Hz)
  esp_timer_create_args_t timer_args;

  timer_args.name = "led_timer";
  timer_args.callback = &timer_callback;
  timer_args.dispatch_method = ESP_TIMER_TASK;
  timer_args.arg = NULL;

  esp_timer_handle_t timer_handle = NULL;

  esp_timer_create(&timer_args, &timer_handle);
  esp_timer_start_periodic(timer_handle, 100); // 1,000,000 us = 1 second
}

//------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------

// mcpwm_timer_handle_t h_timer = NULL;

mcpwm_timer_handle_t h_timer = NULL;
mcpwm_cmpr_handle_t h_comparator = NULL;
mcpwm_timer_config_t timer_config = {};

void config_mcpwm(void)
{

  timer_config.group_id = 0;
  timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  timer_config.resolution_hz = PWM_RESOLUTION_HZ;
  timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  timer_config.intr_priority = 0;

  timer_config.period_ticks = PWM_RESOLUTION_HZ / frec_reloj_filtro; // periodo en ticks

  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &h_timer));

  // operador
  mcpwm_oper_handle_t oper;
  mcpwm_operator_config_t operator_config = {.group_id = 0};
  mcpwm_new_operator(&operator_config, &oper);
  mcpwm_operator_connect_timer(oper, h_timer);

  // Comparador
  mcpwm_comparator_config_t comparator_config = {};
  comparator_config.flags.update_cmp_on_tez = true;
  mcpwm_new_comparator(oper, &comparator_config, &h_comparator);

  // Generador
  mcpwm_gen_handle_t gen;
  mcpwm_generator_config_t gen_config = {
      .gen_gpio_num = PIN_DE_SALIDA_SENAL,
      .flags =
          {
              .invert_pwm = false,
              .io_loop_back = false,
              .io_od_mode = false,
              .pull_up = true,
              .pull_down = false}};

  mcpwm_new_generator(oper, &gen_config, &gen);

  mcpwm_comparator_set_compare_value(h_comparator, (timer_config.period_ticks * duty) / 100);
  // mcpwm_comparator_set_compare_value(h_comparator, 500000); // 50% duty
  //  Configurar acciones
  mcpwm_generator_set_action_on_timer_event(gen,
                                            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                                         MCPWM_TIMER_EVENT_EMPTY,
                                                                         MCPWM_GEN_ACTION_HIGH));

  mcpwm_generator_set_action_on_compare_event(gen,
                                              MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                                             h_comparator,
                                                                             MCPWM_GEN_ACTION_LOW));

  // Iniciar
  mcpwm_timer_enable(h_timer);
  mcpwm_timer_start_stop(h_timer, MCPWM_TIMER_START_NO_STOP);

  // mcpwm_timer_disable(timer_pwm);
  //  si lo quiero borrar
  // mcpwm_del_timer(timer_pwm);

  // si le quiero cambiar el periodo
  // mcpwm_timer_set_period(timer, period_ticks)
}

void init_nvs()
// Inicializar NVS
{
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
}

esp_err_t guardar_completo_nvs(mi_config_t *config)
{
  nvs_handle_t handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
  if (err != ESP_OK)
    return err;

  // Guardar toda la estructura como un blob
  err = nvs_set_blob(handle, "config", config, sizeof(mi_config_t));
  if (err == ESP_OK)
  {
    err = nvs_commit(handle);
  }
  nvs_close(handle);
  return err;
}

esp_err_t leer_completo_nvs(mi_config_t *config)
{
  nvs_handle_t handle;
  esp_err_t err = nvs_open("storage", NVS_READONLY, &handle);
  if (err != ESP_OK)
    return err;

  size_t longitud = sizeof(mi_config_t);
  err = nvs_get_blob(handle, "config", config, &longitud);
  nvs_close(handle);
  return err;
}

void muestra_configuracion_nvs(mi_config_t *config)
{
  ESP_LOGI(TAG, "Configuracion leida de NVS:");
  for (int i = 0; i < N_CANALES; i++)
  {
    ESP_LOGI(TAG, "CANAL %d: GPIO=%d, DPIN=%d, CAP_TIMER=%d, ID=%d, CODE=%d, ENABLE=%d, UI_MEAN=%d",
             i,
             config->CANAL[i].gpio_num,
             config->CANAL[i].dpin,
             config->CANAL[i].cap_timer,
             config->CANAL[i].ID,
             config->CANAL[i].code,
             config->CANAL[i].enable,
             (int)config->CANAL[i].ui_mean);
  }
  ESP_LOGI(TAG, "MODO_INICIAL=%d", config->modo_inicial);
  ESP_LOGI(TAG, "FREC_EMULA=%d", config->frec_emula);
  ESP_LOGI(TAG, "CONTADOR=%d", config->contador);

  return;
}

void inicia_variables_globales(mi_config_t *config)
{
  for (int i = 0; i < N_CANALES; i++)
  {
    CANAL[i].code = config->CANAL[i].code;
    CANAL[i].dpin = config->CANAL[i].dpin;
    CANAL[i].gpio_num = config->CANAL[i].gpio_num;
    CANAL[i].cap_timer = config->CANAL[i].cap_timer;
    CANAL[i].ID = config->CANAL[i].ID;
    CANAL[i].enable = config->CANAL[i].enable;
    CANAL[i].ui_mean[0] = config->CANAL[i].ui_mean;
    CANAL[i].ui_mean[1] = config->CANAL[i].ui_mean;
    CANAL[i].prescaler = config->CANAL[i].prescaler;
    CANAL[i].delta_max[0] = config->CANAL[i].delta_max; // 10% de tolerancia
    CANAL[i].delta_max[1] = config->CANAL[i].delta_max;
  }
  return;
}