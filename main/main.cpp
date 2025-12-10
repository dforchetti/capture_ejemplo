
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
  int gpio_num;
  int dpin;
  int cap_timer;
  int ID;
  int code;
  bool enable;
  int ui_mean;
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

#define PIN_DE_SALIDA_SENAL GPIO_NUM_1
#define DEBUG_PIN_1 GPIO_NUM_10
#define DEBUG_PIN_2 GPIO_NUM_11
#define DEBUG_PIN_3 GPIO_NUM_12
#define DEBUG_PIN_4 GPIO_NUM_13
#define N_CANALES 4
#define CAPTURE_PRESCALER 20;

#define PWM_RESOLUTION_HZ 80000000 // 80 MHz
#define DECIMACION 10000           // frec_reloj_filtro / CAPTURE_PRESCALER
#define ACTUALIZA 100              // frec_reloj_filtro / CAPTURE_PRESCALER

int frec_reloj_filtro = 150000;      // frecuencia de reloj del filtro en Hz
int periodo = 2 / frec_reloj_filtro; // periodo en us

modo estado_actual = ENCENDIDO;

const char *TAG = "capture";
const char *DATOS = "DATA";

mcpwm_cap_channel_handle_t cap_chan[4] = {NULL, NULL, NULL, NULL};
bool _simula_ = true;

data CANAL[N_CANALES];

bool led_state = false;
static bool debug_state = false;
volatile bool f_envioExitoso = true;
volatile bool f_calibra_media = false;
bool f_arraque = false;

mi_config_t config_default = {
    .CANAL = {
        {.gpio_num = 2, .dpin = DEBUG_PIN_1, .cap_timer = 0, .ID = 0, .code = 111, .enable = 1, .ui_mean = 0},
        {.gpio_num = 4, .dpin = DEBUG_PIN_2, .cap_timer = 0, .ID = 1, .code = 222, .enable = 1, .ui_mean = 0},
        {.gpio_num = 16, .dpin = DEBUG_PIN_3, .cap_timer = 1, .ID = 2, .code = 333, .enable = 1, .ui_mean = 0},
        {.gpio_num = 17, .dpin = DEBUG_PIN_4, .cap_timer = 1, .ID = 3, .code = 444, .enable = 1, .ui_mean = 0}},
    .modo_inicial = APAGADO,
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
  static int64_t t_anterior, t_actual;

  t_anterior = esp_timer_get_time();
  t_actual = t_anterior;

  config_GPIO();

  while (1)
  {
    t_actual = esp_timer_get_time();

    //    if (t_actual - t_anterior >= 60-30*debug_state) {
    //    if (t_actual - t_anterior >= 75-32*debug_state) {
    if (t_actual - t_anterior >= 750 - 320 * debug_state)
    {
      // if (t_actual - t_anterior >= 6000-3000*debug_state) {
      t_anterior = t_actual;
      debug_state = !debug_state;
      // gpio_set_level(DEBUG_PIN, debug_state);
      //  gpio_set_level(PIN_DE_SALIDA_SENAL, debug_state);
    }
    // gpio_set_level(DEBUG_PIN, 1);
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
  // Create a task to handler UART event from ISR
  xTaskCreate(uart_event_task, "uart_event_task", 10000, NULL, 12, NULL);

  xTaskCreate(task1,         // Función de la tarea
              "Task simula", // Nombre de la tarea
              10000,         // Tamaño del stack
              NULL,          // Parámetros
              1,             // Prioridad
              NULL           // Handle de la tarea
  );

  // debug();

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
  // config_capture();

  // Bucle principal
  //------------------------------------------------------------------------------------------

  ESP_LOGI(TAG, "Arrancanding CORE (%d)", xPortGetCoreID());

  bool estado = false;
  long tanterior = esp_timer_get_time();
  long tactual;

  /*printf("ACA...\n");
  while (1)
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }*/
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
      printf("ESTADO...%d\n", estado);

      /*
      if (estado)
      {
        mcpwm_capture_channel_enable(cap_chan[3]);
      }
      else
      {
        mcpwm_capture_channel_disable(cap_chan[3]);
      }
      */
    }
    /*
        estado = !estado;
        //   gpio_set_level(PIN_DE_SALIDA_SENAL, estado);
        contador++;
        printf("%d (%ld).\n", contador, (tactual - tanterior) / 1000);
        vTaskDelay(500 / portTICK_PERIOD_MS);*/
  }
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
int32_t delta;
int CH;

//------------------------------------------------------------------------------------------
static bool capture_callback(mcpwm_cap_channel_handle_t cap_chan,
                             const mcpwm_capture_event_data_t *edata,
                             void *user_data)
{

  static int cont = 0;
  static CaptureEvent event;

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

  delta = value - dato->t_anterior; //
  dato->t_anterior = value;

  dato->count[edge]++; // son dos contadores independientes para cada flanco

  // Almacena los valores maximos y minimos del periodo
  if (delta > dato->max[edge])
  {
    dato->max[edge] = delta;
  }
  if (delta < dato->min[edge])
  {
    dato->min[edge] = delta;
  }

  // ui_mean es un promedio entero para comparar en la isr

  if (delta >= (dato->ui_mean[edge] + dato->delta_max[edge])) // si DELTA > (MEDIA + UMBRAL) -> trigger MAXIMO++
  {
    dato->contador_disparos_max[edge]++;

    if (dato->contador_disparos_max[edge] >= dato->n_max_disparos[edge]) // si trigger MAXIMO alcanza el numero de disparos maximos
    {
      dato->flag_evento[edge] = true;
    }
  }
  else if (delta <= (dato->ui_mean[edge] - dato->delta_max[edge])) // si DELTA < (MEDIA - UMBRAL) -> trigger MINIMO++
  {

    dato->contador_disparos_min[edge]++;

    if (dato->contador_disparos_min[edge] >= dato->n_max_disparos[edge]) // si trigger MINIMO alcanza el numero de disparos maximos
    {
      dato->flag_evento[edge] = true;
    }
  }
  else // si no se supera ningun
  {
    dato->contador_disparos_min[edge] = 0;
    dato->contador_disparos_max[edge] = 0;
  }

  if (f_calibra_media == true)
  {

    dato->ui_mean_sum[edge] = dato->ui_mean_sum[edge] + delta;
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
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)CANAL[i].gpio_num, 0));
  }

  io_conf.pin_bit_mask = 1ULL << PIN_DE_SALIDA_SENAL;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  ESP_ERROR_CHECK(gpio_set_level(PIN_DE_SALIDA_SENAL, 0));
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

void config_mcpwm(void)
{
  ESP_LOGI(TAG, "Configurando Timer");

  mcpwm_timer_config_t c_timer;
  mcpwm_timer_handle_t h_timer = NULL;

  c_timer.group_id = 0;
  c_timer.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  c_timer.resolution_hz = PWM_RESOLUTION_HZ; // 100MHz, 0.1us per tick
  c_timer.intr_priority = 0;
  c_timer.count_mode = MCPWM_TIMER_COUNT_MODE_UP;

  c_timer.period_ticks = c_timer.resolution_hz / frec_reloj_filtro; // periodo en ticks

  int comparacion = c_timer.period_ticks / 2;

  ESP_ERROR_CHECK(mcpwm_new_timer(&c_timer, &h_timer));

  mcpwm_oper_handle_t h_operator = NULL;
  mcpwm_operator_config_t c_operator;
  c_operator.group_id = 0;
  c_operator.intr_priority = 0;

  ESP_LOGI(TAG, "Configurando Operador");
  ESP_ERROR_CHECK(mcpwm_new_operator(&c_operator, &h_operator));

  ESP_LOGI(TAG, "Connect timer and operator");
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(h_operator, h_timer));

  ESP_LOGI(TAG, "Configurando Comparador");

  mcpwm_cmpr_handle_t h_comparator = NULL;
  mcpwm_comparator_config_t c_comparator;
  c_comparator.intr_priority = 0;
  // c_comparator.flags.update_cmp_on_tep = 1;
  c_comparator.flags.update_cmp_on_tez = true;

  ESP_ERROR_CHECK(
      mcpwm_new_comparator(h_operator, &c_comparator, &h_comparator));

  ESP_LOGI(TAG, "Configurando Generador");

  mcpwm_gen_handle_t h_generator = NULL;
  mcpwm_generator_config_t c_generator;
  c_generator.gen_gpio_num = PIN_DE_SALIDA_SENAL; // GPIO_NUM_1;
  c_generator.flags.pull_up = true;

  // c_generator.flags.invert_pwm = 1;

  ESP_ERROR_CHECK(mcpwm_new_generator(h_operator, &c_generator, &h_generator));

  ESP_ERROR_CHECK(
      mcpwm_comparator_set_compare_value(h_comparator, comparacion));

  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
      h_generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                MCPWM_TIMER_EVENT_EMPTY,
                                                MCPWM_GEN_ACTION_HIGH)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
      h_generator,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, h_comparator,
                                     MCPWM_GEN_ACTION_LOW)));

  ESP_LOGI(TAG, "habilitando timers PWM");

  ESP_ERROR_CHECK(mcpwm_timer_enable(h_timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(h_timer, MCPWM_TIMER_START_NO_STOP));

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
             config->CANAL[i].ui_mean);
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
  }
  return;
}