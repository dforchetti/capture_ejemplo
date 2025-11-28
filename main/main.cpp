
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

const static char *TAG = "capture";

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

int frec_reloj_filtro = 150000;      // frecuencia de reloj del filtro en Hz
int periodo = 2 / frec_reloj_filtro; // periodo en us

#define DECIMACION 100000 // frec_reloj_filtro / CAPTURE_PRESCALER
#define ACTUALIZA 1000    // frec_reloj_filtro / CAPTURE_PRESCALER

data CANAL[N_CANALES];

bool led_state = false;
static bool debug_state = false;
volatile bool f_envioExitoso = true;
//------------------------------------------------------------------------------------------

// funciones
//------------------------------------------------------------------------------------------
void debug(void);
void config_GPIO(void);
void config_capture(void);
void config_mcpwm(void);
void config_timer(void);
static void timer_callback(void *arg);
static bool capture_callback(mcpwm_cap_channel_handle_t cap_chan,
                             const mcpwm_capture_event_data_t *edata,
                             void *user_data);
//------------------------------------------------------------------------------------------

// Crear mutex
portMUX_TYPE spinlock_isr = portMUX_INITIALIZER_UNLOCKED;
QueueHandle_t xQueue = NULL;

// estructura para el intercambio de datos con la interrupcion
struct CaptureEvent
{
  bool edge;
  int cont;
  int M2[2];
  int n_muestras[2];
  int max[2];
  int min[2];
  float mean[2];

  gpio_num_t gpio_num;
};

struct CaptureEvent dato;

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

  xQueue = xQueueCreate(200, sizeof(struct CaptureEvent)); // cola para hasta 10 entero

  // struct CaptureEvent dato;

  // Crear primera tarea (LED)
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
  // config_GPIO();

  // capture channels
  //------------------------------------------------------------------------------------------
  config_capture();

  // Config TIMER
  //------------------------------------------------------------------------------------------
  // config_timer();

  // config PWM
  //------------------------------------------------------------------------------------------
  config_mcpwm();

  // config_GPIO();

  // Bucle principal
  //------------------------------------------------------------------------------------------

  ESP_LOGI(TAG, "Arrancanding CORE (%d)", xPortGetCoreID());

  while (1) //{vTaskDelay(10);}
  {
    /*

        //    if (xQueueReceive(xQueue, &dato, portMAX_DELAY)) {
        //    if (xQueueReceive(xQueue, &dato, 0)) {
          if (f_envioExitoso == false)
          {
            ESP_LOGE(TAG, "fallo en la cola del ISR");
            f_envioExitoso = true;
            }
*/
    if (xQueueReceiveFromISR(xQueue, &dato, 0))
    {

      dato.mean[0] = (float)(dato.M2[0] / dato.n_muestras[0] / 80.0); // 80 MHz de reloj
      dato.mean[1] = (float)(dato.M2[1] / dato.n_muestras[1] / 80.0); // 80 MHz de reloj

      ESP_LOGI(TAG, "<%i>, GPIO:%i DU(%6.2fu) MAX(%6.2f%%) MIN(%6.2f%%) N(%d), DD(%6.2fu) MAX(%6.2f%%) MIN(%6.2f%%) N(%d)", dato.cont, dato.gpio_num, dato.mean[0], 100.0 * (dato.max[0] / 80.0 - dato.mean[0]) / dato.mean[0], 100.0 * (dato.min[0] / 80.0 - dato.mean[0]) / dato.mean[0], dato.n_muestras[0], dato.mean[1], 100.0 * (dato.max[1] / 80.0 - dato.mean[1]) / dato.mean[1], 100.0 * (dato.min[1] / 80.0 - dato.mean[1]) / dato.mean[1], dato.n_muestras[1]);
      //   ESP_LOGI(TAG, "<%i>, GPIO:%i M2U: = %d NU %d, M2D: = %d ND %d", dato.cont, dato.gpio_num,dato.M2[0],dato.n_muestras[0],dato.M2[0],dato.n_muestras[1]);
    }
    vTaskDelay(500);
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

  // entrando a zona crítica
  portENTER_CRITICAL_ISR(&spinlock_isr);
  edge = edata->cap_edge;
  value = edata->cap_value;
  portEXIT_CRITICAL_ISR(&spinlock_isr);

  dato->dir_flanco_actual = edge;

  // para saltear los controles posteriores
  if (dato->count[edge] == 0)
  {

    dato->dir_flanco_anterior = !edge;
  }

  // contabiliza si hubo algun error en el patron de conmutaciones
  if (dato->dir_flanco_anterior == dato->dir_flanco_actual)
  {
    dato->cont_errores++;
    dato->f_error = true;
  }

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

  dato->ui_mean_sum[edge] = dato->ui_mean_sum[edge] + delta;

  if (dato->count[edge] == ACTUALIZA)
  {
    dato->ui_mean[edge] = dato->ui_mean_sum[edge] / dato->count[edge];
    dato->ui_mean_sum[edge] = 0;
    dato->count[edge] = 0;
  }

  cont++;

  if (cont == DECIMACION) // cada cierto numero de muestras envio un resumen
  {
    //  if (cont == 100) {

    event.cont++;
    event.edge = edata->cap_edge;
    event.gpio_num = (gpio_num_t)dato->code;

    event.M2[0] = dato->M2[0];
    event.M2[1] = dato->M2[1];
    event.n_muestras[0] = dato->count[0];
    event.n_muestras[1] = dato->count[1];
    event.max[0] = dato->max[0];
    event.max[1] = dato->max[1];
    event.min[0] = dato->min[0];
    event.min[1] = dato->min[1];

    dato->M2[0] = 0;
    dato->M2[1] = 0;
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

//------------------------------------------------------------------------------------------
void config_capture(void)
{
  ESP_LOGI(TAG, "Install capture timers");

  mcpwm_cap_timer_handle_t cap_timer[2] = {NULL, NULL};

  mcpwm_capture_timer_config_t cap_conf[2];

  // cap_conf[0].clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
  cap_conf[0].clk_src = MCPWM_CAPTURE_CLK_SRC_APB;
  cap_conf[0].group_id = 0;
  //  cap_conf[1].clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
  cap_conf[1].clk_src = MCPWM_CAPTURE_CLK_SRC_APB;
  cap_conf[1].group_id = 1;

  ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf[0], &cap_timer[0]));
  ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf[1], &cap_timer[1]));

  mcpwm_cap_channel_handle_t cap_chan[4] = {NULL, NULL, NULL, NULL};

  CANAL[0].gpio_num = 2;
  CANAL[0].dpin = DEBUG_PIN_1;
  CANAL[0].cap_timer = 0;
  CANAL[0].ID = 0;
  CANAL[0].code = 111;

  CANAL[1].gpio_num = 2;
  CANAL[1].dpin = DEBUG_PIN_2;
  CANAL[1].cap_timer = 0;
  CANAL[1].ID = 1;
  CANAL[1].code = 222;

  CANAL[2].gpio_num = 2;
  CANAL[2].dpin = DEBUG_PIN_3;
  CANAL[2].cap_timer = 1;
  CANAL[2].ID = 2;
  CANAL[2].code = 333;

  CANAL[3].gpio_num = 2;
  CANAL[3].dpin = DEBUG_PIN_4;
  CANAL[3].cap_timer = 1;
  CANAL[3].ID = 3;
  CANAL[3].code = 444;

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
  ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer[0]));
  ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer[1]));

  ESP_LOGI(TAG, "start timer");
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
void config_mcpwm(void)
{
  ESP_LOGI(TAG, "Configurando Timer");

  mcpwm_timer_handle_t h_timer = NULL;
  mcpwm_timer_config_t c_timer;

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
