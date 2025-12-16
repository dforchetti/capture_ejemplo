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

// Palabra específica a buscar
extern const char *TAG;
extern mcpwm_cap_timer_handle_t cap_timer[];
extern bool f_arraque;

class data
{

private:
  // const int TRIGG_MAX = 0;
  // const int TRIGG_MIN =0;
  const int N_disparos_MAX = 10;
  const int N_disparos_MIN = 10;
  const int ciclos_calibracion_default = 10;

public:
  //-----------------------------------------------------------------------------------
  // config
  //-----------------------------------------------------------------------------------
  gpio_num_t gpio_num;       // el GPIO asociado a la captura
  gpio_num_t dpin;           // el pin asociado al debug del GPIO
  int cap_timer;             // el timer asociado a la medicion de tiempo 0/1
  int ID;                    // un numero identificatorio 1,2,3,..etc
  int code;                  // un codigo identificatorio cualqueira 111,222, etc
  bool enable;               // canal habilitado/deshabilitado
  int prescaler;             // prescaler cada cuantos flanco quiero que se capture
  int ciclos_de_calibracion; // numero de ciclos para calibrar la media
  //-----------------------------------------------------------------------------------
  // trigger el indice 2 obedece a estadoUP, estado DOWN
  //-----------------------------------------------------------------------------------
  // uint32_t delta_trigger_max[2];     // umbral maximo de disparo
  // uint32_t delta_trigger_min[2];     // umbral maximo de disparo
  bool f_error;             // flag de error true/false desde la ultima vez que se controló
  int cont_errores;         // contador de errores en el procesamiento de los datos
  bool dir_flanco_anterior; // UP/DOWN
  bool dir_flanco_actual;   // UP/DOWN
  uint32_t t_anterior;
  uint32_t delta[2];
  uint32_t delta_max[2];        // error maximo de tiempo
  bool f_calibra[2];            // flag_para calibrar la media
  uint32_t contador_calibra[2]; // contador para la calibracion de la media

  int contador_disparos_max[2];   // contador histórico de disparos consecutivos
  int contador_disparos_min[2];   // contador histórico de disparos consecutivos
  int contador_error_disparos[2]; // si no se logra alcanzar el valor del
                                  // contador de disparos up, se resetea la
                                  // cuenta y se incrementa este contador
  int32_t max[2];                 // valor maximo desde el ultimo control
  int32_t min[2];                 // valor minimo desde el ultimo control
  int n_max_disparos[2];          // numero maximo de disparos para conseguir un evento
  bool flag_evento_max[2];
  bool flag_evento_min[2];

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
  void reinicia_variables(void);
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
  this->gpio_num = gpio_num_t(0);
  this->dpin = gpio_num_t(0);
  this->cap_timer = 0;
  this->ID = 0;
  this->code = 0;
  this->enable = false;
  this->prescaler = 1;
  this->ciclos_de_calibracion = ciclos_calibracion_default;

  this->f_error = false;
  this->cont_errores = 0;
  this->dir_flanco_anterior = false;
  this->dir_flanco_actual = false;

  this->t_anterior = 0;

  this->reinicia_variables();
}
void data::reinicia_variables(void)
{
  for (int i = 0; i < 2; i++)
  {
    this->delta[i] = 0;
    this->delta_max[i] = 0;
    this->contador_disparos_max[i] = 0;
    this->contador_disparos_min[i] = 0;
    this->contador_error_disparos[i] = 0;
    this->f_calibra[i] = false;
    this->contador_calibra[i] = 0;

    this->max[i] = 0;
    this->min[i] = 0x7FFFFFFF;
    this->n_max_disparos[i] = N_disparos_MAX;
    this->flag_evento_max[i] = false;
    this->flag_evento_min[i] = false;

    this->ui_mean[i] = 0;
    this->M2[i] = 0;
    this->ui_mean_sum[i] = 0;
    this->count[i] = 0;
  }
}

#endif /* INCLUDE_MI_LIBRERIA_H_ */
