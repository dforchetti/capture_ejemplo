#ifndef INCLUDE_MI_LIBRERIA_H_
#define INCLUDE_MI_LIBRERIA_H_

#include "math.h"
#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"

using namespace std;

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

#endif /* INCLUDE_MI_LIBRERIA_H_ */
