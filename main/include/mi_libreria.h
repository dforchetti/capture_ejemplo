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

  float mean[2];
  uint32_t ui_mean[2]; // para poder comparar en la isr
  uint64_t M2[2];
  int count[2];

  //-----------------------------------------------------------------------------------
  // funciones
  //-----------------------------------------------------------------------------------
  // constructor
  data();
  // destructor
  ~data();

  void reset();
  void init_stats(void);
  float get_variance(int i);
  float get_std_dev(int i);
  void update_stats(int, float);
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
    this->mean[i] = 0;
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

void data::update_stats(int i, float new_value)
{
  this->count[i]++;
  float delta = new_value - this->mean[i];
  this->mean[i] += delta / this->count[i];
  float delta2 = new_value - this->mean[i];
  this->M2[i] += delta * delta2;
}

void data::init_stats(void)
{
  for (int i = 0; i < 2; i++)
  {
    this->mean[i] = 0.0;
    this->M2[i] = 0.0;
    this->count[i] = 0;
  }
}

float data::get_variance(int i)
{
  if (this->count[i] < 2)
    return 0.0;
  return this->M2[i] / (this->count[i] - 1); // Varianza muestral
};

float data::get_std_dev(int i)
{
  return sqrt(get_variance(i));
}
//----------------------------------

/*





typedef struct {
  //-----------------------------------------------------------------------------------
  // config
  //-----------------------------------------------------------------------------------
  int gpio_num;             // el GPIO asociado a la captura
  int cap_timer;            // el timer asociado a la medicion de tiempo 0/1
  int ID;                   // un numero identificatorio 1,2,3,..etc
  int code;                 // un codigo identificatorio cualqueira 111,222, etc
  int dir_flanco_actual;    // UP/DOWN
  bool dir_flanco_anterior; // UP/DOWN
  int cont_errores; // contador de errores en el procesamiento de los datos
  bool f_error; // flag de error true/false desde la ultima vez que se controló
  //-----------------------------------------------------------------------------------
  // trigger
  //-----------------------------------------------------------------------------------
  int delta_trigger_max[2];     // umbral maximo de disparo
  int delta_trigger_min[2];     // umbral maximo de disparo
  int contador_disparos_max[2]; // contador histórico de disparos consecutivos
  int contador_disparos_min[2]; // contador histórico de disparos consecutivos
  int contador_error_disparos[2]; // si no se logra alcanzar el valor del
                                  // contador de disparos up, se resetea la
                                  // cuenta y se incrementa este contador
  int max[2];                     // valor maximo desde el ultimo control
  int min[2];                     // valor minimo desde el ultimo control
  int n_max_disparos[2]; // numero maximo de disparos para conseguir un evento
  bool flag_evento[2];
  //-----------------------------------------------------------------------------------
  // estadísticas
  //-----------------------------------------------------------------------------------
  RunningStats stats; // datos para procesamiento estadísitico
  // funciones de procesamiento estadístico
  float (*get_var)(RunningStats *);
  float (*get_s)(RunningStats *);
  void (*init)(RunningStats *);
  void (*update_d)(RunningStats *, float);
  // void (*reset_data)(capture_user_data_t *,RunningStats *)
} capture_user_data_t;

void init_sruct(capture_user_data_t *elemento);

// calculos estadísticos
//-----------------------------------------------------------------------------

void init_sruct(capture_user_data_t *elemento) {
  elemento->get_var = &get_variance;
  elemento->get_s = &get_std_dev;
  elemento->init = &init_stats;
  elemento->update_d = &update_running_stats;

  elemento->init(&elemento->stats);
}

void reset(capture_user_data_t *elemento) {
  elemento->gpio_num = 0;
  elemento->cap_timer = 0;
  elemento->ID = 0;
  elemento->code = 0;
  elemento->dir_flanco_actual = 0;
  elemento->dir_flanco_anterior = 0;
  elemento->cont_errores = 0;
  elemento->f_error = 0;
  elemento->stats.count = 0;
  elemento->stats.M2 = 0;
  elemento->stats.mean = 0;
}

void init_stats(RunningStats *stats) {
  stats->mean = 0.0;
  stats->M2 = 0.0;
  stats->count = 0;
}

void update_running_stats(RunningStats *stats, float new_value) {
  stats->count++;
  float delta = new_value - stats->mean;
  stats->mean += delta / stats->count;
  float delta2 = new_value - stats->mean;
  stats->M2 += delta * delta2;
}

float get_variance(RunningStats *stats) {
  if (stats->count < 2)
    return 0.0;
  return stats->M2 / (stats->count - 1); // Varianza muestral
}

float get_std_dev(RunningStats *stats) { return sqrt(get_variance(stats)); }
//-----------------------------------------------------------------------------

int contador2 = 0;
*/
#endif /* INCLUDE_MI_LIBRERIA_H_ */
