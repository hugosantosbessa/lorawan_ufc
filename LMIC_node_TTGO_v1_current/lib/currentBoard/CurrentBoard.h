#ifndef CURRENT_BOARD_H
#define CURRENT_BOARD_H

#include "Wire.h"
#include <Arduino.h>


// ############### CONSTANTS ################# //

//General
const int PERIODICITY=5000; // Defined milliseconds


//Device-specific
#define I2Caddress 0x48
#define MARGEMDEERRO 200
#define HOURSTOMILLIS 3600000.0   //Valor de 1 hora em millisegundos
#define CORRENTEMAXIMA 52     //Corrente maxima enviada para o ads1115 convertida na proporção 100A:50mA
#define TENSAOMAXIMA 0.512    //Tensão maxima lida no ads1115

class CurrentBoard{
  public:
    float getElectricalConsumption();

  private:
    float consumoTotal = 0;
    unsigned long inicioTempoLigado =0;
    unsigned long fimTempoLigado =0;
    bool estaLigado = false;
    double kwValue;
    double maxValue = 0, correnteAtual = 0, maxValueAnt = 0;

    int i = 0;
    float multiplier = 0.000015625;
    
    int16_t convertedValue;
    int16_t actualValue[19];

    void deviceControl();
    void calculateConsumption();


};



#endif