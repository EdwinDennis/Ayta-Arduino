#ifndef  _BLE_MINI_H
#define _BLE_MINI_H

#include <Arduino.h>

void BLEMini_begin(unsigned long bound);
int recibiendo();
void enviarDato(unsigned char dat);
void enviarArregloDatos(unsigned char *arreglo, unsigned char longitud);
int leerDato();

#endif
