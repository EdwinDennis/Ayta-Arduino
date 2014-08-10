
#include "ble_mini.h"

// UNO
// TX: pin 1
// RX: pin 0
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328P__)
	#define BLEMINI Serial

// other board
// board					TX			RX
// Leonardo			        1			0
// MEGA			    		18			19
// DUE						18			19
#else
	#define BLEMINI Serial1
#endif

void BLEMini_begin(unsigned long bound)
{
		BLEMINI.begin(bound);
}

int recibiendo()
{
		return BLEMINI.available();
}

void enviarDato(unsigned char dat)
{
		BLEMINI.write(dat);
}


/* Escribe datos binarios al puerto serie. Estos datos se envían como un byte o una serie de bytes
    arreglo -> arreglo para enviar como una serie de bytes
    longitud -> longitud de la memoria intermedia
*/
void enviarArregloDatos(unsigned char *arreglo, unsigned char longitud)
{
		delay(10);
		if(longitud > 0)
			BLEMINI.write(arreglo, longitud);
}

int leerDato()
{
	delay(10);
	return BLEMINI.read();
}
