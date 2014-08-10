/*
 *  When use DUE board, the Arduino IDE should be the version of 1.5.4 or above.
 *  Board       BLEMini(TX, RX) 
 *  DUE          (18, 19)
 *  MEGA         (18, 19)
 *  UNO          (1, 0)
 *  LEONARDO     (1, 0)
 */

#include <Servo.h>
#include "Boards.h"
#include <ble_mini.h>

#define PROTOCOL_MAJOR_VERSION   0 //
#define PROTOCOL_MINOR_VERSION   0 //
#define PROTOCOL_BUGFIX_VERSION  2 // bugfix

#define PIN_CAPABILITY_NONE      0x00
#define PIN_CAPABILITY_DIGITAL   0x01
#define PIN_CAPABILITY_ANALOG    0x02
#define PIN_CAPABILITY_PWM       0x04
#define PIN_CAPABILITY_SERVO     0x08
#define PIN_CAPABILITY_I2C       0x10

// modos de pin y asignacion de direcciones

#define ANALOG                  0x02 // pin analogico en modo analogInput
#define PWM                     0x03 //  pin digital en modo de salida PWM 
#define SERVO                   0x04 // pin digital en modo de salida Servo

byte modosPin[TOTAL_PINS];
byte estadosPin[TOTAL_PINS];
byte pin_pwm[TOTAL_PINS];
byte pin_servo[TOTAL_PINS];

Servo servos[MAX_SERVOS];

/* variables del temporizador */
unsigned long valorActualMilisegundos;        // almacena el valor actual de millis ()
unsigned long compararMilisegundos;       // para comparacion con  valorActualMilisegundos
int frecuenciaEjecucion = 5;          // la frecuencia para ejecutar el bucle principal (en ms)

void setup()
{
  BLEMini_begin(57600);
  
  #if !defined(__AVR_ATmega328P__)
  Serial.begin(57600);
  //while(!Serial);  // Habilitar esta línea si desea depurar en Le
  Serial.println("BLE Arduino Slave ");
  #endif

  /* Por defecto todos a la entrada digital */
  for (int pin = 0; pin < TOTAL_PINS; pin++)
  {
    // Establecer pin para entrada con  INPUT_PULLUP (Entidendo que es como la inversa si esta en HIGH  que se apague un led por ejemplo, si es low que se encienda)
    if(IS_PIN_DIGITAL(pin))
    {
      pinMode(pin, INPUT);
    }      
    digitalWrite(pin, HIGH);
    //  guarda el modo del pin y su estado
    modosPin[pin] = INPUT;
    estadosPin[pin] = LOW;
  } 
}

static byte buf_len = 0;

//Entrada de informe digital
byte reportDigitalInput()
{
  static byte pin = 0;
  byte report = 0;
  
  if (!IS_PIN_DIGITAL(pin))
  {
    pin++;
    if (pin >= TOTAL_PINS)
      pin = 0;
    return 0;
  }
  
  if (modosPin[pin] == INPUT)
  {
      byte estadoActual = digitalRead(pin);
            
      if (estadosPin[pin] != estadoActual)
      {
        estadosPin[pin] = estadoActual;
        byte buf[] = {'G', pin, INPUT, estadoActual};
        enviarArregloDatos(buf, 4);
        
        report = 1;
      }
  }
  
  pin++;
  if (pin >= TOTAL_PINS)
    pin = 0;
    
  return report;
}

//Reportar capacidad de pin
void reportPinCapability(byte pin)
{
  byte buf[] = {'P', pin, 0x00};
  byte pin_cap = 0;
                    
  if (IS_PIN_DIGITAL(pin))
    pin_cap |= PIN_CAPABILITY_DIGITAL;
            
  if (IS_PIN_ANALOG(pin))
    pin_cap |= PIN_CAPABILITY_ANALOG;

  if (IS_PIN_PWM(pin))
    pin_cap |= PIN_CAPABILITY_PWM;

  if (IS_PIN_SERVO(pin))
    pin_cap |= PIN_CAPABILITY_SERVO;

  buf[2] = pin_cap;
  enviarArregloDatos(buf, 3);
}

void reportPinServoData(byte pin)
{

  byte value = pin_servo[pin];
  byte mode = modosPin[pin];
  byte buf[] = {'G', pin, mode, value};         
  enviarArregloDatos(buf, 4);
}

byte reportPinAnalogData()
{
  static byte pin = 0;
  byte report = 0;
  
  if (!IS_PIN_DIGITAL(pin))
  {
    pin++;
    if (pin >= TOTAL_PINS)
      pin = 0;
    return 0;
  }
  
  if (modosPin[pin] == ANALOG)
  {
    uint16_t value = analogRead(pin);
    byte value_lo = value;
    byte value_hi = value>>8;
    
    byte mode = modosPin[pin];
    mode = (value_hi << 4) | mode;
  
    byte buf[] = {'G', pin, mode, value};         
    enviarArregloDatos(buf, 4);
  }
  
  pin++;
  if (pin >= TOTAL_PINS)
    pin = 0;
    
  return report;
}

void reportPinDigitalData(byte pin)
{
  byte state = digitalRead(pin);
  byte mode = modosPin[pin];
  byte buf[] = {'G', pin, mode, state};         
  enviarArregloDatos(buf, 4);
}

void reportPinPWMData(byte pin)
{
  byte value = pin_pwm[pin];
  byte mode = modosPin[pin];
  byte buf[] = {'G', pin, mode, value};         
  enviarArregloDatos(buf, 4);
}

void sendCustomData(uint8_t *buf, uint8_t len)
{
  uint8_t data[20] = "Z";
  memcpy(&data[1], buf, len);
  enviarArregloDatos(data, len+1);
}

byte queryDone = false;


//INICIA LOOP

void loop()
{
  while(recibiendo())
  {
    byte cmd;
    cmd = leerDato();

#if !defined(__AVR_ATmega328P__) // don't print out on UNO
    Serial.write(cmd);
#endif

    // Analizar los datos aqui
    switch (cmd)
    {
      case 'V': // versión del protocolo de consulta
        {
          queryDone = false;
          
          byte buf[] = {'V', 0x00, 0x00, 0x01};
          enviarArregloDatos(buf, 4);          
        }
        break;
      
      case 'C': // recuento total de pines
        {
          byte buf[2];
          buf[0] = 'C';
          buf[1] = TOTAL_PINS; 
          enviarArregloDatos(buf, 2);
        }        
        break;
      
      case 'M': // consulta el modo del pin 
        {  
          byte pin = leerDato();
          byte buf[] = {'M', pin, modosPin[pin]}; // reportar modo del pin
          enviarArregloDatos(buf, 3);
        }  
        break;
      
      case 'S': // establece modo del pin
        {
          byte pin = leerDato();
          byte mode = leerDato();
          
          if (IS_PIN_SERVO(pin) && mode != SERVO && servos[PIN_TO_SERVO(pin)].attached())//attach true si el servo está conectado al pin; false en caso contrario.
            servos[PIN_TO_SERVO(pin)].detach(); //Separar la variable Servo de su pin
  
          /* ToDo: Comprobar el modo si esta en su capacidad o no  */
          /* asumir siempre OK*/
          if (mode != modosPin[pin])
          {              
            pinMode(pin, mode);
            modosPin[pin] = mode;
          
            if (mode == OUTPUT)
            {
              digitalWrite(pin, LOW);
              estadosPin[pin] = LOW;
            }
            else if (mode == INPUT)
            {
              digitalWrite(pin, HIGH);
              estadosPin[pin] = HIGH;
            }
            else if (mode == ANALOG)
            {
              if (IS_PIN_ANALOG(pin)) {
                if (IS_PIN_DIGITAL(pin)) {
                  pinMode(PIN_TO_DIGITAL(pin), LOW);
                }
              }
            }
            else if (mode == PWM)
            {
              if (IS_PIN_PWM(pin))
              {
                pinMode(PIN_TO_PWM(pin), OUTPUT);
                analogWrite(PIN_TO_PWM(pin), 0);
                pin_pwm[pin] = 0;
                modosPin[pin] = PWM;
              }
            }
            else if (mode == SERVO)
            {
              if (IS_PIN_SERVO(pin))
              {
                pin_servo[pin] = 0;
                modosPin[pin] = SERVO;
                if (!servos[PIN_TO_SERVO(pin)].attached())
                  servos[PIN_TO_SERVO(pin)].attach(PIN_TO_DIGITAL(pin));
              }
            }
          }
            
  //        if (mode == ANALOG)
  //          reportPinAnalogData(pin);
          if ( (mode == INPUT) || (mode == OUTPUT) )
            reportPinDigitalData(pin);
          else if (mode == PWM)
            reportPinPWMData(pin);
          else if (mode == SERVO)
            reportPinServoData(pin);
        }
        break;

      case 'G': // query pin data
        {
          byte pin = leerDato();
          reportPinDigitalData(pin);
        }
        break;
        
      case 'T': // set pin digital state
        {
          byte pin = leerDato();
          byte state = leerDato();
          
          digitalWrite(pin, state);
          reportPinDigitalData(pin);
        }
        break;
      
      case 'N': // set PWM
        {
          byte pin = leerDato();
          byte value = leerDato();
          
          analogWrite(PIN_TO_PWM(pin), value);
          pin_pwm[pin] = value;
          reportPinPWMData(pin);
        }
        break;
      
      case 'O': // set Servo
        {
          byte pin = leerDato();
          byte value = leerDato();

          if (IS_PIN_SERVO(pin))
            servos[PIN_TO_SERVO(pin)].write(value);
          pin_servo[pin] = value;
          reportPinServoData(pin);
        }
        break;
      
      case 'A': // query all pin status
        for (int pin = 0; pin < TOTAL_PINS; pin++)
        {
          reportPinCapability(pin);
          if ( (modosPin[pin] == INPUT) || (modosPin[pin] == OUTPUT) )
            reportPinDigitalData(pin);
          else if (modosPin[pin] == PWM)
            reportPinPWMData(pin);
          else if (modosPin[pin] == SERVO)
            reportPinServoData(pin);  
        }
        
        queryDone = true; 
        
        {
          uint8_t str[] = "ABC";
          sendCustomData(str, 3);
        }
       
        break;
          
      case 'P': // query pin capability
        {
          byte pin = leerDato();
          reportPinCapability(pin);
        }
        break;
        
      case 'Z':
        {
          byte len = leerDato();
          byte buf[len];
          for (int i=0;i<len;i++)
            buf[i] = leerDato();

#if !defined(__AVR_ATmega328P__)  
          Serial.println("->");
          Serial.print("Received: ");
          Serial.print(len);
          Serial.println(" byte(s)");
#endif          
        }
    }
    
    return; // sólo hacer esta tarea en este bucle
  }


    
  if (queryDone) // sólo informar los datos después de que el estado de consulta
  { 
    byte input_data_pending = reportDigitalInput();  
    if (input_data_pending)  
      return; // sólo hacer esta tarea en

    valorActualMilisegundos = millis();
    if (valorActualMilisegundos - compararMilisegundos > frecuenciaEjecucion)
    {
      compararMilisegundos += frecuenciaEjecucion;
  
      reportPinAnalogData();
    }
  }  
}

