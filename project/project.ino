#include <Arduino_FreeRTOS.h>

#include "db.h"

void TaskSerialRead( void *pvParameters );
void TaskTempLog( void *pvParameters );

DB db;

void setup() {
  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  db.init();

  xTaskCreate(
    TaskSerialRead
    ,  (const portCHAR *)"SerialRead"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL // *pvParameters
    ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL ); // Reference to task

  xTaskCreate(
    TaskTempLog
    ,  (const portCHAR *)"TempratureLog"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL // *pvParameters
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL ); // Reference to task
}

void loop() { /* NOT USED */ }

void TaskSerialRead( void *pvParameters ) {
  while(1) {
    if (Serial.available() > 0) {
      int inputByte = Serial.read();

      switch(inputByte) {
        case('1'):
          db.printLatest();
          break;
        case('2'):
          // low power mode
          break;
        case('3'):
          db.printAll();
          break;
        case('c'):
          db.clear();
          break;
        case('\n'):
          // Skip newline chars
          break;
        default:
          Serial.println("Unkown command");
          break;
      }
    }
  }
}

void TaskTempLog( void *pvParamaters) {
  TempReading tr;
  while(1) {
    tr.temp = GetTemp();
    tr.timestamp = millis();
    db.append(&tr);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

double GetTemp(void) { 
  double t;
  
  // Set the internal reference and mux for the ATmega32U4.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX0) | _BV(MUX1) | _BV(MUX2)); 
  ADCSRB |= (1 << MUX5); // enable the ADC
  
  delay(5); // wait for voltages to become stable.
  ADCSRA |= _BV(ADSC); // Start the ADC
  
  // Detect end-of-conversion
  while (bit_is_set(ADCSRA, ADSC));
  unsigned int w = ADCW;
  t = w;
  t = (t - 273.0 + 5.0) / 1.00;
  return (t);
}

void TempReading::print() {
  Serial.print(timestamp);
  Serial.print(": ");
  Serial.print(static_cast<int>(temp));
  Serial.println(" °C");
}