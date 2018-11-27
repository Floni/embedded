#include <Arduino_FreeRTOS.h>
#include <avr/sleep.h> 
#include <avr/wdt.h>

#include "db.h"

#define WAKE_PIN 2
#define INPUT_PIN 7
#define OUTPUT_PIN 13

void TaskSerialRead( void *pvParameters );
void TaskTempLog( void *pvParameters );
void TaskInterrupt(void* pvParameters);

void inputInterrupt();
void wakeUpInterrupt();

DB db;

TaskHandle_t interruptTaskHandle;

void setup() {
  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  Serial.println("Starting");

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

   xTaskCreate(
    TaskInterrupt
    ,  (const portCHAR *)"InterruptTask"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL // *pvParameters
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &interruptTaskHandle ); // Reference to task
  
  pinMode(INPUT_PIN, INPUT_PULLUP);
  pinMode(WAKE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), inputInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(WAKE_PIN), wakeUpInterrupt, LOW);
}

void loop() { /* NOT USED */ }

void sleep() {

  set_sleep_mode( SLEEP_MODE_PWR_DOWN );
 
  portENTER_CRITICAL();
  
  wdt_disable();
  
  //reset watchdog
  wdt_reset();
  
  sleep_enable();
   
  // Only if there is support to disable the brown-out detection.
  // If the brown-out is not set, it doesn't cost much to check.
  #if defined(BODS) && defined(BODSE)
  Qsleep_bod_disable();
  #endif
   
  portEXIT_CRITICAL();
  
  sleep_cpu(); // Good night.

   
  // Ugh. Yawn... I've been woken up. Better disable sleep mode.
  // Reset the sleep_mode() faster than sleep_disable();
  sleep_reset();
  
  //set up WDT Interrupt (rather than the WDT Reset).
  wdt_interrupt_enable( portUSE_WDTO );

  
}

void TaskSerialRead( void *pvParameters ) {
  while(1) {
    if (Serial.available() > 0) {
      int inputByte = Serial.read();

      switch(inputByte) {
        case('1'):
          db.printLatest();
          break;
        case('2'):
          Serial.println("Entering sleep...");
          delay(5);
         
          sleep();

          Serial.println("Woke up");
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
    vTaskDelay(1);
  }
}

void TaskTempLog( void *pvParamaters) {
  TempReading tr;
  while(1) {
    tr.temp = GetTemp();
    //tr.timestamp = millis();
    db.append(&tr);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void TaskInterrupt(void* pvParameters) {
  while (1) {
    int notified = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000));
  
    if (notified == 1) {
      digitalWrite(OUTPUT_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(1000));
      digitalWrite(OUTPUT_PIN, LOW);
    }
  }
}

void inputInterrupt() {
  vTaskNotifyGiveFromISR(interruptTaskHandle, NULL);
}

void wakeUpInterrupt() {
  
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
  t = (t - 273.0) / 1.00;
  return (t);
}

void TempReading::print(CounterType counter) {
  Serial.print(counter);
  Serial.print(": ");
  Serial.print(static_cast<int>(temp));
  Serial.println(" °C");
}
