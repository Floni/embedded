#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include <avr/sleep.h> 
#include <avr/wdt.h>

#include "db.h"

#define WAKE_PIN 2
#define INPUT_PIN 7
#define OUTPUT_PIN 13

#define TEMP_READING_INTERVAL 500
#define SLEEP_DELAY 100

#define COLLISION_HIGH_DELAY 1000
#define COLLISION_NOTIFY_MAX_DELAY 1000

#define SERIAL_BUAD 9600

void TaskSerialRead( void *pvParameters );
void TaskTempLog( void *pvParameters );
void TaskInterrupt(void* pvParameters);

void inputInterrupt();
void wakeUpInterrupt();

// The mutex and database
SemaphoreHandle_t DBmutex;
DB db;

// task handle of the task to wake when a collision interrupt occurs
TaskHandle_t interruptTaskHandle;

void setup() {
  Serial.begin(SERIAL_BUAD);

  // init the database, reading from EEPROM
  db.init();
  DBmutex = xSemaphoreCreateMutex();

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
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &interruptTaskHandle ); // Reference to task

  // set both interrupt pins as inputs
  pinMode(INPUT_PIN, INPUT_PULLUP);
  pinMode(WAKE_PIN, INPUT_PULLUP);

  // enable the interrupts
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), inputInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(WAKE_PIN), wakeUpInterrupt, LOW);
}

void loop() { /* NOT USED */ }

/**
 * Take the database mutex,
 * must be called before all database operations
 */
static void takeDBmutex() {
  while (xSemaphoreTake(DBmutex, 5) == pdFALSE) {
    vTaskDelay(1);
  }
}

/**
 * Releases the database mutex, 
 * must be called after all database operations
 */
static void releaseDBmutex() {
  xSemaphoreGive(DBmutex); 
}

/**
 * Puts the arduino in sleep mode,
 * also disables the watchdog timer to prevent
 * the arduino from waking up every 15ms.
 */
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
  sleep_bod_disable();
  #endif
   
  portEXIT_CRITICAL();
  
  sleep_cpu(); // Good night.

   
  // Ugh. Yawn... I've been woken up. Better disable sleep mode.
  // Reset the sleep_mode() faster than sleep_disable();
  sleep_reset();
  
  //set up WDT Interrupt (rather than the WDT Reset).
  wdt_interrupt_enable( portUSE_WDTO );

}

/**
 * Task that reads from the serial port and executes the required command.
 */
void TaskSerialRead( void *pvParameters ) {

  // wait for serial port to start
  while (!Serial) { vTaskDelay(1); }
  
  for (;;) {
    
    // execute all commands that are available
    while (Serial.available() > 0) {
      int inputByte = Serial.read();

      switch(inputByte) {
        case('1'): // print the latest temperature reading
          takeDBmutex();
          db.printLatest();
          releaseDBmutex();
          break;
        case('2'): // put arduino to sleep
          Serial.println("Sleeping");
          delay(SLEEP_DELAY);
          sleep();
          break;
        case('3'): // print all temperature reading
          takeDBmutex();
          db.printAll();
          releaseDBmutex();
          break;
        case('c'): // clear the database
          takeDBmutex();
          db.clear();
          releaseDBmutex();
          break;
        case('\n'): // Skip newline chars
          break;
        default:
          Serial.println("Unkown command");
          break;
      }
    }

    // put this task to sleep while no serial data is abailable
    vTaskDelay(1);
  }
}

/**
 * Task that logs the temperature to the database every 500ms.
 */
void TaskTempLog( void *pvParamaters) {
  TempReading tr;
  TickType_t lastWakeTime = xTaskGetTickCount();
  
  for (;;) {
    // take temperature reading
    tr.temp = GetTemp();

    // save to database
    takeDBmutex();
    db.append(&tr);
    releaseDBmutex();

    // delay 500ms taking into account 
    // the time the reading and writing took
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TEMP_READING_INTERVAL));
  }
}

/**
 * High priority task that responds to the collision interrupt.
 */
void TaskInterrupt(void* pvParameters) {
  for (;;) {
    // wait to be notified from the ISR
    int notified = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(COLLISION_NOTIFY_MAX_DELAY));

    // if notified turn the output pin on
    if (notified == 1) {
      digitalWrite(OUTPUT_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(COLLISION_HIGH_DELAY));
      digitalWrite(OUTPUT_PIN, LOW);
    }
  }
}

/**
 * ISR for the collision interrupt, wakes the interrupt task.
 */
void inputInterrupt() {
  // notify the high priority task
  vTaskNotifyGiveFromISR(interruptTaskHandle, NULL);

  // normally portYIELD_FROM_ISR would be called
  // to ensure that the next task that runs is the
  // highest priority task, but arduino rtos library
  // doesn't implement this, so we trust the scheduler
}

/**
 * ISR that is used to wake the arduino.
 * Does nothing but waking up from sleep
 */
void wakeUpInterrupt() {/* nop */}

/**
 * Returns the current internal temperature of the arduino
 */
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

/**
 * Prints a temperature reading,
 * implemented here because it needs access to the Serial object
 */
void TempReading::print(CounterType counter) {
  Serial.print(counter);
  Serial.print(": ");
  Serial.print(static_cast<int>(temp));
  Serial.println(" °C");
}
