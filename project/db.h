#pragma once

#include <HardwareSerial.h>

using IdxType = unsigned int;
using CounterType = unsigned int;
using TempType = unsigned char;

struct TempReading {
  TempType temp;

  /**
   * Print the temperature reading,
   * the given counter is it order in the database
   */
  void print(CounterType counter);
};

struct DB {
  IdxType startIdx;
  IdxType endIdx;

  CounterType startCounter;
  CounterType endCounter;

  /**
   * Initialize the database, 
   * must be called before any operations
   */
  void init();

  /**
   * Clear the database,
   * earasing all content.
   */
  void clear();

  /**
   * Append the given temperature reading to the database
   */
  void append(TempReading* tr);

  /**
   * Prints the latest temperature reading
   * by calling the print method.
   */
  void printLatest();

  /**
   * Prints all temperature readings by
   * calling print on them.
   */
  void printAll();
};
