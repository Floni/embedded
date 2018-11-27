#pragma once

#include <HardwareSerial.h>

using IdxType = unsigned int;
using CounterType = unsigned int;
using TempType = unsigned char;

struct TempReading {
  TempType temp;

  void print(CounterType counter);
};

struct DB {
  IdxType startIdx;
  IdxType endIdx;

  CounterType startCounter;
  CounterType endCounter;

  void clear();

  void init();

  void append(TempReading* tr);

  void printLatest();

  void printAll();
};
