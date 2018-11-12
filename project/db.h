#pragma once

#include <HardwareSerial.h>

using IdxType = unsigned int;
using TempType = unsigned char;

struct TempReading {
  TempType temp;
  unsigned int timestamp;

  void print();
};

struct DB {
  IdxType startIdx;
  IdxType endIdx;

  void clear();

  void init();

  void append(TempReading* tr);

  void printLatest();

  void printAll();
};