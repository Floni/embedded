#include "db.h"

#include <EEPROM.h>

#define DB_DATA_ADDR sizeof(DB)

constexpr IdxType getAddress(IdxType idx) {
  return idx + DB_DATA_ADDR;
}

static void increment(IdxType& idx) {
  idx += sizeof(TempReading);
  if (getAddress(idx) >= E2END) {
    idx = 0;
  }
}

void DB::clear() {
    this->startIdx = this->endIdx = 0;
    this->startCounter = this->endCounter = 0;
    EEPROM.put(0, *this);
}

void DB::init() {
    EEPROM.get(0, *this);
}

void DB::append(TempReading* tr) {
    EEPROM.put(getAddress(endIdx), *tr);
    this->endCounter++;

    // increment end index
    increment(this->endIdx);

    // inc start index if needed
    if (this->endIdx == this->startIdx) {
      increment(this->startIdx);
      this->startCounter++;
    }
    
    EEPROM.put(0, *this);
}

void DB::printLatest() {
    TempReading tr;
    
    int lastIdx = static_cast<int>(endIdx) - sizeof(TempReading);
    if (lastIdx < 0) {
      // TODO: alignment?
      lastIdx = E2END;
    }
    
    EEPROM.get(getAddress(lastIdx), tr);
    tr.print(this->endCounter);
}

void DB::printAll() {
    TempReading tr;
    
    auto counter = this->startCounter;
    auto idx = this->startIdx;
    
    while (idx != this->endIdx) {

      EEPROM.get(getAddress(idx), tr);
      tr.print(counter);
      
      counter++;
      increment(idx);
    }
}
