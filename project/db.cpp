#include "db.h"

#include <EEPROM.h>

#define DB_DATA_ADDR sizeof(DB)


void DB::clear() {
    this->startIdx = this->endIdx = 0;
    this->startCounter = this->endCounter = 0;
    EEPROM.put(0, *this);
}

void DB::init() {
    EEPROM.get(0, *this);
}

void DB::append(TempReading* tr) {
    EEPROM.put(DB_DATA_ADDR + endIdx, *tr);
    this->endCounter++;

    // check for wrap
    if (this->endIdx >= E2END) {
      this->endIdx = 0;
    } else {
      this->endIdx += sizeof(TempReading);
    }

    // inc start_idx if needed
    if (this->endIdx == this->startIdx) {
      // TODO: wrap startIdx
      this->startIdx += sizeof(TempReading);
      this->startCounter++;
    }
    
    EEPROM.put(0, *this);
}

void DB::printLatest() {
    TempReading tr;
    // TODO: wrap endIdx - 1
    EEPROM.get(DB_DATA_ADDR + endIdx - sizeof(TempReading), tr);
    tr.print(this->endCounter);
}

void DB::printAll() {
    TempReading tr;
    IdxType counter = this->startCounter;
    for (auto idx = this->startIdx; idx != this->endIdx; idx  = (idx >= E2END ? DB_DATA_ADDR : idx + sizeof(TempReading))) {
        EEPROM.get(DB_DATA_ADDR + idx, tr);
        tr.print(counter);
        counter++;
    }
}
