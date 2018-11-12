#include "db.h"

#include <EEPROM.h>

#define DB_START_IDX_ADDR 0
#define DB_END_IDX_ADDR sizeof(IdxType)
#define DB_DATA_ADDR (2*sizeof(IdxType))

void DB::clear() {
    this->startIdx = this->endIdx = DB_DATA_ADDR;
    EEPROM.put(DB_START_IDX_ADDR, *this);
    static_assert(sizeof(DB) == DB_DATA_ADDR, "Padding makes db larger than it should be");
}

void DB::init() {
    EEPROM.get(DB_START_IDX_ADDR, *this);
}

void DB::append(TempReading* tr) {
    EEPROM.put(DB_DATA_ADDR + endIdx*sizeof(TempReading), *tr);
    this->endIdx++; // TODO: overflow
    EEPROM.put(DB_START_IDX_ADDR, *this);
}

void DB::printLatest() {
    TempReading tr;
    EEPROM.get(DB_DATA_ADDR + (endIdx - 1) * sizeof(TempReading), tr);
    tr.print();
}

void DB::printAll() {
    TempReading tr;
    for (auto idx = this->startIdx; idx < this->endIdx; idx++) {
        EEPROM.get(DB_DATA_ADDR + idx * sizeof(TempReading), tr);
        tr.print();
    }
}