#pragma once
#include "driver/pulse_cnt.h"
#include <array>
namespace pcnt_fake {
enum class Operation { NONE, LOW_WATCH, HIGH_WATCH, CLEAR, ENABLE, START, STOP, READ };
extern Operation failNext;
extern int failWheel;
extern unsigned orderingErrors, liveUnits;
void reset();
void advance(unsigned wheel, int pulses);
void setCount(unsigned wheel, int count);
void reportBeforeOverflowISR(unsigned wheel, int limit);
}
