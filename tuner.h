#ifndef TUNER_H
#define TUNER_H

#include <vector>
#include <regex>
#include <map>
#include "messaging.h"
#include "motor_control.h"

#define TUNER_DURATION_CONST 0.125

extern void tune_parser(std::string melody);

extern void playTune();

#endif
