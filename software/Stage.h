/*
  Stage.h - Library for simulating rocket stages (enum only)
*/

#include "Arduino.h"

#ifndef STAGE_H
#define STAGE_H

enum Stage {
  WAIT,
  IGNITION,
  ASCENT,
  DESCENT,
  FINAL_DECEL,
  ERROR
};

#endif