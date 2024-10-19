/*
  SatNav.h - Library for controlling NEO-6M GPS
  Current state: emulation only.
*/

#include "Arduino.h"
#include "Stage.h"

#ifndef SATNAV_H
#define SATNAV_H
class SatNav {
  public:
    SatNav(Stage s);
    void getTarget(float &x, float &y, float &z);
    void currentLocation(float &x, float &y, float &z);
    void get_distance_displacement(float &x, float &y, float &z);
  private:
    Stage _stage;
};
#endif