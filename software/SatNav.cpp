#include "SatNav.h"

float emulatedGPSCoordinates[3] = {0, 0, 0};

SatNav::SatNav(Stage s) {
  _stage = s;
}

void SatNav::getTarget(float &x, float &y, float &z) {
  // emulate for now
  x = -126;
  y = 0;
  z = 477;
}

void SatNav::currentLocation(float &x, float &y, float &z) {
  if(_stage == Stage::IGNITION) {
    x = 0;
    y = 0;
    z = 0;
  } else {
    // emulate for now
    x = emulatedGPSCoordinates[0];
    y = emulatedGPSCoordinates[1];
    z = emulatedGPSCoordinates[2];
  }
}

void SatNav::get_distance_displacement(float &x, float &y, float &z) {
  float tX, tY, tZ, lX, lY, lZ;
  getTarget(tX, tY, tZ);
  currentLocation(lX, lY, lZ);

  x = tX - lX;
  y = tY - lY;
  z = tZ - lZ;
}