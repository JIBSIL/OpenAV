/*
  VectorMath.h - Library for calculating angle of launch and midair calculations
*/

#include "Arduino.h"
#include "SatNav.h"

#define DEGREE_RAD_CONVERSION 0.0174533
#define NAV_COMPUTER_SIMULATION_TOLERANCE 0.1

// optimizations for launch angle algo
#define FAST_HALF_PRECISION_Z_ACCURACY true
#define FAST_QUARTER_PRECISION_Z_ACCURACY true
#define USE_ALTERNATIVE_ACCURATE_SEEKING false

#ifndef VECTORMATH_H
#define VECTORMATH_H
class VectorMath {
  public:
    VectorMath(SatNav& sn, float rocketArea);
    void original_optimize(float &angleX, float &angleZ);
    void alternative_seeking(float &angleX, float &angleZ);
  private:
    SatNav& _satnav;
    //float _rocketArea;
};
#endif