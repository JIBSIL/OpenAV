#include "VectorMath.h" 

#define distanceDebug false

const float rocketMass = 0.5;
const float defaultSeaLevelAirDensity = 1.225;
const int num_points = 18;

float _rocketArea;
float sea_level_temp = 15.0;
float sea_level_humidity = 60.0;

// air density calculation constants
const float p0 = 101325; // Sea level standard pressure (Pa)
const float T0 = 288.15; // Sea level standard temperature (K)
const float L = 0.0065; // Temperature lapse rate (K/m)
const float R = 287.05; // Specific gas constant for dry air (J/kg/K)
const float g = 9.80665; // Gravitational acceleration (m/s^2)
const float M = 0.0289644; // Molar mass of Earth's air (kg/mol)

// humidity calculation constants
// ONLY accurate up to 2-3km
const float H_humid = 2500.0;

// temperature calculation constant with alt
// only works up to 11km
const float lapse_rate = 0.0065;

float thrust_time[num_points] = {
  0.000, 0.005, 0.046, 0.059, 0.100, 0.200, 0.400, 0.500, 0.600, 0.796, 0.894, 0.951, 
  0.964, 1.000, 1.052, 1.106, 1.144, 1.180
};

// this will be filled by the launch helper
float time_squared_table[num_points];
float precomputed_density[50];

float thrust_force[num_points] = {
  0.000, 40.438, 817.754, 813.261, 772.822, 736.877, 696.439, 669.480, 620.055, 
  539.178, 485.261, 453.809, 435.836, 274.082, 152.767, 62.904, 13.480, 0.000
};

float calculate_air_density(float altitude, float temperature, float humidity) {  
  float T = temperature + 273.15;

  // pressure at altitude
  float pressure = p0 * pow((1 - (L * altitude) / T0), (g * M) / (R * L));
  float es = 6.1078 * pow(10.0, (7.5 * temperature) / (temperature + 237.3)) * 100; // convert hPa to Pa
  float e = humidity * es;
  float r = 0.622 * (e / (pressure - e));
  float Tv = T * (1 + 0.61 * r);
  float air_density = pressure / (R * Tv);
  return air_density;
}

// simple exponential decay
float calculate_humidity_at_altitude(float sea_level_humidity, float altitude) {
  float humidity_at_altitude = sea_level_humidity * exp(-altitude / H_humid);
  if (humidity_at_altitude < 0) {
    humidity_at_altitude = 0;
  }
  return humidity_at_altitude;
}

// linear
float calculate_temperature_at_altitude(float sea_level_temperature, float altitude) {
  float temperature_at_altitude = sea_level_temperature - (lapse_rate * altitude);
  return temperature_at_altitude;
}

// basic noncomputed drag curve for max speed
float drag_coeff(float mach_number) {
  float drag_coefficient;
  if (mach_number < 0.8) {
      drag_coefficient = 0.3;
  } else if (mach_number < 1.0) {
      drag_coefficient = 0.5;
  } else if (mach_number < 1.5) {
      drag_coefficient = 0.8;
  } else {
      drag_coefficient = 1.0;
  }
  return drag_coefficient;
}

VectorMath::VectorMath(SatNav& sn, float rocketArea) {
  _satnav = sn;
  _rocketArea = rocketArea;

  Serial.println("Preloading density...");
  for(int i = 0; i < 50; i++) {
    float dy = 75 * i;
    float temp = calculate_temperature_at_altitude(sea_level_temp, dy);
    float humidity = calculate_humidity_at_altitude(sea_level_humidity, dy);

    float airDensity = calculate_air_density(dy, temp, humidity);
    precomputed_density[i] = airDensity;
  }
}

// this is used to calculate "n" coordinate distance (notation uses X direction for clarity)
void calculate_n_distance_from_origin(int angleX, float &dx) {
  float vx = 0;

  float cosXMass = cos(DEGREE_RAD_CONVERSION * float(angleX)) / rocketMass;

  for(int i = 1; i < num_points; i++) {
    float accelX = cosXMass * thrust_force[i];
    float dragCoeff = drag_coeff(vx / 343.0);

    float temp = calculate_temperature_at_altitude(sea_level_temp, dx);
    float humidity = calculate_humidity_at_altitude(sea_level_humidity, dx);

    float airDensity = calculate_air_density(dx, temp, humidity);
    float drag_force = 0.5 * dragCoeff * airDensity * _rocketArea * vx * vx;

    // vf^2=vi^2+2ad
    dx = (vx * thrust_time[i]) + (0.5 * accelX * time_squared_table[i]);

    vx += sqrt(2.0 * accelX * dx);
    
    // test for nan
    if(vx != vx) {
      vx = 0;
    }
  }
}

float signum(float x) {
  if (x > 0.0f) return 1;
  if (x < 0.0f) return -1;
  return 0;
}

// this is used to calculate "n" coordinate distance (notation uses X direction for clarity)
void calculate_supersonic_n_distance_from_origin(int angleX, float &dx, float &maxdzy) {
  float vx = 0.0f, vy = 0.0f, dy = 0.0f;

  float cosX = cos(DEGREE_RAD_CONVERSION * float(angleX));
  float sinX = sin(DEGREE_RAD_CONVERSION * float(angleX));

  for(int i = 1; i < num_points; i++) {
    float timestep = thrust_time[i] - thrust_time[i - 1];
    float dt2 = timestep * timestep;
    float dragCoeff = drag_coeff(vy / 343.0);
    
    float vel = sqrt((vx * vx) + (vy * vy));
    float airDensity = precomputed_density[(int) floor(dy / 75)];
    float drag_force = 0.5 * dragCoeff * airDensity * _rocketArea * vel * vel;

    float fY = thrust_force[i] * sinX;
    float fX = thrust_force[i] * cosX;

    float dragY = drag_force * sinX * (signum(fY) * -1);
    float dragX = drag_force * cosX * (signum(fX) * -1);

    float accelY = (fY - g + dragY) / rocketMass;
    float accelX = (fX + drag_force) / rocketMass;

    if(dy > maxdzy) {
      maxdzy = dy;
    }

    //vx = sqrt((vx * vx) + (2.0 * accelX * dx));
    //vy = sqrt((vy * vy) + (2.0 * accelY * dy));
    vx += accelX * timestep;
    vy += accelY * timestep;

    // vf^2=vi^2+2ad
    // d=vit + 1/2at^2
    dx += (vx * timestep) + (0.5 * accelX * dt2);
    dy += (vy * timestep) + (0.5 * accelY * dt2);

    if(distanceDebug) {
      Serial.println("");
      Serial.println("AngleX " + String(angleX));
      Serial.println("DragCoeff " + String(dragCoeff));
      Serial.println("Vel " + String(vel));
      Serial.println("AirDensity " + String(airDensity));
      Serial.println("DragForce " + String(drag_force));
      Serial.println("SinX " + String(sinX));
      Serial.println("ThrustForce " + String(thrust_force[i]));
      Serial.println("AccelY " + String(accelY));
      Serial.println("fY " + String(fY));
      Serial.println("fX " + String(fX));
      Serial.println("CosX " + String(cosX));
      Serial.println("AccelX " + String(accelX));
      Serial.println("Dx " + String(dx));
      Serial.println("Dy " + String(dy));
    }
    
    // test for nan
    if(vx != vx) {
      vx = 0;
    }

    if(vy != vy) {
      vy = 0;
    }
  }
}

void VectorMath::original_optimize(float &angleX, float &angleZ) {
  float displX, displY, displZ, pct, bestPercentageX, bestPercentageZ;
  _satnav.get_distance_displacement(displX, displY, displZ);
  
  bool found = false;

  // make t^2 multiplication table to optimize
  // use global time^2 table
  for(int i = 0; i < num_points; i++) {
    time_squared_table[i] = thrust_time[i] * thrust_time[i];
  }

  for(int i = 0; i < 180; i++) {
    if(found) {
      continue;
    }

    float x, y;
    // do expensive x calculations 180 times rather than 180^2 times
    calculate_supersonic_n_distance_from_origin(180 - i, x, y);
    
    float j = 0;
    while(j < 180) {
      //if(found) {
      //  continue;
      //}

      #if FAST_QUARTER_PRECISION_Z_ACCURACY
      j += 4;
      #elif FAST_HALF_PRECISION_Z_ACCURACY
      j += 2;
      #else
      j++;
      #endif

      float z = 0.0f, maxdzy = 0.0f;
      calculate_supersonic_n_distance_from_origin(180 - j, z, maxdzy);

      float percentageZ = abs((displZ - z) / z);
      float percentageX = abs((displX - x) / x);

      // Serial.println(String(x) + "X " + String(z) + "Z");
      // Serial.println(percentageX);
      // Serial.println(percentageZ);

      if((!bestPercentageX) || (percentageX < bestPercentageX)) {
        bestPercentageX = percentageX;
      }

      if((!bestPercentageZ) || (percentageZ < bestPercentageZ)) {
        bestPercentageZ = percentageZ;
      }

      if(int(j) % 100 == 0) {
        Serial.println("[LAUNCH HELPER] Best percentage so far is " + String(bestPercentageX) + "X, " + String(bestPercentageZ) + "Z");
      }

      if((percentageX <= NAV_COMPUTER_SIMULATION_TOLERANCE) && (percentageZ <= NAV_COMPUTER_SIMULATION_TOLERANCE)) {
        angleX = 180 - i;
        angleZ = 180 - j;
        found = true;
        Serial.println("[LAUNCH HELPER] Angle calculated with " + String(100 - (100 * ((percentageX + percentageZ) / 2))) + "% accuracy! Max altitude: " + String(maxdzy));
        break;
      }
    }

    if(int(i) % 2) {
      Serial.println("[LAUNCH HELPER] progress: optimization at " + String(100.0 * (i / 180.0)) + "%");
    }
  }

  if(!found) {
    Serial.println(F("[LAUNCH HELPER] No optimal launch angle found (try lowering NAV_COMPUTER_SIMULATION_TOLERANCE)!"));
  }
}

void VectorMath::alternative_seeking(float &angleX, float &angleZ) {
  float displX, displY, displZ;
  _satnav.get_distance_displacement(displX, displY, displZ);

  float simtol;

  #if USE_ALTERNATIVE_ACCURATE_SEEKING
  simtol = 0.2;
  #else
  simtol = NAV_COMPUTER_SIMULATION_TOLERANCE;
  #endif

  // Make t^2 multiplication table to optimize
  for(int idx = 0; idx < num_points; idx++) {
    time_squared_table[idx] = thrust_time[idx] * thrust_time[idx];
  }

  #if USE_ALTERNATIVE_ACCURATE_SEEKING
  // Variables for angleX
  float bestPercentageX = 1.0f;
  float bestValueX = 0.0f;
  bool initializedX = false;

  // Variables for angleZ
  float bestPercentageZ = 1.0f;
  float bestValueZ = 0.0f;
  bool initializedZ = false;
  #endif

  // Optimize angleX
  float iIncrement = 8.0f;
  int ioriginal;
  for(float i = 0; i < 180;) {
    ioriginal++;
    float x;
    calculate_n_distance_from_origin(180 - i, x);
    float percentageX = abs((displX - x) / x);

    #if USE_ALTERNATIVE_ACCURATE_SEEKING
    if(initializedX) {
      if(percentageX < bestPercentageX) {
        bestPercentageX = percentageX;
        bestValueX = 180 - i;
      }
      if(i >= 180) {
        angleX = bestValueX;
        break;
      }
    }

    if(percentageX <= simtol) {
      if(!initializedX) {
        initializedX = true;
        i = max(0.0f, i - iIncrement);
        iIncrement = 0.2f;
        bestPercentageX = percentageX;
        bestValueX = 180 - i;
        continue; // Restart 'i' loop with finer increments
      }
    }
    #else
    if(percentageX <= simtol) {
      angleX = 180 - i;
      break;
    }
    #endif

    i += iIncrement;

    if(ioriginal % 64 == 0) {
      Serial.println("[LAUNCH HELPER] progress: optimization at " + String(100.0 * (i / 180.0)) + "%");
    }
  }

  // Check if angleX was found
  #if USE_ALTERNATIVE_ACCURATE_SEEKING
  if(!initializedX) {
    Serial.println(F("[LAUNCH HELPER] No optimal launch angleX found (try lowering NAV_COMPUTER_SIMULATION_TOLERANCE)!"));
    return;
  }
  #endif

  // Optimize angleZ
  float jIncrement = 8.0f;
  for(float j = 0; j < 180;) {
    float z;
    calculate_n_distance_from_origin(180 - j, z);
    float percentageZ = abs((displZ - z) / z);

    #if USE_ALTERNATIVE_ACCURATE_SEEKING
    if(initializedZ) {
      if(percentageZ < bestPercentageZ) {
        bestPercentageZ = percentageZ;
        bestValueZ = 180 - j;
      }
      if(j >= 180) {
        angleZ = bestValueZ;
        break;
      }
    }

    if(percentageZ <= simtol) {
      if(!initializedZ) {
        initializedZ = true;
        j = max(0.0f, j - jIncrement);
        jIncrement = 0.2f;
        bestPercentageZ = percentageZ;
        bestValueZ = 180 - j;
        continue; // Restart 'j' loop with finer increments
      }
    }
    #else
    if(percentageZ <= simtol) {
      angleZ = 180 - j;
      break;
    }
    #endif

    j += jIncrement;
  }

  // Check if angleZ was found
  #if USE_ALTERNATIVE_ACCURATE_SEEKING
  if(!initializedZ) {
    Serial.println(F("[LAUNCH HELPER] No optimal launch angleZ found (try lowering NAV_COMPUTER_SIMULATION_TOLERANCE)!"));
    return;
  }
  #endif

  // Output the final angles and accuracy
  #if USE_ALTERNATIVE_ACCURATE_SEEKING
  float averagePercentage = (bestPercentageX + bestPercentageZ) / 2.0f;
  Serial.println("[LAUNCH HELPER] Angle calculated with " + String(100 - (100 * averagePercentage)) + "% accuracy");
  angleX = bestValueX;
  angleZ = bestValueZ;
  return;
  #endif
}