/*
  StageManager.h - Library for managing global access to rocket stages
*/

#include "Arduino.h"
#include "Stage.h"

class StageManager {
  public:
    StageManager();
    Stage StageManager::stage;
};