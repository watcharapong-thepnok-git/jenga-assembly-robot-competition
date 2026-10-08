#ifndef MISSION_2_H
#define MISSION_2_H

#include "Robot.h"

namespace Robot {

class Mission2 { /* For Red Team */
private:
  class Clock {
    private:
      uint32_t startTime = 0;

    public:
      void tick() { startTime = millis(); }
      bool hasElapsed(uint32_t duration) { return millis() - startTime >= duration; }
  };

  class Counter {
    private:
      uint32_t count = 0;

    public:
      void set(uint32_t number) { count = number; }

      void reset() { count = 0; } 
      void next()  { count++;   }

      uint32_t current() { return count; }
      bool equal(uint32_t number) { return count == number; }
  };

  Clock clock;  Clock clock2; 
  Counter step; Counter cycle;

  using StateFunc = void (Mission2::*)();
  StateFunc currentState;

  void transitionTo(StateFunc nextState);

  /* For testing */
  void stateRotationTest();
  void stateTranslationTest(); 

  /* In zone */
  void stateStartZone();
  void stateLoadZone();
  void stateAssemblyZone();

  /* Between zone */
  void stateStartToLoadZone();
  void stateLoadToAssemblyZone();

public:
  Mission2();
  void update();
};

} /* namespace Robot */ 

#endif