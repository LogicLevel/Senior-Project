#ifndef Gesture_h
#define Gesture_h

#include "Arduino.h"

  class Gesture
  {
    public:
      // the flag must be set high when a gesture is sensed
      byte flag = 0;

      // navigational gestures
      byte up = 0;
      byte down = 0;
      byte left = 0;
      byte right = 0;

      // interfacing gestures
      byte select = 0;
      // byte variableSet = 0;
      byte cancel = 0;
      byte home = 0;

      // this value measures tilt of the hand when the variableSet byte is one
      //   this value goes positive 20 to negative 20
      // int variableSetInclination = 0;
      // const int MAX_INCLINATION = 20;

    private:

  }

#endif
