#ifndef Haptic_h
#define Haptic_h

#include "Arduino.h"

  class Haptic
  {
    public:
      // the flag must be set high when a gesture is sensed
      Haptic(int p){
        pin = p;
      }

      void setup(){
        pinMode(pin,OUTPUT);
      }

      byte select = 0;
      byte reject = 0;

      void compute() {
        if (select == 1)
          sel();
        else if (reject == 1)
          rej();

        // process timers
        if (millis() < timer_1) { // turn on buzzer
          digitalWrite(pin, HIGH);
        } else if ( (timer_2_on < millis()) && (timer_2_off > millis()) ) {
          digitalWrite(pin, HIGH);
        } else {
          digitalWrite(pin, LOW);
        }
      }

    private:
      int pin;

      // on time
      long timer_1 = 0;
      long timer_1_on_time = 200;
      long timer_1_off_time = 200;

      long timer_2_on = 0;
      long timer_2_off = 0;
      long timer_2_on_time = 200;
      //long timer_2_off_time = 200;

      byte buzz1_active = 0;
      // loads timers for an acceptable action
      void sel(){
        timer_1 = millis() + timer_1_on_time;
        select = 0;
      }
      // loads timers for an invalid action
      void rej(){
        timer_1 = millis() + timer_1_on_time;
        timer_2_on = timer_1 + timer_1_off_time;
        timer_2_off = timer_2_on + timer_2_on_time;
        reject = 0;
      }

  };

#endif
