#ifndef Frame_h
#define Frame_h

#include "Arduino.h"



  class Frame
  {
    public:
      Frame(String n);

      // pointers to fram links
      Frame *up = NULL;
      Frame *down = NULL;
      Frame *left = NULL;
      Frame *right = NULL;

      // define alowable gestures
      byte *select = NULL;
      //byte *variableSet = NULL;
      byte *cancel = NULL;
      byte *home = NULL;

      String name;

    private:

  };

#endif
