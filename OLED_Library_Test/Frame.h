#ifndef Frame_h
#define Frame_h

#include "Arduino.h"



  class Frame
  {
    public:
      Frame(String n);

      // pointers to fram links
      Frame *front = NULL;
      Frame *back = NULL;
      Frame *left = NULL;
      Frame *right = NULL;

      // define alowable gestures
      byte *toggle = NULL;
      byte *variableSet = NULL;
      byte *cancel = NULL;
      byte *home = NULL;

      String name;

    private:

  };

#endif
