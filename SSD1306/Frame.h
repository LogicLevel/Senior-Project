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

      // execution pointer for
      void addAction(int i);

      String name;

    private:

  };

#endif
