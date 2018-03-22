#ifndef Frame_h
#define Frame_h

#include "Arduino.h"

const char hueUsername[] = "5Wp1uZobMvFdn91W9-8NHqagJYwzBvgHNvxXHEI3";  // Hue username
const int hueHubPort = 80;

  class Frame
  {
    public:
      Frame()

      // pointers to fram links
      Frame *front = NULL;
      Frame *back = NULL;
      Frame *left = NULL;
      Frame *right = NULL;

      void addAction(int i);

      char name[];

    private:

  };

#endif
