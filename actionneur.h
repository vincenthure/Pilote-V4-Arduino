#ifndef actionneur_h
#define actionneur_h

#define VERIN_NULL    -1
#define VERIN_STOP     0
#define VERIN_EXTEND   1
#define VERIN_RETRACT  2

#include "pin.h"

class Actionneur
  {
  public:

    Actionneur();
    int Actionneur::action(double, double);
    int getVerin();
    void setThreshold(double);

  private:
  
    int verin_state;
    double threshold;
  };

#endif
