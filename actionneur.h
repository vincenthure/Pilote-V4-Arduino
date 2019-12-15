#ifndef actionneur_h
#define actionneur_h

#define VERIN_STOP     0
#define VERIN_EXTEND   1
#define VERIN_RETRACT  2

#include "pin.h"

class Actionneur
  {
  public:

    Actionneur();
    void Actionneur::action(double, double);
    char getVerin();
    void setThreshold(double);

  private:
  
    char verin_state;
    double threshold;
  };

#endif
