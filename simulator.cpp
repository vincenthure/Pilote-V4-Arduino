#include "simulator.h"
#include "actionneur.h"

Simulator::Simulator(unsigned long pilote_loop_time)
      {
      barre = 0;
      loop_time = pilote_loop_time;  
      }

double Simulator::boat(double angle_barre)
      {
      static double heading = 0;

      heading += K_BOAT * loop_time * angle_barre;
      return heading;
      }

double Simulator::verin(char action)
    {
    switch(action)
        {
        case VERIN_EXTEND  : barre += K_VERIN * loop_time;
                             break;
                             
        case VERIN_RETRACT : barre -= K_VERIN * loop_time;
                             break;        
        }
    return barre;
    }
