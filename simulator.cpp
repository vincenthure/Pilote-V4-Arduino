#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "simulator.h"
#include "actionneur.h"

Simulator::Simulator(unsigned long pilote_loop_time)
      {
      kboat  = pilote_loop_time * K_BOAT; 
      kverin = pilote_loop_time * K_VERIN;
      }

double Simulator::boat(double angle_barre)
      {
      static double heading = 0;

      heading += kboat * angle_barre;
      return heading;
      }

double Simulator::verin(char action)
    {
    static double        barre = 0;  
    static char          last_action = VERIN_STOP;
    static unsigned long start_time_action = 0;
    
    if( last_action != action )
            start_time_action = millis();
        
    last_action = action;
    double attack = (double)((millis() - start_time_action))/TIME_RAMPE;
    if(attack>1)  attack=1;
      
    switch(action)   
        {
        case VERIN_EXTEND  : barre +=  attack * kverin;
                             break;
                             
        case VERIN_RETRACT : barre -=  attack * kverin;
                             break;        
        }

    return barre;
    }
