
#include "Arduino.h"
#include "Navh.h"


Navh::Navh(navh* navh)
    {
    Serial.begin(NAVH_BAUD);
    data = navh; 
    }


boolean Navh::read()
    {
    while(1)
          {
          char c = Serial.read();
          if(c==-1)    
              return 0;

          if(c=='F')
              if(Serial.read() == 18)   
                  return Serial.readBytes((byte*)data,68); 
          }
    }

void Navh::ask()
    {
    Serial.write('F');
    }
