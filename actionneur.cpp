#include "actionneur.h"
#include "Arduino.h"

Actionneur::Actionneur()
    {
    pinMode(PIN_EXTEND, OUTPUT); 
    pinMode(PIN_RETRACT,OUTPUT);
      
    verin_state = VERIN_STOP;
    }

void Actionneur::setThreshold(double Threshold)
    {
    threshold = Threshold;  
    }

void Actionneur::action(double boat, double capteur)
    {
    double value = boat - capteur;
    char new_state = VERIN_STOP;

    if      (value >  threshold)       new_state = VERIN_RETRACT; 
    else if (value < -threshold)       new_state = VERIN_EXTEND; 

    if((verin_state != VERIN_STOP) && (new_state == VERIN_STOP))     
                {
                digitalWrite(PIN_RETRACT, LOW);
                digitalWrite(PIN_EXTEND, LOW);
                //bluetooth.send( OUTPUT_STOP ); 
                }
                
    else if ((verin_state != VERIN_EXTEND)  && (new_state == VERIN_EXTEND))   
                {
                digitalWrite(PIN_RETRACT, LOW);
                digitalWrite(PIN_EXTEND, HIGH); 
                //bluetooth.send( OUTPUT_EXTEND );
                }
                
    else if ((verin_state != VERIN_RETRACT) && (new_state == VERIN_RETRACT))
                {
                digitalWrite(PIN_RETRACT,HIGH);
                digitalWrite(PIN_EXTEND,  LOW);
                //bluetooth.send( OUTPUT_RETRACT ); 
                }

    verin_state =  new_state; 
    }

char Actionneur::getVerin()
    {
    return verin_state; 
    }
