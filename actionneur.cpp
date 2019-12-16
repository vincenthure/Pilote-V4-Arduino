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

int Actionneur::action(double boat, double capteur)
    {
    double value = boat - capteur;
    int new_state = VERIN_STOP;
    int action    = VERIN_NULL;

    if      (value >  threshold)       new_state = VERIN_RETRACT; 
    else if (value < -threshold)       new_state = VERIN_EXTEND; 

    if((verin_state != VERIN_STOP) && (new_state == VERIN_STOP))     
                {
                digitalWrite(PIN_RETRACT, LOW);
                digitalWrite(PIN_EXTEND, LOW);
                action = VERIN_STOP; 
                }
                
    else if ((verin_state != VERIN_EXTEND)  && (new_state == VERIN_EXTEND))   
                {
                digitalWrite(PIN_RETRACT, LOW);
                digitalWrite(PIN_EXTEND, HIGH); 
                action =  VERIN_EXTEND;
                }
                
    else if ((verin_state != VERIN_RETRACT) && (new_state == VERIN_RETRACT))
                {
                digitalWrite(PIN_RETRACT,HIGH);
                digitalWrite(PIN_EXTEND,  LOW);
                action =  VERIN_RETRACT; 
                }

    verin_state =  new_state; 
    return action;
    }

int Actionneur::getVerin()
    {
    return verin_state; 
    }
