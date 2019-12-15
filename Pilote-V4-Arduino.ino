
#include <EEPROM.h>

#include "pin.h"
#include "Navh.h"
#include "simulator.h"
#include "bluetooth.h"
#include "actionneur.h"
#include "pid.h"
#include "AnalogButtons.h"

#define   PILOTE_LOOP_TIME   100

//******************** declaration ***************************

double   gap, 
         barre,
         target = 0;

struct   navh       imu;
struct   parametres parametre;

PID           asservissement(&gap, &barre, &target, 0, 0, 0, DIRECT);

Navh          navh(&imu);
Simulator     simulator(PILOTE_LOOP_TIME);
Bluetooth     bluetooth(&parametre);
Actionneur    actionneur;
AnalogButtons analogButtons(PIN_POUSSOIRS, INPUT);


//******************************************************************************

void setup()
      {
      EEPROM.get(0, parametre);   // restore parametres from EEprom 
      
      asservissement.SetMode(AUTOMATIC);
      asservissement.SetSampleTime(PILOTE_LOOP_TIME);
      asservissement.SetOutputLimits(-parametre.barre_max, parametre.barre_max);
      asservissement.SetTunings(parametre.kp, parametre.ki, parametre.kd);
        
      bluetooth.begin();
      actionneur.setThreshold(parametre.threshold);
       
      Button btn1 = Button(510, &fnc1);
      Button btn2 = Button(681, &fnc2);
      Button btn3 = Button(768, &fnc3);
      Button btn4 = Button(819, &fnc4);
      Button btn5 = Button(  0, &fnc5);
      analogButtons.add(btn1);
      analogButtons.add(btn2);
      analogButtons.add(btn3);
      analogButtons.add(btn4);
      analogButtons.add(btn5);
      }
      
void loop()
      {
      unsigned long end_time = millis() + PILOTE_LOOP_TIME;
      
      double capteur_barre = simulator.verin(actionneur.getVerin());     
      imu.yaw   = simulator.boat(capteur_barre);
      //navh.read();
      //navh.ask();

      gap = imu.yaw - parametre.cap;

      asservissement.Compute();                       // input gap, output barre
      
      if(parametre.stanby==false)
              actionneur.action(capteur_barre , barre);  // input ecart de la barre, output commande verrin stop, extend, retract

      bluetooth.arduino_refresh(imu.yaw, capteur_barre, barre);  // monitoring android
      
      if( bluetooth.arduino_scan() )
                        {
                        EEPROM.put(0,parametre); 
                        asservissement.SetOutputLimits(-parametre.barre_max, parametre.barre_max);
                        asservissement.SetTunings(parametre.kp, parametre.ki, parametre.kd);
                        actionneur.setThreshold(parametre.threshold); 
                        }
                        
      
      
      while(millis() < end_time)   analogButtons.check();    // scan les boutons
      }

void fnc1()
    {
    bluetooth.click_minus_10();     
    }
      
void fnc2()
    {
    bluetooth.click_minus_1();     
    }

void fnc3()
    {
    bluetooth.click_plus_1();     
    }

void fnc4()
    {
    bluetooth.click_plus_10();     
    }

void fnc5()
    {
    bluetooth.click_stanby();     
    }
