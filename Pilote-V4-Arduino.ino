
#include <PID_v1.h>
#include <AltSoftSerial.h>
#include <EEPROM.h>
#include <AnalogButtons.h>

#include "Navh.h"
#include "pilote.h"
#include "simulator.h"

//******************** declaration ***************************

double   pid_gap    = 0, 
         pid_barre  = 0,
         pid_target = 0;

char     verin_state = VERIN_STOP;

struct   navh imu;
struct   parametres parametre;

AltSoftSerial Bluetooth; // RX pin 8, TX pin 9, unusable PWM 10
PID           asservissement(&pid_gap, &pid_barre, &pid_target, parametre.kp, parametre.ki, parametre.kd, DIRECT);
AnalogButtons analogButtons(PIN_POUSSOIRS, INPUT);
Navh          navh(&imu);
Simulator     simulator;


//******************************************************************************

void setup()
      {
      // initialise Arduino

      Bluetooth.begin(BLUETOOTH_BAUD); // connection Bluetooth
      pinMode(PIN_EXTEND, OUTPUT); 
      pinMode(PIN_RETRACT,OUTPUT);
                 
      parametre.stanby = true;
      load_parametres();   // restore parametres from EEprom 
      pid_refresh();        // initialise PID
            
      buttons_init();       // initialise les boutons
      }
      
void loop()
      {
      unsigned long end_time = millis() + PILOTE_LOOP_TIME;
      
      double capteur_barre = simulator.verin(verin_state);     
      imu.yaw   = simulator.boat(capteur_barre);  // input ange de barre reel , output cap du bateau
      navh.read();
      navh.ask();

      pid_gap = imu.yaw - parametre.cap;
      //Serial.println(pid_gap);
      if(pid_gap >  180) pid_gap -= 360;
      if(pid_gap < -180) pid_gap += 360;

      asservissement.Compute();                       // input pid_gap, output pid_barre
      if(parametre.stanby==false)
              verin_state = actionneur(capteur_barre , pid_barre, verin_state);  // input ecart de la barre, output commande verrin stop, extend, retract

      bluetooth(imu.yaw, capteur_barre, pid_barre);  // envoi info à android pour le monitoring, check bouton pressé sur android
 
 // attends pour faire une boucle bien periodique en scannant les boutons
 
      while(millis() < end_time)       analogButtons.check();  
      }

void pid_refresh()
     {
     asservissement.SetMode(AUTOMATIC);
     asservissement.SetOutputLimits(-parametre.barre_max, parametre.barre_max);
     asservissement.SetSampleTime(PILOTE_LOOP_TIME);
     asservissement.SetTunings(parametre.kp, parametre.ki, parametre.kd);  
     }
     
char actionneur(double boat, double capteur, char verin)
    {
    double value = boat - capteur;
    char new_state = VERIN_STOP;

    if      (value >  parametre.threshold)       new_state = VERIN_RETRACT; 
    else if (value < -parametre.threshold)       new_state = VERIN_EXTEND; 

    if((verin != VERIN_STOP) && (new_state == VERIN_STOP))     
                {
                digitalWrite(PIN_RETRACT, LOW);
                digitalWrite(PIN_EXTEND, LOW);
                bluetooth_send( OUTPUT_STOP ); 
                }
                
    else if ((verin != VERIN_EXTEND)  && (new_state == VERIN_EXTEND))   
                {
                digitalWrite(PIN_RETRACT, LOW);
                digitalWrite(PIN_EXTEND, HIGH); 
                bluetooth_send( OUTPUT_EXTEND );
                }
                
    else if ((verin != VERIN_RETRACT) && (new_state == VERIN_RETRACT))
                {
                digitalWrite(PIN_RETRACT,HIGH);
                digitalWrite(PIN_EXTEND,  LOW);
                bluetooth_send( OUTPUT_RETRACT ); 
                }

    return new_state; 
    }
   
void click_reset_parametres()
      {
      parametre.kp        = KP_INIT,
      parametre.ki        = KI_INIT,
      parametre.kd        = KD_INIT,
      parametre.barre_max = BARRE_MAX_INIT;
      parametre.threshold = THRESHOLD_INIT;
      }

double cap_limit( double val )
      {
      if(val<0)     val+=360;
      if(val>359)   val-=360;
      return val;  
      }
  
double k_limit( double val )
      {
      val=max(val,0);
      val=min(val,254);
      return val;  
      }
      
//***************************** Button ****************************************

Button btn1 = Button(510, &click_minus_10);
Button btn2 = Button(681, &click_minus_1);
Button btn3 = Button(768, &click_plus_1);
Button btn4 = Button(819, &click_plus_10);
Button btn5 = Button(  0, &click_stanby);

void buttons_init()
      {
      analogButtons.add(btn1);
      analogButtons.add(btn2);
      analogButtons.add(btn3);
      analogButtons.add(btn4);
      analogButtons.add(btn5);
      }

void click_minus_10()
      {
      parametre.cap = cap_limit(parametre.cap-10);
      bluetooth_send_param(OUTPUT_CAP, parametre.cap);
      }

void click_minus_1()
      {
      parametre.cap = cap_limit(parametre.cap-1); 
      bluetooth_send_param(OUTPUT_CAP, parametre.cap);
      }

void click_plus_1()
      {
      parametre.cap = cap_limit(parametre.cap+1);  
      bluetooth_send_param(OUTPUT_CAP, parametre.cap);
      }

void click_plus_10()
      {
      parametre.cap = cap_limit(parametre.cap+10);  
      bluetooth_send_param(OUTPUT_CAP, parametre.cap);
      }

void click_stanby()
      {
      parametre.stanby = !parametre.stanby;
      if(parametre.stanby)   bluetooth_send( OUTPUT_ACTIF );
      else                   bluetooth_send( OUTPUT_PAUSE );
      }

void click_kp_moins_pilote()
      {
      parametre.kp = k_limit(parametre.kp-1);
      bluetooth_send_param(OUTPUT_KP,parametre.kp);        
      }

void click_kp_plus_pilote()
      {
      parametre.kp = k_limit(parametre.kp+1);
      bluetooth_send_param(OUTPUT_KP,parametre.kp); 
      }
      
void click_ki_moins_pilote()
      {
      parametre.ki = k_limit(parametre.ki-1);
      bluetooth_send_param(OUTPUT_KI,parametre.ki);
      }

void click_ki_plus_pilote()
      {
      parametre.ki = k_limit(parametre.ki+1);
      bluetooth_send_param(OUTPUT_KI,parametre.ki);  
      }
      
void click_kd_moins_pilote()
      {
      parametre.kd = k_limit(parametre.kd-1);
      bluetooth_send_param(OUTPUT_KD,parametre.kd); 
      } 

void click_kd_plus_pilote()
      {
      parametre.kd = k_limit(parametre.kd+1);
      bluetooth_send_param(OUTPUT_KD,parametre.kd); 
      } 

void click_barre_max_moins()
      {
      parametre.barre_max -=1;
      bluetooth_send_param(OUTPUT_BARRE_MAX,parametre.barre_max); 
      } 

void click_barre_max_plus()
      {
      parametre.barre_max += 1;
      bluetooth_send_param(OUTPUT_BARRE_MAX,parametre.barre_max); 
      } 
      
void click_parametres()
      {
      bluetooth_send_param( OUTPUT_KP, parametre.kp );
      bluetooth_send_param( OUTPUT_KI, parametre.ki );
      bluetooth_send_param( OUTPUT_KD, parametre.kd );
      bluetooth_send_param( OUTPUT_BARRE_MAX, parametre.barre_max );    
      }
      
void click_cap()
      {
      bluetooth_send_param( OUTPUT_CAP, parametre.cap);
      if(parametre.stanby)   bluetooth_send( OUTPUT_ACTIF );
      else           bluetooth_send( OUTPUT_PAUSE );
      }
      
//******************************** Bluetooth ***************************

void  bluetooth(double param1, double param2, double param3)
      {
      bluetooth_check();

      bluetooth_send_param  ( OUTPUT_YAW,   param1 );
      bluetooth_send_param  ( OUTPUT_BARRE_BOAT, param2 );
      bluetooth_send_param  ( OUTPUT_BARRE,     param3 );
      bluetooth_send        ( OUTPUT_REDRAW );
      }
     
void  bluetooth_check()
      { 
      if(Bluetooth.available()>1)
        {
        if( Bluetooth.read() != INPUT_START ) return;
        
        switch( Bluetooth.read() )
              {
              case INPUT_PARAMETRES_REQUEST : click_parametres();
                                              return;

              case INPUT_CAP_REQUEST :        click_cap();
                                              return;
                                              
              case INPUT_CAP_MOINS_10 :       click_minus_10();
                                              break;                         
                          
              case INPUT_CAP_MOINS_1  :       click_minus_1();
                                              break;   
                            
              case INPUT_CAP_PLUS_1 :         click_plus_1();
                                              break;    
                           
              case INPUT_CAP_PLUS_10 :        click_plus_10();
                                              break;    
                          
              case INPUT_STANBY :             click_stanby();
                                              break;

              case INPUT_KP_MOINS :           click_kp_moins_pilote();
                                              break;
                           
              case INPUT_KP_PLUS :            click_kp_plus_pilote();
                                              break;
                           
              case INPUT_KI_MOINS :           click_ki_moins_pilote(); 
                                              break;
                           
              case INPUT_KI_PLUS :            click_ki_plus_pilote(); 
                                              break;
                           
              case INPUT_KD_MOINS    :        click_kd_moins_pilote();
                                              break;

              case INPUT_KD_PLUS :            click_kd_plus_pilote();
                                              break;
                                           
              case INPUT_BARRE_MAX_MOINS:     click_barre_max_moins();
                                              break;
 
              case INPUT_BARRE_MAX_PLUS :     click_barre_max_plus();
                                              break;
                                          
              case INPUT_PARAMETRES_RESET :   click_reset_parametres();
                                              click_parametres();
                                              break;
             }
          save_parametres();
        }
      }
     

  void bluetooth_send(char prefix)
    {
    Bluetooth.write(OUTPUT_START);
    Bluetooth.write(prefix);
    Bluetooth.println();
    }
    
void bluetooth_send_param(char prefix, double val)
    {
    Bluetooth.write(OUTPUT_START);
    Bluetooth.write(prefix);
    Bluetooth.print(val,0);
    Bluetooth.println();
    }

void bluetooth_send_info(String str, int val)
    {
    Bluetooth.write(OUTPUT_START);
    Bluetooth.write(OUTPUT_INFO);
    Bluetooth.print(str);
    Bluetooth.print(val);
    Bluetooth.println();
    }
    
//********************** EEPROM ************************************
      
void load_parametres()
      {
      EEPROM.get(0, parametre);
      pid_refresh();              
      }
      
void save_parametres()
      {
      EEPROM.put(0,parametre); 
      pid_refresh();              
      }
      
