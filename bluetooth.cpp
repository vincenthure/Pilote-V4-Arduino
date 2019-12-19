#include "bluetooth.h"
#include "Arduino.h"

Bluetooth::Bluetooth(parametres* param)
     {
     parametre = param; 
     }

void Bluetooth::begin()
     {
     arduino.begin(BLUETOOTH_BAUD);  
     }
     
boolean Bluetooth::arduino_scan()
      {
      if(arduino.available()>1)
        {
        if( arduino.read() != INPUT_START ) return;
        
        switch( arduino.read() )
              {
              case INPUT_PARAMETRES_REQUEST : click_parametres();
                                              return false;

              case INPUT_CAP_REQUEST :        click_cap();
                                              return false;
                                              
              case INPUT_CAP_MOINS_10 :       click_minus_10();
                                              return true;                         
                          
              case INPUT_CAP_MOINS_1  :       click_minus_1();
                                              return true;   
                            
              case INPUT_CAP_PLUS_1 :         click_plus_1();
                                              return true;    
                           
              case INPUT_CAP_PLUS_10 :        click_plus_10();
                                              return true;    
                          
              case INPUT_STANBY :             click_stanby();
                                              return true;

              case INPUT_KP_MOINS :           click_kp_moins_pilote();
                                              return true;
                           
              case INPUT_KP_PLUS :            click_kp_plus_pilote();
                                              return true;
                           
              case INPUT_KI_MOINS :           click_ki_moins_pilote(); 
                                              return true;
                           
              case INPUT_KI_PLUS :            click_ki_plus_pilote(); 
                                              return true;
                           
              case INPUT_KD_MOINS    :        click_kd_moins_pilote();
                                              return true;

              case INPUT_KD_PLUS :            click_kd_plus_pilote();
                                              return true;
                                           
              case INPUT_BARRE_MAX_MOINS:     click_barre_max_moins();
                                              return true;
 
              case INPUT_BARRE_MAX_PLUS :     click_barre_max_plus();
                                              return true;
                                          
              case INPUT_PARAMETRES_RESET :   click_reset_parametres();
                                              click_parametres();
                                              return true;
             }
        }
      }
 

void Bluetooth::click_minus_10()
      {
      parametre->cap = cap_limit(parametre->cap-10);
      send_param(OUTPUT_CAP, parametre->cap);
      }

void Bluetooth::click_minus_1()
      {
      parametre->cap = cap_limit(parametre->cap-1); 
      send_param(OUTPUT_CAP, parametre->cap);
      }

void Bluetooth::click_plus_1()
      {
      parametre->cap = cap_limit(parametre->cap+1);  
      send_param(OUTPUT_CAP, parametre->cap);
      }

void Bluetooth::click_plus_10()
      {
      parametre->cap = cap_limit(parametre->cap+10);  
      send_param(OUTPUT_CAP, parametre->cap);
      }

void Bluetooth::click_stanby()
      {
      parametre->stanby = !parametre->stanby;
      if(parametre->stanby)   send_( OUTPUT_PAUSE );
      else                    send_( OUTPUT_ACTIF );
      }

void Bluetooth::click_kp_moins_pilote()
      {
      parametre->kp = k_limit(parametre->kp-1);
      send_param(OUTPUT_KP,parametre->kp);        
      }

void Bluetooth::click_kp_plus_pilote()
      {
      parametre->kp = k_limit(parametre->kp+1);
      send_param(OUTPUT_KP,parametre->kp); 
      }
      
void Bluetooth::click_ki_moins_pilote()
      {
      parametre->ki = k_limit(parametre->ki-1);
      send_param(OUTPUT_KI,parametre->ki);
      }

void Bluetooth::click_ki_plus_pilote()
      {
      parametre->ki = k_limit(parametre->ki+1);
      send_param(OUTPUT_KI,parametre->ki);  
      }
      
void Bluetooth::click_kd_moins_pilote()
      {
      parametre->kd = k_limit(parametre->kd-1);
      send_param(OUTPUT_KD,parametre->kd); 
      } 

void Bluetooth::click_kd_plus_pilote()
      {
      parametre->kd = k_limit(parametre->kd+1);
      send_param(OUTPUT_KD,parametre->kd); 
      } 

void Bluetooth::click_barre_max_moins()
      {
      parametre->barre_max -=1;
      send_param(OUTPUT_BARRE_MAX,parametre->barre_max); 
      } 

void Bluetooth::click_barre_max_plus()
      {
      parametre->barre_max += 1;
      send_param(OUTPUT_BARRE_MAX,parametre->barre_max); 
      } 
      
void Bluetooth::click_parametres()
      {
      send_param( OUTPUT_KP, parametre->kp );
      send_param( OUTPUT_KI, parametre->ki );
      send_param( OUTPUT_KD, parametre->kd );
      send_param( OUTPUT_BARRE_MAX, parametre->barre_max );    
      }
      
void Bluetooth::click_cap()
      {
      send_param( OUTPUT_CAP, parametre->cap);
      if(parametre->stanby)    send_( OUTPUT_ACTIF );
      else                     send_( OUTPUT_PAUSE );
      }
 
void Bluetooth::arduino_refresh(double yaw, double capteur_barre, double barre )
    {
    send_param  ( OUTPUT_YAW,   yaw );
    send_param  ( OUTPUT_BARRE_BOAT, capteur_barre );
    send_param  ( OUTPUT_BARRE,     barre );
    send_       ( OUTPUT_REDRAW ); 
    }

void Bluetooth::click_reset_parametres()
      {
      parametre->kp        = KP_INIT,
      parametre->ki        = KI_INIT,
      parametre->kd        = KD_INIT,
      parametre->barre_max = BARRE_MAX_INIT;
      parametre->threshold = THRESHOLD_INIT;
      }

double Bluetooth::cap_limit( double val )
      {
      if(val<0)     val+=360;
      if(val>359)   val-=360;
      return val;  
      }
  
double Bluetooth::k_limit( double val )
      {
      val=max(val,0);
      val=min(val,254);
      return val;  
      }
      
void Bluetooth::send_(char prefix)
    {
    arduino.write(OUTPUT_START);
    arduino.write(prefix);
    arduino.println();
    }
    
void Bluetooth::send_param(char prefix, double val)
    {
    arduino.write(OUTPUT_START);
    arduino.write(prefix);
    arduino.print(val,0);
    arduino.println();
    }

void Bluetooth::send_info(String str, int val)
    {
    arduino.write(OUTPUT_START);
    arduino.write(OUTPUT_INFO);
    arduino.print(str);
    arduino.print(val);
    arduino.println();
    }

void Bluetooth::stop()
    {
    send_(OUTPUT_STOP);
    }

void Bluetooth::extend()
    {
    send_(OUTPUT_EXTEND);
    }

void Bluetooth::retract()
    {
    send_(OUTPUT_RETRACT);
    }
