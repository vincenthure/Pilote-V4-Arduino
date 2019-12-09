
#include <PID_v1.h>
#include <AltSoftSerial.h>
#include <EEPROM.h>
#include <AnalogButtons.h>

#define   PIN_POUSSOIRS       A0
#define   PIN_EXTEND           3
#define   PIN_RETRACT          4
#define   BLUETOOTH_BAUD   57600
#define   NAVH_BAUD       115200

#define   BARRE_MAX_INIT      20
#define   KP_INIT             12
#define   KI_INIT              4
#define   KD_INIT             24
#define   THRESHOLD_INIT       2
#define   CAP_INIT             0

#define   PILOTE_LOOP_TIME   100

//*************** Serial ***************

#define OUTPUT_START             'Z'
#define OUTPUT_CAP               'A'
#define OUTPUT_REDRAW            'B'
#define OUTPUT_BARRE             'C'
#define OUTPUT_KP                'D'
#define OUTPUT_KI                'E'
#define OUTPUT_KD                'F'
#define OUTPUT_BARRE_BOAT        'G'
#define OUTPUT_YAW               'H'
#define OUTPUT_PAUSE             'I'
#define OUTPUT_ACTIF             'J'
#define OUTPUT_BARRE_MAX         'K'
#define OUTPUT_EXTEND            'L'
#define OUTPUT_RETRACT           'M'
#define OUTPUT_STOP              'N'
#define OUTPUT_INFO              'O'

#define INPUT_START              'Z'
#define INPUT_CAP_MOINS_10       'A'
#define INPUT_CAP_MOINS_1        'B'
#define INPUT_CAP_PLUS_1         'C'
#define INPUT_CAP_PLUS_10        'D'
#define INPUT_STANBY             'E'
#define INPUT_KP_MOINS           'F'
#define INPUT_KP_PLUS            'G'
#define INPUT_KI_MOINS           'H'
#define INPUT_KI_PLUS            'I'
#define INPUT_KD_MOINS           'J'
#define INPUT_KD_PLUS            'K'
#define INPUT_PARAMETRES_REQUEST 'L'
#define INPUT_CAP_REQUEST        'M'
#define INPUT_PARAMETRES_RESET   'N'
#define INPUT_BARRE_MAX_MOINS    'O'
#define INPUT_BARRE_MAX_PLUS     'P'


//**********************************************

#define VERIN_STOP     0
#define VERIN_EXTEND   1
#define VERIN_RETRACT  2

//**********************************************
struct navh
      {
      float roll,pitch,yaw,
            q0,q1,q2,q3,
            p,q,r,
            ax,ay,az,
            mx,my,mz,
            menu,cpu;
      };
      
struct parametres
      {
      double  kp,
              ki,
              kd,
              barre_max,
              threshold,
              cap;  
      };
      
//***********************************************

double     pid_gap    = 0, 
           pid_barre  = 0,
           pid_target = 0;

char       verin_state = VERIN_STOP;

boolean    pid_mode;

struct navh imu;
struct parametres parametre;

AltSoftSerial Bluetooth; // RX pin 8, TX pin 9, unusable PWM 10

PID pilote(&pid_gap, &pid_barre, &pid_target, parametre.kp, parametre.ki, parametre.kd, DIRECT);

AnalogButtons analogButtons(PIN_POUSSOIRS, INPUT);

//******************************************************************************

void setup()
      {
      // initialise Arduino

      Serial.begin(NAVH_BAUD);         // connection capteur IMU AHRS
      Bluetooth.begin(BLUETOOTH_BAUD); // connection Bluetooth
      pinMode(PIN_EXTEND, OUTPUT); 
      pinMode(PIN_RETRACT,OUTPUT);
                 
      // initialise PID

      pid_mode = true;
      pilote.SetMode(AUTOMATIC);
      pilote.SetOutputLimits(-parametre.barre_max, parametre.barre_max);
      pilote.SetSampleTime(PILOTE_LOOP_TIME); 
   
      load_parametres();   // restore parametres from EEprom 
      
      buttons_init();       // initialise les boutons
      }

void loop()
      {
      unsigned long end_time = millis() + PILOTE_LOOP_TIME;
      
      double barre_boat = SimulatorVerin(verin_state);     
      imu.yaw   = simulator_boat(barre_boat);  // input ange de barre reel , output cap du bateau
      //read_navh( &imu );
      //ask_navh();

      pid_gap = imu.yaw - parametre.cap;
      //Serial.println(pid_gap);
      if(pid_gap >  180) pid_gap -= 360;
      if(pid_gap < -180) pid_gap += 360;

      pilote.Compute();                       // input pid_gap, output pid_barre
      barre_Compute(barre_boat - pid_barre);  // input ecart de la barre, output commande verrin stop, extend, retract

      bluetooth(imu.yaw, barre_boat, pid_barre);  // envoi info à android pour le monitoring, check bouton pressé sur android
 
 // attends pour faire une boucle bien periodique en scannant les boutons
  
      while(millis() < end_time)      analogButtons.check();  
      }

void barre_Compute(double value)
    {
    static int lastState=0;
    verin_state = VERIN_STOP;

    if      (value >  parametre.threshold)       verin_state = VERIN_RETRACT; 
    else if (value < -parametre.threshold)       verin_state = VERIN_EXTEND; 

    if      ((lastState != VERIN_STOP)    && (verin_state == VERIN_STOP))     verin(VERIN_STOP);
    else if ((lastState != VERIN_EXTEND)  && (verin_state == VERIN_EXTEND))   verin(VERIN_EXTEND);
    else if ((lastState != VERIN_RETRACT) && (verin_state == VERIN_RETRACT))  verin(VERIN_RETRACT);

    lastState = verin_state; 
    }

void verin(int val)
      {
      switch(val)
          {
          case VERIN_STOP :    digitalWrite(PIN_RETRACT, LOW);
                               digitalWrite(PIN_EXTEND, LOW);
                               bluetooth_send( OUTPUT_STOP ); 
                               break;

         case VERIN_EXTEND  :  digitalWrite(PIN_RETRACT, LOW);
                               digitalWrite(PIN_EXTEND, HIGH); 
                               bluetooth_send( OUTPUT_EXTEND ); 
                               break; 

          case VERIN_RETRACT : digitalWrite(PIN_RETRACT,HIGH);
                               digitalWrite(PIN_EXTEND,  LOW);
                               bluetooth_send( OUTPUT_RETRACT );  
                               break; 
          }
      }
      
void reset_parametres()
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
      save_parametres();
      }

void click_minus_1()
      {
      parametre.cap = cap_limit(parametre.cap-1); 
      bluetooth_send_param(OUTPUT_CAP, parametre.cap);
      save_parametres();
      }

void click_plus_1()
      {
      parametre.cap = cap_limit(parametre.cap+1);  
      bluetooth_send_param(OUTPUT_CAP, parametre.cap);
      save_parametres();
      }

void click_plus_10()
      {
      parametre.cap = cap_limit(parametre.cap+10);  
      bluetooth_send_param(OUTPUT_CAP, parametre.cap);
      save_parametres();
      }

void click_stanby()
      {
      pid_mode = !pid_mode;
      if(pid_mode)      bluetooth_send( OUTPUT_ACTIF );
      else              bluetooth_send( OUTPUT_PAUSE );
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
      if(pid_mode)   bluetooth_send( OUTPUT_ACTIF );
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
              case INPUT_CAP_MOINS_10 :   click_minus_10();
                                          return;                         
                          
              case INPUT_CAP_MOINS_1  :   click_minus_1();
                                          return;   
                            
              case INPUT_CAP_PLUS_1 :     click_plus_1();
                                          return;    
                           
              case INPUT_CAP_PLUS_10 :    click_plus_10();
                                          return;    
                          
              case INPUT_STANBY :         click_stanby();
                                          return;

              case INPUT_PARAMETRES_REQUEST :    
                                          click_parametres();
                                          return;

              case INPUT_CAP_REQUEST :    click_cap();
                                          return;

              case INPUT_KP_MOINS :       click_kp_moins_pilote();
                                          break;
                           
              case INPUT_KP_PLUS :        click_kp_plus_pilote();
                                          break;
                           
              case INPUT_KI_MOINS :       click_ki_moins_pilote(); 
                                          break;
                           
              case INPUT_KI_PLUS :        click_ki_plus_pilote(); 
                                          break;
                           
              case INPUT_KD_MOINS    :    click_kd_moins_pilote();
                                          break;

              case INPUT_KD_PLUS :        click_kd_plus_pilote();
                                          break;
                                           
              case INPUT_BARRE_MAX_MOINS: click_barre_max_moins();
                                          break;
 
              case INPUT_BARRE_MAX_PLUS : click_barre_max_plus();
                                          break;
                                          
              case INPUT_PARAMETRES_RESET :      reset_parametres();
                                          click_parametres();
                                          break;
             }
          save_parametres();
        }
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
    
void bluetooth_send(char prefix)
    {
    Bluetooth.write(OUTPUT_START);
    Bluetooth.write(prefix);
    Bluetooth.println();
    }
    
//********************** EEPROM ************************************
      
void load_parametres()
      {
      EEPROM.get(0, parametre);
      pilote.SetTunings(parametre.kp, parametre.ki, parametre.kd);   
      pilote.SetOutputLimits(-parametre.barre_max, parametre.barre_max);              
      }
      
void save_parametres()
      {
      EEPROM.put(0,parametre); 
      pilote.SetTunings(parametre.kp, parametre.ki, parametre.kd);   
      pilote.SetOutputLimits(-parametre.barre_max, parametre.barre_max);              
      }

//************************************************************************

int read_navh( navh *buffer )
      {
      while(1)
          {
          char c = Serial.read();
          if(c==-1)    return 0;
          if(c=='F')
              if(Serial.read() == 18)   return Serial.readBytes((byte*)buffer,68); 
              else                      return 0;
          }
      }
      
void ask_navh()
    {
    Serial.write('F');  
    }

//**************************** Simulator **************************

#define K_BOAT  0.0004

double simulator_boat(double angle_barre)
      {
      static double heading = 0;

      heading += K_BOAT * PILOTE_LOOP_TIME * angle_barre;
      return heading;
      }

#define K_VERIN 0.01

double SimulatorVerin(char action)
    {
    static double Barre = 0;
    switch(action)
        {
        case VERIN_EXTEND  : Barre += K_VERIN * PILOTE_LOOP_TIME;
                             break;
                             
        case VERIN_RETRACT : Barre -= K_VERIN * PILOTE_LOOP_TIME;
                             break;        
        }
    return Barre;
    }
        

    
