
#include <PID_v1.h>
#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include <EEPROM.h>
#include <AnalogButtons.h>

#define   pin_Poussoirs     A0
#define   pin_Pwm            3
#define   pin_Reverse        4
#define   pin_Rx_Imu        12
#define   pin_Tx_Imu        13
#define   pin_Rx_Bluetooth   8
#define   pin_Tx_Bluetooth   9

#define   barre_max_init    45

#define   Kp_pilote_init    16
#define   Ki_pilote_init     3
#define   Kd_pilote_init    58
#define   Kp_moteur_init     6
#define   Ki_moteur_init     0 
#define   Kd_moteur_init     0

#define   pilote_loop_time 100

#define   tension_max      254
#define   tension_min     -254

#define   K_verin      0.00006
#define   K_boat        0.0002

//*************** output Serial ***************

#define prefix_cap            'A'
#define prefix_redraw         'B'
#define prefix_tensionMoteur  'C'
#define prefix_angleBarre     'D'
#define prefix_kp             'E'
#define prefix_ki             'F'
#define prefix_kd             'G'
#define prefix_angleBarreReel 'H'
#define prefix_capReel        'I'
#define prefix_kp2            'J'
#define prefix_ki2            'K'
#define prefix_kd2            'L'
#define prefix_pause          'M'
#define prefix_actif          'N'
#define prefix_barre_max      'O'

//**************** Input Serial ****************

#define prefix_cap_moins_10    'A'
#define prefix_cap_moins_1     'B'
#define prefix_cap_plus_1      'C'
#define prefix_cap_plus_10     'D'
#define prefix_stanby          'E'
#define prefix_kp_moins        'F'
#define prefix_kp_plus         'G'
#define prefix_ki_moins        'H'
#define prefix_ki_plus         'I'
#define prefix_kd_moins        'J'
#define prefix_kd_plus         'K'
#define prefix_pid_request     'L'
#define prefix_cap_request     'M'
#define prefix_kp2_moins       'N'
#define prefix_kp2_plus        'O'
#define prefix_ki2_moins       'P'
#define prefix_ki2_plus        'Q'
#define prefix_kd2_moins       'R'
#define prefix_kd2_plus        'S'
#define prefix_reset_pid       'T'
#define prefix_barre_max_moins 'U'
#define prefix_barre_max_plus  'V'

//**********************************************

double     Cap               = 0,
           Cap_Reel          = 0,
           Zero              = 0,
           Ecart_Cap         = 0,
           Tension_Moteur    = 0,
           Angle_Barre       = 0,
           Angle_Barre_Reel  = 0,
           Ecart_Angle_Barre = 0,
          
           Kp_moteur         = Kp_moteur_init,
           Ki_moteur         = Ki_moteur_init,
           Kd_moteur         = Kd_moteur_init,
           
           Kp_pilote         = Kp_pilote_init, 
           Ki_pilote         = Ki_pilote_init, 
           Kd_pilote         = Kd_pilote_init,
           
           barre_max         = barre_max_init;
           
boolean    PID_mode;

SoftwareSerial Navh(pin_Rx_Imu, pin_Tx_Imu); // RX, TX
AltSoftSerial Bluetooth; // RX, TX
PID pilote(&Ecart_Cap, &Angle_Barre, &Zero, Kp_pilote, Ki_pilote, Kd_pilote, DIRECT);
PID moteur(&Ecart_Angle_Barre, &Tension_Moteur, &Zero , Kp_moteur, Ki_moteur, Kd_moteur, DIRECT);
AnalogButtons analogButtons(pin_Poussoirs, INPUT);

//******************************************************************************

void setup()
      {
      // initialise Arduino

      Serial.begin(9600);
      Navh.begin(115200);    // connection capteur IMU AHRS
      Bluetooth.begin(115200); // connection Bluetooth
      pinMode(pin_Reverse, INPUT_PULLUP); 
      pinMode(pin_Pwm, INPUT_PULLUP);
                 
      // initialise PID

      PID_mode = true;
      pilote.SetMode(AUTOMATIC);
      pilote.SetOutputLimits(-barre_max, barre_max);
      pilote.SetSampleTime(pilote_loop_time); 
  
      moteur.SetMode(AUTOMATIC);
      moteur.SetOutputLimits(tension_min, tension_max);
      moteur.SetSampleTime(pilote_loop_time); 

      // restore parametres from EEprom

      load_Kparam();    
      load_cap();
      
      buttons_init();  // initialise les boutons
      }

void loop()
      {
      unsigned long starttime = millis();
      unsigned long duration = 0;
      
      Cap_Reel = simulatorBoat(Angle_Barre_Reel);  // input ange de barre reel , output cap du bateau
      //Cap_reel = read_cap();
      
      Ecart_Cap = Cap_Reel - Cap;
      if( Ecart_Cap >  180 ) Ecart_Cap -= 360;
      if( Ecart_Cap < -180 ) Ecart_Cap += 360;
 
      pilote.Compute();  // input ecart par rapport au cap, output angle de barre
      
      Angle_Barre_Reel = simulatorVerin(Tension_Moteur);  // input tension moteur, output angle de barre reel
      Ecart_Angle_Barre = Angle_Barre_Reel - Angle_Barre;

      moteur.Compute();   // input ecart angle de barre , output tension moteur
      
      if (PID_mode)     drive_motor();

      bluetooth();

      while  (duration<pilote_loop_time)   
          {
          duration = millis() - starttime;
          analogButtons.check();
          }
      }

void reset_PID()
      {
      Kp_moteur = Kp_moteur_init,
      Ki_moteur = Ki_moteur_init,
      Kd_moteur = Kd_moteur_init,
           
      Kp_pilote = Kp_pilote_init, 
      Ki_pilote = Ki_pilote_init, 
      Kd_pilote = Kd_pilote_init;

      barre_max = barre_max_init;
      }

double Cap_limit( double val )
      {
      if(val<0)     val+=360;
      if(val>359)   val-=360;
      return val;  
      }
  
double K_limit( double val )
      {
      val=max(val,0);
      val=min(val,254);
      return val;  
      }
      
void drive_motor()
      {
      (Tension_Moteur<0)?digitalWrite(pin_Reverse, HIGH):digitalWrite(pin_Reverse, LOW); 
      analogWrite(pin_Pwm, abs(Tension_Moteur)); 
      }
          
//***************************** Button ****************************************

Button btn1 = Button(510, &click_minus_10, &click_minus_10, 1000, 500);
Button btn2 = Button(681, &click_minus_1,  &click_minus_1,  1000, 500);
Button btn3 = Button(768, &click_plus_1,   &click_plus_1,   1000, 500);
Button btn4 = Button(819, &click_plus_10,  &click_plus_10,  1000, 500);
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
      Cap = Cap_limit(Cap-10);
      if(Serial) bluetooth_send_param(prefix_cap, Cap);
      save_cap();
      }

void click_minus_1()
      {
      Cap = Cap_limit(Cap-1); 
      if(Serial) bluetooth_send_param(prefix_cap, Cap);
      save_cap();
      }

void click_plus_1()
      {
      Cap = Cap_limit(Cap+1);  
      if(Serial) bluetooth_send_param(prefix_cap, Cap);
      save_cap();
      }

void click_plus_10()
      {
      Cap = Cap_limit(Cap+10);  
      if(Serial) bluetooth_send_param(prefix_cap, Cap);
      }

void click_stanby()
      {
      PID_mode = !PID_mode;
      if(PID_mode)      bluetooth_send( prefix_actif );
      else              bluetooth_send( prefix_pause );
      }

void click_kp_moins_pilote()
      {
      Kp_pilote = K_limit(Kp_pilote-1);
      bluetooth_send_param(prefix_kp,Kp_pilote);        
      }

void click_kp_plus_pilote()
      {
      Kp_pilote = K_limit(Kp_pilote+1);
      bluetooth_send_param(prefix_kp,Kp_pilote); 
      }
      
void click_ki_moins_pilote()
      {
      Ki_pilote = K_limit(Ki_pilote-1);
      bluetooth_send_param(prefix_ki,Ki_pilote);
      }

void click_ki_plus_pilote()
      {
      Ki_pilote = K_limit(Ki_pilote+1);
      bluetooth_send_param(prefix_ki,Ki_pilote);  
      }
      
void click_kd_moins_pilote()
      {
      Kd_pilote = K_limit(Kd_pilote-1);
      bluetooth_send_param(prefix_kd,Kd_pilote); 
      } 

void click_kd_plus_pilote()
      {
      Kd_pilote = K_limit(Kd_pilote+1);
      bluetooth_send_param(prefix_kd,Kd_pilote); 
      } 

void click_kp_moins_moteur()
      {
      Kp_moteur = K_limit(Kp_moteur-1);
      bluetooth_send_param(prefix_kp2,Kp_moteur);        
      }

void click_kp_plus_moteur()
      {
      Kp_moteur = K_limit(Kp_moteur+1);
      bluetooth_send_param(prefix_kp2,Kp_moteur); 
      }
      
void click_ki_moins_moteur()
      {
      Ki_moteur = K_limit(Ki_moteur-1);
      bluetooth_send_param(prefix_ki2,Ki_moteur);
      }

void click_ki_plus_moteur()
      {
      Ki_moteur = K_limit(Ki_moteur+1);
      bluetooth_send_param(prefix_ki2,Ki_moteur);  
      }
      
void click_kd_moins_moteur()
      {
      Kd_moteur = K_limit(Kd_moteur-1);
      bluetooth_send_param(prefix_kd2,Kd_moteur); 
      } 

void click_kd_plus_moteur()
      {
      Kd_moteur = K_limit(Kd_moteur+1);
      bluetooth_send_param(prefix_kd2,Kd_moteur); 
      } 

void click_barre_max_moins()
      {
      barre_max -=1;
      bluetooth_send_param(prefix_barre_max,barre_max); 
      } 

void click_barre_max_plus()
      {
      barre_max += 1;
      bluetooth_send_param(prefix_barre_max,barre_max); 
      } 
      
void click_PID()
      {
      bluetooth_send_param( prefix_kp,        Kp_pilote );
      bluetooth_send_param( prefix_ki,        Ki_pilote );
      bluetooth_send_param( prefix_kd,        Kd_pilote );
      bluetooth_send_param( prefix_kp2,       Kp_moteur );
      bluetooth_send_param( prefix_ki2,       Ki_moteur );
      bluetooth_send_param( prefix_kd2,       Kd_moteur ); 
      bluetooth_send_param( prefix_barre_max, barre_max );    
      }
      
void click_cap()
      {
      bluetooth_send_param( prefix_cap, Cap);
      if(PID_mode)      bluetooth_send( prefix_actif );
      else              bluetooth_send( prefix_pause );
      }
      
//******************************** Bluetooth ***************************

void  bluetooth()
      {
      bluetooth_check();
      bluetooth_send_param  ( prefix_tensionMoteur,  Tension_Moteur   );
      bluetooth_send_param  ( prefix_capReel,        Cap_Reel         );
      bluetooth_send_param  ( prefix_angleBarreReel, Angle_Barre_Reel );
      bluetooth_send_param  ( prefix_angleBarre,     Angle_Barre      );
      bluetooth_send        ( prefix_redraw );
      }
      
void  bluetooth_check()
      { 
      if(Bluetooth.available())
      
        switch( Bluetooth.read() )
              {
              case prefix_cap_moins_10 :   click_minus_10();
                                           return;                         
                          
              case prefix_cap_moins_1  :   click_minus_1();
                                           return;   
                            
              case prefix_cap_plus_1 :     click_plus_1();
                                           return;    
                           
              case prefix_cap_plus_10 :    click_plus_10();
                                           return;    
                          
              case prefix_stanby :         click_stanby();
                                           return;

              case prefix_pid_request :    click_PID();
                                           return;

              case prefix_cap_request :    click_cap();
                                           return;

              case prefix_kp_moins :       click_kp_moins_pilote();
                                           break;
                           
              case prefix_kp_plus :        click_kp_plus_pilote();
                                           break;
                           
              case prefix_ki_moins :       click_ki_moins_pilote(); 
                                           break;
                           
              case prefix_ki_plus :        click_ki_plus_pilote(); 
                                           break;
                           
              case prefix_kd_moins    :    click_kd_moins_pilote();
                                           break;

              case prefix_kd_plus :        click_kd_plus_pilote();
                                           break;
                                          
              case prefix_kp2_moins :      click_kp_moins_moteur();
                                           break;
                           
              case prefix_kp2_plus :       click_kp_plus_moteur();
                                           break;
                           
              case prefix_ki2_moins :      click_ki_moins_moteur(); 
                                           break;
                           
              case prefix_ki2_plus :       click_ki_plus_moteur(); 
                                           break;
                           
              case prefix_kd2_moins:       click_kd_moins_moteur();
                                           break;

              case prefix_kd2_plus :       click_kd_plus_moteur();
                                           break;
                                           
              case prefix_barre_max_moins: click_barre_max_moins();
                                           break;
 
              case prefix_barre_max_plus : click_barre_max_plus();
                                           break;
                                          
              case prefix_reset_pid :      reset_PID();
                                           click_PID();
                                           break;
             }
          save_Kparam();
      }
   
void bluetooth_send_param(char prefix, double val)
    {

        Bluetooth.write(prefix);
        Bluetooth.print(val,0);
        Bluetooth.println();

    }

void bluetooth_send(char prefix)
    {

        Bluetooth.write(prefix);
        Bluetooth.println();
 
    }
    
//********************** EEPROM ************************************

void load_cap()
      {
      Cap = (double)EEPROM.read(7);  
      }
      
void save_cap()
      {
      EEPROM.write(7,(byte)Cap);  
      }
      
void load_Kparam()
      {
      Kp_pilote = (double)EEPROM.read(0);
      Ki_pilote = (double)EEPROM.read(1);
      Kd_pilote = (double)EEPROM.read(2);
      if(Kp_pilote!=255 && Ki_pilote!=255 && Kd_pilote!=255) 
                    pilote.SetTunings(Kp_pilote, Ki_pilote, Kd_pilote);
  
      Kp_moteur = (double)EEPROM.read(3);
      Ki_moteur = (double)EEPROM.read(4);
      Kd_moteur = (double)EEPROM.read(5);
      if(Kp_moteur!=255 && Ki_moteur!=255 && Kd_moteur!=255) 
                    moteur.SetTunings(Kp_moteur, Ki_moteur, Kd_moteur);
                    
      barre_max = (double)EEPROM.read(6);
      if(barre_max!=255)
                  pilote.SetOutputLimits(-barre_max, barre_max);              
      }
      
void save_Kparam()
      {
      EEPROM.write(0,(byte)Kp_pilote);  
      EEPROM.write(1,(byte)Ki_pilote);  
      EEPROM.write(2,(byte)Kd_pilote); 
      EEPROM.write(3,(byte)Kp_moteur);  
      EEPROM.write(4,(byte)Ki_moteur);  
      EEPROM.write(5,(byte)Kd_moteur);
      EEPROM.write(6,(byte)barre_max);  

      pilote.SetTunings(Kp_pilote, Ki_pilote, Kd_pilote);
      moteur.SetTunings(Kp_moteur, Ki_moteur, Kd_moteur);
      pilote.SetOutputLimits(-barre_max, barre_max); 
      }

//************************************************************************

struct data
{
char f;
char n;
float Phi,Theta,Psi,Q0,Q1,Q2,Q3,P,Q,R,Ax,Ay,Az,Ma,Mb,Mz,Menu,Cpu;  
};

float read_Cap()
      {
      struct data val;
      char buffer[4];
      
      while( Navh.read() != -1 );   //vide le buffer du port serie
      Navh.write('F');            // envoie une requete

Serial.readBytes(buffer,74);

      if( Navh.read() != 'F' ) return -1;   //verifie le premier byte
      Navh.read();                          // lit le second byte
      for(int i=0; i<4; i++) buffer[i]=Navh.read();  //lit les quatres bytes suivant
   
      return *(float *)(buffer);
      }

//**************************** Simulator **************************

double  old_angle_barre   = 0,
        old_Cap           = 0;
        
double  simulatorVerin(double tension)
      {
      double alpha = old_angle_barre + ( K_verin * pilote_loop_time * tension );
      if (alpha > barre_max)      alpha = barre_max;
      if (alpha < -barre_max)     alpha = -barre_max;
      old_angle_barre = alpha;
      return alpha;
      }
    
double simulatorBoat(double alpha)
      {
      double new_Cap = old_Cap + ( K_boat * pilote_loop_time * alpha);
      old_Cap = new_Cap;
      return new_Cap;
      }
        

    
