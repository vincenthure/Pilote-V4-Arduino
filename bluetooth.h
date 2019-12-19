#ifndef bluetooth_h
#define bluetooth_h

#include <AltSoftSerial.h>

#define BLUETOOTH_BAUD        57600

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
#define OUTPUT_THRESHOLD         'P'

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
#define INPUT_THRESHOLD_MOINS    'Q'
#define INPUT_THRESHOLD_PLUS     'R'

#define   BARRE_MAX_INIT      20
#define   KP_INIT             12
#define   KI_INIT              4
#define   KD_INIT             24
#define   THRESHOLD_INIT       2
#define   CAP_INIT             0
#define   STANBY_INIT          1


struct parametres
      {
      double  kp,
              ki,
              kd,
              barre_max,
              threshold,
              cap;
      char    stanby;  
      };

class Bluetooth
  {
  public:
  
    Bluetooth( parametres*); 

    void     arduino_refresh(double,double,double);
    boolean  arduino_scan();
    void     begin();
    void click_minus_10();
    void click_minus_1();
    void click_plus_1();
    void click_plus_10();
    void click_stanby();
    void stop();
    void extend();
    void retract(); 
 
       
  private:

    AltSoftSerial arduino; // RX pin 8, TX pin 9, unusable PWM 10
    parametres *parametre;
    
    void click_kp_moins_pilote();
    void click_kp_plus_pilote();
    void click_ki_moins_pilote();
    void click_ki_plus_pilote();
    void click_kd_moins_pilote();
    void click_kd_plus_pilote();
    void click_barre_max_moins();
    void click_barre_max_plus();
    void click_threshold_moins();
    void click_threshold_plus();
    void click_parametres();
    void click_cap();
    void click_reset_parametres();
    double cap_limit( double );
    double k_limit( double );
    
    void  send_(char);
    void  send_param(char, double);
    void  send_info(String, int);
  };

#endif
