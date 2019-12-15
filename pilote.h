#ifndef pilote_h
#define pilote_h

#define   PIN_POUSSOIRS       A0
#define   PIN_EXTEND           3
#define   PIN_RETRACT          4
#define   BLUETOOTH_BAUD   57600
#define   BARRE_MAX_INIT      20
#define   KP_INIT             12
#define   KI_INIT              4
#define   KD_INIT             24
#define   THRESHOLD_INIT       2
#define   CAP_INIT             0
#define   STANBY_INIT          1

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

#endif
