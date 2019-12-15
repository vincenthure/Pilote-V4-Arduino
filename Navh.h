
#ifndef Navh_h
#define Navh_h

#define   NAVH_BAUD  115200

struct navh
      {
      float roll,pitch,yaw,
            q0,q1,q2,q3,
            p,q,r,
            ax,ay,az,
            mx,my,mz,
            menu,cpu;
      };

class Navh
  {
  public:

    Navh( navh* );
    boolean read( void );
    void    ask( void );

  private:

    navh *data;
  };

#endif
