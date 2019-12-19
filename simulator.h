#ifndef simulator_h
#define simulator_h

#define K_BOAT       0.0004
#define K_VERIN      0.01
#define TIME_RAMPE 300

class Simulator
    {
    public:

    Simulator(unsigned long);
    double boat(double);
    double verin(char);

    private:

    double  kboat,
            kverin;
    };

#endif
