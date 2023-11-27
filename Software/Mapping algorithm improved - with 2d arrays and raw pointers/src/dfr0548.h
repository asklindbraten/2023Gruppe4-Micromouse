#include <Arduino.h>

class dfr0548
{

public:
static dfr0548    *instance;  // A singleton reference, used purely by the interrupt service routine.

    dfr0548();


    void Initialize();
    //for example rf_fwd = 4095 and rf_bwd = 0 to move wheel rf forward 
              //  rf_fwd = 0    and rf_bwd = 4095 to move wheel rf backward
              //  anything higher than 1 will map to 4095. 0 is stop
    void SetPWMWheels(int rf_fwd, int rf_bwd, int rb_fwd, int rb_bwd, int lf_fwd, int lf_bwd, int lb_fwd, int lb_bwd); 

private:
    void SetFrequencyHz (int freq);
    void ConvertWheelPWMRegisters(uint8_t (& buffer) [8], int speedFwd, int speedBwd);
};