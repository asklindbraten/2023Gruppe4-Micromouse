#include <Arduino.h>
#include <dfr0548.h>
#include <wire.h>

dfr0548* dfr0548::instance = NULL;

  const uint8_t dfr0548Addr = 0x40;
  const uint8_t MODE1 = 0x00;
  const uint8_t PRESCALE = 0xFE;
  const uint8_t SLEEP = 0x10;
  const uint8_t START = 0x20;
  const uint8_t SOFT_RESET  = 0x06;
  const uint8_t LED0_ON_L = 0x06;
  uint8_t bufferLF[8]={0,0,0,0,0,0,0,0};
  uint8_t bufferLB[8]={0,0,0,0,0,0,0,0};
  uint8_t bufferRF[8]={0,0,0,0,0,0,0,0};
  uint8_t bufferRB[8]={0,0,0,0,0,0,0,0};
dfr0548::dfr0548()
{
	
instance = this;
}

void dfr0548::Initialize ()
{
       //MANUAL: https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf
      //RESET. Since volatile memory, reset to all zero.
      //Note power down/up cycle is needed for all values to be lost.
      //Or we do a softreset

       Wire.beginTransmission(MODE1);
       Wire.write(SOFT_RESET); 
       Wire.endTransmission(); 
         
       Wire.beginTransmission(dfr0548Addr);
       Wire.write(MODE1); 
       Wire.write(MODE1); 
       Wire.endTransmission(); 
       
       
       Serial.println("Init Done");

    SetFrequencyHz(50);
}

 void dfr0548::SetFrequencyHz (int freq)
 {
      // compute prescaler based on desired frequency
      // see formula p.25, 50Hz has a prescaler of 121 = 0x79
      //prescaler := Compute_Prescaler_From_Frequency(Freq);

      //SLEEP. (set 4th bit high = 0x10) -> disable osc so we can write to prescale register (page 13-15)
      Wire.beginTransmission(dfr0548Addr);
      Wire.write(MODE1); 
      Wire.write(SLEEP); 
      Wire.endTransmission();
      
    delayMicroseconds (500);
      //SET PRESCALE.
      Wire.beginTransmission(dfr0548Addr);
      Wire.write(PRESCALE); 
      Wire.write(0x79); 
      Wire.endTransmission();
      
      delayMicroseconds (500);
      //RESTART
      Wire.beginTransmission(dfr0548Addr);
      Wire.write(MODE1); 
      Wire.write(START); 
      Wire.endTransmission();

       Serial.println("Set Freq Done");
}


void dfr0548::ConvertWheelPWMRegisters(uint8_t (& buffer) [8], int speedFwd, int speedBwd) {
      if ((speedFwd == 0) && (speedBwd == 0))  //stop motor
      {
        buffer[0] = 0x00;
        buffer[1] = 0x00;
        buffer[2] = 0x00;
        buffer[3] = 0x00;
        buffer[4] = 0x00;
        buffer[5] = 0x00;
        buffer[6] = 0x00;
        buffer[7] = 0x00;
        Serial.println("STOP");
     }
     else
     {
        if (speedFwd > 0) //Set motor spinning forward
        {
            // set PWM signal at time X. Time X can be set for both ON and OFF part of the signal
            // It is always ON at time 0 if value is 0, but it is OFF at time w.SpeedForward
            // this means that if w.SpeedForward has value 0, then it is immediately off,
            // and if it has the max value, 12bits is 4096 = FFF, then it is never off.
             buffer[0] = 0x00; //forward on L
             buffer[1] = 0x00; //forward on H
             buffer[2] = 0xFF; //forward off L
             buffer[3] = 0x0F; //forward off H

            //Buffer (3) := UInt8(w.SpeedForward and 16#FF#); -- FORWARD OFF L
            //Buffer (4) := UInt8(Shift_Right (UInt16(w.SpeedForward), 8)); -- FORWARD OFF H
          

             buffer[4] = 0x00; //backward on L
             buffer[5] = 0x00; //backward on H
             buffer[6] = 0x00; //backward off L
             buffer[7] = 0x00; //backward off H
              Serial.println("FORWARD");
        }
        else //Set motor spinning backward
        {
             buffer[0] = 0x00; //forward on L
             buffer[1] = 0x00; //forward on H
             buffer[2] = 0x00; //forward off L
             buffer[3] = 0x00; //forward off H

             buffer[4] = 0x00; //backward on L
             buffer[5] = 0x00; //backward on H
             buffer[6] = 0xFF; //backward off L
             buffer[7] = 0x0F; //backward off H
            
            //Buffer (7) := UInt8(w.SpeedBackward and 16#FF#); -- FORWARD OFF L
            //Buffer (8) := UInt8(Shift_Right (UInt16(w.SpeedBackward), 8)); -- FORWARD OFF H
             Serial.println("BACKWARD");
        }   
     }
  }

void dfr0548::SetPWMWheels(int rf_fwd, int rf_bwd, int rb_fwd, int rb_bwd, int lf_fwd, int lf_bwd, int lb_fwd, int lb_bwd)
 {
  Serial.println();
  Serial.println(rf_fwd);
  Serial.println(rf_bwd);
  
  ConvertWheelPWMRegisters (bufferRF, rf_fwd, rf_bwd);
  ConvertWheelPWMRegisters (bufferRB, rb_fwd, rb_bwd);
  ConvertWheelPWMRegisters (bufferLF, lf_fwd, lf_bwd);
  ConvertWheelPWMRegisters (bufferLB, lb_fwd, lb_bwd);
  
      Wire.beginTransmission(dfr0548Addr);
      Wire.write(LED0_ON_L); 
      Wire.write(bufferRF[0]); 
      Wire.write(bufferRF[1]); 
      Wire.write(bufferRF[2]); 
      Wire.write(bufferRF[3]); 
      Wire.write(bufferRF[4]); 
      Wire.write(bufferRF[5]); 
      Wire.write(bufferRF[6]); 
      Wire.write(bufferRF[7]); 
     
      Wire.write(bufferRB[0]); 
      Wire.write(bufferRB[1]); 
      Wire.write(bufferRB[2]); 
      Wire.write(bufferRB[3]); 
      Wire.write(bufferRB[4]); 
      Wire.write(bufferRB[5]); 
      Wire.write(bufferRB[6]); 
      Wire.write(bufferRB[7]); 

      Wire.write(bufferLF[0]); 
      Wire.write(bufferLF[1]); 
      Wire.write(bufferLF[2]); 
      Wire.write(bufferLF[3]); 
      Wire.write(bufferLF[4]); 
      Wire.write(bufferLF[5]); 
      Wire.write(bufferLF[6]); 
      Wire.write(bufferLF[7]); 

      Wire.write(bufferLB[0]); 
      Wire.write(bufferLB[1]); 
      Wire.write(bufferLB[2]); 
      Wire.write(bufferLB[3]); 
      Wire.write(bufferLB[4]); 
      Wire.write(bufferLB[5]); 
      Wire.write(bufferLB[6]); 
      Wire.write(bufferLB[7]); 
     
      Wire.endTransmission();

 Serial.println("Set Wheels Done");
 }