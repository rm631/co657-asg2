#include "LM75B.h"

LM75B::LM75B(PinName sda, PinName scl) : i2c(sda, scl)
{
   char cmd[2];
   cmd[0]    = LM75B_Conf;
   cmd[1]    = 0x0;   
   i2c.write( LM75B_ADDR, cmd, 2);
}



LM75B::~LM75B()
{

}

uint16_t LM75B::read()
{
    union {
        char cmd[2];
        uint16_t tmp;
    } x;
    x.cmd[0] = LM75B_Temp;
   
    i2c.write( LM75B_ADDR, x.cmd, 1); // Send command string
    i2c.read( LM75B_ADDR, x.cmd, 2); // Send command string    
    return x.tmp;
}
