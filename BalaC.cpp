#include "BalaC.h"

#define BALAC_ADDRESS  0X38
#define BALAC_SDA      0
#define BALAC_SCL      26
#define BALAC_IICFREQ  400000


void IIC_Write_1bytes(uint8_t address,uint8_t Register_address,uint8_t data)
{
    Wire.beginTransmission(address);
    Wire.write(Register_address);
    Wire.write(data);
    Wire.endTransmission();
}

void IIC_Write_2bytes(uint8_t address,uint8_t Register_address,uint16_t data)
{
    Wire.beginTransmission(address);
    Wire.write(Register_address);
    Wire.write(data >> 8); //MSB
    Wire.write(data & 0xFF); //LSB
    Wire.endTransmission();
}

uint8_t readBytes(uint8_t address, uint8_t subAddress, uint8_t count,uint8_t * dest) 
{
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    uint8_t i = 0;
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(address, (uint8_t)count)) 
    {
        while (Wire.available()) 
        {
            dest[i++]=Wire.read();
        }
        return true;
    }
    return false;
}

/*******************************************************************************/


void BalaC_Init()
{
  Wire.begin(BALAC_SDA, BALAC_SCL, BALAC_IICFREQ);
  BalaC_SetPowerA(0);
  BalaC_SetPowerB(0);
}

void BalaC_SetPowerA(int8_t power)
{
  if (power!=0) power += 7 *(power/abs(power));
  power = constrain(power, -100, 100);
  IIC_Write_1bytes(BALAC_ADDRESS, 0, power);
}

void BalaC_SetPowerB(int8_t power)
{
  power = constrain(power, -100, 100);
  IIC_Write_1bytes(BALAC_ADDRESS, 1, power);
}
