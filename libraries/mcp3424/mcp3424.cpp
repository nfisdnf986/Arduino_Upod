// library for interface with mcp3424 ADC
// library is lean; i.e. constrained to PGA=1, res=18bit

#include "Wire.h"
#include "mcp3424.h"

#define MCP3424_COMMON_ADDR 0x68
#define CONFIG_REGISTER 0x8C;
char address;

mcp3424::mcp3424()
{
	Wire.begin();
}

void mcp3424::GetAddress(char Addr0, char Addr1)
{
 //take argument A0 and A1, and define the address used by I2C to address MCP3424
 //valid inputs 'G' grounded, 'H' high, 'F' float
 //return full address (appends the COMMON_ADDR)
 //NOTE: both floating is same as both grounded
/*
A2  A1  A0||Addr0  Addr1
0   0   0 ||  0    0
0   0   1 ||  0    F
0   1   0 ||  0    1
1   0   0 ||  1    0
1   0   1 ||  1    F
1   1   0 ||  1    1
0   1   1 ||  F    0
1   1   1 ||  F    1
0   0   0 ||  F    F
*/

 char mask=0;

  switch (Addr0)
  {
   case 'G':
   if(Addr1=='G') mask=0x0;
   if(Addr1=='F') mask=0x1;
   if(Addr1=='H') mask=0x2;
   break;
   
   case 'H':
   if(Addr1=='G') mask=0x4;
   if(Addr1=='F') mask=0x5;
   if(Addr1=='H') mask=0x6;
   break;
  
   case 'F':
   if(Addr1=='G') mask=0x3;
   if(Addr1=='F') mask=0x7;
   if(Addr1=='H') mask=0x0;
   break; 
   }
   
  address=(MCP3424_COMMON_ADDR | mask); //apply mask to common address for full address  
 }



float mcp3424::GetValue(int channel)
{
  long ADC_value=0; //initialize to prevent false reading
  char channel_register=CONFIG_REGISTER;
  
  switch (channel)  //modify config_register with channel requesed
  {
   case 1:
   channel_register=channel_register; //default of '0'
   break;
   
   case 2:
   channel_register=channel_register | (1 <<5); 
   break;
   
   case 3:
   channel_register=channel_register | (2 <<5);
   break;
   
   case 4:
   channel_register=channel_register | (3 <<5); 
   break;   
  }
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 // address=0x69;  //NOTE: OVERWRITE ADDRESS for G,F config
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  int write_status;
  Wire.beginTransmission(address);
  Wire.send(channel_register); //note: first bit of register is '1', thus a request to read ADC
  write_status=Wire.endTransmission();
  delay(1000);
  
 // Serial.print("Write_status_sendRegister: ");
 // Serial.println(write_status);
 //Serial.print("Address: ");
// Serial.println((int)address);
// Serial.print("register: ");
// Serial.println((int)channel_register);
  
  unsigned char byte0;
  unsigned char byte1;
  unsigned char byte2;
  unsigned char byte3;
  delay(350);
  Wire.beginTransmission(address);
  Wire.requestFrom((int)address,4);
    
  byte0=Wire.receive();
  delay(10);
  byte1=Wire.receive();
  delay(10);
  byte2=Wire.receive();
  delay(10);
  byte3=Wire.receive(); //this is a 'configuration byte' and does not contain conversion data
  
  write_status=Wire.endTransmission();
  
  //For debugging
/*  Serial.print((unsigned int)byte0);
  Serial.println();
  Serial.print((unsigned int)byte1);
  Serial.println();
  Serial.print((unsigned int)byte2);
  Serial.println();
  Serial.print((unsigned int)byte3);
  Serial.println(); */
  
  
  //  SEE DATA SHEET FOR HOW TO CONCATENATE BYTES
  //note: need to check if negative b/c if so, values are 2's complement
  char sign=(byte0 & 0x2); //MSB of ADC_value
  if(sign)
  {
  	  byte0=~byte0;
  	  byte1=~byte1;
  	  byte2=~byte2;
  }
  ADC_value= ADC_value | (byte0 & 0x01);
  ADC_value=ADC_value<<16;
  ADC_value= ADC_value | (((unsigned int)byte1)<<8);
  ADC_value= ADC_value | byte2; 
  
  if(sign)
  {
  	ADC_value=-ADC_value;
  }
  
  float ADC_mVoltage=ADC_value*15.625;
  return ADC_mVoltage;
}
		