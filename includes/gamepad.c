unsigned char PS2byte;
unsigned char PS2data4;
unsigned char PS2data5;
unsigned char i;
  
#include <avr/io.h>
#include "delays.h"

#define PS2clk 3
#define PS2cmd 2
#define PS2att 1
#define PS2dat 0
#define PS2PORT PORTB
#define PS2IN PINB

#define SET(x,y) (x|=(1<<y))
#define CLR(x,y) (x&=(~(1<<y)))
#define CHK(x,y) (x & (1<<y))
#define TOG(x,y) (x^=(1<<y)) 

void ReadGamepad()
{
   SET(PS2PORT,PS2cmd); 
   SET(PS2PORT,PS2clk); 
   CLR(PS2PORT,PS2att); // low enable joystick

   Delay(1);
   PS2byte=0x01; // first command
   for(i=0;i<8;i++)
   {
      if(CHK(PS2byte,i)) SET(PS2PORT,PS2cmd);
	  else  CLR(PS2PORT,PS2cmd);
      CLR(PS2PORT,PS2clk); //Delay(1); 
      SET(PS2PORT,PS2clk); 
   }
   SET(PS2PORT,PS2cmd); 

   Delay(1);
   PS2byte=0x42; // sec command
   for(i=0;i<8;i++)
   {
      if(CHK(PS2byte,i)) SET(PS2PORT,PS2cmd);
	  else  CLR(PS2PORT,PS2cmd);
      CLR(PS2PORT,PS2clk); //Delay(1); 
      SET(PS2PORT,PS2clk); 
   }
   SET(PS2PORT,PS2cmd); 

   Delay(1);
   for(i=0;i<8;i++)  // 3 byte
   {
      CLR(PS2PORT,PS2cmd);
      CLR(PS2PORT,PS2clk); //Delay(1); 
      SET(PS2PORT,PS2clk); 
   }
   SET(PS2PORT,PS2cmd); 

   Delay(1);
   PS2data4=0;
   for(i=0;i<8;i++)  // 4 byte
   {
      CLR(PS2PORT,PS2cmd);
      CLR(PS2PORT,PS2clk); Delay(1);
	  if(CHK(PS2IN,PS2dat)) SET(PS2data4,i); 
      SET(PS2PORT,PS2clk); 
   }
   SET(PS2PORT,PS2cmd); 

   Delay(1);
   PS2data5=0;
   for(i=0;i<8;i++) // 5 byte
   {
      CLR(PS2PORT,PS2cmd);
      CLR(PS2PORT,PS2clk); Delay(1); 
	  if(CHK(PS2IN,PS2dat)) SET(PS2data5,i);
      SET(PS2PORT,PS2clk); 
   }
   SET(PS2PORT,PS2cmd); 
   SET(PS2PORT,PS2att); // HI disable joystick
}
