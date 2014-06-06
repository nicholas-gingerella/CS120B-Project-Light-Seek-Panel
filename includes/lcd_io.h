/*	Nicholas Gingerella nging001@ucr.edu
 *  Lab Section: 22
 *  Assignment: Custom Lab - Light Seeker
 *  Project Description:
 *      A miniature solar panel that automatically adjusts its orientation to point towards
 *		the most light. Uses PWM, ADC, two photoresistors, one servo motor, and one analog
 *      joystick
 *
 *  I acknowledge all content contained herein, excluding template or example
 *  code, is my own original work
 */

#ifndef _LCDIO_H
#define _LCDIO_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define SET_BIT(p,i) ((p) |= (1 << (i)))
#define CLR_BIT(p,i) ((p) &= ~(1 << (i)))
#define GET_BIT(p,i) ((p) & (1 << (i)))
          
/*-------------------------------------------------------------------------*/

#define DATA_BUS PORTD		// port connected to pins 7-14 of LCD display
#define CONTROL_BUS PORTC	// port connected to pins 4 and 6 of LCD disp.
#define RS 0			// pin number of uC connected to pin 4 of LCD disp.
#define E 1			// pin number of uC connected to pin 6 of LCD disp.

/*-------------------------------------------------------------------------*/

void LCD_ClearScreen(void) {
   LCD_WriteCommand(0x01);
}

void LCD_init(void) {

    //wait for 100 ms.
	delay_ms(100);
	LCD_WriteCommand(0x38);
	LCD_WriteCommand(0x06);
	LCD_WriteCommand(0x0f);
	LCD_WriteCommand(0x01);
	LCD_WriteCommand(0x0C); //turn cursor off
	delay_ms(10);						 
}

void LCD_WriteCommand (unsigned char Command) {
   CLR_BIT(CONTROL_BUS,RS);
   DATA_BUS = Command;
   SET_BIT(CONTROL_BUS,E);
   asm("nop");
   CLR_BIT(CONTROL_BUS,E);
   delay_ms(2); // ClearScreen requires 1.52ms to execute
}

void LCD_WriteData(unsigned char Data) {
   SET_BIT(CONTROL_BUS,RS);
   DATA_BUS = Data;
   SET_BIT(CONTROL_BUS,E);
   asm("nop");
   CLR_BIT(CONTROL_BUS,E);
   delay_ms(1);
}

void LCD_DisplayString( unsigned char column, const unsigned char* string) {
   //LCD_ClearScreen();
   unsigned char c = column;
   while(*string) {
      LCD_Cursor(c++);
      LCD_WriteData(*string++);
   }
}

void LCD_Cursor(unsigned char column) {
   if ( column < 17 ) { // 16x1 LCD: column < 9
						// 16x2 LCD: column < 17
      LCD_WriteCommand(0x80 + column - 1);
   } else {
      LCD_WriteCommand(0xB8 + column - 9);	// 16x1 LCD: column - 1
											// 16x2 LCD: column - 9
   }
}


//Input:
// location: location where you want to store
// 0,1,2,....7
// ptr: Pointer to pattern data
//
//Usage:
// pattern[8]={0x04,0x0E,0x0E,0x0E,0x1F,0x00,0x04,0x00};
// LCD_build(1,pattern);
//
//LCD Ports are same as discussed in previous sections

void LCD_build(unsigned char location, unsigned char *ptr){
	unsigned char i;
	if(location<8){
		LCD_WriteCommand(0x40+(location*8));
		for(i=0;i<8;i++)
		LCD_WriteData(ptr[ i ]);
	}
	
}


void delay_ms(int miliSec) //for 8 Mhz crystal
{
	int i,j;
	for(i=0; i < miliSec; i++){
		for(j=0; j < 775; j++)
		{
			asm("nop");
		}
	}
}

#endif //_LCDIO_H