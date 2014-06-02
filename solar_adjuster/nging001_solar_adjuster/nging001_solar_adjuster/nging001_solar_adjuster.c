#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "bit.h"
#include "timer.h"
#include "lcd_io.h"
#include "adc.h"
#include "servo.h"

//find max value of two numbers
unsigned short maxValue(unsigned short x, unsigned short y){
	return x > y ? x : y;
}

//find the absolute difference between two numbers
unsigned short absDifference(unsigned short x, unsigned short y){
	return x > y ? (x - y) : (y - x);
}


//#################### (MAIN) LIGHT SEEK SM #########################
//shared variables
unsigned short lightDiff;
unsigned char rotateLeft;	//1=rotate left, 0=no rotate
unsigned char rotateRight; //1=rotate right, 0=no rotate
unsigned char systemOff;		//1=off, 0=On
unsigned char DetectedHuman = 0; //input to main system and output of detect light (OUTPUT LED: B0, SENSOR INPUT: B1)
unsigned char pirSensor;
unsigned short LightSeek_Period = 1;

enum LightSeek_States {LightSeekInit, Off, PowerOn, ShutDown, On, FilterRotate, RotateLeft, RotateRight} LightSeek_State;
void LightSeek_TickFct(){
	static unsigned short lightL, lightR, lightMax;
	static unsigned char filterCnt, shutdownCnt, powerOnCnt, servoAngle;
	
	// range thresholds for system operation
	const unsigned char lightDiffThresh = 20;	// the difference needed to init seeking logic
	const unsigned char systemOnThresh = 100;	// the minimum light reading needed for system power
	
	const unsigned char filterMax = 10;		// 1s = 1000ms -> 1000/period -> 1000/50 = 20
	const unsigned char shutdownMax = 50;	// 10s = 10000ms -> 10000/period -> 10000/50 = 200
	const unsigned char powerOnMax = 50;	// 10s = 10000ms -> 10000/period -> 10000/50 = 200
	
	//transitions
	switch(LightSeek_State){
		case LightSeekInit:
			LightSeek_State = Off;
			break;
		case Off:
			if( lightMax < systemOnThresh ){
				LightSeek_State = Off;
			}
			else{
				powerOnCnt = 0;
				LightSeek_State = PowerOn;
			}
			break;
		case PowerOn:
			if( (powerOnCnt < powerOnMax) && (lightMax >= systemOnThresh) ){
				powerOnCnt++;
				LightSeek_State = PowerOn;
			}
			else if ( (powerOnCnt >= powerOnMax) && (lightMax >= systemOnThresh) ){
				LightSeek_State = On;
			}
			else {
				LightSeek_State = Off;
			}
			break;
		case On:
			if( (lightDiff < lightDiffThresh) && (lightMax >= systemOnThresh) ){
				LightSeek_State = On;
			}
			else if( (lightDiff >= lightDiffThresh) && (lightMax >= systemOnThresh) && !DetectedHuman){
				filterCnt = 0;
				LightSeek_State = FilterRotate;
			}
			else {
				shutdownCnt = 0;
				LightSeek_State = ShutDown;
			}
			break;
		case FilterRotate:
			if( (filterCnt < filterMax) && (lightDiff >= lightDiffThresh) && !DetectedHuman ){
				filterCnt++;
				LightSeek_State = FilterRotate;
			}
			else if( (filterCnt >= filterMax) && (lightDiff >= lightDiffThresh) && (lightL > lightR) && !DetectedHuman){
				rotateLeft = 1;
				rotateRight = 0;
				LightSeek_State = RotateLeft;
			}
			else if( (filterCnt >= filterMax) && (lightDiff >= lightDiffThresh) && (lightL < lightR) && !DetectedHuman){
				rotateLeft = 0;
				rotateRight = 1;
				LightSeek_State = RotateRight;
			}
			else {
				LightSeek_State = On;
			}
			break;
		case RotateRight:
			if( (lightDiff > (lightDiffThresh - (lightDiffThresh/2))) && (servoAngle > 0) && (lightL < lightR) && !DetectedHuman){
				LightSeek_State = RotateRight;
			}
			else{
				rotateLeft = 0;
				rotateRight = 0;
				LightSeek_State = On;
			}
			break;
		case RotateLeft:
			if( (lightDiff > (lightDiffThresh - (lightDiffThresh/2))) && (servoAngle < 180) && (lightL > lightR) && !DetectedHuman){
				LightSeek_State = RotateLeft;
			}
			else{
				rotateLeft = 0;
				rotateRight = 0;
				LightSeek_State = On;
			}
			break;
		case ShutDown:
			if( (shutdownCnt < shutdownMax) && (lightMax < systemOnThresh) ){
				shutdownCnt++;
			}
			else if( (shutdownCnt >= shutdownMax) && (lightMax < systemOnThresh) ){
				LightSeek_State = Off;
			}
			else{
				LightSeek_State = On;
			}
			break;
		default:
			LightSeek_State = LightSeekInit;
			break;
	}//transitions
	
	
	//actions
	switch(LightSeek_State){
		case LightSeekInit:
			lightL = 0;
			lightR = 0;
			rotateLeft = 0;
			rotateRight = 0;
			systemOff = 1;
			lightMax = 0;
			servoAngle = 90;
			break;
		case Off:
			lightL = LEFT_LIGHT_SENSOR;
			lightR = RIGHT_LIGHT_SENSOR;
			rotateLeft = 0;
			rotateRight = 0;
			systemOff = 1;
			lightMax = maxValue(lightL, lightR);
			servoAngle = 90;
			break;
		case PowerOn:
			lightL = LEFT_LIGHT_SENSOR;
			lightR = RIGHT_LIGHT_SENSOR;
			rotateLeft = 0;
			rotateRight = 0;
			systemOff = 1;
			lightMax = maxValue(lightL, lightR);
			break;
		case On:
			lightL = LEFT_LIGHT_SENSOR;
			lightR = RIGHT_LIGHT_SENSOR;
			rotateLeft = 0;
			rotateRight = 0;
			systemOff = 0;
			lightMax = maxValue(lightL, lightR);
			lightDiff = absDifference(lightL,lightR);
			break;
		case FilterRotate:
			lightL = LEFT_LIGHT_SENSOR;
			lightR = RIGHT_LIGHT_SENSOR;
			rotateLeft = 0;
			rotateRight = 0;
			systemOff = 0;
			lightMax = maxValue(lightL, lightR);
			lightDiff = absDifference(lightL,lightR);
			break;
		case RotateRight:
			lightL = LEFT_LIGHT_SENSOR;
			lightR = RIGHT_LIGHT_SENSOR;
			rotateRight = 1;
			rotateLeft = 0;
			systemOff = 0;
			lightDiff = absDifference(lightL,lightR);
			break;
		case RotateLeft:
			lightL = LEFT_LIGHT_SENSOR;
			lightR = RIGHT_LIGHT_SENSOR;
			rotateRight = 0;
			rotateLeft = 1;
			systemOff = 0;
			lightDiff = absDifference(lightL,lightR);
			break;
		case ShutDown:
			lightL = LEFT_LIGHT_SENSOR;
			lightR = RIGHT_LIGHT_SENSOR;
			rotateLeft = 0;
			rotateRight = 0;
			systemOff = 0;
			lightMax = maxValue(lightL, lightR);
			break;
		default:
			lightL = 0;
			lightR = 0;
			rotateLeft = 0;
			rotateRight = 0;
			systemOff = 1;
			lightMax = 0;
			servoAngle = 90;
			break;
	}//actions
	
}//LightSeekSM
//#################### (MAIN) LIGHT SEEK SM #########################



//##################### SERVO CONTROL SM ############################
//shared variables
unsigned char servoAngle;	//current servo angle of the system

enum ServoControl_States {ServoInit, ServoWait, ServoDefaultPos, ServoRotateLeft, ServoRotateRight} ServoControl_State;
void ServoControl_TickFct(){
	
	switch(ServoControl_State){
		case ServoInit:
			ServoControl_State = ServoWait;
			break;
		case ServoWait:
			if(systemOff){
				ServoControl_State = ServoDefaultPos;
			}
			else if(rotateLeft && !rotateRight){
				ServoControl_State = ServoRotateLeft;
			}
			else if(!rotateLeft && rotateRight){
				ServoControl_State = ServoRotateRight;
			}
			else{
				ServoControl_State = ServoWait; 
			}
			break;
		case ServoDefaultPos:
			if(systemOff){
				ServoControl_State = ServoDefaultPos;
			}
			else{
				ServoControl_State = ServoWait;
			}
			break;
		case ServoRotateLeft:
			if((servoAngle < 180) && rotateLeft && !rotateRight){
				ServoControl_State = ServoRotateLeft;
			}
			else{
				ServoControl_State = ServoWait;
			}
			break;
		case ServoRotateRight:
			if((servoAngle > 0) && !rotateLeft && rotateRight){
				ServoControl_State = ServoRotateRight;
			}
			else{
				ServoControl_State = ServoWait;
			}
			break;
		default:
			ServoControl_State = ServoInit;
			break;
	}
	
	switch(ServoControl_State){
		case ServoInit:
			servoAngle = 90;
			Servo_SetAngle(servoAngle);
			break;
		case ServoWait:
			break;
		case ServoDefaultPos:
			servoAngle = 90;
			Servo_SetAngle(servoAngle);
			break;
		case ServoRotateLeft:
			Servo_SetAngle(servoAngle++);
			break;
		case ServoRotateRight:
			servoAngle--;
			if(servoAngle == 0){
				Servo_SetAngle(servoAngle);
				servoAngle = 1;
			} else {
				Servo_SetAngle(servoAngle);
			}
			break;
		default:
			servoAngle = 90;
			Servo_SetAngle(servoAngle);
			break;
	}
}
//##################### SERVO CONTROL SM ############################

/*
//##################### HUMAN DETECTOR SM ############################
//shared variables
//unsigned char DetectedHuman; //input to main system and output of detect light (OUTPUT LED: B0, SENSOR INPUT: B1)

enum HumanDetect_States {InitDetection, HumanNotDetected, HumanDetected} HumanDetect_State;
void HumanDetect_TickFct(){
	static unsigned char initCnt = 0;
	static unsigned char initMax = 20; //10s
	
	static unsigned char detectCnt = 0;
	static unsigned char detectMax = 20; //5s
	
	//transitions
	switch(HumanDetect_State){
		case InitDetection:
			if(initCnt < initMax){
				initCnt++;
				HumanDetect_State = InitDetection;
			}
			else{
				initCnt = 0;
				HumanDetect_State = HumanNotDetected;
			}
			break;
		case HumanNotDetected:
			if(!pirSensor){
				HumanDetect_State = HumanNotDetected;
			}
			else {
				HumanDetect_State = HumanDetected;
				detectCnt = 0;
			}
			break;
		case HumanDetected:
			if(detectCnt < detectMax){
				detectCnt++;
				HumanDetect_State = HumanDetected;
			}
			else {
				detectCnt = 0;
				HumanDetect_State = HumanNotDetected;
			}
			break;
		default:
			initCnt = 0;
			HumanDetect_State = InitDetection;
			break;
	}//transitions
	
	
	//actions
	switch(HumanDetect_State){
		case InitDetection:
			DetectedHuman = 0;
			//PORTB = SetBit(PORTB,0,0);
			LCD_ClearScreen();
			LCD_DisplayString(1,"InitDetect");
			break;
		case HumanNotDetected:
			DetectedHuman = 0;
			//PORTB = SetBit(PORTB,0,0);
			LCD_ClearScreen();
			LCD_DisplayString(1,"Not Detect");
			break;
		case HumanDetected:
			DetectedHuman = 1;
			//PORTB = SetBit(PORTB,0,1);
			LCD_ClearScreen();
			LCD_DisplayString(1,"Detect");
			break;
		default:
			//DetectedHuman = 0;
			//PORTB = SetBit(PORTB,0,0);
			break;
	}//actions
}
//##################### HUMAN DETECTOR SM ############################
*/

//##################### LCD DISPLAY SM ############################
enum LcdDisplay_States {LcdInit, DisplayLeft, DisplayRight} LcdDisplay_State;
void LcdDisplay_TickFct(){
	static char int_buffer[10];
	static char cursorPos;
	static short lightL;
	static short lightR;
	
	//transitions
	switch(LcdDisplay_State){
		case LcdInit:
			cursorPos = 1;
			LcdDisplay_State = DisplayLeft;
			break;
		case DisplayLeft:
			cursorPos = 13;
			LcdDisplay_State = DisplayRight;
			break;
		case DisplayRight:
			cursorPos = 1;
			LcdDisplay_State = DisplayLeft;
			break;
	}//transitions
	
	//actions
	switch(LcdDisplay_State)
	{
		case LcdInit:
			//						 ______________
			//initialize display to |000        000|
			//						|______________|
			
			
			//initialize left		 ______________
			//initialize display to |000           |
			//						|______________|
			cursorPos = 1;
			LCD_ClearScreen();
			for(unsigned char i=0;i<3;i++){
				LCD_WriteData(0 + '0');
				LCD_Cursor(++cursorPos);
			}
			
			//initialize right		 ______________
			//initialize display to |           000|
			//						|______________|
			cursorPos = 13;
			for(unsigned char i=0;i<3;i++){
				LCD_WriteData(0 + '0');
				LCD_Cursor(++cursorPos);
			}
			break;
		case DisplayLeft:
			lightL = LEFT_LIGHT_SENSOR;
			itoa(lightL, int_buffer,10);
			LCD_Cursor(cursorPos);
			
			//						 ______________
			//2 leading 0s needed   |00x           |
			//						|______________|
			if(lightL < 10){
				LCD_WriteData(0 + '0');
				cursorPos++;
				LCD_WriteData(0 + '0');
				cursorPos++;
			}
			//						 ______________
			//1 leading 0s needed   |0xx           |
			//						|______________|
			else if ( lightL >= 10 && lightL < 100){
				LCD_WriteData(0 + '0');
				cursorPos++;
			}
			
			//if neither of the above are true, then 
			//we need 0 leading 0s, since the reading
			//is a 3 digit number
			//						 ______________
			//0 leading 0s needed   |xxx           |
			//						|______________|
			
			//print out contents of the light sensor reading
			for(unsigned char i=0; int_buffer[i]; i++){
				LCD_WriteData(int_buffer[i]);
				cursorPos++;
			}
			break;
		case DisplayRight:
			lightR = RIGHT_LIGHT_SENSOR;
			itoa(lightR, int_buffer,10);
			LCD_Cursor(cursorPos);
			
			//						 ______________
			//2 leading 0s needed   |           00x|
			//						|______________|
			if(lightR < 10){
				LCD_WriteData(0 + '0');
				cursorPos++;
				LCD_WriteData(0 + '0');
				cursorPos++;
			}
			//						 ______________
			//1 leading 0s needed   |           0xx|
			//						|______________|
			else if ( lightR >= 10 && lightR < 100){
				LCD_WriteData(0 + '0');
				cursorPos++;
			}
			
			//if neither of the above are true, then
			//we need 0 leading 0s, since the reading
			//is a 3 digit number
			//						 ______________
			//0 leading 0s needed   |           xxx|
			//						|______________|
			for(unsigned char i=0; int_buffer[i]; i++){
				LCD_WriteData(int_buffer[i]);
				cursorPos++;
			}
			break;
	}//actions
}
//##################### LCD DISPLAY SM ############################



int main(void)
{
	DDRB = 1 << 6 | 1 << 0; PORTB = 0xBE;
	DDRD = 0xFF;
	DDRC = 0x03;
	
	//set and turn on system timer
	TimerSet(50);
	TimerOn();
	
	LCD_init();//initialize lcd display for use
	ADC_init();//initialize analog to digital converter
	PWM_on();//turn on PWM and servo control
	
	//Init SM states
	LcdDisplay_State = LcdInit;
	//HumanDetect_State = InitDetection;
	LightSeek_State = LightSeekInit;
	ServoControl_State = ServoInit;
	
	//main loop
	while(1)
	{
		pirSensor = GetBit(~PINB,1); //get reading from pir sensor
		
		LcdDisplay_TickFct();
		//HumanDetect_TickFct();
		LightSeek_TickFct();
		ServoControl_TickFct();
		while(!TimerFlag);
		TimerFlag = 0;
	}
}