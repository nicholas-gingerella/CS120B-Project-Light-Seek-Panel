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


//shared variables
unsigned short lightDiff;
unsigned char rotateLeft;	//1=rotate left, 0=no rotate
unsigned char rotateRight; //1=rotate right, 0=no rotate
unsigned char systemOff;		//1=off, 0=On
unsigned char DetectedHuman; //input to main system and output of detect light (OUTPUT LED: B0, SENSOR INPUT: B1)
unsigned char pirSensor;	//passive infrared sensor input
unsigned short joystickReading; //analog reading from joystick
unsigned char overrideBtn;	//override button for manual override of system
unsigned char OverrideOn;	//1 = manual override on, 0 = manual override off
unsigned char servoAngle;	//current servo angle of the system
char* currentActionMsg;		//display message for current light detector action



//#################### (MAIN) LIGHT SEEK SM #########################
unsigned short LightSeek_Period = 1;

enum LightSeek_States {LightSeekInit, Off, PowerOn, ShutDown, On, FilterRotate, 
	RotateLeft, RotateRight, ManualModeDown, ManualMode, ManualRight, ManualLeft,
	AutoModeDown} LightSeek_State;
void LightSeek_TickFct(){
	static unsigned short lightL, lightR, lightMax;
	static unsigned char filterCnt, shutdownCnt, powerOnCnt, servoAngle;
	
	// range thresholds for system operation
	const unsigned char lightDiffThresh = 20;	// the difference needed to init seeking logic
	const unsigned char systemOnThresh = 100;	// the minimum light reading needed for system power
	
	const unsigned char filterMax = 40;		// 2s @50ms
	const unsigned char shutdownMax = 100;	// 5s @50ms
	const unsigned char powerOnMax = 100;	// 5s @50ms
	
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
			
			//manual override
			if(overrideBtn){
				OverrideOn = 1;
				LightSeek_State = ManualModeDown;
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
			
			//manual override
			if(overrideBtn){
				OverrideOn = 1;
				LightSeek_State = ManualModeDown;
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
			
			//manual override
			if(overrideBtn){
				OverrideOn = 1;
				LightSeek_State = ManualModeDown;
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
			
			//manual override
			if(overrideBtn){
				OverrideOn = 1;
				LightSeek_State = ManualModeDown;
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
			
			//manual override
			if(overrideBtn){
				OverrideOn = 1;
				LightSeek_State = ManualModeDown;
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
			
			//manual override
			if(overrideBtn){
				OverrideOn = 1;
				LightSeek_State = ManualModeDown;
			}
			break;
		case ShutDown:
			if( (shutdownCnt < shutdownMax) && (lightMax < systemOnThresh) ){
				shutdownCnt++;
				LightSeek_State = ShutDown;
			}
			else if( (shutdownCnt >= shutdownMax) && (lightMax < systemOnThresh) ){
				LightSeek_State = Off;
			}
			else{
				LightSeek_State = On;
			}
			
			//manual override
			if(overrideBtn){
				LightSeek_State = ManualModeDown;
			}
			break;
		case ManualModeDown: //pressing down on manual mode btn
			if(overrideBtn){
				LightSeek_State = ManualModeDown;
			} else{
				LightSeek_State = ManualMode;
			}
			break;
		case ManualMode: //officially in manual mode
			/*
			 *if joystick gives a left reading, go to ManualLeft
			 *else if if joystick gives a right reading, go to ManualRight
			 */
			
			if(OverrideOn && (joystickReading < 500) && !DetectedHuman){
				LightSeek_State = ManualLeft;
			}
			else if(OverrideOn && (joystickReading > 600) && !DetectedHuman){
				LightSeek_State = ManualRight;
			}
			
			
			if(OverrideOn && overrideBtn){
				LightSeek_State = AutoModeDown;
			}
			break;
		case ManualLeft: //manually rotate left (set rotateLeft)
			if(OverrideOn && (joystickReading < 500) && !DetectedHuman){
				LightSeek_State = ManualLeft;
			} else {
				LightSeek_State = ManualMode;
			}
			break;
		case ManualRight: //manually rotate right (set rotateRight)
			if(OverrideOn && (joystickReading > 600) && !DetectedHuman){
				LightSeek_State = ManualRight;
			} else {
				LightSeek_State = ManualMode;
			}
			break;
		case AutoModeDown://pressing down on manual mode btn (goes to On state)
			if(overrideBtn){
				LightSeek_State = AutoModeDown;
			} else {
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
			currentActionMsg = "Sys Off         ";
			lightL = LEFT_LIGHT_SENSOR;
			lightR = RIGHT_LIGHT_SENSOR;
			rotateLeft = 0;
			rotateRight = 0;
			systemOff = 1;
			lightMax = maxValue(lightL, lightR);
			servoAngle = 90;
			break;
		case PowerOn:
			currentActionMsg = "Power On         ";
			lightL = LEFT_LIGHT_SENSOR;
			lightR = RIGHT_LIGHT_SENSOR;
			rotateLeft = 0;
			rotateRight = 0;
			systemOff = 1;
			lightMax = maxValue(lightL, lightR);
			break;
		case On:
			currentActionMsg = "Sys On          ";
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
			currentActionMsg = "Searching         ";
			lightL = LEFT_LIGHT_SENSOR;
			lightR = RIGHT_LIGHT_SENSOR;
			rotateRight = 1;
			rotateLeft = 0;
			systemOff = 0;
			lightDiff = absDifference(lightL,lightR);
			break;
		case RotateLeft:
			currentActionMsg = "Searching          ";
			lightL = LEFT_LIGHT_SENSOR;
			lightR = RIGHT_LIGHT_SENSOR;
			rotateRight = 0;
			rotateLeft = 1;
			systemOff = 0;
			lightDiff = absDifference(lightL,lightR);
			break;
		case ShutDown:
			currentActionMsg = "Shutdown          ";
			lightL = LEFT_LIGHT_SENSOR;
			lightR = RIGHT_LIGHT_SENSOR;
			rotateLeft = 0;
			rotateRight = 0;
			systemOff = 0;
			lightMax = maxValue(lightL, lightR);
			break;
		case ManualModeDown: //pressing down on manual mode btn
			rotateLeft = 0;
			rotateRight = 0;
			OverrideOn = 1;
			currentActionMsg = "Manual          ";
			break;
		case ManualMode: //officially in manual mode
			rotateLeft = 0;
			rotateRight = 0;
			OverrideOn = 1;
			joystickReading = ANALOG_STICK_READING;
			currentActionMsg = "Manual          ";
			PORTB = SetBit(PORTB,3,1);
			break;
		case ManualLeft: //manually rotate left (set rotateLeft)
			rotateLeft = 1;
			rotateRight = 0;
			OverrideOn = 1;
			joystickReading = ANALOG_STICK_READING;
			currentActionMsg = "Left          ";
			break;
		case ManualRight: //manually rotate right (set rotateRight)
			rotateLeft = 0;
			rotateRight = 1;
			OverrideOn = 1;
			joystickReading = ANALOG_STICK_READING;
			currentActionMsg = "Right          ";
			break;
		case AutoModeDown://pressing down on manual mode btn (goes to On state)
			OverrideOn = 0;
			rotateLeft = 0;
			rotateRight = 0;
			PORTB = SetBit(PORTB,3,0);
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
enum ServoControl_States {ServoInit, ServoWait, ServoDefaultPos, ServoRotateLeft, ServoRotateRight} ServoControl_State;
void ServoControl_TickFct(){
	
	switch(ServoControl_State){
		case ServoInit:
			ServoControl_State = ServoWait;
			break;
		case ServoWait:
			if(systemOff && !OverrideOn){
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
			if(systemOff && !OverrideOn){
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
	
	//actions
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
			if(servoAngle >= 180){
				servoAngle = 180;
				Servo_SetAngle(servoAngle);
			}
			else{
				servoAngle++;
				Servo_SetAngle(servoAngle);
			}
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
}//Servo Control SM
//##################### SERVO CONTROL SM ############################


//##################### HUMAN DETECTOR SM ############################
enum HumanDetect_States {InitDetection, HumanNotDetected, HumanDetected} HumanDetect_State;
void HumanDetect_TickFct(){
	static unsigned char initCnt = 0;
	static unsigned char initMax = 60; //3s @50ms give sensor time to initialize
	
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
			}
			break;
		case HumanDetected:
			if(pirSensor){
				HumanDetect_State = HumanDetected;
			}
			else {
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
			break;
		case HumanNotDetected:
			DetectedHuman = 0;
			break;
		case HumanDetected:
			DetectedHuman = 1;
			break;
		default:
			DetectedHuman = 0;
			break;
	}//actions
	
	if(!systemOff || OverrideOn){
		PORTB = SetBit(PORTB,0,DetectedHuman); //turn on detection LED
	} else {
		PORTB = SetBit(PORTB,0,0);
	}
}
//##################### HUMAN DETECTOR SM ############################


//##################### LCD DISPLAY SM ############################
enum LcdDisplay_States {LcdInit, DisplayLeft, DisplayRight, DisplayAngle, DisplayAction, DisplaySun, DisplayMoon} LcdDisplay_State;

//custom patterns
const char angleLcdPattern[8] = {0x0, 0x1, 0x2, 0x4, 0x8, 0x10, 0x1f, 0x0};
const char sunLcdPattern[8] = {0x04, 0x11, 0x04, 0x0e, 0x0e, 0x04, 0x11, 0x04};
const char moonLcdPattern[8] = {0x00, 0x1e, 0x0f, 0x07, 0x07, 0xf, 0x1e, 0x00};
	
void LcdDisplay_TickFct(){
	static char int_buffer[10];
	static char cursorPos;
	static short lightL;
	static short lightR;
	static unsigned char sunImgPos;
	
	// sun/moon positions on lcd display, where the positions stand for lcd col number
	const unsigned char LPos3 = 5, LPos2 = 6, LPos1 = 7, MiddlePos = 8, RPos1 = 9, RPos2 = 10, RPos3 = 11, RPos4 = 12;
	
	
	const unsigned char angleImg = 0x00, sunImg = 0x01, moonImg = 0x02;
	
	//transitions
	switch(LcdDisplay_State){
		case LcdInit:
			cursorPos = 1; //cursor start position for dispLeft
			LcdDisplay_State = DisplayLeft;
			break;
		case DisplayLeft:
			cursorPos = 14; //cursor start position for dispRight
			LcdDisplay_State = DisplayRight;
			break;
		case DisplayRight:	//cursor start position for dispAngleImg
			cursorPos = 17;
			LcdDisplay_State = DisplayAngle;
			break;
		case DisplayAngle:
			cursorPos = 23;
			LcdDisplay_State = DisplayAction;
			break;
		case DisplayAction:
			cursorPos = 1;
			if(!systemOff || OverrideOn){
				LcdDisplay_State = DisplaySun;
			}
			else {
				LcdDisplay_State = DisplayMoon;
			}
			break;
		case DisplaySun:
			cursorPos = 1;
			LcdDisplay_State = DisplayLeft;
			break;
		case DisplayMoon:
			cursorPos = 1;
			LcdDisplay_State = DisplayLeft;
			break;
	}//transitions
	
	//actions
	switch(LcdDisplay_State)
	{
		case LcdInit:
			//build custom characters for use throughout program
			//LCD_build(0,angleLcdPattern);
			//LCD_build(1,sunLcdPattern);
			//LCD_build(2,moonLcdPattern);
			
			LCD_ClearScreen();
			
			//initialize left		 ______________
			//initialize display to |000           |
			//						|______________|
			cursorPos = 1;
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
		case DisplayAngle:
			//draw angle image
			LCD_Cursor(cursorPos);
			LCD_WriteData(angleImg);
			cursorPos++;
		
			itoa(servoAngle, int_buffer,10);
			LCD_Cursor(cursorPos);
			
			//						 ______________
			//2 leading 0s needed   |              |
			//						|a00x__________|
			if(servoAngle < 10){
				LCD_WriteData(0 + '0');
				cursorPos++;
				LCD_WriteData(0 + '0');
				cursorPos++;
			}
			//						 ______________
			//1 leading 0s needed   |              |
			//						|a0xx__________|
			else if ( servoAngle >= 10 && servoAngle < 100){
				LCD_WriteData(0 + '0');
				cursorPos++;
			}
			
			//if neither of the above are true, then
			//we need 0 leading 0s, since the reading
			//is a 3 digit number
			//						 ______________
			//0 leading 0s needed   |              |
			//						|axxx__________|
			for(unsigned char i=0; int_buffer[i]; i++){
				LCD_WriteData(int_buffer[i]);
				cursorPos++;
			}
			break;
		case DisplayAction:
			LCD_Cursor(cursorPos);
			if(DetectedHuman && (!systemOff || OverrideOn)){
				LCD_DisplayString(cursorPos,"WARNING!!!!       ");
			}
			else{
				LCD_DisplayString(cursorPos,currentActionMsg);
			}
			break;
		case DisplaySun:
			//determine where the sun symbol should be based on angle
			if((servoAngle < 23) && (servoAngle >= 0)){
				sunImgPos = RPos4;
			}
			else if((servoAngle >= 23) && (servoAngle < 46)){
				sunImgPos = RPos3;
			}
			else if((servoAngle >= 46) && (servoAngle < 69)){
				sunImgPos = RPos2;
			}
			else if((servoAngle >= 69) && (servoAngle < 90)){
				sunImgPos = RPos1;
			}
			else if((servoAngle >= 90) && (servoAngle < 113)){
				sunImgPos = MiddlePos;
			}
			else if((servoAngle >= 113) && (servoAngle < 136)){
				sunImgPos = LPos1;
			}
			else if((servoAngle >= 136) && (servoAngle < 158)){
				sunImgPos = LPos2;
			}
			else if((servoAngle >= 158) && (servoAngle <= 180)){
				sunImgPos = LPos3;
			}
			
			for(unsigned char i = 5; i<=12; i++){
				LCD_Cursor(i);
				if(sunImgPos == i){
					LCD_WriteData(sunImg);
				}
				else {
					LCD_WriteData(' ');
				}
			}
			
			break;
		case DisplayMoon:
			for(unsigned char i = 5; i<=12; i++){
				LCD_Cursor(i);
				if(MiddlePos == i){
					LCD_WriteData(moonImg);
				}
				else {
					LCD_WriteData(' ');
				}
			}
			break;
	}//actions
}
//##################### LCD DISPLAY SM ############################



int main(void)
{
	// TODO: setup inputs and outputs for override
	// Joystick taken care of by ADC A2, just need
	// output for manual override led
	DDRB = 1 << 6 | 1 << 3 | 1 << 0; PORTB = 0xB6; //1011 0110
	DDRD = 0xFF;
	DDRC = 0x03;
	
	//set and turn on system timer
	TimerSet(50);
	TimerOn();
	
	LCD_init();//initialize lcd display for use
	//build custom characters for use throughout program
	LCD_build(0,angleLcdPattern);
	LCD_build(1,sunLcdPattern);
	LCD_build(2,moonLcdPattern);
	
	ADC_init();//initialize analog to digital converter
	PWM_on();//turn on PWM and servo control
	
	//Init SM states
	LcdDisplay_State = LcdInit;
	HumanDetect_State = InitDetection;
	LightSeek_State = LightSeekInit;
	ServoControl_State = ServoInit;
	
	//main loop
	while(1)
	{
		pirSensor = GetBit(PINB,1); //get reading from pir sensor
		overrideBtn = GetBit(~PINB,2); //get joystick button press
		
		HumanDetect_TickFct();
		LightSeek_TickFct();
		ServoControl_TickFct();
		LcdDisplay_TickFct();
		while(!TimerFlag);
		TimerFlag = 0;
	}
}