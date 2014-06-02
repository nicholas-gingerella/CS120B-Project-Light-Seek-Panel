// File: servo.h
// Author: Nicholas Gingerella
#ifndef SERVO_H
#define SERVO_H

//-------------------------------------------------------------------
//  Timer/Counter PWM For Servo (ASSUMES 8MHz System Clock)
//-------------------------------------------------------------------
//PWM uses Timer3 on the Atmega1284P
void PWM_on(){
	//0 out the Timer3 Timer/Counter Control Registers
	TCCR3A = 0x00;
	TCCR3B = 0x00;
	
	//Settings for Timer3: Fast PWM Mode with prescaler 8 and non-inverting compare output on OCR3A
	//TOP = sys_clock/(2*prescaler*desired_clock) ----> TOP = 8e6/(2*8*50)-1 = 19999
	ICR3 = 19999; // TOP value for Timer3 (50Hz PWM)
	TCCR3A = 0x82; // TCCR3A = 1000 0010 ----> TCCR3A = (1 << COM3A1) | (1 << WGM31);
	TCCR3B = 0x1A; //TCCR3B = 0001 1010 ----> TCCR3B = (1 << CS31) | (1 << WGM33) | (1 << WGM32) ;
	TCNT3 = 0; // resets counter
}


//Turns off Timer3 and PWM
void PWM_off() {
	TCCR3A = 0x00;
	TCCR3B = 0x00;
}


//Solar Panel Orientation: 90 = Horizontal | 0 or 180 = Vertical | 0 < angle < 90 = right tilt | 90 < angle < 180 = left tilt
//Sets angle (degrees) of servo hooked to OCR3A (PinB6)
//Range is 0 to 180
void Servo_SetAngle( char angle){
	short servo_angle;
	
	//if passed large angle, default to 180 degrees (max)
	if(angle > 180){
		servo_angle = 2400;
	}
	else {
		//Linear relation between PWM high pulse and angle
		//high_time = ((angle / (180 / 1800)) + 600) ---> high_time = ((angle / 0.1) + 600)
		//add 600 to offset hightime from 0 to 600 (which is 0 deg for servo)
		servo_angle = (angle*10) + 600;
	}
	
	OCR3A = servo_angle;
}
//-------------------------------------------------------------------
#endif //SERVO_H