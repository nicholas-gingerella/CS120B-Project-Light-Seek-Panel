//File: servo_control.c
//Author: Nicholas Gingerella
//Date: 05/22/14
#include <avr/io.h>
#include <avr/interrupt.h>



volatile unsigned char TimerFlag = 0; // TimerISR() sets this to 1. C programmer should clear to 0.

// Internal variables for mapping AVR's ISR to our cleaner TimerISR model.
unsigned long _avr_timer_M = 1; // Start count from here, down to 0. Default 1ms
unsigned long _avr_timer_cntcurr = 0; // Current internal count of 1ms ticks

// Set TimerISR() to tick every M ms
void TimerSet(unsigned long M) {
	_avr_timer_M = M;
	_avr_timer_cntcurr = _avr_timer_M;
}


void TimerOn() {
	// AVR timer/counter controller register TCCR1
	TCCR1B 	= 0x0B;	// bit3 = 1: CTC mode (clear timer on compare)
	// bit2bit1bit0=011: prescaler /64
	// 00001011: 0x0B
	// SO, 8 MHz clock or 8,000,000 /64 = 125,000 ticks/s
	// Thus, TCNT1 register will count at 125,000 ticks/s

	// AVR output compare register OCR1A.
	OCR1A 	= 125;	// Timer interrupt will be generated when TCNT1==OCR1A
	// We want a 1 ms tick. 0.001 s * 125,000 ticks/s = 125
	// So when TCNT1 register equals 125,
	// 1 ms has passed. Thus, we compare to 125.
	// AVR timer interrupt mask register

	TIMSK1 	= 0x02; // bit1: OCIE1A -- enables compare match interrupt

	//Initialize avr counter
	TCNT1 = 0;

	// TimerISR will be called every _avr_timer_cntcurr milliseconds
	_avr_timer_cntcurr = _avr_timer_M;

	//Enable global interrupts
	SREG |= 0x80;	// 0x80: 1000000
}

void TimerOff() {
	TCCR1B 	= 0x00; // bit3bit2bit1bit0=0000: timer off
}

void TimerISR() {
	TimerFlag = 1;
}

// In our approach, the C programmer does not touch this ISR, but rather TimerISR()
ISR(TIMER1_COMPA_vect)
{
	// CPU automatically calls when TCNT0 == OCR0 (every 1 ms per TimerOn settings)
	_avr_timer_cntcurr--; 			// Count down to 0 rather than up to TOP
	if (_avr_timer_cntcurr == 0) { 	// results in a more efficient compare
		TimerISR(); 				// Call the ISR that the user uses
		_avr_timer_cntcurr = _avr_timer_M;
	}
}





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
void Servo_SetAngle(unsigned char angle){
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



int main(void)
{
	//port B is output
	DDRB = 0xFF;
	
	//set and turn on timer
	TimerSet(500);
	TimerOn();
	
	//turn on PWM and set servo angle
	PWM_on();
	
	unsigned char testAngles[] = {0, 30, 60, 90, 120, 150, 180};
	unsigned char i = 0;
	//main loop
	while(1)
	{	
		
		
		if(i < 7){
			Servo_SetAngle(testAngles[i++]);
		}
		else{
			i = 0;
			Servo_SetAngle(testAngles[i++]);
		}
		
		
		while(!TimerFlag);
		TimerFlag = 0;
	}
}
