/*
  ServoShield.h - Decade counter driven Servo library for Arduino using one 8 bit timer and 4 DIO to control up to 16 servos
  Revision 1.1
  Copyright (c) 2009 Adriaan Swanepoel.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "ServoShield.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <WProgram.h> 

volatile uint16_t counter1servopositions[10];
volatile uint16_t counter2servopositions[10];
volatile uint16_t *counter1currentservo;
volatile uint16_t *counter2currentservo;
volatile uint16_t *counter1deadperiod;
volatile uint16_t *counter2deadperiod;
volatile uint16_t counter1current;
volatile uint16_t counter2current;
uint16_t servosmax[16];
uint16_t servosmin[16];

int outputmap[] = {0, 1, 4, 5, 2, 6, 3, 7, 8, 3, 0, 4, 1, 5, 2, 6};

#ifdef HIGHACCURACY
uint16_t CalcCounterValue(int Time) 
{ 
   return (uint16_t)(65535 - (Time * 16) + 1); 
} 

ISR(TIMER1_OVF_vect) 
{
	counter1cntport |= _BV(counter1cntpin); 
	*counter1deadperiod -= *counter1currentservo;
	TCNT1 = CalcCounterValue(*counter1currentservo++); 
	counter1cntport &= ~_BV(counter1cntpin);	
	
	if (counter1currentservo > &counter1servopositions[9])
	{
		counter1currentservo = &counter1servopositions[0];
		*counter1deadperiod = deadperiod;
	}		
}

#if defined(__AVR_ATmega1280__)
ISR(TIMER3_OVF_vect) 
{
	counter2cntport |= _BV(counter2cntpin); 
	*counter2deadperiod -= *counter2currentservo;
	TCNT3 = CalcCounterValue(*counter2currentservo++); 
	counter2cntport &= ~_BV(counter2cntpin);
	
	if (counter2currentservo > &counter2servopositions[9])
	{
		counter2currentservo = &counter2servopositions[0];
		*counter2deadperiod = deadperiod;
	}		
}
#endif //__AVR_ATmega1280__
#endif //HIGHACCURACY

ISR(TIMER2_OVF_vect)  
{
	#ifndef HIGHACCURACY
	counter1current += step;
	//Time to pulse counter 1?
	if (counter1current >= *counter1currentservo)
	{
		counter1cntport |= _BV(counter1cntpin);
		counter1current = 0;
		*counter1deadperiod -= *counter1currentservo;
		
		if (counter1currentservo > &counter1servopositions[8])
		{
			counter1currentservo = &counter1servopositions[0];
			*counter1deadperiod = deadperiod;
		}		
			else
			
				counter1currentservo++;	
			
		counter1cntport &= ~_BV(counter1cntpin); 
	}
	#endif	
	
	counter2current += step;
	//Time to pulse counter 2?
	if (counter2current >= *counter2currentservo)
	{
		counter2cntport |= _BV(counter2cntpin); 		
		counter2current = 0;		
		*counter2deadperiod -= *counter2currentservo;
		
		if (counter2currentservo > &counter2servopositions[8])
		{
			counter2currentservo = &counter2servopositions[0];
			*counter2deadperiod = deadperiod;
		}		
			else
			
				counter2currentservo++;		
			
		counter2cntport &= ~_BV(counter2cntpin);  
	}	
	
	TCNT2 = 176;	// reinit counter
}

void timer_init()
{
	#ifdef TCCR2B
	TCCR2B = (1<<CS20); // prescaler = 1
	#else
	TCCR2 = (1<<CS20); // prescaler = 1
	#endif //TCCR2B
	
	#ifdef HIGHACCURACY
	//Need to clear first
	TIMSK1 = 0;  
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1C = 0;
	
	TCCR1B = (1 << CS10);
	TIMSK1 = (1 << TOIE1);
	TCNT1 = CalcCounterValue(*counter1currentservo++); 
	
	#ifdef __AVR_ATmega1280__
	//Need to clear first
	TIMSK3 = 0;
	TCCR3A = 0;
	TCCR3B = 0;
	TCCR3C = 0;
	
	TCCR3B = (1 << CS30); 
	TIMSK3 = (1 << TOIE3);	
	TCNT3 = CalcCounterValue(*counter2currentservo++); 
	#else
	TIMSK2 = (1 << TOIE2); 		 // Timer2 Overflow Interrupt Enable
	TCNT2 = 176;				 // init counter
	#endif //__AVR_ATmega1280__
	#else
	TIMSK2 = (1 << TOIE2); 	 	// Timer2 Overflow Interrupt Enable
	TCNT2 = 176;				 // init counter
	#endif //HIGHACCURACY
}

ServoShield::ServoShield()
{
	//Set all servos to default center
	for (int servo = 0; servo < 10; servo++) 
	{
		counter1servopositions[servo] = 1500;
		counter2servopositions[servo] = 1500;
		servosmax[servo] = 2000;
		servosmin[servo] = 1000;
	}
	
	//Setup pin modes
	pinMode(counter1resetpin, OUTPUT);
	pinMode(counter2resetpin, OUTPUT);
	counter1cntddr |= _BV(counter1cntpin); 
	counter2cntddr |= _BV(counter2cntpin); 
}

int ServoShield::setposition(int servo, int position)
{
	if ((position >= servosmin[servo]) && (position <= servosmax[servo]))
	{
		//Servo 0 to 8 on counter 1
		if ((servo >= 0) && (servo < 9))
		{
			counter1servopositions[outputmap[servo]] = position;
			return 0;
		}

		//Servo 9 to 16 on counter 2
		if ((servo >= 9) && (servo < 16))
		{
			counter2servopositions[outputmap[servo]] = position;
			return 0;
		}
		
		return 1;
	}
	
	return 1;
}

int ServoShield::getposition(int servo)
{
	//Servo 0 to 8 on counter 1
	if ((servo >= 0) && (servo < 9))
		return counter1servopositions[outputmap[servo]];

	//Servo 9 to 16 on counter 2
	if ((servo >= 9) && (servo < 16))
		return counter2servopositions[outputmap[servo]];
		
	return -1;
}

int ServoShield::setbounds(int servo, int minposition, int maxposition)
{
	if (servo < 16)
	{
		servosmax[servo] = maxposition;
		servosmin[servo] = minposition;
		return 0;
	}
	
	return 1;
}

int ServoShield::start()
{
	//Reset counters
	digitalWrite(counter1resetpin, HIGH);  
	digitalWrite(counter2resetpin, HIGH);  
	counter1current = 0;
	counter2current = 0;
	
	//Set servo pointers
	counter1currentservo = &counter1servopositions[0];
	counter2currentservo = &counter2servopositions[0];
	counter1deadperiod = &counter1servopositions[9];
	counter2deadperiod = &counter2servopositions[9];
	
	//Set dead periods
	*counter1deadperiod = deadperiod;
	*counter2deadperiod = deadperiod;
	
	timer_init();
	
	digitalWrite(counter1resetpin, LOW);  
	digitalWrite(counter2resetpin, LOW); 
}

int ServoShield::stop()
{
	TIMSK2 &= ~(1 << TOIE2); 
	
	#ifdef HIGHACCURACY
	TIMSK1 &= ~(1 << TOIE1);
	#ifdef __AVR_ATmega1280__
	TIMSK3 &= ~(3 << TOIE3);
	#endif //__AVR_ATmega1280__
	#endif //HIGHACCURACY
}