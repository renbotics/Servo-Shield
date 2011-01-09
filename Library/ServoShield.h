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
#ifndef ServoShield_h
#define ServoShield_h

#define ServoShieldVersion 1.5

#define HIGHACCURACY
#define counter1resetpin 7
#define counter2resetpin 9
#define deadperiod 18500
#define step 26

//Direct IO for faster access
#if defined(__AVR_ATmega1280__)

#define counter1cntddr	DDRH
#define counter1cntport PORTH
#define counter1cntpin 	PH4
#define counter2cntddr	DDRH
#define counter2cntport	PORTH
#define counter2cntpin 	PH5

#else

#define counter1cntddr	DDRD
#define counter1cntport PORTD
#define counter1cntpin 	PORTD6
#define counter2cntddr	DDRB
#define counter2cntport	PORTB
#define counter2cntpin 	PORTB0

#endif

class ServoShield
{
private:
	
public:
	ServoShield();
	int setposition(int servo, int position);
	int setbounds(int servo, int minposition, int maxposition);
	int getposition(int servo);	
	int start();
	int stop();
};

#endif