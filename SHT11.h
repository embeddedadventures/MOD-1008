/*
Copyright (c) 2016, Embedded Adventures
All rights reserved.
Contact us at source [at] embeddedadventures.com
www.embeddedadventures.com
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
- Neither the name of Embedded Adventures nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
THE POSSIBILITY OF SUCH DAMAGE.
*/

// SHT11/15 MOD-1008 temperature and humidity sensor Arduino library
// Written originally by Embedded Adventures


#ifndef __SHT11_H
#define __SHT11_H

#include "inttypes.h"

#define SIZE_BYTE		8
#define DELAY_TIME		10
#define RESET_TIME		15

#define MEAS_TEMP		0x03
#define MEAS_HUMID		0x05
#define READ_SREG		0x07
#define WRITE_SREG		0x06
#define SOFT_RESET		0x1E

#ifndef uns8
	#define uns8	unsigned char
#endif

#ifndef uns16
	#define uns16	unsigned int
#endif

class SHT11Class {
private:
	int		data_pin;
	int		clk_pin;
	uns16	raw_data;
	uns8	crc;
	
	//Constant coefficients for 14-bit/12-bit precision, at 3.3V
	const float		C1 = -4.0;
	const float		C2 = 0.0405;
	const double	C3 = -0.0000028;
	const float		T1 = 0.01;
	const float		T2 = 0.00008;
	const float		d1 = -38.4;
	const float		d2 = 0.0098;

	uns8	readIn();
	bool	dataReady();
	void	acknowledge();
	void	transmissionStart();
	void	sendCommand(uns8 cmd);
	void	readRaw();
	
public:
	void	begin(int data, int clock);
	float	getTemperature();
	float	getHumidity();
};

extern SHT11Class mod1008;

#endif