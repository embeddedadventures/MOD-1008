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

#include "SHT11.h"
#include "Arduino.h"

#define DEBUG

#ifdef DEBUG
	#define DEBUGHEX(x) Serial.println(x, HEX)
#else
	#define DEBUGHEX(x)
#endif

void SHT11Class::begin(int data, int clock) {
	data_pin = data;
	clk_pin = clock;
	sendCommand(SOFT_RESET);
	delay(RESET_TIME);
}

void SHT11Class::transmissionStart() {
	pinMode(data_pin, OUTPUT);
	pinMode(clk_pin, OUTPUT);
	digitalWrite(data_pin, HIGH);
	digitalWrite(clk_pin, HIGH);
	digitalWrite(data_pin, LOW);
	digitalWrite(clk_pin, LOW);
	digitalWrite(clk_pin, HIGH);
	digitalWrite(data_pin, HIGH);
	digitalWrite(clk_pin, LOW);
}

void SHT11Class::acknowledge() {
	pinMode(data_pin, OUTPUT);
	digitalWrite(data_pin, HIGH);
	digitalWrite(data_pin, LOW);
	digitalWrite(clk_pin, HIGH);
	digitalWrite(clk_pin, LOW);
}

uns8 SHT11Class::readIn() {
	int data = 0;
	pinMode(data_pin, INPUT);
	pinMode(clk_pin, OUTPUT);
	
	for (int i = 0; i < SIZE_BYTE; ++i) {
		digitalWrite(clk_pin, HIGH);
		delay(DELAY_TIME);
		data = data * 2 + digitalRead(data_pin);
		digitalWrite(clk_pin, LOW);
	}

	return data;
}

void SHT11Class::sendCommand(uns8 cmd) {
	int ack;
	
	transmissionStart();
	shiftOut(data_pin, clk_pin, MSBFIRST, cmd);
	
	//Clock pulse for the ACK
	digitalWrite(clk_pin, HIGH);
	digitalWrite(clk_pin, LOW);
}

bool SHT11Class::dataReady() {
	pinMode(data_pin, INPUT);
	delay(DELAY_TIME);
	return (digitalRead(data_pin) == LOW) ? true : false;

}

float SHT11Class::getTemperature() {
	float result;
	
	sendCommand(MEAS_TEMP);
	
	while (!dataReady()) {}
	
	raw_data = readIn();
	acknowledge();
	raw_data = raw_data << 8;

	raw_data |= readIn();
	acknowledge();

	crc = readIn();
	acknowledge();
	
	result = (raw_data * d2) + d1;
	return result;
}

float SHT11Class::getHumidity() {
	float  rh_linear, rh_correct, temperature;

	temperature = getTemperature();
	
	sendCommand(MEAS_HUMID);
	
	while (!dataReady()) {}
	
	raw_data = readIn();
	acknowledge();
	raw_data = raw_data << 8;

	raw_data |= readIn();
	acknowledge();

	crc = readIn();
	acknowledge();
	
	rh_linear = C1 + (C2 * raw_data) + (C3 * raw_data * raw_data);
	
	rh_correct = ((temperature - 25.0) * (T1 + (T2 * raw_data))) + rh_linear;
	return rh_correct;
}

SHT11Class mod1008;

