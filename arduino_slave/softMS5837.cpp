#include "softMS5837.h"
#include <SoftWire.h>

uint8_t sdaPin = A2;
uint8_t sclPin = A3;
SoftWire softwire(sdaPin, sclPin);

#define MS5837_ADDR               0x76  
#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1_8192    0x4A
#define MS5837_CONVERT_D2_8192    0x5A

const float softMS5837::Pa = 100.0f;
const float softMS5837::bar = 0.001f;
const float softMS5837::mbar = 1.0f;

const uint8_t softMS5837::MS5837_30BA = 0;
const uint8_t softMS5837::MS5837_02BA = 1;

uint8_t txbuffer[10];
uint8_t rxbuffer[10];

softMS5837::softMS5837() {
	fluidDensity = 1029;
}

bool softMS5837::init() {
    softwire.setDelay_us(5);
    softwire.setTxBuffer(txbuffer, 10);
    softwire.setRxBuffer(rxbuffer, 10);
    softwire.begin();

	// Reset the softMS5837, per datasheet
	softwire.beginTransmission(MS5837_ADDR);
    softwire.write(MS5837_RESET);
	softwire.endTransmission();

	// Wait for reset to complete
	delay(10);

	// Read calibration values and CRC
	for ( uint8_t i = 0 ; i < 7 ; i++ ) {
		softwire.beginTransmission(MS5837_ADDR);
		softwire.write(MS5837_PROM_READ+i*2);
		softwire.endTransmission();

		softwire.requestFrom(MS5837_ADDR,2);
		C[i] = (softwire.read() << 8) | softwire.read();
	}

	// Verify that data is correct with CRC
	uint8_t crcRead = C[0] >> 12;
	uint8_t crcCalculated = crc4(C);

	if ( crcCalculated == crcRead ) {
		return true; // Initialization success
	}

	return false; // CRC fail
}

void softMS5837::setModel(uint8_t model) {
	_model = model;
}

void softMS5837::setFluidDensity(float density) {
	fluidDensity = density;
}

void softMS5837::read() {
	// Request D1 conversion
	softwire.beginTransmission(MS5837_ADDR);
	softwire.write(MS5837_CONVERT_D1_8192);
	softwire.endTransmission();

	delay(20); // Max conversion time per datasheet
	
	softwire.beginTransmission(MS5837_ADDR);
	softwire.write(MS5837_ADC_READ);
	softwire.endTransmission();

 	softwire.requestFrom(MS5837_ADDR,3);
	D1 = 0;
	D1 = softwire.read();
	D1 = (D1 << 8) | softwire.read();
	D1 = (D1 << 8) | softwire.read();
	
	// Request D2 conversion
	softwire.beginTransmission(MS5837_ADDR);
	softwire.write(MS5837_CONVERT_D2_8192);
	softwire.endTransmission();

	delay(20); // Max conversion time per datasheet
	
	softwire.beginTransmission(MS5837_ADDR);
	softwire.write(MS5837_ADC_READ);
	softwire.endTransmission();

	softwire.requestFrom(MS5837_ADDR,3);
	D2 = 0;
	D2 = softwire.read();
	D2 = (D2 << 8) | softwire.read();
	D2 = (D2 << 8) | softwire.read();

	calculate();
}

void softMS5837::calculate() {
	// Given C1-C6 and D1, D2, calculated TEMP and P
	// Do conversion first and then second order temp compensation
	
	int32_t dT = 0;
	int64_t SENS = 0;
	int64_t OFF = 0;
	int32_t SENSi = 0;
	int32_t OFFi = 0;  
	int32_t Ti = 0;    
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;
	
	// Terms called
	dT = D2-uint32_t(C[5])*256l;
	if ( _model == MS5837_02BA ) {
		SENS = int64_t(C[1])*65536l+(int64_t(C[3])*dT)/128l;
		OFF = int64_t(C[2])*131072l+(int64_t(C[4])*dT)/64l;
		P = (D1*SENS/(2097152l)-OFF)/(32768l);
	} else {
		SENS = int64_t(C[1])*32768l+(int64_t(C[3])*dT)/256l;
		OFF = int64_t(C[2])*65536l+(int64_t(C[4])*dT)/128l;
		P = (D1*SENS/(2097152l)-OFF)/(8192l);
	}
	
	// Temp conversion
	TEMP = 2000l+int64_t(dT)*C[6]/8388608LL;
	
	//Second order compensation
	if ( _model == MS5837_02BA ) {
		if((TEMP/100)<20){         //Low temp
			Serial.println("here");
			Ti = (11*int64_t(dT)*int64_t(dT))/(34359738368LL);
			OFFi = (31*(TEMP-2000)*(TEMP-2000))/8;
			SENSi = (63*(TEMP-2000)*(TEMP-2000))/32;
		}
	} else {
		if((TEMP/100)<20){         //Low temp
			Ti = (3*int64_t(dT)*int64_t(dT))/(8589934592LL);
			OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
			SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
			if((TEMP/100)<-15){    //Very low temp
				OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
				SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
			}
		}
		else if((TEMP/100)>=20){    //High temp
			Ti = 2*(dT*dT)/(137438953472LL);
			OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
			SENSi = 0;
		}
	}
	
	OFF2 = OFF-OFFi;           //Calculate pressure and temp second order
	SENS2 = SENS-SENSi;
	
	if ( _model == MS5837_02BA ) {
		TEMP = (TEMP-Ti);
		P = (((D1*SENS2)/2097152l-OFF2)/32768l)/100;
	} else {
		TEMP = (TEMP-Ti);
		P = (((D1*SENS2)/2097152l-OFF2)/8192l)/10;
	}
}

float softMS5837::pressure(float conversion) {
	return P*conversion;
}

float softMS5837::temperature() {
	return TEMP/100.0f;
}

float softMS5837::depth() {
	return (pressure(softMS5837::Pa)-101300)/(fluidDensity*9.80665);
}

float softMS5837::altitude() {
	return (1-pow((pressure()/1013.25),.190284))*145366.45*.3048;
}


uint8_t softMS5837::crc4(uint16_t n_prom[]) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}
	
	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}