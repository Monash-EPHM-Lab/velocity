/*
BoSL Velocity Firmware rev: 0.1.2
*/

//FFT library
#include "src\arduinoFFTfix.h"
#include "src\I2C.h"
#include <LowPower.h>

#define SCAN 8
#define THRESHOLD 8000
#define SAMPLES 256
#define ADCADR 0x34
#define INTPIN 2

////arrays and constants 
const float calArray[5] PROGMEM = {1, 9.72, 19.7, 35.7, 69.7}; //fft bin to frequency (Hz)
const float convgd[5] PROGMEM = {-0.242, -0.3449, 0, 0.3449, 0.242}; //convlution which takes derivative and gausian blur (sigma = 1)
//psd inverse to multiply ACC with
#if SCAN != 8
	#error PSD will be wrong if scans is not 8, need to change psd.
#endif

//this psd has been normalised to have a sum of 128 and then the reciprocal was taken to optimise division 
const float psd[128] PROGMEM = {0.0486, 0.0515, 0.1005, 0.1684, 0.2097, 0.2548, 0.2918, 0.2941, 0.3260, 0.3872, 0.4247, 0.4557, 0.5112, 0.5283, 0.5929, 0.6724,
								0.7019, 0.7564, 0.7914, 0.8685, 1.0031, 1.1231, 1.1873, 1.3456, 1.4145, 1.5398, 1.7043, 1.6475, 1.8733, 2.1320, 2.1425, 2.3349,
								2.4325, 2.6274, 2.7041, 2.7515, 2.8640, 2.8970, 2.8848, 2.8773, 2.9352, 2.9080, 2.9468, 2.9932, 3.0037, 3.0858, 3.1253, 2.9065,
								2.7438, 2.6662, 2.6206, 2.4676, 2.2633, 2.4108, 2.7908, 3.1709, 3.1852, 3.2739, 3.5544, 3.6771, 3.6985, 3.8011, 3.8440, 3.8430,
								3.8374, 3.8902, 3.9454, 3.9505, 4.0462, 4.1509, 4.1788, 4.2397, 4.0877, 4.1413, 4.3790, 4.3199, 4.1033, 4.0549, 3.8106, 3.4437,
								3.1874, 3.0327, 2.6496, 2.8251, 3.2223, 3.6684, 3.8046, 3.7911, 3.9244, 4.1388, 4.1237, 4.1424, 4.1158, 4.3081, 4.3293, 4.3082,
								4.2095, 4.3283, 4.4344, 4.4344, 4.4511, 4.5481, 4.4713, 4.4890, 4.4620, 4.5747, 4.5343, 4.5284, 4.5218, 4.3216, 4.4784, 4.4617,
								4.5578, 4.7484, 4.6178, 4.5849, 4.5438, 4.7071, 4.7038, 4.6925, 4.5123, 4.5673, 4.7224, 4.7807, 4.6650, 4.6258, 4.7032, 4.6332};


//FFT sample buffers
int16_t read[SAMPLES];
float acc[SAMPLES/2];

float convstack[3];

float rangeScaler;


float vem; //velocity measurement mean
float ves; //velocity standard deviation
float vea; //velocity amplitude score


bool debug = 0;
//Define FFT object
arduinoFFTfix FFTfix = arduinoFFTfix();

void doFFT(void){
	//create imaginary array for FFT //must be here for mem reasons
	int16_t imag[SAMPLES];
    //set imaginary componet of FFT to zero
    for (int i = 0; i < SAMPLES; i++) {
      imag[i] = 0;
    }

    // //Compute FFT
    rangeScaler = FFTfix.RangeScaling(read, SAMPLES);
    FFTfix.Windowing(read, SAMPLES, FFT_FORWARD);
    FFTfix.Compute(read, imag, SAMPLES, FFT_FORWARD);
    FFTfix.ComplexToMagnitude(read, imag, SAMPLES);
		
}

void getVel(int velMulti, int scans, bool plot = 0) {
  
	rangeScaler = 0;
	
	for(int i = 0; i < SAMPLES/2; i++){
		acc[i] = 0;
	}

  for (int iter = 0; iter < scans; iter++) {

    if (velMulti == 4) {
      sampleFast();
    }
    if (velMulti == 3) {
      sampleSlow(20);
    }
    if (velMulti == 2) {
      sampleSlow(200);
    }
    if (velMulti == 1) {
      sampleSlow(600);
    }
    if (velMulti == 0) {
      sampleSlow(1400);
    }

    //delete DC component of signal
    delDCcomp();

	doFFT();
   
	for(int i = 0; i<SAMPLES/2; i++){
		float temp = (((float)read[i])/(((float)rangeScaler)*4.0));
		acc[i] += temp*temp;
	}
	
    
    }
	for(int i = 0; i<SAMPLES/2; i++){
		acc[i] += sqrt(acc[i]);

	}
	
   if(plot){
	   plotFFT();
   }
   if(debug){
	    //plotFFT();
	    //Serial.println();
	   printFFT();
   }

   maxPeak(velMulti);
   
// returns in vem, ves, vea
}

void setup() {

  Serial.begin(9600);
  Serial.print('R');
  
}


void loop() {

  uint8_t cmd;

  if (Serial.available() > 0) {
    cmd = Serial.read();

	if (cmd == 'V') {
      getFFT(0);
    }
	if (cmd == 'F') {
      getFFT(1);
    }
	if (cmd == 'D') {
      printRes();
    }
    if (cmd == 'S') {
      sleepF();
    }
	if (cmd == 'I') {
      getInf();
    }
	
	//debug mode
	if (cmd == 'B') {
	  debug = 1;
      while(1){
	  getFFT(0);
	  }
    }

  }


}

void sleepF() {
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void printRes(){
	Serial.print(",");
    Serial.print(vem, 0);
	Serial.print(",");
    Serial.print(ves, 0);
	Serial.print(",");
    Serial.print(vea, 0);
	Serial.print("T");
}

void getFFT(bool fft){
	pinMode(3, OUTPUT);
	pinMode(9, OUTPUT);
	pinMode(A0, OUTPUT);
	pinMode(A1, OUTPUT);
	digitalWrite(9, HIGH);
	digitalWrite(3, HIGH);
	digitalWrite(A0, HIGH);
	digitalWrite(A1, HIGH);

	I2c.begin();
	I2c.setSpeed(1); //Note 200 kHz Bus Speed
	
	if(fft){
		 getVel(4, SCAN, 1);	
	}else{
		 getVel(4, SCAN, 0);
		 
		if(not debug){
			printRes();
		}
	}
	
	digitalWrite(9, LOW);
    digitalWrite(3, LOW);
    digitalWrite(A0, LOW);
    digitalWrite(A1, LOW);
    I2c.end();
	
}

void maxPeak(int velMulti){
	
	vem = 0;
	ves = 0;
	vea = 0;
	
	int max_bin = 0;


	//whiten noise
	whiten();
	
	
	//find maximum bin and maximum value
	for (int i = 3; i < (SAMPLES / 4); i++)
    {
      if (acc[i] > vea) {
        vea = acc[i];
        max_bin = i;
      }

    }
	
	//this find a peak outside the low noise range if present
	//maybe this is a bad idea, should be safe to comment out if so
	if (max_bin < 8){
		float max_val = THRESHOLD;
		for (int i = 8; i < 16; i++)
		{
		if (acc[i] > max_val) {
			max_val = acc[i];
			max_bin = i;
			vea = max_val;
		}
		}
	}
	
    for (int i = 3; i < (SAMPLES / 4); i++) {
      if (acc[i] < THRESHOLD) {
        acc[i] = 0;
      }

    }

    bool notPeak = 0;
    for (int i = max_bin; i < (SAMPLES / 4); i++) {
      if (acc[i] == 0) {
        notPeak = 1;
      }
      if (notPeak) {
        acc[i] = 0;
      }
    }
    notPeak = 0;
    for (int i = max_bin; i > 0; i--) {
      if (acc[i] == 0) {
        notPeak = 1;
      }
      if (notPeak) {
        acc[i] = 0;
      }
    }

	float norm = 0;

    for (int i = 3; i < (SAMPLES / 4) ; i++) {
      vem += (float)acc[i] * (float)i;
      norm += (float)acc[i];
    }

    vem = vem / norm;


    for (int i = 3; i < (SAMPLES / 4); i++) {
      ves += ((float)(i - vem))*((float)(i - vem))*((float)acc[i]);
    }
	
	ves = sqrt(ves / norm);


    //converts frequency to mm/s
    ves = 1.15*(ves) * (pgm_read_float_near(&calArray[velMulti]))*0.75  /(SAMPLES / 128);
    vem = 1.15*(vem) * (pgm_read_float_near(&calArray[velMulti]))*0.75  /(SAMPLES / 128); 
	vea = norm/1E5;

}


void doConv(){
	stackSet();
	for(int i = 0; i< SAMPLES/4; i++){
		float sum = 0;
		float pop = 0;
		for(int j = 0; j < 5; j++){
			sum += pgm_read_float_near(&convgd[j])*getAcc(i-j);
		}
		pop = stackPush(sum);
		setAcc(i-4, pop);
	}
	
}

float getAcc(int idx){
	if((idx >= 0) and (idx < SAMPLES/2)){
		return acc[idx];
	}else{
		return 0;
	}
}

void setAcc(int idx, float val){
	if((idx >= 0) and (idx < SAMPLES/2)){
		acc[idx] = val;
	}
}

void whiten(){
	for(int i = 0; i < SAMPLES/4 ; i++){
		acc[i] = acc[i]*pgm_read_float_near(&psd[i]);
	}	
}

void stackSet(){
	for(int i =0; i < 4; i++){
	convstack[i] = 0;
	}
}

float stackPush(float val){
	float popped = convstack[0];
	for(int i =1; i < 3; i++){
	convstack[i] = convstack[i-1];
	}
	convstack[2] = val;
	return popped;
}


void sampleFast() {
  uint8_t adcData[SAMPLES * 2];

  I2c.write(ADCADR, 0b10101100);
  I2c.write(ADCADR, 0b00000010);

  I2c.readex(ADCADR, SAMPLES * 2, adcData);

  for (int i = 0; i < SAMPLES * 2; i += 2) {
    int rsult = (adcData[i] - 240) * 256 + adcData[i + 1];
    if (rsult > 2048) {
      rsult = rsult - 4096;
    }
    read[i / 2] = rsult;
  }

}

void sampleSlow(int delay) {
  I2c.write(ADCADR, 0b10100100);
  I2c.write(ADCADR, 0b00000010);

  uint8_t MSB;
  uint8_t LSB;


  for (int i = 0; i < SAMPLES * 2; i += 2) {
    I2c.read(ADCADR, 2);

    MSB = I2c.receive();
    LSB = I2c.receive();

    int16_t rsult = (MSB - 240) * 256 + LSB;
    if (rsult > 2048) {
      rsult = rsult - 4096;
    }
    read[i / 2] = rsult;
    delayMicroseconds(delay);
  }



}


void plotRAW() {
  for (int i = 0; i < (SAMPLES); i++)
  {
    Serial.println(acc[i]);
  }
}

void printFFT() {

  for (int i = 0; i < (SAMPLES / 2); i++)
  {
    Serial.println(acc[i]);

  }

  for (int i = 0; i < (1); i++)
  {
    Serial.println(-10);
  }
}

void plotFFT() {
  Serial.print(',');
  for (int i = 0; i < (SAMPLES / 2); i++)
  {
    Serial.print(acc[i]);
	Serial.print(',');
  }

	Serial.print('J');
	Serial.print(',');
	Serial.print(rangeScaler);
	Serial.print(',');
    Serial.print('T');
}


void clearPlot() {
  delay(3000);
  for (int i = 0; i < 500; i++)
  {
    Serial.println(0);
  }
}

void delDCcomp() {
  int32_t average = 0;

  for ( int16_t i = 0; i < SAMPLES; i++) {
    average += read[i];
  }
  average = (average / SAMPLES);

  for ( int16_t i = 0; i < SAMPLES; i++) {
    read[i] -= average;
  }
}


void getInf(){
	Serial.println(F("ID: 00x"));
	Serial.println(F("Firmware rev: 0.1.2"));
	Serial.println(F("Hardware rev: 0.1.3"));
}

