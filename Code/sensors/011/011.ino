/*
BoSL Velocity Firmware rev: 0.1.2
PSD Spectrum for hardware ID 011
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
const float psd[128] PROGMEM = {0.1190, 0.0431, 0.0513, 0.0767, 0.1182, 0.1641, 0.2135, 0.2489, 0.3046, 0.3652, 0.4439, 0.5077, 0.5546, 0.6432, 0.6924, 0.7752,
0.8561, 0.9897, 1.0549, 1.1519, 1.2206, 1.2647, 1.3248, 1.3861, 1.4064, 1.4651, 1.5360, 1.5837, 1.5873, 1.6836, 1.7374, 1.8015,
1.7212, 1.8639, 1.9165, 1.9512, 1.9826, 2.0282, 2.2922, 2.3183, 2.2097, 2.4486, 2.5692, 2.6301, 2.7784, 2.9694, 3.2571, 3.2495,
3.5600, 3.7536, 3.6837, 3.8661, 3.9239, 3.9755, 4.3310, 4.4134, 4.8677, 5.8614, 5.9702, 6.4277, 6.6313, 7.4972, 8.0835, 8.4297,
8.4391, 9.3837, 10.1615, 11.0232, 12.1373, 12.7341, 13.1216, 14.7059, 16.1102, 17.0426, 18.9381, 20.0187, 20.0103, 22.2896, 24.0206, 26.7728,
26.6864, 29.0576, 30.7565, 32.9480, 32.8170, 36.2774, 37.9682, 39.7271, 39.2222, 43.0233, 47.3772, 50.5411, 52.0441, 50.9836, 52.9876, 53.2288,
56.1474, 59.6919, 59.1355, 61.2987, 64.6382, 65.1281, 66.1069, 67.0081, 72.8592, 71.0363, 76.6804, 80.3730, 83.3300, 79.8095, 83.3134, 84.3982,
85.3160, 86.2760, 87.9523, 88.7293, 92.2861, 93.9119, 93.5426, 88.7436, 93.2114, 90.1741, 90.8061, 90.3708, 90.0521, 93.0348, 91.8071, 92.8513};


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

  Serial.begin(4800);
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
	Serial.println(F("ID: 011"));
	Serial.println(F("Firmware rev: 0.1.2"));
	Serial.println(F("Hardware rev: 0.1.3"));
}

