/*
BoSL Velocity Firmware rev: 0.1.1
*/

//FFT library
#include "src\arduinoFFTfix.h"
#include "src\I2C.h"
#include <LowPower.h>

#define SCAN 8
#define DEBUG 1
#define MAXTHRESHOLD 70
#define CANNYTHRESHOLD -2000
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
const float psd[128] PROGMEM = {0.0434, 0.1302, 0.3614, 0.5516, 0.7673, 1.0930, 1.2842, 1.4360, 1.7642, 2.1021, 2.2257, 2.1246, 2.2491, 2.3856, 2.2418, 1.9725,
								1.7545, 1.7474, 2.0761, 2.4401, 1.9419, 1.5914, 1.6817, 2.0773, 2.2819, 2.5188, 2.7833, 2.9392, 2.7098, 2.7949, 3.0510, 3.3100, 
								3.2085, 3.3210, 3.3978, 3.3780, 3.2344, 3.1959, 3.2777, 3.4199, 3.3442, 3.3688, 3.1197, 3.2555, 3.2136, 3.0687, 2.9642, 2.7663, 
								2.3861, 2.1492, 2.1230,	2.2483, 2.3645, 2.5430, 2.5027, 2.6091, 2.6616, 2.6996, 2.8795, 2.8860, 2.9746, 3.0419, 3.1340, 3.1399,
								3.2030, 3.3240, 3.2957, 3.3536, 3.4545, 3.5697, 3.5310, 3.5416, 3.6472, 3.6616, 3.6112, 3.6202, 3.6655, 3.6874, 3.6856, 3.7181,
								3.7431, 3.6836, 3.5221, 3.5640, 3.5330, 3.4858, 3.6022, 3.5891, 3.6414, 3.6996, 3.7098, 3.7616, 3.6117, 3.6612, 3.7405, 3.7618,
								3.7321, 3.8477, 3.8426, 3.7889, 3.5972, 3.6667,	3.7568, 3.7154, 3.7625, 3.9065, 3.8923, 3.7489, 3.7727, 3.8224, 3.8191, 3.8335, 
								3.7039, 3.8566, 3.8530, 3.7970, 3.8846, 3.8159, 3.8710,	3.8742, 3.8441, 3.8637, 3.8246, 3.8172, 3.7997, 3.8871, 3.7802, 3.6647};


//FFT sample buffers
int16_t read[SAMPLES];
float acc[SAMPLES/2];

float convstack[3];

float rangeScaler;


float vem; //faling edge velocity
float ves; //velocity standard deviation
float vea; //velocity amplitude score

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

void getVel(int velMulti, int scans, bool plot = 0, bool canny = 0) {
  
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
   if(DEBUG){
	   // plotFFT();
	   // Serial.println();
	   printFFT();
   }
   
   if (canny){
   cannyPeak(velMulti);
   }else{
   maxPeak(velMulti);
   }
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
      getFFT(0,0);
    }
	if (cmd == 'F') {
      getFFT(1,0);
    }
	if (cmd == 'B') {
      getFFT(0,1);
    }
	if (cmd == 'C') {
      getFFT(1,1);
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

  }
  if(DEBUG){
	getFFT(0,0);  
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

void getFFT(bool fft, bool canny){
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
		 getVel(4, SCAN, 1, canny);	
	}else{
		 getVel(4, SCAN, 0, canny);
		 
		if(not DEBUG){
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
	
	for (int i = 2; i < (SAMPLES / 4); i++)
    {
      if (acc[i] > vea) {
        vea = acc[i];
        max_bin = i;
      }

    }

    for (int i = 2; i < (SAMPLES / 4); i++) {
      if (acc[i] < MAXTHRESHOLD) {
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

    for (int i = 2; i < (SAMPLES / 4) ; i++) {
      vem += (float)acc[i] * (float)i;
      norm += (float)acc[i];
    }

    vem = vem / norm;


    for (int i = 2; i < (SAMPLES / 4); i++) {
      ves += ((float)(i - vem))*((float)(i - vem))*((float)acc[i]);
    }
	
	ves = sqrt(ves / norm);


    //converts frequency to mm/s
    ves = (ves) * (pgm_read_float_near(&calArray[velMulti]))*0.75  /(SAMPLES / 128);
    vem = (vem) * (pgm_read_float_near(&calArray[velMulti]))*0.75  /(SAMPLES / 128); 
	vea = norm/1E5;

}

void cannyPeak(int velMulti){
	int max_bin = 0;
	
	vem = 0;
    ves = 0;
	vea = 0;	
	
	//whiten noise
	whiten();
	
	//convolve FFT
	doConv();
	
	//find first peak
	for (int i = SAMPLES/4; i >= 0; i--){
		if (acc[i] < MAXTHRESHOLD){
			if (acc[i] < getAcc(i-1)){
			max_bin = i;
			break;
			}
		}
	}
	//store amplitude
	vea = acc[max_bin];
	
	float nonmaxThresh = vea*0.3;
	
	
	for (int i = 0; i < (SAMPLES / 4); i++) {
      if (acc[i] > nonmaxThresh) {
        acc[i] = 0;
      }

    }

    bool notPeak = 0;
	
	//isolate peak from left and right sides
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
	vem = 0;
	//find vem by average
    for (int i = 2; i < (SAMPLES / 4) ; i++) {
      vem += (float)acc[i] * (float)i;
      norm += (float)acc[i];
    }

    vem = vem / norm;

	//stdev
	ves = 0;

    for (int i = 2; i < (SAMPLES / 4); i++) {
      ves += ((float)(i - vem))*((float)(i - vem))*((float)acc[i]);
    }
	
	ves = sqrt(ves / norm);


    //converts frequency to mm/s
    ves = (ves) * (calArray[velMulti])*0.75  /(SAMPLES / 128);
    vem = (vem) * (calArray[velMulti])*0.75  /(SAMPLES / 128); 
	
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
	for(int i = 0; i < SAMPLES/4  + 5; i++){ //+5 is for future convolution purposes, should be removed later
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
	Serial.println(F("Firmware rev: 0.1.1"));
	Serial.println(F("Hardware rev: 0.1.3"));
}

