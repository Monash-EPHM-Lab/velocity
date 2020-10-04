/*
BoSL Velocity Firmware rev: 0.1.2
PSD Spectrum for hardware ID 010
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
const float psd[128] PROGMEM = {0.1850, 0.1628, 0.3473, 0.5755, 0.7261, 0.8164, 0.8613, 0.8432, 0.7966, 0.7871, 0.7461, 0.7231, 0.6760, 0.6708, 0.6387, 0.6542,
0.6399, 0.6375, 0.6061, 0.6011, 0.5732, 0.5633, 0.5761, 0.5478, 0.5377, 0.5244, 0.5209, 0.5156, 0.5004, 0.4995, 0.5029, 0.4965,
0.5047, 0.5163, 0.5190, 0.5239, 0.5199, 0.5475, 0.5690, 0.5759, 0.5692, 0.5946, 0.6095, 0.6324, 0.6357, 0.6623, 0.6559, 0.6349,
0.6882, 0.7563, 0.8093, 0.8177, 0.8746, 0.9451, 1.0196, 1.0776, 1.0555, 1.1457, 1.1579, 1.2205, 1.2417, 1.3058, 1.4201, 1.4964,
1.4627, 1.5545, 1.6314, 1.6441, 1.6680, 1.7807, 1.7915, 1.7831, 1.8044, 1.9436, 2.0667, 2.1040, 2.0019, 2.0025, 2.1830, 2.2037,
2.1710, 2.3605, 2.4735, 2.4653, 2.4707, 2.5029, 2.6626, 2.7147, 2.6057, 2.6921, 2.7627, 2.7113, 2.6795, 2.8402, 2.8915, 2.9338,
2.9623, 3.1851, 3.3084, 3.2587, 3.3222, 3.4994, 3.3977, 3.4528, 3.6354, 3.8856, 3.8012, 3.9854, 4.0399, 4.0497, 4.2651, 4.2741,
4.2460, 4.3113, 4.3855, 4.5581, 4.5200, 4.6248, 4.7765, 4.7057, 4.6796, 4.8750, 4.9526, 4.7504, 4.6888, 4.9593, 4.7953, 4.7778};


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
	Serial.println(F("ID: 010"));
	Serial.println(F("Firmware rev: 0.1.2"));
	Serial.println(F("Hardware rev: 0.1.3"));
}

