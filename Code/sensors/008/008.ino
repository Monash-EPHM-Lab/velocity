/*
BoSL Velocity Firmware rev: 0.1.2
PSD Spectrum for hardware ID 008
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
const float psd[128] PROGMEM = {0.3806, 0.1303, 0.1485, 0.1726, 0.1932, 0.2132, 0.2211, 0.2311, 0.2448, 0.2513, 0.2731, 0.2856, 0.3034, 0.3139, 0.3261, 0.3392,
								0.3420, 0.3443, 0.3655, 0.4015, 0.3938, 0.3966, 0.4051, 0.4822, 0.4973, 0.5126, 0.4977, 0.5558, 0.5854, 0.6108, 0.6206, 0.6196,
								0.6933, 0.7234, 0.7162, 0.8556, 0.8723, 0.9217, 0.9391, 0.9697, 1.0188, 1.1093, 1.1726, 1.1923, 1.2873, 1.4040, 1.5122, 1.6894,
								1.7648, 1.8897, 2.0254, 2.1058, 2.4119, 2.5388, 2.5408, 2.8293, 3.1712, 3.3052, 3.5925, 3.9158, 4.2484, 5.0744, 5.7451, 5.9234,
								6.1793, 6.8837, 7.8759, 8.5415, 9.7165, 10.1505, 11.2395, 12.8534, 14.3306, 16.1929, 18.2473, 19.9281, 22.2674, 25.6449, 28.7420, 31.4176,
								35.2440, 39.7451, 47.5246, 51.6912, 56.4388, 65.3383, 72.9289, 81.7954, 88.3413, 97.0156, 104.3109, 110.8951, 110.8787, 115.1556, 118.5631, 127.7645,
								118.4262, 119.4189, 112.6865, 109.1256, 101.3627, 92.4494, 88.2445, 84.4878, 70.8996, 59.6193, 52.7538, 47.6588, 50.5984, 58.6009, 57.9473, 53.0576,
								49.1523, 49.5528, 47.5766, 46.7391, 45.2846, 40.2341, 39.2350, 38.3590, 39.9614, 38.5653, 37.0822, 37.2440, 39.0274, 37.4776, 37.1228, 36.8253};


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

  Serial.begin(2400);
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
	//vea = norm/1E3; //return vea as peak height

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
	Serial.println(F("ID: 008"));
	Serial.println(F("Firmware rev: 0.1.2"));
	Serial.println(F("Hardware rev: 0.1.3"));
}

