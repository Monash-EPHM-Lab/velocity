/*
BoSL Velocity Firmware rev: 0.2.0
PSD Spectrum for hardware ID xxx								
*/

//FFT library
#include "src\arduinoFFTfix.h"
#include "src\I2C.h"
#include <LowPower.h>
#include <math.h>

#define SCAN 8
#define THRESHOLD_SLOPE -0.35
#define THRESHOLD_AREA -1
#define SAMPLES 256
#define ADCADR 0x34
#define INTPIN 2

////arrays and constants 
const float calArray[5] PROGMEM = {1, 9.72, 19.7, 35.7, 69.7}; //fft bin to frequency (Hz)
const float convgd[9] PROGMEM = {0.056, 0.094, 0.111, 0.078, 0, -0.0780, -0.111, -0.094, -0.056}; //convlution which takes derivative and gausian blur (sigma = 2)
//psd inverse to multiply ACC with
#if SCAN != 8
	#error PSD will be wrong if scans is not 8, need to change psd.
#endif

//this psd has been normalised to have a sum of 128 and then the log was taken to optimise division 
const float psd[128] PROGMEM = {1.7025,2.8384,2.667,2.3104,2.0386,1.817,1.6309,1.4694,1.3649,1.25,1.1571,1.0574,1.0068,0.9056,0.8216,0.7779,
								0.723,0.635,0.5665,0.506,0.4527,0.3852,0.3417,0.2717,0.2461,0.1822,0.1304,0.0594,0.0621,-0.036,-0.1163,-0.1745,
								-0.2055,-0.2748,-0.3405,-0.4152,-0.4505,-0.4997,-0.5583,-0.6129,-0.6708,-0.7386,-0.8072,-0.8576,-0.9116,-0.9838,-1.0504,-1.0984,
								-1.1451,-1.231,-1.3071,-1.372,-1.4313,-1.5146,-1.5847,-1.6497,-1.7043,-1.797,-1.8682,-1.9366,-2.0212,-2.1136,-2.1768,-2.2614,
								-2.331,-2.4236,-2.5108,-2.6269,-2.6824,-2.7742,-2.8665,-2.972,-3.0449,-3.1653,-3.2781,-3.3992,-3.4868,-3.6119,-3.7214,-3.8351,
								-3.9477,-4.0923,-4.2064,-4.3351,-4.4654,-4.6152,-4.7444,-4.8665,-4.9762,-5.0995,-5.2214,-5.3602,-5.4262,-5.4968,-5.5468,-5.5994,
								-5.5994,-5.5994,-5.5728,-5.5468,-5.4727,-5.4491,-5.4037,-5.3185,-5.2591,-5.203,-5.116,-5.036,-5.0056,-4.9477,-4.8929,-4.8283,
								-4.7795,-4.7444,-4.6886,-4.6565,-4.6356,-4.6152,-4.5854,-4.5756,-4.5282,-4.5099,-4.5099,-4.483,-4.4654,-4.4568,-4.4397,-4.4568};


//FFT sample buffers
int16_t read[SAMPLES];
float acc[SAMPLES/2];

float convstack[8];

float rangeScaler;


float vem; //velocity measurement mean
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
	    plotFFT();
	    Serial.println();
	   //printFFT();
   }

   maxPeak(velMulti);
   
// returns in vem, vea
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
    Serial.print("NA");
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
	delay(20);
	digitalWrite(3, HIGH);
	delay(20);
	digitalWrite(A0, HIGH);
	delay(20);
	digitalWrite(A1, HIGH);
	delay(20);

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
	
	#define IDX_LOW 3
	#define IDX_HIGH 64
	
	vem = 0;
	vea = 0;
	
	int max_bin = 0;


	
	for (int i = 0; i < (SAMPLES/4 +7); i++){ //+7 for later convolution
		acc[i] = log(acc[i]);
	}
	//log whiten noise
	whitenLog();
	
	
	doConv();
	
	bool peakCan = false;
	bool firstZero = true;
	int lstZero = SAMPLES/4;
	float norm = 0;
	for(int i = SAMPLES/4 - 1; i >= 0; i--){
		if (not peakCan){
			if (acc[i] > 0){
				lstZero = i;
				norm = 0;
			}else{
				norm += acc[i];
			}
			
			if (acc[i] > THRESHOLD_SLOPE){
				max_bin = i;
			}else if (acc[i] < acc[i-1]){
				max_bin = i;
				peakCan = true;
			}
			
		}else{
			norm += acc[i];
			if (firstZero){
				if (acc[i] > 0){
					firstZero = false;
				}
			}else{
				if (acc[i] < 0){
					if (norm < THRESHOLD_AREA){
						break;
					}else{
						peakCan = false;
						firstZero = true;
						lstZero = i;
						norm = 0;
					}
				}
			}
		}	
	}
	
	for(int i = 0; i < SAMPLES/4; i++){
		if (acc[i] > 0){
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
	
	norm = 0;

    for (int i = 3; i < (SAMPLES / 4) ; i++) {
      vem += (float)acc[i] * (float)i;
      norm += (float)acc[i];
    }

    vem = vem / norm;


    //converts frequency to mm/s
    vem = 0.6*1.15*(vem) * (pgm_read_float_near(&calArray[velMulti]))*0.75  /(SAMPLES / 128); 
	vea = -norm;//return vea as max signal strength

}


void doConv(){
	stackSet();
	for(int i = 0; i< SAMPLES/4 + 7; i++){
		float sum = 0;
		float pop = 0;
		for(int j = 0; j < 9; j++){
			sum += pgm_read_float_near(&convgd[j])*getAcc(i-j+4);
		}
		pop = stackPush(sum);
		setAcc(i-4, pop);
	}
	setAcc(0, 0);
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

void whitenLog(){
	for(int i = 0; i < SAMPLES/4 +7 ; i++){ //+7 for later convolution
		acc[i] = acc[i] - pgm_read_float_near(&psd[i]);
	}	
}

void stackSet(){
	for(int i =0; i < 8; i++){
	convstack[i] = 0;
	}
}

float stackPush(float val){
	float popped = convstack[0];
	for(int i = 0; i < 8; i++){
	convstack[i] = convstack[i+1];
	}
	convstack[3] = val;
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
	Serial.println(F("Firmware rev: 0.2.0"));
	Serial.println(F("Hardware rev: 0.1.3"));
}

