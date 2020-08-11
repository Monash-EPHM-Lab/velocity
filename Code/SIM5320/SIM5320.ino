//setup quick scan and record function from website...so it reads a file and then enters a quick scan and upload sequence until that file is changed again.

#include <SoftwareSerial.h>
#include <JeeLib.h> // Low power functions library
ISR(WDT_vect) { Sleepy::watchdogEvent(); } // Setup the watchdog
#include <avr/wdt.h>

SoftwareSerial Sim900(2,3);
SoftwareSerial velPort(10,11);

float temp, EC, Depth, TempVar, ECVar, DepthVar, lastECwritten, lastTempwritten, lastDepthwritten, ECDiff, TempDiff, DepthDiff, FasterA, ECThresholdForForcedLoggingAtMinInterval;
short counter, DailyCounter;
int a, ScanInterval, MinLogInterval, MaxLogInterval, Writer;
String AAA, BBB;
unsigned long previous;
uint8_t x, answer;
char response[196];
char vel[80];
char ok[3] = {'O', 'K', '\0'};

int TEMPsensorInput = 6;


void setup() {

  TWBR = 158;
  TWSR |= bit (TWPS1);
  ScanInterval = 60; //seconds
  MinLogInterval = 60; //seconds
  MaxLogInterval = 3600; //seconds
  ECThresholdForForcedLoggingAtMinInterval = 0.8;
  pinMode(A2,INPUT); //this is for autologging function at SCAN interval! Just place a 100ohm shunt from 5v to A5 and it should enter into this logging
  
  pinMode(2,INPUT);
  pinMode(3,OUTPUT);
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);

  Serial.begin(19200);
  Sim900.begin(19200);
  Sim900.listen();

  
  temp = 0.0;
  EC = 0.0;
  Depth = 0.0;


	memset(vel, '\0', 80);
	 Serial.println("VEL");
      velread();
	  for(int j = 0; j < 79; j++){
		  Serial.print(vel[j]);
	  }
	  Serial.println(vel[79]);
	

  //set placeholdernumbers
  lastECwritten = ECVar;
  lastTempwritten = TempVar;
  lastDepthwritten = DepthVar;
  EC = ECVar;
  //EC = 0;
  temp = TempVar;
  Depth = DepthVar;

  //set Ec and temp
  SendToWeb222(1);

  EC = 0.0;
  temp = 0.0;
  Depth = 0.0;
}

void loop() {
  //turn off the TX and RX pins so the RX and TX pins of the SIM go low - saves power!
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);

  //get sensor of A5 to see if we need to do quick things
  FasterA = analogRead(A2)*1.0;

  //go and get temperature and EC
  memset(vel, '\0', 80);
	 Serial.println("VEL");
      velread();
	  for(int j = 0; j < 79; j++){
		  Serial.print(vel[j]);
	  }
	  Serial.println(vel[79]);

  //add these values to the averagers
  EC = EC + ECVar;
  temp = temp + TempVar;
  Depth = Depth + DepthVar;

  //increment counters
  counter = counter + ScanInterval;
  DailyCounter = DailyCounter + ScanInterval;

  
  
  
      //ok we should send to the web because we have some changing datasets.
      //turn on the TX pin so the RX of the SIM goes high
      digitalWrite(3,HIGH);
      lastECwritten = EC;
      lastTempwritten = temp;
      lastDepthwritten = Depth;
      SendToWeb222(0);
      Serial.print(F("Sent to web. "));
      //turn off the TX and RX pins so the RX and TX pins of the SIM go low - saves power!
      digitalWrite(3,LOW);
      DailyCounter = 0;

    //resent counters
    counter = 0;
    EC = 0;
    temp = 0;
    Depth = 0;
  
  Serial.println(F(", delaying."));
  //Sleepy::loseSomeTime(1000 * ScanInterval);
  //delay(1000 * ScanInterval);

}

void velread(){
	uint32_t tout = 0;
	char bytin = 0;
	uint8_t i = 0;
	

	pinMode(7, OUTPUT);
	digitalWrite(7, LOW);
	delay(10);
	digitalWrite(7, HIGH);
	pinMode(7, INPUT);

	velPort.begin(9600);
	velPort.listen();
	
	tout = millis(); 
	while(millis() - tout < 10000){
        if(velPort.available() != 0){ 
			bytin = velPort.read();
			if(bytin == 'R'){

				break;
			}
		}
	}	
	
	
	velPort.print('V');
	
	tout = millis();
	while((millis()- tout < 10000) and (i<80)){
		if(velPort.available() != 0){
           
		   bytin = velPort.read();
		   if(bytin == ' '){
			continue;
           }
		   vel[i] = bytin;
            // check if the desired answer is in the response of the module
            if (vel[i] == 'T')    
            {
                vel[i] = '\0';
				break;
            }
			i++;
        } 
		
	 }


	velPort.print('S');
	velPort.flush();
	velPort.end();
	Sim900.listen();
}


void SendToWeb222(int abc) {
  a = 0;

  //first check to see if the modem is still on - if it is, turn it off
  while (sendATcommand(F("AT"),ok,2000) == 1 && a < 5) {
   TurnOnOffSim();
   a++;
   delay(2000);
  } 
  
  //OK, it should now be off, now to turn it on
  a=0;
  while (sendATcommand(F("AT"),ok,2000) == 0 && a<5) {
   TurnOnOffSim();
   a++;
   delay(2000);
  }
 
  if(abc==1){
      //FIRST TIME RUN ONLY!!!
      a=0;
      do{a++;}while (sendATcommand(F("AT+IPR=19200"),ok,10000) == 0 && a<5);
      a=0;
      do{a++;}while (sendATcommand(F("ATE1"),ok,10000) == 0 && a<5);
      a=0;
      do{a++;}while (sendATcommand(F("AT&W0"),ok,10000) == 0 && a<5);
  }
  
  a=0;
  do{a++;}while (sendATcommand(F("AT+CGDCONT=1,\"IP\",\"mdata.net.au\",\"0.0.0.0\""), ok,10000) == 0 && a<5);
  a=0;
  do{a++;}while (sendATcommand(F("AT+CGSOCKCONT=1,\"IP\",\"mdata.net.au\""), ok,10000) == 0 && a<5);
  a=0;
  do{a++;}while (sendATcommand(F("AT+CSOCKSETPN=1"), ok,10000) == 0 && a<10);
  a=0;
  do{a++;}while (sendATcommand(F("AT+CHTTPSSTART"), ok,10000) == 0 && a<2);
  a=0;
  
  do{a++;}while (sendATcommand(F("AT+CHTTPSOPSE=\"www.bosl.com.au\",80,1"), ok,10000) == 0 && a<5);
//you may need to move the /EoDC/ to the below line bewfore the "WriteMe.php... so it looks like "EoDC/WriteMe.php.... but try the way i have it first
  AAA = F("GET /");
  AAA += F("IoT/testing/scripts/WriteMe.php?SiteName=VEL_SD_TEST.csv&T=");
  AAA += "B";
  AAA += "&V=";
  AAA += vel;
  AAA += F(" HTTP/1.1\r\nHost: www.bosl.com.au:80\r\n\r\n");
  BBB = F("AT+CHTTPSSEND=");
  BBB +=AAA.length();

  a=0;
  do{a++;}while (sendATcommand(BBB, ">",10000) == 0 && a<5);

  a=0;
  do{a++;}while (sendATcommand(AAA, ok,10000) == 0 && a<5);

  delay(1000);

  a=0;
  do{a++;}while (sendATcommand(F("AT+CHTTPSCLSE"), ok,5000) == 0 && a<1);

  a=0;
  do{a++;}while (sendATcommand(F("AT+CHTTPSSTOP"), ok,10000) == 0 && a<1);

//  a=0;
//  do{a++;}while (sendATcommand(F("AT+CPOF"), ok,10000) == 0 && a<1);

  //check to see if the modem is still on - if it is, turn it off
  while (sendATcommand(F("AT"),ok,2000) == 1 && a < 5) {
   TurnOnOffSim();
   a++;
   delay(2000);
  } 
   
}

void SendToWeb(int abc) {
  a = 0;
  if(abc==1){
        //first check to see if the modem is still on - if it is, turn it off

      while (sendATcommand(F("AT"),ok,2000) == 1 && a < 5) {
         TurnOnOffSim();
         a++;
         delay(2000);
      } 
       
      //OK, it should now be off, now to turn it on
      a=0;
      while (sendATcommand(F("AT"),ok,2000) == 0 && a<5) {
         TurnOnOffSim();
         a++;
         delay(2000);
      }
     
      //OK, the system should now be on. 
      a=0;
      do{a++;}while (sendATcommand(F("AT+IPR=19200"),ok,10000) == 0 && a<5);
      a=0;
      do{a++;}while (sendATcommand(F("ATE0"),ok,10000) == 0 && a<5);
      a=0;
      do{a++;}while (sendATcommand(F("AT&W0"),ok,10000) == 0 && a<5);
      a=0;
      do{a++;}while (sendATcommand(F("AT+CGDCONT=1,\"IP\",\"mdata.net.au\",\"0.0.0.0\""), ok,10000) == 0 && a<5);
      a=0;
      do{a++;}while (sendATcommand(F("AT+CGSOCKCONT=1,\"IP\",\"mdata.net.au\""), ok,10000) == 0 && a<5);
      a=0;
      do{a++;}while (sendATcommand(F("AT+CSOCKSETPN=1"), ok,10000) == 0 && a<10);
      a=0;
      do{a++;}while (sendATcommand(F("AT+CHTTPSSTART"), ok,10000) == 0 && a<2);
      a=0;
  }
  
  do{a++;}while (sendATcommand(F("AT+CHTTPSOPSE=\"www.bosl.com.au\",80,1"), ok,10000) == 0 && a<5);
//you may need to move the /EoDC/ to the below line bewfore the "WriteMe.php... so it looks like "EoDC/WriteMe.php.... but try the way i have it first
  AAA = F("GET /");
  AAA += F("IoT/testing/scripts/WriteMe.php?SiteName=VEL5K6.csv&T=");
  AAA += "B";
  AAA += "&V=";
  AAA += vel;
  AAA += F(" HTTP/1.1\r\nHost: www.bosl.com.au:80\r\n\r\n");
  BBB = F("AT+CHTTPSSEND=");
  BBB +=AAA.length();

  a=0;
  do{a++;}while (sendATcommand(BBB, ">",10000) == 0 && a<5);

  a=0;
  do{a++;}while (sendATcommand(AAA, ok,10000) == 0 && a<5);

  delay(1000);

  a=0;
  do{a++;}while (sendATcommand(F("AT+CHTTPSCLSE"), ok,5000) == 0 && a<3);

  //a=0;
  //do{a++;}while (sendATcommand(F("AT+CHTTPSSTOP"), ok,10000) == 0 && a<5);

  //a=0;
  //do{a++;}while (sendATcommand(F("AT+CPOF"), ok,10000) == 0 && a<5);
   
}

int8_t sendATcommand(String ATcommand, char* expected_answer1, unsigned int timeout){
    Serial.println(ATcommand);
    x=0;
    answer=0;
    
    previous = 0;

    memset(response, '\0', 196);    // Initialize the string

    delay(100);

    while( Sim900.available() > 0) Sim900.read();    // Clean the input buffer

    Sim900.println(ATcommand);    // Send the AT command 


        x = 0;
    previous = millis();

    // this loop waits for the answer
    do{
        if(Sim900.available() != 0){    
            response[x] = Sim900.read();
            x++;
            // check if the desired answer is in the response of the module
            if (strstr(response, expected_answer1) != NULL)    
            {
                answer = 1;
            }
        }
        // Waits for the asnwer with time out
    }
    while((answer == 0) && ((millis() - previous) < timeout));    
    Serial.println(answer);
    return answer;
}


void TurnOnOffSim() {
   digitalWrite(8,LOW);
   delay(1000);
   digitalWrite(8,HIGH);
   delay(1200);
   digitalWrite(8,LOW);
   delay(5000); 
}

