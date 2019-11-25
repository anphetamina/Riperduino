/*

   _____          __                .__           _________              __                       
  /  _  \   _____/  |_  ____   ____ |__| ____    /   _____/____    _____/  |_  ___________  ____  
 /  /_\  \ /    \   __\/  _ \ /    \|  |/  _ \   \_____  \\__  \  /    \   __\/  _ \_  __ \/  _ \ 
/    |    \   |  \  | (  <_> )   |  \  (  <_> )  /        \/ __ \|   |  \  | (  <_> )  | \(  <_> )
\____|__  /___|  /__|  \____/|___|  /__|\____/  /_______  (____  /___|  /__|  \____/|__|   \____/ 
        \/     \/                 \/                    \/     \/     \/                          

*/

// gestione retroilluminazione
// ram del display che glitcha
// funzione di deumidificazione

#include <LiquidCrystal.h>
#include <dht.h>

#define DHT22_PIN 2

#define buzzerPin 13

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

#define RS        8
#define E         9
#define D4        4
#define D5        5
#define D6        6
#define D7        7

#define motorPin  11 // motore frigo
#define pumpPin   12 // ventola umidità
#define fanPin    3 // ventola

// default
#define TEMP      10
#define HUMID     75
#define TIME      1.5 // minutes

#define MAX_TEMP  30
#define MIN_TEMP  0
#define MAX_HUM   95
#define MIN_HUM   0
#define MAX_PER   5 // minutes
#define MIN_PER   0

// INPUT VARIABLES
const int numOfInputs=5;
unsigned long lastDebounceTime[numOfInputs] = {0,0,0,0,0};
unsigned long debounceDelay = 50;
bool lastInputState[numOfInputs] = {LOW,LOW,LOW,LOW,LOW};
bool inputState[numOfInputs] = {LOW,LOW,LOW,LOW,LOW};
bool currentState[numOfInputs] = {LOW,LOW,LOW,LOW,LOW};
bool inputFlag[numOfInputs] = {LOW,LOW,LOW,LOW,LOW};
int adc_key_in = 0;
int reading = btnNONE;
dht DHT;

// LCD LOGIC
LiquidCrystal lcd(RS,E,D4,D5,D6,D7);
const int numOfScreens = 4;
int currentScreen = 0;
String screens[numOfScreens][2] = {{"", ""}, {"Temperatura", "C"}, {"Umidita'", "%"}, {"Min intervento", "min"}};
float parameters[numOfScreens] = {0,TEMP,HUMID,TIME};
float edges[numOfScreens][2] = {{0,0}, {MIN_TEMP,MAX_TEMP}, {MIN_HUM,MAX_HUM}, {MIN_PER,MAX_PER}};
unsigned long jumpToDefaultScreenDelay = 30*1000; // milliseconds
unsigned long jumpToDefaultScreenTime = 0;
bool canSpam = false;
unsigned long refreshDelay = 2*1000; // milliseconds
unsigned long refreshTime = 0;

byte degree[8] = {
  0b01100,
	0b10010,
	0b10010,
	0b01100,
	0b00000,
	0b00000,
	0b00000,
	0b00000
};

byte thermometer[8] = {
  0b00100,
  0b01010,
  0b01010,
  0b01110,
  0b01110,
  0b11111,
  0b11111,
  0b01110
};

byte dropper[8] = {
  0b00100,
  0b00100,
  0b01010,
  0b01010,
  0b10001,
  0b10001,
  0b10001,
  0b01110
};

byte F[8] = {
	0b00000,
	0b00000,
	0b00000,
	0b01110,
	0b01000,
	0b01110,
	0b01000,
	0b00000
};

byte A[8] = {
	0b00000,
	0b00000,
	0b00000,
	0b01110,
	0b01010,
	0b01110,
	0b01010,
	0b00000
};

byte N[8] = {
	0b00000,
	0b00000,
	0b00000,
	0b01001,
	0b01101,
	0b01011,
	0b01001,
	0b00000
};

byte O[8] = {
	0b00000,
	0b00000,
	0b00000,
	0b01110,
	0b01010,
	0b01010,
	0b01110,
	0b00000
};

// RELAYS LOGIC
float T = TEMP;
float H = HUMID;
float P = TIME; // minutes
float currentTemp;
float currentHum;
bool tempIsHigh;
bool waitingForTemp;
float tempThreshold = 2;
bool humIsLow;
bool waitingForHum;
float humThreshold = 3;
bool isTime = true;
unsigned long period = 0;
unsigned long lastReading = 0;
unsigned long pollingRate = 3*1000; // milliseconds

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  pinMode(fanPin,OUTPUT);
  pinMode(pumpPin,OUTPUT);
  pinMode(motorPin,OUTPUT);
  pinMode(buzzerPin,OUTPUT);
  lcd.createChar(0, degree);
  lcd.createChar(1, thermometer);
  lcd.createChar(2, dropper);
  lcd.createChar(3, F);
  lcd.createChar(4, A);
  lcd.createChar(5, N);
  lcd.createChar(6, O);
  readSensor();
  lcd.setCursor(2,0);
  lcd.print("v0.93-beta");
  delay(2500);
}

void loop() {
  takeMeasurements();
  manageRelays();
  getInput();
  printLCD();
}

void getInput(){
  reading = getButton();
  for(int i=0; i<numOfInputs; i++){
    if(reading == i) currentState[i] = HIGH;
    else currentState[i] = LOW;
    if(currentState[i] && !lastInputState[i] && millis()-lastDebounceTime[i] > debounceDelay){
      inputFlag[i] = HIGH;
      lastDebounceTime[i] = millis();
    }
    else if(currentState[i] && lastInputState[i] && millis()-lastDebounceTime[i] > debounceDelay+150 && canSpam){
      inputFlag[i] = HIGH;
      lastDebounceTime[i] = millis();
    }
    else inputFlag[i] = LOW;
    if(inputFlag[i]) getOutput(i);
    if(reading == i) lastInputState[i] = HIGH;
    else lastInputState[i] = LOW;
  }
}

int getButton(){
  adc_key_in = analogRead(0);

  if (adc_key_in > 1000) return btnNONE;
  if (adc_key_in < 50)   return btnRIGHT;
  if (adc_key_in < 250)  return btnUP;
  if (adc_key_in < 450)  return btnDOWN;
  if (adc_key_in < 650)  return btnLEFT;
  if (adc_key_in < 850)  return btnSELECT;

  return btnNONE;
}

void getOutput(int button){
  switch (button) {

    case btnLEFT:
      Serial.println("LEFT");
      lcd.clear();
      if(currentScreen == 0 || currentScreen == 1) currentScreen = numOfScreens-1;
      else currentScreen--;
      jumpToDefaultScreenTime = millis();
      canSpam = false;
      break;

    case btnRIGHT:
      Serial.println("RIGHT");
      lcd.clear();
      if(currentScreen == numOfScreens-1) currentScreen = 1;
      else currentScreen++;
      jumpToDefaultScreenTime = millis();
      canSpam = false;
      break;

    case btnUP:
      Serial.println("UP");
      if(parameters[currentScreen] < edges[currentScreen][1]) parameters[currentScreen] += 0.5;
      jumpToDefaultScreenTime = millis();
      canSpam = true;
      break;

    case btnDOWN:
      Serial.println("DOWN");
      if(edges[currentScreen][0] < parameters[currentScreen]) parameters[currentScreen] -= 0.5;
      jumpToDefaultScreenTime = millis();
      canSpam = true;
      break;

    case btnSELECT:
      Serial.println("SELECT");
      T = parameters[1];
      H = parameters[2];
      P = parameters[3];
      lcd.clear();
      currentScreen = 0;
      canSpam = false;
      break;

    default:
      break;
  }
}

void printLCD(){
  if(currentScreen == 0){
    lcd.setCursor(1,0);
    lcd.write((uint8_t)1);
    lcd.print(" ");
    lcd.print(currentTemp,0);
    lcd.write((uint8_t)0);
    lcd.print("C  ");
    lcd.write((uint8_t)2);
    lcd.print(" ");
    lcd.print(currentHum,0);
    lcd.print("%  ");
    lcd.setCursor(3,1);
    if(P != 0)  lcd.print(abs(((P*60*1000)+period-millis())/1000),1);
    printFan(9);
  }
  else{
    lcd.setCursor(0,0);
    lcd.print(screens[currentScreen][0]);
    lcd.setCursor(0,1);
    lcd.print(parameters[currentScreen]);
    lcd.print(" ");
    lcd.setCursor(6,1);
    lcd.print(screens[currentScreen][1]);
    if(millis() - jumpToDefaultScreenTime > jumpToDefaultScreenDelay){
      Serial.println("Jumping to default screen");
      lcd.clear();
      currentScreen = 0;
    }
  }
}

void printFan(int cursor){
  lcd.setCursor(cursor,1);
  lcd.write((uint8_t)3);
  lcd.write((uint8_t)4);
  lcd.write((uint8_t)5);
  lcd.print(" ");
  if(isTime){
    lcd.write((uint8_t)6);
    lcd.write((uint8_t)5);
    lcd.print(" ");
  }
  else{
    lcd.write((uint8_t)6);
    lcd.write((uint8_t)3);
    lcd.write((uint8_t)3);
  }
}

void takeMeasurements(){
  if(millis() - lastReading > pollingRate) readSensor();

  if(P == 0){
    isTime = true;
  }
  else if(millis() - period >= P*60*1000){
    isTime = !isTime;
    period = millis();
  }

  if(currentTemp > T && !waitingForTemp){
    tempIsHigh = true;
  }
  else if(currentTemp < T+tempThreshold){
    waitingForTemp = true;
  }
  else{
    tempIsHigh = false;
    waitingForTemp = false;
  }

  if(currentHum < H && !waitingForHum){
    humIsLow = true;
  }
  else if(currentHum > H-humThreshold){
    waitingForHum = true;
  }
  else{
    humIsLow = false;
    waitingForHum = false;
  }
}

void readSensor(){
  int chk = DHT.read22(DHT22_PIN);
  Serial.print(millis()/1000);
  Serial.print(". ");

    switch (chk)
    {
    case DHTLIB_OK:
        Serial.print("OK ~ ");
        currentTemp = DHT.temperature;
        currentHum = DHT.humidity;
        break;
    case DHTLIB_ERROR_CHECKSUM:
        Serial.print("Checksum error ~ ");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Checksum error");
        delay(1000);
        tone(buzzerPin,10000,1000);
        lcd.clear();
        break;
    case DHTLIB_ERROR_TIMEOUT:
        Serial.print("Time out error ~ ");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Time out error");
        delay(1000);
        tone(buzzerPin,10000,1000);
        lcd.clear();
        break;
    case DHTLIB_ERROR_CONNECT:
        Serial.print("Connect error ~ ");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Connect error");
        delay(1000);
        tone(buzzerPin,10000,1000);
        lcd.clear();
        break;
    case DHTLIB_ERROR_ACK_L:
        Serial.print("Ack Low error ~ ");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Ack Low error");
        delay(1000);
        tone(buzzerPin,10000,1000);
        lcd.clear();
        break;
    case DHTLIB_ERROR_ACK_H:
        Serial.print("Ack High error ~ ");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Ack High error");
        delay(1000);
        tone(buzzerPin,10000,1000);
        lcd.clear();
        break;
    default:
        Serial.print("Unknown error ~ ");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Unknown error");
        delay(1000);
        tone(buzzerPin,10000,1000);
        lcd.clear();
        break;
    }

  Serial.print("Temperature: ");
  Serial.print(currentTemp);
  Serial.print(" C ~ Humidity: ");
  Serial.print(currentHum);
  Serial.print(" %\n");
  lastReading = millis();
}

void manageRelays(){
  relay1();
  relay2();
  relay3();
  if(millis() - refreshTime > refreshDelay){
    lcd.begin(16,2);
    lcd.createChar(0, degree);
    lcd.createChar(1, thermometer);
    lcd.createChar(2, dropper);
    lcd.createChar(3, F);
    lcd.createChar(4, A);
    lcd.createChar(5, N);
    lcd.createChar(6, O);
    refreshTime = millis();
  }
}

void relay1(){
  if(tempIsHigh && !waitingForTemp) decreaseTemp();
  else increaseTemp();
}

void relay2(){
  if(humIsLow && !waitingForHum) increaseHum();
  else decreaseHum();
}

void relay3(){
  if(isTime) turnFanOn();
  else turnFanOff();
}

void increaseTemp(){
  digitalWrite(motorPin,LOW);
}

void decreaseTemp(){
  digitalWrite(motorPin,HIGH);
}

void increaseHum(){
  digitalWrite(pumpPin,HIGH);
}

void decreaseHum(){
  digitalWrite(pumpPin,LOW);
}

void turnFanOn(){
  digitalWrite(fanPin,HIGH);
}

void turnFanOff(){
  digitalWrite(fanPin,LOW);
}

// void getInput2(){
//   reading = getButton();
//   for(int i=0; i<numOfInputs; i++){
//     if(reading == i) currentState[i] = HIGH;
//     else currentState[i] = LOW;
//
//     if(currentState[i] != lastInputState[i]){
//       lastDebounceTime[i] = millis();
//     }
//
//     if((millis() - lastDebounceTime[i]) > debounceDelay){
//       if(currentState[i] != inputState[i]){
//         inputState[i] = lastInputState[i];
//         if(inputState[i] == HIGH){
//           getOutput(i);
//         }
//       }
//       if(currentState[i] && inputState[i] && (millis() - lastDebounceTime[i]) > debounceDelay+150 && canSpam){
//           getOutput(i);
//           lastDebounceTime[i] = millis();
//       }
//     }
//
//     if(reading == i) lastInputState[i] = HIGH;
//     else lastInputState[i] = LOW;
//   }
// }
