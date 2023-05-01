#include<ADS1115_WE.h>
#include<Wire.h>

#define I2C_ADDRESS 0x48

#define SAMPLE_DIV 10 //steps per sample

ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);

#define STEP_PIN 3
#define DIR_PIN 4
#define EN_PIN 2
#define ENDSW_PIN 5
#define STEPS_PER_REV 200
#define MS 16 //microsteps
#define REV STEPS_PER_REV * MS //Steps per 360 deg
#define LEAD_SCREW_PITCH 8 //in mm
#define DEZIMALS 6          //num of dezimal digits

boolean homed = false;
int zPos = 9999;


void disableStepper();
void simpleHoming(int nDelay);
void userMoveStepper();

void setup() {
  Serial.begin(9600);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(ENDSW_PIN, INPUT);

  digitalWrite(EN_PIN,HIGH);

  printSetupPrompt();

  adcInit();
}

void loop() {
  serialPrompt();
}

void homingRoutine() {
  Serial.println("Homing...");

  simpleHoming(200);
  moveStepper(1,REV / 2, 100); //1 = back
  simpleHoming(400);
  zPos = 0;
  moveStepper(1,REV + REV / 4, 100);

  homed = true; 
}

void serialPrompt() {
  Serial.print("Prompt");
  Serial.print(" z axis pos: ");
  Serial.println(zPos);
  Serial.print(zPosInMM(),5);
  Serial.println(" mm");
  
  int userInput = readUserInput();

  switch (userInput) {
    case 1:
      homingRoutine();
      break;
    case 2:
      calibrationRoutine();
      break;
    case 3:
      endSwitchStatus();
      break;
    case 4:
      testStepper();
      break;
    case 5:
      userMoveStepper();
      break;
    case 6:
      adcTest();
      break;
    case 7:
      errorCalibration();
      break;
    case 8:
      staticCalibration();
      break;
  }

}

int readUserInput() {
  int input;
  boolean validInput = false;

  while (!validInput) {
    Serial.println(" homing: 1 / calibration: 2 / Endswitch status: 3 / Stepper Test: 4/ move stepper: 5");
    Serial.println(" ADC test: 6 / Error cal.: 7 / static cal.: 8 /");

    input = readInput();

    if (input == 1 || input == 2 || input == 3 || input == 4 || input == 5 || input == 6 || input == 7 || input ==8) {
      validInput = true;
    } else {
      Serial.println("Invalid Entry try again!");
    }
  }
  return input;
}

void printSetupPrompt() {
  Serial.println("SetupPrompt");
  //TODO
}

void calibrationRoutine() {
  Serial.println("calibration Routine");
  Serial.println("HllSns_mV pos_in_mm line");

  int lneCnt = 0;
  
  if(homed){
    while(digitalRead(ENDSW_PIN)){
      moveStepper(0,SAMPLE_DIV,1);
      Serial.print(readChannel(),6);
      Serial.print(" ");
      Serial.print(zPosInMM(),6);
      Serial.print(" ");
      Serial.println(lneCnt);
      lneCnt++;
    }
  }else{
    Serial.println();
    Serial.println("Not homed yet!");
    Serial.println();
    return;
  }
  Serial.println();
  Serial.println("Calibration done");
  Serial.println();

  moveStepper(1,REV + REV / 4, 100);
}

int readInput() {
  //wait for input
  while (Serial.available() == 0) {};

  int input = Serial.parseInt(SKIP_NONE);

  while (!Serial.available() == 0) {
    String bin = Serial.readString();
  }

  return input;
}

void endSwitchStatus() {
  for (int i = 0; i < 20; i++) {
    Serial.println(digitalRead(ENDSW_PIN));
    delay(200);
  }
}

void testStepper(){
  digitalWrite(EN_PIN,LOW);
  delay(100);
  moveStepper(LOW ,REV , 50);

  delay(1000);

  moveStepper(HIGH ,REV , 200);
  
  digitalWrite(EN_PIN,HIGH);
}

void moveStepper(boolean dir, int nSteps, int nDelay){
  digitalWrite(EN_PIN, LOW); //enable driver
  digitalWrite(DIR_PIN, dir);  

  for(int i = 0; i < nSteps; i++){
    digitalWrite(STEP_PIN,HIGH);
    delayMicroseconds(nDelay);
    digitalWrite(STEP_PIN,LOW);
    delayMicroseconds(nDelay);

  }

  if(!dir){ //closing
    zPos -= nSteps;
  }else{
    zPos += nSteps;
  }

  if(zPos < 0){
    while(1){
      Serial.println("neg Pos please reset");
      delay(500);
      }
  }

  /*

  int cnt = 0;
  boolean suc = false;
  
  while(!suc){
    moveStepper(LOW,1,nDelay);

    if(!digitalRead(ENDSW_PIN)){
      delay(100);
      suc = true;
      break;
    }
  }
  */
}


void simpleHoming(int nDelay){
  int cnt = 0;
  boolean suc = false;
  
  while(!suc){
    moveStepper(LOW,1,nDelay);

    if(!digitalRead(ENDSW_PIN)){
      delay(100);
      suc = true;
      break;
    }

    if(cnt > 200 * 5 * 16){
      while(1){
      Serial.println("Error while homing please reset");  
      delay(1000);
      }
    }
    cnt++;
  }

  delay(10);
}

void disableStepper(){
  digitalWrite(EN_PIN, HIGH);
}

void userMoveStepper(){}

void adcTest(){
  Serial.println("Reading ADC");
  for(int i = 0; i < 20; i++){
    Serial.println(readChannel());
  }
}

void adcInit(){
  Wire.begin();

  if(!adc.init()){
    Serial.println("ADS1115 not connected!");
  }

  adc.setVoltageRange_mV(ADS1115_RANGE_4096);
  adc.setCompareChannels(ADS1115_COMP_0_GND);
  adc.setMeasureMode(ADS1115_CONTINOUS);
}

float readChannel(){
  float voltage = 0.0;
  adc.setCompareChannels(ADS1115_COMP_0_GND);
  voltage = adc.getResult_mV();
  return voltage;
}

double zPosInMM(){
  return  (double)LEAD_SCREW_PITCH / 3200.0 * zPos; //IDK REV isn't working here :/
}

double polynomApprox(double x){
  long double a = -2.809957114218647   * pow(10,-17);
  long double b =  3.020236658403055   * pow(10,-13);
  long double c = -1.2900143056821008  * pow(10,-9);
  long double d =  2.8338332872291313  * pow(10,-6);
  long double e = -0.003394828809185907* pow(10,0);
  long double f = 2.1118342751329493   * pow(10,0);
  long double g = -532.8529044996867   * pow(10,0);
  //g + f*x + e*x^2 + d*x^3 + c*x^4 + b*x^5 + a*x^6}},

  long double approx =
  a * pow(x, 6) +
  b * pow(x, 5) +
  c * pow(x, 4) +
  d * pow(x, 3) +
  e * pow(x, 2) +
  f * pow(x, 1) +
  g * pow(x, 0);

  return approx;
 
}

void errorCalibration(){
  Serial.println("Error calibration Routine");
  Serial.println("HllSns_mV pos_in_mm line approx error line");

  int lneCnt = 0;
  
  if(homed){
    while(digitalRead(ENDSW_PIN)){
      moveStepper(0,SAMPLE_DIV,1);
      Serial.print(readChannel(),DEZIMALS);
      Serial.print(" ");
      Serial.print(zPosInMM(),DEZIMALS);
      Serial.print(" ");
      Serial.print(polynomApprox(readChannel()));
      Serial.print(" ");
      Serial.println(lneCnt);
      lneCnt++;
    }
  }else{
    Serial.println();
    Serial.println("Not homed yet!"); 
    Serial.println();
    return;
  }
  Serial.println();
  Serial.println("Calibration done");
  Serial.println();

  moveStepper(1,REV + REV / 4, 100);
}

void staticCalibration(){

  int nStops = 5;
  int nSamples = 100;
  int prevZPos = zPos;
  
  Serial.println();
  Serial.println("Starting Static Calibration...");
  Serial.println();
  
  if(homed){
    for(int i =0; i < nStops; i++){
      moveStepper(0, prevZPos / nStops ,200);
      
      Serial.println();
      Serial.print("Holding Pos: ");
      Serial.print(zPosInMM());
      Serial.println(" Holding pos. sensor readout:");
      Serial.println();
      
      for(int j = 0; j < nSamples; j++){
        delay(2); //860Hz  1/860 = 0.0012ms wait for new data to arrive
        Serial.println(readChannel(),DEZIMALS); 
      }
    }
    
  }else{
    Serial.println();
    Serial.println("Not homed yet!"); 
    Serial.println();
    return;
  }

  moveStepper(1, REV * 2,200); //move motor back
  
  Serial.println();
  Serial.println("Static calibration Done");
  Serial.println();
}
