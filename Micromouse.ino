#include <Wire.h>
#include <cmath> //for abs()
#define spdArraySize 100
#define TIE 0x2
#define TEN 0x1
#define NORTH 1
#define EAST 2
#define SOUTH 4
#define WEST 8
#define STKSIZE 50
#define mSize 16
#define cellLength  180
#define oneCellDistance 21500
#define speed_to_counts(spd)  ((float)spd*oneCellDistance/cellLength/1000)   //mm to counts
#define counts_to_speed(counts)   ( ((float)counts) *cellLength/oneCellDistance*1000)  //counts to mm

//Data
const uint16_t NORTHMASK = 0b00000001;
const uint16_t EASTMASK = 0b00000010;
const uint16_t SOUTHMASK = 0b00000100;
const uint16_t WESTMASK = 0b00001000;
int minOpenNeighbor = mSize*mSize - 2;

typedef struct coord {
    uint8_t x;
    uint8_t y;
}crd;

struct stack {
   crd stk[STKSIZE];
   int top;
}st;

//Motors
int EN1 = 22;
int EN2 = 21;
int M1TermA = 5;
int M1TermB = 6;
int M2TermA = 7;
int M2TermB = 8;


//Encoders
int EAA = 0;
int EAB = 1;
int EBA = 14;
int EBB = 2;
int pulseA = 0;
int pulseB = 0;

//IR Sensors
int IRA = 26;
int IRB = 27;
int IRC = 28;
int IRD = 29;
int IRE = 30;
int SWI1 = 15;
int SWI2 = 16;
int SWI3 = 17;


//PID

float kpX = 0.3;
float kdX = 0.2;
float kpW = 0.03;
float kdW = 0.09;

int16_t DLSensor, DRSensor, leftSensor, rightSensor, frontSensor;
//int16_t trueF, trueL, trueR, trueDR, trueDL;
int16_t DLMiddleValue = 110;
int16_t DRMiddleValue = 91;
int16_t LMiddleValue = 405;
int16_t RMiddleValue = 293;
int16_t leftBaseSpeed = 60, rightBaseSpeed = 60;
int32_t leftCount = 0, rightCount = 0;
int32_t leftEncoderChange = 0, rightEncoderChange = 0;
int32_t leftCountOld = 0, rightCountOld = 0;
int32_t enc_lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
int16_t sensorError = 0;
float curSpeedX, targetSpeedX, curSpeedW, targetSpeedW, endSpeedX;
float accX = 500;//1m/s/s
float decX = 500;
float accW = 0;
float decW = 0;
int32_t decNeeded = 0;
float encoderFeedbackX, encoderFeedbackW;
int32_t encoderCount = 0, oldEncoderCount = 0, distanceLeft = 0;
bool  useIR = false, usePID = false, addErr = true;
/*
float errorP = 0, errorD = 0, oldErrorP = 0, totalError = 0;
float kp = 0.3;
float kd = 0.3;

int16_t DLSensor, DRSensor, leftSensor, rightSensor, frontSensor;
int16_t DLMiddleValue = 66; 
int16_t DRMiddleValue = 53;
int16_t leftBaseSpeed = 60, rightBaseSpeed = 60;
int32_t leftCount = 0, rightCount = 0;
int32_t leftEncoderChange = 0, rightEncoderChange = 0;
int32_t leftCountOld = 0, rightCountOld = 0;
int32_t enc_lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
*/
int16_t tick;
//Gyro
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t x,y,z;

//FF values
int maze[10][10];
//uint8_t walls[6][6] = {{14,10,10,9,12,9},{12,10,10,2,0,11},{5,13,12,9,5,13},{4,3,4,3,6,1},{4,8,0,10,9,7},{7,7,6,13,6,13}};
//uint8_t walls[6][6] = {{14,8,8,8,8,9},{12,0,0,0,0,1},{4,0,0,0,0,1},{4,0,0,0,0,1},{4,0,0,0,0,1},{6,2,2,2,2,3}};
uint8_t walls[10][10] = {{14,8,8,8,8,8,8,8,8,9},{12,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,1},{6,2,2,2,2,2,2,2,2,3}};

int stksize = STKSIZE;
int currentCellX, currentCellY;

//Info for Direction the mouse is pointing towards
int Direction = 0;

//Distance Travelled
double distance = 0;

//Keep track of Current State
int state = 0;
//state 0 = search mode (default)
//state 1 = speed run


//Instantiated Functions
void setLeftPwm(int16_t spd);
void setRightPwm(int16_t spd);
void PID();
void initialMaze();
void gyro();
bool stkempty();
crd  stkpop();
void stkpush(crd val);
void init_stk();
void ffConfigure();
void currentDirection();
void getWallValues();
void getIRsensorValue();
void faceNorth();
void faceEast();
void faceSouth();
void faceWest();
void spin(int16_t dir, int16_t angle);
void moveCell(int32_t dist, int32_t maxSpd, int32_t endSpd);
void printSensorValue();
void motorRamp();
void leftISR();
void rightISR();
void getEncoderSpeed();
void getSensorError();
void updateCurrentSpeed();
int needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd);
void blinkLEDOne(int rate);
void blinkLEDTwo(int rate);
int smallerOpenCells(int x, int y);
void clearStack();
//void sensorSamples();

void pit0_isr(void){ //PID ISR routing  
  if(useIR){getIRsensorValue();}
  if(usePID){PID();}
  PIT_TFLG0 = 1; //
}

void setup() {
  //Timer
  NVIC_SET_PRIORITY(IRQ_PORTA, 16);
  NVIC_SET_PRIORITY(IRQ_PORTB, 16);
  NVIC_SET_PRIORITY(IRQ_PORTC, 16);
  NVIC_SET_PRIORITY(IRQ_PORTD, 16);
  NVIC_SET_PRIORITY(IRQ_PORTE, 16);
  NVIC_SET_PRIORITY(IRQ_PIT_CH0, 64);
  SCB_SHPR3 = 0x30200000;//set systick priority to 48
  
  SIM_SCGC6 |= SIM_SCGC6_PIT;
  PIT_MCR = 0x00;
  NVIC_ENABLE_IRQ(IRQ_PIT_CH0); //Enable PIT ch0 ARM interrupt
  PIT_LDVAL0 = 48000; //Periodic Interval Timer Load Value
  PIT_TCTRL0 = TIE; //Periodic Interval Timer Timer Enable
  PIT_TCTRL0 |= TEN;
  PIT_TFLG0 |= 1;
  
  //Motors
  pinMode(EN1, OUTPUT);
  pinMode(M1TermA, OUTPUT);
  pinMode(M1TermB, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(M2TermA, OUTPUT);
  pinMode(M2TermB, OUTPUT);
  //IR Recievers
  pinMode(IRA, INPUT);
  pinMode(IRB, INPUT);
  pinMode(IRC, INPUT);
  pinMode(IRD, INPUT);
  pinMode(IRE, INPUT);
  //IR Emitters
  pinMode(SWI1, OUTPUT);
  pinMode(SWI2, OUTPUT);
  pinMode(SWI3, OUTPUT);
  //Gyro
//  Wire.begin();
//  Wire.beginTransmission(MPU_addr);
//  Wire.write(0x6B);  // PWR_MGMT_1 register
//  Wire.write(0);     // set to zero (wakes up the MPU-6050)
//  Wire.endTransmission(true);
  //Encoder
  pinMode(EAA, INPUT);
  pinMode(EAB, INPUT);
  pinMode(EBA, INPUT);
  pinMode(EBB, INPUT);
  attachInterrupt(digitalPinToInterrupt(EBA), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EBB), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EAA), rightISR, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(EAB), rightISR, CHANGE);
  //LEDs
  pinMode(13, OUTPUT);
  pinMode(20, OUTPUT);
  Serial.begin(9600);
  targetSpeedW = 0;
  targetSpeedX = 0;
}

void loop() {

  int16_t NWO, EWO, SWO, WWO; //Check if Walls are open
  int16_t PlusX, PlusY, MinusX, MinusY; //Check adj Cells
  bool caseN, caseE, caseS, caseW; //Different cases
  bool left = 0;
  bool right = 1;
  
  useIR = true;
  while (DLSensor < 300){
    digitalWrite(13, HIGH);
    delay(60);
    digitalWrite(13, LOW);
    delay(60);
  }
  useIR = false;
  digitalWrite(13, HIGH);
  delay(3000);
  digitalWrite(13, LOW);
  
  //getIRsensorValue(); //This is to get which state we should be in
  //The state determines if we speed run or search run
//  if (a >= 600 && c >= 600){
//    state = 1;
//    //Add feedback from LED to be certain you get this state
//  }
//  else {
//    state = 0;
//    //Add feedback from LED to be certain you are in this state
//  }
  state = 0;
  if (state == 0){
    usePID = true;
    useIR = true;
    delay(2);
    initialMaze();
    init_stk();
    delay(1);
    moveCell(26500, 500, 0);
    blinkLEDOne(50);
    //spin(1,360);
    currentCellX = 0;
    currentCellY = 1;
    while (maze[currentCellX][currentCellY] != 0){
   /* while(1){
      if (rightSensor < 123){
        useIR = false;
        spin(right, 90);
        delay(1);
        useIR = true;
        delay(10);
      }
      else if (leftSensor < 210){
        useIR = false;
        spin(left, 90);
        delay(1);
        useIR = true;
        delay(10);
      }
      else if (frontSensor > 50){
        useIR = false;
        spin(left, 90);
        delay(1);
        spin(left, 90);
        useIR = true;
        delay(10);
      }
      delay(200);
      moveCell(oneCellDistance, 500, 0);
      */
      
      getWallValues();
      NWO = walls[currentCellX][currentCellY] & NORTHMASK;
      EWO = walls[currentCellX][currentCellY] & EASTMASK;
      SWO = walls[currentCellX][currentCellY] & SOUTHMASK;
      WWO = walls[currentCellX][currentCellY] & WESTMASK;

      PlusY = currentCellY + 1; //Adj North
      PlusX = currentCellX + 1; //Adj East
      MinusY = currentCellY - 1; //Adj South
      MinusX = currentCellX - 1; //Adj West
      
      caseN = (NWO == 0x00) && (maze[currentCellX][currentCellY] == (maze[currentCellX][PlusY] + 1)) && (PlusY <= 9);
      caseE = (EWO == 0x00) && (maze[currentCellX][currentCellY] == (maze[PlusX][currentCellY] + 1)) && (PlusX <= 9);
      caseS = (SWO == 0x00) && (maze[currentCellX][currentCellY] == (maze[currentCellX][MinusY] + 1)) && (MinusY >= 0);
      caseW = (WWO == 0x00) && (maze[currentCellX][currentCellY] == (maze[MinusX][currentCellY] + 1)) && (MinusX >= 0);
      
      if (caseN){
      //means NORTH wall is open && less than 1
          useIR = false;
          faceNorth();
          delay(1);
          useIR = true;
          delay(10);
          //setLeftPwm(0);
          //setRightPwm(0);
          delay(1);
          moveCell(oneCellDistance, 500, 0);
          delay(200);
          currentCellY++;
          
      }   
      else if (caseE){
      //means EAST wall is open && less than 1
          useIR = false;
          faceEast();
          delay(1);
          useIR = true;
          delay(2);
          //setLeftPwm(0);
          //setRightPwm(0);
          delay(1);
          moveCell(oneCellDistance, 500, 0);
          delay(200);
          currentCellX++;
       }
       else if (caseS){
       //means SOUTH wall is open && less than 1
          useIR = false;
          faceSouth();
          delay(1);
          useIR = true;
          delay(2);
          //setLeftPwm(0);
          //setRightPwm(0);
          delay(1);
          moveCell(oneCellDistance, 500, 0);
          delay(200);
          currentCellY--;
       }
       else if (caseW){
       //means WEST wall is open && less than 1
          useIR = false;
          faceWest();
          delay(1);
          useIR = true;
          delay(2);
          //setLeftPwm(0);
          //setRightPwm(0);
          delay(1);
          moveCell(oneCellDistance, 500, 0);
          delay(200);
          currentCellX--;
       }
       else {
           ffConfigure();
       }
       
    }
    while(state != 1)
    {
      useIR = true;
      delay(10);
        if (rightSensor > 700){
          state++;
          digitalWrite(13, LOW);
          delay(3000);
        }
    }
  /*getIRsensorValue();
  while (frontSensor < 85){
    moveCell(5000,500, 500);
    getIRsensorValue();
  }*/
  }
}
/*
void sensorSample(){
  int i;
  trueR = 0;
  trueL = 0;
  trueDR = 0;
  trueDL = 0;
  getIRsensorValue();
  for (i = 0; i < 3; i++){
    if(trueR <= rightSensor){
      trueR = rightSensor;
    }
  getIRsensorValue();
  }
  for (i = 0; i < 3; i++){
    if(trueL <= leftSensor){
      trueL = leftSensor;
    }
  getIRsensorValue();
  }
  for (i = 0; i < 3; i++){
    if(trueF <= frontSensor){
      trueF = frontSensor;
    }
  getIRsensorValue();
  }
  for (i = 0; i < 3; i++){
    if(trueDR <= DRSensor){
      trueDR = DRSensor;
    }
  getIRsensorValue();
  }
  for (i = 0; i < 3; i++){
    if(trueDL <= DLSensor){
      trueDL = DLSensor;
    }
  getIRsensorValue();
  }
}
*/
void blinkLEDOne (int rate){
  digitalWrite(13, HIGH);
  delay(rate);
  digitalWrite(13, LOW);
  delay(rate);
}

void blinkLEDTwo (int rate){
  digitalWrite(20, HIGH);
  delay(rate);
  digitalWrite(20, LOW);
  delay(rate);
}

void faceNorth(){
/*  if (Direction == 0){
    spin(1, 360);
  }*/
  bool r = 1;
  bool l = 0;
  addErr = false;
  if (Direction == 1){
    //blinkLEDOne(1000);
    spin(l, 90);
  }
  else if (Direction == 2){
    //blinkLEDOne(1000);
    spin(l, 90);
    delay(100);
    spin(l, 90);
  }
  else if (Direction == 3){
    //blinkLEDOne(1000);
    spin(r, 90);
  }
  Direction = 0;
  addErr = true;
}

void faceEast(){
  bool r = 1;
  bool l = 0;
  addErr = false;
  if (Direction == 0){
    //blinkLEDOne(1000);
    spin(r, 90);
  }
  /*else if (Direction == 1){
    spin(-1, 360);
  }*/
  else if (Direction == 2){
    //blinkLEDOne(1000);
    spin(l, 90);
  }
  else if (Direction == 3){
    //blinkLEDOne(1000);
    spin(l,90);
    delay(100);
    spin(l,90);
  }
  Direction = 1;
  addErr = true;
}

void faceSouth(){
  bool r = 1;
  bool l = 0;
  addErr = false;
  if (Direction == 0){
    //blinkLEDOne(1000);
    spin(l, 90);
    delay(100);
    spin(l, 90);
  }
  else if (Direction == 1){
    //blinkLEDOne(1000);
    spin(r, 90);
  }
  /*else if (Direction == 2){
    spin(-1, 360);
  }*/
  else if (Direction == 3){
    //blinkLEDOne(1000);
    spin(l, 90);
  }
  Direction = 2;
  addErr = true;
}

void faceWest(){
  bool r = 1;
  bool l = 0;
  addErr = false;
  if (Direction == 0){
    //blinkLEDOne(1000);
    spin(r, 90);
  }
  else if (Direction == 1){
    //blinkLEDOne(1000);
    spin(l, 90);
    delay(100);
    spin(l, 90);
  }
  else if (Direction == 2){
    //blinkLEDOne(1000);
    spin(l, 90);
  }
  /*else if (Direction == 3){
    spin(1, 360);
  }*/
  Direction = 3;
  addErr = true;
}

void getWallValues(){
  useIR = true;
  delay(10);
  int front = 10;
  int left = 145;
  int right = 100;
  if (Direction == 0){
    if (frontSensor >= front){
      walls[currentCellX][currentCellY] = (NORTH | walls[currentCellX][currentCellY]);
    }
    if (rightSensor >= right){
      walls[currentCellX][currentCellY] = (EAST | walls[currentCellX][currentCellY]);
    }
    if (leftSensor >= left){
      walls[currentCellX][currentCellY] = (WEST | walls[currentCellX][currentCellY]);
    }
  }
  else if (Direction == 1){
    if (frontSensor >= front){
      walls[currentCellX][currentCellY] = (EAST | walls[currentCellX][currentCellY]);
    }
    if (rightSensor >= right){
      walls[currentCellX][currentCellY] = (SOUTH | walls[currentCellX][currentCellY]);
    }
    if (leftSensor >= left){
      walls[currentCellX][currentCellY] = (NORTH | walls[currentCellX][currentCellY]);
    }
  }
  else if (Direction == 2){
    if (frontSensor >= front){
      walls[currentCellX][currentCellY] = (SOUTH | walls[currentCellX][currentCellY]);      
    }
    if (rightSensor >= right){
      walls[currentCellX][currentCellY] = (WEST | walls[currentCellX][currentCellY]);
    }
    if (leftSensor >= left){
      walls[currentCellX][currentCellY] = (EAST | walls[currentCellX][currentCellY]);
    }
  }
  else if (Direction == 3){
    if (frontSensor >= front){
      walls[currentCellX][currentCellY] = (WEST | walls[currentCellX][currentCellY]);
    }
    if (rightSensor >= right){
      walls[currentCellX][currentCellY] = (NORTH | walls[currentCellX][currentCellY]);
    }
    if (leftSensor >= left){
      walls[currentCellX][currentCellY] = (SOUTH | walls[currentCellX][currentCellY]);
    }
  }
}

void leftISR(){ //left encoder interrupt service routing
  static int32_t val = 0;
  val = val << 2;
  val = val | ( digitalReadFast(EBA) << 1 |  digitalReadFast(EBB) );
  leftCount += enc_lookup_table[val & 0b1111];
}

void rightISR(){  //right encoder interrupt service routing
  static int32_t val = 0;
  val = val << 2;
  val = val | ( digitalReadFast(EAA) << 1 |  digitalReadFast(EAB) );
  rightCount += enc_lookup_table[val & 0b1111];
}

void getIRsensorValue(){
  leftSensor = analogRead(IRB);//read ambient offset
  rightSensor = analogRead(IRA);  
  digitalWrite(SWI3, HIGH);//turn on emitter
  delayMicroseconds(80);//wait for receiver to charge
  leftSensor = analogRead(IRB) - leftSensor;//read receiver then subtract with the ambient offset
  rightSensor = analogRead(IRA) - rightSensor;
  digitalWrite(SWI3, LOW);//turn off emitter
  delayMicroseconds(80);//wait until other receiver fully discharged

  DLSensor = analogRead(IRC);
  DRSensor = analogRead(IRE);  
  digitalWrite(SWI1, HIGH);
  delayMicroseconds(80);
  DLSensor = analogRead(IRC) - DLSensor;
  DRSensor = analogRead(IRE) - DRSensor;
  digitalWrite(SWI1, LOW);
  delayMicroseconds(80);
  
  frontSensor = analogRead(IRD);
  digitalWrite(SWI2, HIGH);
  delayMicroseconds(80);
  frontSensor = analogRead(IRD) - frontSensor;
  digitalWrite(SWI2, LOW);

  //sensor value error conditioning
  if (leftSensor < 0){
    leftSensor = 0;
  }
  if (rightSensor < 0){
    rightSensor = 0;
  }
  if (DLSensor < 0){
    DLSensor = 0;
  }
  if (DRSensor < 0){
    DRSensor = 0;
  }
  if (frontSensor < 0){
    frontSensor = 0;
  }
}

void moveCell(int32_t dist, int32_t maxSpd, int32_t endSpd){
  distanceLeft = dist;
  oldEncoderCount = encoderCount;
  targetSpeedX = speed_to_counts(maxSpd*2);
  endSpeedX = speed_to_counts(endSpd*2);
  do
  {
     needToDecelerate(dist,curSpeedX, endSpeedX);
//    /*you can call int needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd) 
//    here with current speed and distanceLeft to decide if you should start to decelerate or not.*/
    if (decNeeded > decX)
       targetSpeedX = endSpeedX;  
     delay(1);//needed, stupid compiler
  }
  while( (encoderCount-oldEncoderCount) < dist);
  targetSpeedX = 0;

  oldEncoderCount = encoderCount; //update here for next movement to minimized the counts loss between cells.
}

void motorRamp(){
    static int spd = 0;
    for (spd = 0; spd < 256; spd++ ){
      setLeftPwm(spd);
      setRightPwm(spd);
      delay(10);
    }
    for (spd = 255; spd > -256; spd-- ){
      setLeftPwm(spd);
      setRightPwm(spd);
      delay(10);
    } 
      
    for (spd = -255; spd < 0; spd++ ){
      setLeftPwm(spd);
      setRightPwm(spd);
      delay(10);
    }  
}

int needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd){//speed are in encoder counts/ms, dist is in encoder counts 
  if (curSpd < 0){
    curSpd = -curSpd;
  }
  if (endSpd < 0){
    endSpd = -endSpd;
  }
  if (dist < 0){
    dist = 1;//-dist;
  }
  if (dist == 0){
    dist = 1;  //prevent divide by 0
  }
return (abs(((curSpd*curSpd - endSpd*endSpd)*100/((float)dist)/8)*cellLength*1000/oneCellDistance ) );
  //calculate deceleration rate needed with input distance, input current speed and input targetspeed to determind if the deceleration is needed
  //use equation 2*a*S = Vt^2 - V0^2  ==>  a = (Vt^2-V0^2)/2/S
  //because the speed is the sum of left and right wheels(which means it's doubled), that's why there is a "/4" in equation since the square of 2 is 4
}

void updateCurrentSpeed(){
  if (curSpeedX < targetSpeedX){
    curSpeedX += (float)(speed_to_counts(accX*2)/100); //speed is twice so add twice of acceleration
    if(curSpeedX > targetSpeedX){
      curSpeedX = targetSpeedX;
    }
  }
  else if (curSpeedX > targetSpeedX){
    curSpeedX -= (float)speed_to_counts(decX*2)/100;
    if(curSpeedX < targetSpeedX){
      curSpeedX = targetSpeedX;
    }
  }
  if (curSpeedW < targetSpeedW){
    curSpeedW += accW;
    if(curSpeedW > targetSpeedW){
      curSpeedW = targetSpeedW;
    }
  }
  else if (curSpeedW > targetSpeedW){
    curSpeedW -= decW;
    if(curSpeedW < targetSpeedW){
      curSpeedW = targetSpeedW;
    }
  } 
}

void getSensorError(){ 
  if(useIR){
      if(DLSensor > DLMiddleValue && DRSensor < DRMiddleValue){//case 1
        sensorError = DLSensor - DLMiddleValue;
      }
      else if(DRSensor > DRMiddleValue && DLSensor < DLMiddleValue){//case 2
        sensorError = DRMiddleValue - DRSensor;
      }
      else{ //case 3 do nothing, let encoder to align the mouse
        sensorError = 0;  
      }
  }
  else{
    sensorError = 0;
  }
  if (addErr = false){
    sensorError = 0; 
  }
}

void setLeftPwm(int16_t spd){
  if (spd > 255){
    spd = 255;
  }
  if (spd < -255){
    spd = -255;
  }
  if (spd >= 0){
    analogWrite(EN2, spd); 
    digitalWrite(M2TermA, LOW);
    digitalWrite(M2TermB, HIGH);   
  }
  else {
    analogWrite(EN2, -spd); 
    digitalWrite(M2TermA, HIGH);
    digitalWrite(M2TermB, LOW);     
  }
} 

void setRightPwm(int16_t spd){
  if (spd > 255){
    spd = 255;
  }
  if (spd < -255){
    spd = -255;
  }
  if (spd >= 0){
    analogWrite(EN1, spd); 
    digitalWrite(M1TermA, LOW);
    digitalWrite(M1TermB, HIGH);   
   }
  else {
    analogWrite(EN1, -spd); 
    digitalWrite(M1TermA, HIGH);
    digitalWrite(M1TermB, LOW);     
   }
}

void printSensorValue(){
  Serial.print("front=");Serial.print(frontSensor);Serial.print("  ");  
  Serial.print("left=");Serial.print(leftSensor);Serial.print("  ");
  Serial.print("right=");Serial.print(rightSensor);Serial.print("  ");
  Serial.print("DL=");Serial.print(DLSensor);Serial.print("  ");
  Serial.print("DR=");Serial.println(DRSensor);Serial.print("  ");
  delay(20);
} 

void getEncoderSpeed(){
  static int32_t pos = 0;//position for moving average array

  leftEncoderChange = leftCount - leftCountOld;
  rightEncoderChange = rightCount - rightCountOld;
  leftCountOld = leftCount;
  rightCount = rightCountOld;

  encoderFeedbackX = (leftEncoderChange + rightEncoderChange)*100;//translation feedback is twice of the speed, in order to maximized the resolution
  encoderFeedbackW = (rightEncoderChange - leftEncoderChange)*100;

  encoderCount += encoderFeedbackX/2;
  distanceLeft -= encoderFeedbackX/2;
//calculate deceleration rate needed in order to make decision if it is the time to decelerate.
//this variable will be used by other funtion who run in the main routing
  decNeeded = needToDecelerate(distanceLeft, curSpeedX, endSpeedX);
}

void PID(){
  static float posErrorX = 0, posErrorW = 0, posPwmX, posPwmW, oldPosErrorX = 0, oldPosErrorW = 0;  
  static float rotationalFeedback = 0;
  
  updateCurrentSpeed();
  getEncoderSpeed();
  getSensorError();

  rotationalFeedback = encoderFeedbackW + sensorError/3;

  posErrorX += (curSpeedX - encoderFeedbackX);//get position error, similar to errorP
  posErrorW += targetSpeedW - rotationalFeedback;//equivalent to errorD

  posPwmX = kpX * posErrorX + kdX * (posErrorX - oldPosErrorX);  //PD calculation
  posPwmW = kpW * posErrorW + kdW * (posErrorW - oldPosErrorW);
  
  oldPosErrorX = posErrorX;
  oldPosErrorW = posErrorW;

  setLeftPwm(posPwmX - posPwmW);
  setRightPwm(posPwmX + posPwmW);
}

void spin(bool face, int16_t angle){
  int16_t endCounts = angle*95/90;
  int16_t dir;
  int32_t leftInitialCount, rightInitialCount;
  if (face == 1){
    dir = -1;
  }
  else if (face == 0){
    dir = 1;
  }
  
  leftInitialCount = leftCount;
  rightInitialCount = rightCount;
  targetSpeedX = 0;
  targetSpeedW = 0;
  do{
    targetSpeedW = 80 * dir;
    delay(1);
  }
  while( ( abs(leftCount - leftInitialCount) + abs(rightCount - rightInitialCount) ) < endCounts  );
  targetSpeedW = 0;
  oldEncoderCount = encoderCount;
}

int smallerOpenCells(int x, int y){
  int16_t north, south, east, west, counter;
  int Nopen, Eopen, Sopen, Wopen, i, j, leastVal;
  int leastNESW[4] = {255, 255, 255, 255};
  bool caseN, caseE, caseS, caseW, out;
  counter = 0;
  north = y + 1;
  east = x + 1;
  south = y - 1;
  west = x - 1;

  Nopen = walls[x][y] & NORTHMASK; //North Wall Mask Check
  Eopen = walls[x][y] & EASTMASK;   //East Wall Mask Check
  Sopen = walls[x][y] & SOUTHMASK; //South Wall Mask Check
  Wopen = walls[x][y] & WESTMASK;   //West Wall Mask Check

  caseN = (north <= 15) && (Nopen == 0) && ((maze[x][y] - 1) == maze[x][north]);
  caseE = (east <= 15) && (Eopen == 0) && ((maze[x][y] - 1) == maze[east][y]);
  caseS = (south >= 0) && (Sopen == 0) && ((maze[x][y] - 1) == maze[x][south]);
  caseW = (west >= 0) && (Wopen == 0) && ((maze[x][y] - 1) == maze[west][y]);

  if (caseN == 1){
    counter++;
  }
  else if ((north <= 9) && (Nopen == 0)){
        leastNESW[0] = maze[x][north];
  }
  if (caseE == 1){
    counter++;
  }
    else if ((east <= 9) && (Eopen == 0)){
        leastNESW[1] = maze[east][y] ;
  }
  if (caseS == 1){
    counter++;
  }
  else if ((south >= 0) && (Sopen == 0)){
        leastNESW[2] = maze[x][south];
  }
  if (caseW == 1){
    counter++;
  }
  else if ((west >= 0) && (Wopen == 0)){
        leastNESW[3] = maze[west][y];
  }

  if (counter != 0){
    out = 1;
  }

  else {
        leastVal = leastNESW[0];
        for (i = 0; i < 3; i++){
            if(leastVal >= leastNESW[i+1]){
                leastVal = leastNESW[i+1];
            }
        }
        minOpenNeighbor = leastVal;
    out = 0;
  }
return out;
}
  
void ffConfigure(){
    int anyopen;
    if (maze[currentCellX][currentCellY] == 0){
        //printf("Find more cells");
        while(1){
          digitalWrite(13, HIGH);
          delay(500);
          digitalWrite(13, LOW);
          delay(500);
        }
       
    }
    else {
        //Get wall values functions, for now we'll use a dummy global holding values
        digitalWrite(13,HIGH);
        int cellCoordXMM;
        int cellCoordYMM;
        int cellCoordXPP;
        int cellCoordYPP;
        bool caseNW, caseSW, caseEW, caseWW;
        int left = currentCellX -  1;
        int right = currentCellX + 1;
        int up = currentCellY + 1;
        int down = currentCellY - 1;
        crd curr, adjN, adjE, adjS, adjW, cell_coord;
        crd openAdjN; //Open Adjacent Cell North
        crd openAdjE; //Open Adjacent Cell East
        crd openAdjS; //Open Adjacent Cell South
        crd openAdjW; //Open Adjacent Cell West
        curr.x = currentCellX;
        curr.y = currentCellY;
        adjN.x = currentCellX;
        adjN.y = up;
        adjE.x = right;
        adjE.y = currentCellY;
        adjS.x = currentCellX;
        adjS.y = down;
        adjW.x = left;
        adjW.y = currentCellY;


        stkpush(curr);
        /*
        if (adjN.y <= 5){stkpush(adjN);}
        if (adjE.x <= 5){stkpush(adjE);}
        if (adjW.x >= 0){stkpush(adjW);}
        if (adjS.y >= 0){stkpush(adjS);}
        */

        //getWallsValue();
        while (!stkempty()){
            cell_coord = stkpop();
            int NWopen = walls[cell_coord.x][cell_coord.y] & NORTHMASK; //North Wall Mask Check
            int EWopen = walls[cell_coord.x][cell_coord.y] & EASTMASK;   //East Wall Mask Check
            int SWopen = walls[cell_coord.x][cell_coord.y] & SOUTHMASK; //South Wall Mask Check
            int WWopen = walls[cell_coord.x][cell_coord.y] & WESTMASK;  //West Wall Mask Check

            cellCoordYPP = cell_coord.y + 1;
            cellCoordXPP = cell_coord.x + 1;
            cellCoordYMM = cell_coord.y - 1;
            cellCoordXMM = cell_coord.x - 1;

            caseNW =  (cellCoordYPP <= 9) && (NWopen == 0) && (maze[cell_coord.x][cell_coord.y] != 0) && ((maze[cell_coord.x][cell_coord.y] - 1) != maze[cell_coord.x][cellCoordYPP]);
            caseEW =  (cellCoordXPP <= 9) && (EWopen == 0) && (maze[cell_coord.x][cell_coord.y] != 0) && ((maze[cell_coord.x][cell_coord.y] - 1) != maze[cellCoordXPP][cell_coord.y]);
            caseSW =  (cellCoordYMM >= 0) && (SWopen == 0) && (maze[cell_coord.x][cell_coord.y] != 0) && ((maze[cell_coord.x][cell_coord.y] - 1) != maze[cell_coord.x][cellCoordYMM]);
            caseWW =  (cellCoordXMM >= 0) && (WWopen == 0) && (maze[cell_coord.x][cell_coord.y] != 0) && ((maze[cell_coord.x][cell_coord.y] - 1) != maze[cellCoordXMM][cell_coord.y]);

            anyopen = smallerOpenCells(cell_coord.x, cell_coord.y);
            if (anyopen == 0){
                maze[cell_coord.x][cell_coord.y] = minOpenNeighbor + 1;
                    //printf("%d\n",(SWopen == 0));
               if (caseNW){
                //means NORTH wall is open && not less than 1
                  //distanceVal[cell_coord.x][cell_coord.y] = distanceVal[cell_coord.x][cellCoordYPP] + 1;
                  openAdjN.x = cell_coord.x;
                  openAdjN.y = cellCoordYPP;
                  stkpush(openAdjN);
              }
              if (caseEW){
                  //means EAST wall is open && not less than 1
                  //distanceVal[cell_coord.x][cell_coord.y] = distanceVal[cellCoordXPP][cell_coord.y] + 1;
                  openAdjE.x = cellCoordXPP;
                  openAdjE.y = cell_coord.y;
                  stkpush(openAdjE);
              }
              if (caseSW){
                  //means SOUTH wall is open && not less than 1
                  //distanceVal[cell_coord.x][cell_coord.y] = distanceVal[cell_coord.x][cellCoordYMM] + 1;
                  openAdjS.x = cell_coord.x;
                  openAdjS.y = cellCoordYMM;
                  stkpush(openAdjS);
              }
              if (caseWW){
                  //means WEST wall is open && not less than 1
                  //distanceVal[cell_coord.x][cell_coord.y] = distanceVal[cellCoordXMM][cell_coord.y] + 1;
                  openAdjW.x = cellCoordXMM;
                  openAdjW.y = cell_coord.y;
                  stkpush(openAdjW);
              }
            }
        }
        clearStack();
    }
    digitalWrite(13, LOW);
}


void init_stk() {
   st.top = -1;
}

void clearStack(){
   st.top = -1;
}

void stkpush(crd val) {
   st.top++;
   st.stk[st.top] = val;
}

crd stkpop() {
   crd val;
   val = st.stk[st.top];
   st.top--;
   return val;
}

bool stkempty() {
   if (st.top == -1)
      return 1;
   else
      return 0;
}


void initialMaze(){
  maze[0][0] = 16;
  maze[0][1] = 15;
  maze[0][2] = 14;
  maze[0][3] = 13;
  maze[0][4] = 12;
  maze[0][5] = 11;
  maze[0][6] = 10;
  maze[0][7] = 9;
  maze[0][8] = 8;
  maze[0][9] = 8;

  maze[1][0] = 15;
  maze[1][1] = 14;
  maze[1][2] = 13;
  maze[1][3] = 12;
  maze[1][4] = 11;
  maze[1][5] = 10;
  maze[1][6] = 9;
  maze[1][7] = 8;
  maze[1][8] = 7;
  maze[1][9] = 7;

  maze[2][0] = 14;
  maze[2][1] = 13;
  maze[2][2] = 12;
  maze[2][3] = 11;
  maze[2][4] = 10;
  maze[2][5] = 9;
  maze[2][6] = 8;
  maze[2][7] = 7;
  maze[2][8] = 6;
  maze[2][9] = 5;

  maze[3][0] = 13;
  maze[3][1] = 12;
  maze[3][2] = 11;
  maze[3][3] = 10;
  maze[3][4] = 9;
  maze[3][5] = 8;
  maze[3][6] = 7;
  maze[3][7] = 6;
  maze[3][8] = 5;
  maze[3][9] = 5;

  maze[4][0] = 12;
  maze[4][1] = 11;
  maze[4][2] = 10;
  maze[4][3] = 9;
  maze[4][4] = 8;
  maze[4][5] = 7;
  maze[4][6] = 6;
  maze[4][7] = 5;
  maze[4][8] = 4;
  maze[4][9] = 4;

  maze[5][0] = 11;
  maze[5][1] = 10;
  maze[5][2] = 9;
  maze[5][3] = 8;
  maze[5][4] = 7;
  maze[5][5] = 6;
  maze[5][6] = 5;
  maze[5][7] = 4;
  maze[5][8] = 3;
  maze[5][9] = 3;

  maze[6][0] = 10;
  maze[6][1] = 9;
  maze[6][2] = 8;
  maze[6][3] = 7;
  maze[6][4] = 6;
  maze[6][5] = 5;
  maze[6][6] = 4;
  maze[6][7] = 3;
  maze[6][8] = 2;
  maze[6][9] = 2;

  maze[7][0] = 9;
  maze[7][1] = 8;
  maze[7][2] = 7;
  maze[7][3] = 6;
  maze[7][4] = 5;
  maze[7][5] = 4;
  maze[7][6] = 3;
  maze[7][7] = 2;
  maze[7][8] = 1;
  maze[7][9] = 1;

  maze[8][0] = 8;
  maze[8][1] = 7;
  maze[8][2] = 6;
  maze[8][3] = 5;
  maze[8][4] = 4;
  maze[8][5] = 3;
  maze[8][6] = 2;
  maze[8][7] = 1;
  maze[8][8] = 0;
  maze[8][9] = 0;

  maze[9][0] = 8;
  maze[9][1] = 7;
  maze[9][2] = 6;
  maze[9][3] = 5;
  maze[9][4] = 4;
  maze[9][5] = 3;
  maze[9][6] = 2;
  maze[9][7] = 1;
  maze[9][8] = 0;
  maze[9][9] = 0;
  
}


