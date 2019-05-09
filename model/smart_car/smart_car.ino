#include <Servo.h>
#include <Wire.h>
#include <IRremote.h>
#include <math.h>


/****************************************
 * Bluetooth
 * serial communication protocal
 ****************************************/
#define goFront         0x01 //turn forward
#define goBack          0x02 //turn backwrad
#define goLeft          0x03 //turn left
#define goRight         0x04 //turn right
#define goStop          0x05 //turn stop
#define goRightOriginal 0x06 //turn right in situ
#define goLeftOriginal  0x07 //turn left in situ
#define trailingAuto    0x08 //to trace
#define ultrasonicAuto  0x09 //avoidance by ultrasonic
#define infraredAuto    0x0A //avoidance by infrared
#define servoRight      0x0B //servo turn right(20)
#define servoLeft       0x0C //servo turn left(20)
#define powerFind       0x0D //find power
#define runCircle       0x0E //define the route
#define carryToOld      0x0F //carry the things
#define messageBlue     0x10 //send message by bluetooth
#define getThingOk      0x11 //get things successful

/****************************************
 * HMC5883L
 ****************************************/
#define address 0x1E              //001 1110b(0x3C>>1), HMC5883's address(7 bits)  
#define MagnetcDeclination -5.21  //geomagnetic declination  
#define CalThreshold 0

int offsetX = 12;
int offsetY = -788;
int offsetZ = -244;

int x, y, z;
int angle;

/****************************************
 * L298 motor driver
 ****************************************/
#define motorLeftF      9    //left  motor forward  PWM
#define motorLeftB      5    //left  motor reversal PWM
#define motorRightB     6    //right motor reversal PWM
#define motorRightF     10   //right motor forward  PWM

int motorSpeedRun     = 120; //motor forwrd speed
int motorSpeedStop    = 0;   //motor stop speed

/****************************************
 * Bluetooth Communication
 ****************************************/
byte buffer; //message buffer

/****************************************
 * Trace
 ****************************************/
#define trailingRight  4     //trace of left
#define trailingLeft   7     //trace of right

/****************************************
 * Detection Voltage
 ****************************************/
float voltage;               //battery voltage

#define checkV         A2    //voltage signal
#define powerState     A3    //detect the power state

/****************************************
 * Servo
 ****************************************/
 Servo servo;                //servo object
 
#define servoPin       2     //servo signal
#define servoL         1     //servo turn left
#define servoR         2     //servo turn right

int servoAngleFront   = 65;  //middle  angle
int servoAngleRight   = 0;  //right   angle
int servoAngleLeft    = 180; //left    angle
int servoAngleCurrent = 90;  //current angle
int turnAngle         = 20;  //turn    angle
int pulseWidth;              //pulse   width

/****************************************
 * Infrared
 ****************************************/
#define infraredRight   8   //left  infrared signal
#define infraredLeft   11  //right infrared signal

int infraredLeftState;     //left  infrared state
int infraredRightState;    //right infrared state
int trailingLeftState;     //left  trace    state
int trailingRightState;    //right infrared state

/****************************************
 * Ultrasonic
 ****************************************/
#define ultrasonicEcho A1  //Echo signal 
#define ultrasonicTrig A0  //Trig signal

int distanceFront;         //front distance
int distanceLeft;          //left  distance
int distanceRight;         //right distance

/****************************************
 * infrared receiver
 * define signal pins
 * instantiated infrared receiving class
 ****************************************/
int recvPinFRight  = 3;      //infrared right&front receive

IRrecv irrecvFRight(recvPinFRight);

decode_results resultsFRight; //right&front state

long unsigned int IRSEND = 0x20DF10EF;     //infrared coding

/****************************************
 * get send route
 ****************************************/
int carX, carY;                //coordinate of car
int oldX, oldY;                //coordinate of old man

byte dirPoint[20];             //route point
int  pointLen;                 //the number of route point 
int  pointSize;                //route scale

/****************************************
 * initialize the function
 ****************************************/
void setup() { 
  Serial.begin(9600);

  delay( 5);
  Wire.begin();
  
//  //set HMC5883 work mode  
//  Wire.beginTransmission(address); //begin  communication  
//  Wire.write(0x00);                //choose configuration register A  
//  Wire.write(0x70);                //0111   0000b(value) 
//  Wire.endTransmission();          //end    communication
 
//  Wire.beginTransmission(address); //begin  communication
//  Wire.write(0x01);                //choose configuration register B
//  Wire.write(0x00);                //0000   0000(value) 
//  Wire.endTransmission();
   
  Wire.beginTransmission(address);  
  Wire.write(0x02);                  
  Wire.write(0x00);                  //confinuous measurement mode:0x00  signal measurement mode:0x01  
  Wire.endTransmission();  

  //receive infrared signal
  //irrecvFLeft.enableIRIn();
  //irrecvLeft.enableIRIn();
  //irrecvRight.enableIRIn();
  irrecvFRight.enableIRIn();
  
 //initialization of motor dirver
 pinMode(motorRightB, OUTPUT);
 pinMode(motorRightF, OUTPUT);
 pinMode(motorLeftB,  OUTPUT);
 pinMode(motorLeftF,  OUTPUT);
 
 //initialization of infrared
 pinMode(infraredLeft, INPUT);
 pinMode(infraredRight, INPUT);
 
 //initialization of ultrasonic
 pinMode(ultrasonicEcho, INPUT);
 pinMode(ultrasonicTrig, OUTPUT);
 
 //initialization of servo
 pinMode(servoPin, OUTPUT);
 //servo.attach(servoPin);
 
 //initialization of tracing
 pinMode(trailingLeft, INPUT);
 pinMode(trailingRight, INPUT);

 //initialization the power state 
 pinMode(powerState, INPUT);

 distanceTestFront();
 delay(100);

  //calibration
  calibrateMag(); 
}

/****************************************
 * Ultrasonic
 * distanceTest()         detect distance
 * distanceTestFront()    front  distance
 * distanceTestLeft()     left   distance
 * distanceTestRight()    right  distance
 ****************************************/
float distanceTest(){
  digitalWrite(ultrasonicTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTrig, LOW);
  float dis = pulseIn(ultrasonicEcho, HIGH);
  dis = dis / 58;
  return dis;
}

void distanceTestFront(){
  turnServo(servoAngleFront);
  delay(500);
  servoAngleCurrent = servoAngleFront;
  distanceFront = distanceTest();
}

void distanceTestLeft(){
  turnServo(servoAngleLeft);
  delay(500);
  servoAngleCurrent = servoAngleLeft;
  distanceLeft = distanceTest();
}

void distanceTestRight(){
  turnServo(servoAngleRight);
  delay(500);
  servoAngleCurrent = servoAngleRight;
  distanceRight = distanceTest();
}

/****************************************
 * Control Servo
 * turnServo()     turn the specified angle
 * servoPulse()    pulse function
 * servoControl()  control servo
 * 1-turn left     2-turn right
 ****************************************/
void turnServo(int angle){
  for(int i = 0; i <= 20; i++)
    servoPulse(angle);
}

void servoPulse(int angle){
  pulseWidth = (angle * 11) + 500; //map the pulse width value to 500-2048
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(servoPin, LOW);
  delay(20-pulseWidth/1000);  
}

void servoControl(int dir){
  if(dir == servoL){
    servoAngleCurrent -= turnAngle;
    if(servoAngleCurrent < 0)
      servoAngleCurrent = 0;
    for(int i = 0; i <= 20; i++){
      servoPulse(servoAngleCurrent);
    }
  }
  else if(dir == servoR){
    servoAngleCurrent += turnAngle;
    if(servoAngleCurrent > 180)
      servoAngleCurrent = 180;
    for(int i = 0; i <= 20; i++){
      servoPulse(servoAngleCurrent);
    }
  }
}

/****************************************
 * Move Control
 * turnForward()         forward
 * turnBack()        backward
 * turnLeft()            left
 * turnLeftOriginal()    left      in situ
 * turnRightOriginal()   right     in situ
 * turnRight()           right
 * turnStop()            stop
 ****************************************/
void turnForward(){
  digitalWrite(motorLeftF, HIGH);
  digitalWrite(motorLeftB, LOW);
  analogWrite(motorLeftF, motorSpeedRun + 5);
  digitalWrite(motorRightF, HIGH);
  digitalWrite(motorRightB, LOW);
  analogWrite(motorRightF, motorSpeedRun - 6);
}

void turnBack(){
  digitalWrite(motorLeftF, LOW);
  digitalWrite(motorLeftB, HIGH);
  analogWrite(motorLeftB, motorSpeedRun - 2);
  digitalWrite(motorRightF, LOW);
  digitalWrite(motorRightB, HIGH);
  analogWrite(motorRightB, motorSpeedRun + 2);
}

void turnLeft(){
  digitalWrite(motorRightF, HIGH);
  digitalWrite(motorRightB, LOW);
  analogWrite(motorRightF, motorSpeedRun);
  digitalWrite(motorLeftF, LOW);
  digitalWrite(motorLeftB, LOW);
}

void turnLeftOriginal(){
    digitalWrite(motorRightF, HIGH);
    digitalWrite(motorRightB, LOW);
    analogWrite(motorRightF, motorSpeedRun);
    digitalWrite(motorLeftF, LOW);
    digitalWrite(motorLeftB, HIGH);
    analogWrite(motorLeftB, motorSpeedRun);
}

void turnRight(){
  digitalWrite(motorLeftF, HIGH);
  digitalWrite(motorLeftB, LOW);
  analogWrite(motorLeftF, motorSpeedRun);
  digitalWrite(motorRightF, LOW);
  digitalWrite(motorRightB, LOW); 
}

void turnRightOriginal(){
  digitalWrite(motorRightF, LOW);
  digitalWrite(motorRightB, HIGH);
  analogWrite(motorRightB, motorSpeedRun);
  digitalWrite(motorLeftF, HIGH);
  digitalWrite(motorLeftB, LOW);
  analogWrite(motorLeftF, motorSpeedRun);
}

void turnStop(){
  digitalWrite(motorLeftF,  LOW);
  digitalWrite(motorLeftB,  LOW);
  digitalWrite(motorRightF, LOW);
  digitalWrite(motorRightB, LOW);  
}

/****************************************
 * Power State
 * powerJudge()      detect the power state
 ****************************************/
void powerJudge(){
  float val;//store the voltage value
  val = analogRead(powerState);
  val = val / 1023 * 5.0;
  if(val >= 4)
    sendPowerState('Y');//being charging
  else
    sendPowerState('N');
}

void sendPowerState(char flag){
  Serial.print(flag);
}

/****************************************
 * Infrared receiver&transmitter
 * trailingJudge()    to trace
 ****************************************/
void trailingJudge(){
  while(1){
    trailingLeftState = digitalRead(trailingLeft);
    trailingRightState = digitalRead(trailingRight);
    if(trailingLeftState == LOW && trailingRightState == LOW){
      turnForward();
    }
    //left off the track
    else if(trailingLeftState == LOW && trailingRightState == HIGH){
      turnRight();
    }
    //right off the track
    else if(trailingLeftState == HIGH && trailingRightState == LOW){
      turnLeft();
    }
    //message interrupt
    if(messageJudge())
      break;
  }
}

/****************************************
 * avoidance function
 * infraredJudge()         avoidance by infrared
 * front infrared of receiver and trasmitter
 * ultrasonicJudge()       avoidance by ultrasonic(first version)
 * the servo link the ultrasonic which can turn the direction
 * ultrasonicSecJudge()    avoidance by ultrasonic(second version)
 * realize the function of forward ,avoidance and turn to origianl direction
 * infraUltraJudge()       avoidance by ultrasonic and infrared
 ****************************************/
void infraredJudge(){
  while(1){
    turnForward();
    infraredLeftState = digitalRead(infraredLeft);
    infraredRightState = digitalRead(infraredRight);
    //obstacle on the left
    if(infraredLeftState == LOW && infraredRightState == HIGH){
      turnStop();
      delay(100);
      turnBack();
      delay(300);
      turnRight();
      delay(700);
    }
    //obstacle on the right
    else if(infraredLeftState == HIGH && infraredRightState == LOW){
      turnStop();
      delay(100);
      turnBack();
      delay(300);
      turnLeft();
      delay(700); 
    }
    //obstacle on the right and left
    else if(infraredLeftState == LOW && infraredRight == LOW){
      turnStop();
      delay(100);
      turnBack();
      delay(500);
      turnRight();
      delay(700); 
   }
   else
      turnForward();
   //message interrupt
   if(messageJudge())
      break;
   }
}

void ultrasonicJudge(){
  while(1){
    turnForward();
    distanceTestFront();
    delay(50);
    if(distanceFront < 20){
      turnStop();
      delay(300);
      turnBack();
      delay(300);
      turnStop();
      delay(300);
      distanceTestLeft();
      distanceTestRight();
      if(distanceLeft < 20 && distanceRight < 20){
          turnRightOriginal();
          delay(700);
      }  
      else if(distanceLeft > distanceRight){
         turnLeft();
         delay(700);  
      }
      else if(distanceLeft < distanceRight){
        turnRight();
        delay(700);  
      }
      else
        turnForward();
    }
    //message interrupt
    if(messageJudge())
      break;
  }
}

void ultrasonicSecJudge(){
  int currentAngle; //current angle
  int runDirection; //turn direction 1 left -1 right
  while(1){
    turnForward();
    distanceTestFront();
    if(distanceFront < 15){
      turnStop();
      delay(50);
      turnBack();
      delay(200);
      turnStop();
      delay(50);
      distanceTestLeft();
      distanceTestRight();
      delay(50);
      getAngle();
      currentAngle = angle;
      if(distanceLeft > distanceRight){
          runDirection = 1;
          turnLeft();
          delay(500);
          turnStop();
          delay(50);
      }
      else if(distanceLeft < distanceRight){
          runDirection = -1;
          turnRight();
          delay(500);
          turnStop();
          delay(50);
      }
      turnForCorrect(runDirection);
      turnDir(currentAngle);
    }
    if(messageJudge()){
      break;
    }
  }
}

void turnForCorrect(int dir){
  int runTime = 0;
  if(dir == 1){
    for(int i = 0; i < 2;){
      turnForward();
      delay(100);
      if(i == 0)
        runTime += 100;
      turnStop();
      delay(50);
      distanceTestRight();
      if(distanceRight > 15){
        turnRight();
        delay(500);
        turnStop();
        delay(50);
        i++;
      }
    }
    turnForward();
    delay(runTime);
  }else{
    for(int i = 0; i < 2;){
      turnForward();
      delay(100);
      if(i == 0)
        runTime += 100;
      turnStop();
      delay(50);
      distanceTestLeft();
      if(distanceLeft > 15){
        turnLeft();
        delay(500);
        turnStop();
        delay(50);
        i++;
    }
  }
  turnForward();
  delay(runTime);
  }
}

void infraUltraJudge(){
  while(1){
    turnForward();
    infraredLeftState  = digitalRead(infraredLeft);
    infraredRightState = digitalRead(infraredRight);
    if(infraredLeftState == LOW && infraredRightState == HIGH){
      turnRight();
      delay(300);
    }//left has obstacle
    else if(infraredLeftState == HIGH && infraredRightState == LOW){
      turnLeft();
      delay(300);
    }//right has obstacle
    else if(infraredLeftState == LOW && infraredRightState == LOW){
      turnStop();
      delay(100);
      distanceTestLeft();
      delay(500);
      distanceTestRight();
      delay(500);
      distanceTestFront();
      delay(500);
      if(distanceLeft <= 20 && distanceRight <= 20){
        turnBack();
        delay(700);
        turnRightOriginal();
        delay(1500);
        turnStop();
        delay(100);
      }
      else if(distanceLeft > distanceRight){
        turnLeft();
        delay(1000);
        turnStop();
        delay(100);
      }
      else if(distanceRight > distanceLeft){
        turnRight();
        delay(1000);
        turnStop();
        delay(100);
      }
    }
    else
      turnForward();
    //message interrupt
    if(messageJudge()){
      turnStop();
      break;
    }
  }
}

/****************************************
 * detect battery voltage
 ****************************************/
void testVoltage(){
  int val;
  val = analogRead(checkV);
  voltage = val / 40.92;
}

/****************************************
 * server send the point 
 *   7   0   1
 *    \  |  /
 * 6 --------- 2
 *    /  |  \
 *   5   4   3
 *   runRoute()  route trace
 *   runDir()    forward direction
 *   pointLen    the number of route point 
 *   pointSize   route scale
 *   dirPoint    route point 
 *   getDir()    get next direction
 ****************************************/
void runRoute(){
  int i;
  int turnTime = 590;
  int cnt = 0;
  int times = 0;
  //the number of route poing
  while(1){
    if(Serial.available()){
      pointLen = int(Serial.read());
      break;
    }
  }
  //Serial.println(pointLen);
  //route scale
  while(1){
    if(Serial.available()){
      pointSize = int(Serial.read());
      break;
    }
  }
//  Serial.println(pointSize);
  //route size
  while(1){
    if(Serial.available()){
      dirPoint[cnt++] = byte(Serial.read());
//      Serial.print(dirPoint[cnt-1]);
    }
    if(cnt == pointLen)
      break;
  }
  for(int i = 0; i < pointLen;){
//    motorSpeedRun = 100;
//    if((i && dirPoint[i-1] == dirPoint[i]) || (!i && times && dirPoint[i] == dirPoint[pointLen-1]))
//      turnForward();
//    else{
      runDirSec(dirPoint[i]);
      runDir(dirPoint[i]);
      turnForward();
//    }
    delay(500 * (6 - pointSize));
    turnStop();
    i = i == pointLen-1 ? 0 : i+1;
//    turnRight();
//    delay(1000);
//    turnStop();
//    delay(100);
//    turnForward();
//    delay(500 * (6-pointSize));
//    turnStop();
//    delay(100);
    if(messageJudge()){
//      motorSpeedRun = 110;
      break;
    }
    times++;
  }
}

int getDir(int dir){
  int nextAngle;
  switch(dir){
    case 0:
      nextAngle = 0;
      break;
    case 1:
      nextAngle = 315;
      break;
    case 2:
      nextAngle = 270;
      break;
    case 3:
      nextAngle = 225;
      break;
    case 4:
      nextAngle = 180;
      break;
    case 5:
      nextAngle = 135;
      break;
    case 6:
      nextAngle = 90;
      break;
    case 7:
      nextAngle = 45;
      break;
  }
  return nextAngle;
}

void runDir(int dir){
  int nextAngle = getDir(dir);  //next direction
//  motorSpeedRun = 150;
  getAngle();                   //current angle
  int dis = nextAngle - angle; 
  if(dis >= 180){
    while(1){
      turnRight();
      delay(50);
      turnStop();
      delay(50);
      getAngle();
      delay(100);
      if(angle >= nextAngle - 4 && angle <= nextAngle + 4){
//        turnStop();
        break;
      }
    }
  }
  else if(dis >= 0 && dis < 180){
    while(1){
      turnLeft();
      delay(50);
      turnStop();
      delay(50);
      getAngle();
      delay(100);
      if(angle >= nextAngle - 4 && angle <= nextAngle + 4){
//        turnStop();
        break;
      } 
    }
  }
  else if(dis <= -180){
    if(nextAngle == 0){
      while(1){
        turnLeft();
        delay(50);
        turnStop();
        delay(50);
        getAngle();
        delay(100);
        if(angle >= 360 - 4 || angle <= 0 + 4){
//          turnStop();
          break;
        }
      }
    }
    else{
      while(1){
        turnLeft();
        delay(50);
        turnStop();
        delay(50);
        getAngle();
        delay(100);
        if(angle >= nextAngle - 4 && angle <= nextAngle + 4){
//          turnStop();
          break;
        }
      }
    }
  }
  else if(dis > -180 && dis < 0){
    if(nextAngle == 0){
      while(1){
        turnRight();
        delay(50);
        turnStop();
        delay(50);
        getAngle();
        delay(100);
        if(angle >= 360 - 4 || angle <= 0 + 4){
//          turnStop();
          break;
        }
      }
    }
    else{
      while(1){
        turnRight();
        delay(50);
        turnStop();
        delay(50);
        getAngle();
        delay(100);
        if(angle >= nextAngle - 4 && angle <= nextAngle + 4){
//          turnStop();
          break;
        }
      }
    }
  }
//  motorSpeedRun = 100;
}

int runTime(int runAngle){
  return runAngle * 7.6;
}

void runDirSec(int dir){
  int runAngle;                 //turn angle
  int runT;                     //turn time
  int nextAngle = getDir(dir);  //next angle
//  motorSpeedRun = 150;
  getAngle();                   //get current angle
  delay(100);
  int dis = nextAngle - angle; 
  if(dis >= 180){
    runAngle = angle + 360 - nextAngle;
    runT = runTime(runAngle);
    turnRight();
    delay(runT);
    turnStop();
    delay(100);
  }
  else if(dis >= 0 && dis < 180){
    runAngle = nextAngle - angle;
    runT = runTime(runAngle);
    turnLeft();
    delay(runT);
    turnStop();
    delay(100);
  }
  else if(dis <= -180){
    runAngle = nextAngle + 360 - angle;
    runT = runTime(runAngle);
    turnLeft();
    delay(runT);
    turnStop();
    delay(100);
  }
  else if(dis > -180 && dis < 0){
    runAngle = angle - nextAngle;
    runT = runTime(runAngle);
    turnRight();
    delay(runT);
    turnStop();
    delay(100);
  }
//  motorSpeedRun = 100;
}

/****************************************
 * bluetooth receive message from servo
 * messageJudge()   
 * call function
 ****************************************/
bool messageJudge(){
  if(Serial.available()){
    switch(Serial.read()){
      case goFront:
        turnForward();
        break;
      case goBack:
        turnBack();
        break;
      case goLeft:
        turnLeft();
        break;
      case goRight:
        turnRight();
        break;
      case goStop:
        turnStop();
        break;
      case goRightOriginal:
        turnRightOriginal();
        break;
      case goLeftOriginal:
        turnLeftOriginal();
        break;
      case trailingAuto:
        trailingJudge();
        break;
      case ultrasonicAuto:
        infraUltraJudge();
        break;
      case infraredAuto:
        infraredJudge();
        break;
      case servoRight:
        servoControl(servoR);
        break;
      case servoLeft:
        servoControl(servoL);
        break;
      case powerFind:
        findPower();
        powerJudge();
        break;
      case runCircle:
        runRoute();
        break;
      case carryToOld:
        oldJudge();
        break;
      case messageBlue:
      //send message
        break;
      default:
        break;  
    }  
    return true;  
  }
  return false;
}

/****************************************
*HMC5883L module fuction
*getRawData          collect module data
*calculateHeading    calculate angle 
*calibrateMag        calculate the deviation
*****************************************/
int getAngle(){
  for(int i = 0; i < 7; i++){
    getRawData(&x, &y, &z);
    angle = calculateHeading(&x, &y, &z);
  }
  /*angle -= 90;
  if(angle < 0)
    angle += 360;*/
}

void getRawData(int* x ,int* y,int* z)  
{  
  Wire.beginTransmission(address);  
  Wire.write(0x03);       //read data from register 3  
  Wire.endTransmission();  
                          //data is all 16 bits
  Wire.requestFrom(address, 6);  
  if(6<=Wire.available()){  
    *x = Wire.read()<<8;  //X msb，high 8 bits  
    *x |= Wire.read();    //X lsb，low  8 bits
    *z = Wire.read()<<8;  //Z msb  
    *z |= Wire.read();    //Z lsb  
    *y = Wire.read()<<8;  //Y msb  
    *y |= Wire.read();    //Y lsb  
  }  
}  

int calculateHeading(int* x ,int* y,int* z)  
{  
  //float headingRadians = atan2((double)((*y)-offsetY),(double)((*x)-offsetX));  
  float headingRadians = atan2((double)((*x)-offsetX), (double)((*y)-offsetY)); 
  //map data to 0-2*PI  
  if(headingRadians < 0)  
    headingRadians += 2*PI;  
   
  int headingDegrees = headingRadians * 180/M_PI;  
  //headingDegrees += MagnetcDeclination; //magnetic declination  
   
  //map data to 0-360
  if(headingDegrees > 360)  
    headingDegrees -= 360;  
   
  return headingDegrees;  
}  

void calibrateMag()  
{  
  int x,y,z; //three axes data 
  int xMax, xMin, yMax, yMin, zMax, zMin;  
  //initilization
  getRawData(&x,&y,&z);    
  xMax=xMin=x;  
  yMax=yMin=y;  
  zMax=zMin=z;  
  offsetX = offsetY = offsetZ = 0;  
   
//  Serial.println("Starting Calibration......");  
//  Serial.println("Please turn your device around in 20 seconds");  
   
  for(int i=0;i<200;i++)  
  {  
    getRawData(&x,&y,&z);  
    //calculate maximum and minimum 
    //three axes maximum and minimum 
    if (x > xMax)  
      xMax = x;  
    if (x < xMin )  
      xMin = x;  
    if(y > yMax )  
      yMax = y;  
    if(y < yMin )  
      yMin = y;  
    if(z > zMax )  
      zMax = z;  
    if(z < zMin )  
      zMin = z;  
   
    delay(100);  
   
    /*if(i%10 == 0)  
    {  
      Serial.print(xMax);  
      Serial.print(" ");  
      Serial.println(xMin);
    }*/  
  }  
  //correction amount 
  if(abs(xMax - xMin) > CalThreshold )  
    offsetX = (xMax + xMin)/2;  
  if(abs(yMax - yMin) > CalThreshold )  
    offsetY = (yMax + yMin)/2;  
  if(abs(zMax - zMin) > CalThreshold )  
    offsetZ = (zMax +zMin)/2;  
   
  delay(2000);    
}

/****************************************
 * infrared receiver and transmitter
 * findPower()         positioning charge
 * gotoDestation()     reach charge
 * forLeftState()      left  infrared
 * forRightState()     right infrared 
 * forLeRiState()      left&right infrared
 * getInfraredState()  get infrared state
 ****************************************/
void findPower(){
  int dir = 0;
  int state;
//  motorSpeedRun = 150;
  while(1){
    while(1){
      long unsigned int sFRight = 0;
      if(irrecvFRight.decode(&resultsFRight)){
      sFRight = resultsFRight.value;
      //Serial.print("SFRight: ");
      //Serial.println(sFRight, HEX);
      irrecvFRight.resume();
      }
      delay(100);
      if(sFRight == IRSEND){
        break;
      }
      if(dir == 0)
        turnRight();
      else
        turnLeft();
      delay(50);
      turnStop();
      delay(50);
    }
    turnForward();
    delay(550);
    turnStop();
    delay(100);
    break;
//    distanceTestFront();
//    Serial.println(distanceFront);
//    delay(100);
//    if(distanceFront <= 15){
//      break;
//    }
//    getInfraredState();
//    if(infraredLeftState == LOW && infraredRightState == HIGH){
//      state = 0;
//      break;
//    }
//    else if(infraredLeftState == HIGH && infraredRightState == LOW){
//      state = 1;
//      break;
//    }
//    else if(infraredLeftState == LOW && infraredRightState == LOW){
//      state = 2;
//      break;
//    }
//    if(dir++ == 2)
//        dir = 0;
  }
//  gotoDestation(state);
  goNext();
//  motorSpeedRun = 110;
}

void goNext(){
  int is_ok = 0;
  int base = 1;
  while(1){
    long unsigned int sFRight = 0;
    if(irrecvFRight.decode(&resultsFRight)){
      sFRight = resultsFRight.value;
      //Serial.print("SFRight: ");
      //Serial.println(sFRight, HEX);
      irrecvFRight.resume();
    }
    delay(100);
    if(sFRight == IRSEND){
      turnForward();
      delay(550);
      turnStop();
      delay(50);
      is_ok = 0;
      distanceTestFront();
      delay(100);
      //Serial.println(distanceFront);
      if(distanceFront <= 15){
        turnForward();
        delay(700);
        turnStop();
        break;
      }
      base = 1;
      continue;
    }
    if(is_ok == 0){
      turnLeft();
      delay(20 * base);
      turnStop();
      delay(50);
      is_ok++;
      continue;
    }
    else if(is_ok == 1){
      for(int i = 1; i <= base; i++){
        turnRight();
        delay(15 * base);
        turnStop();
//        delay(50);
        long unsigned int sF = 0;
        if(irrecvFRight.decode(&resultsFRight)){
          sF = resultsFRight.value;
          //Serial.print("SFRight: ");
          //Serial.println(sFRight, HEX);
          irrecvFRight.resume();
        }
        delay(100);
        if(sF == IRSEND){
          turnForward();
          delay(550);
          turnStop();
          delay(100);
          break;
        }
      }
//      turnRight();
//      delay(15 * base);
//      turnStop();
//      delay(50);
      is_ok++;
      base++;
      continue;
    }
    distanceTestFront();
    delay(100);
    //Serial.println(distanceFront);
    if(distanceFront <= 15){
      turnForward();
      delay(700);
      turnStop();
      break;
    }
    else{
      is_ok = 0;
    }
  }
}

void gotoDestation(int state){
  switch(state){
    case 0:
      forLeftState();
      break;
    case 1:
      forRightState();
      break;
//    case 2:
//      forLeRiState();
//      break;
    default:
      break;
  }
}

void forLeftState(){
  while(1){
    getInfraredState();
    if(infraredLeftState == LOW && infraredRightState == HIGH){
      turnRight();
      delay(50);
      turnStop();
      turnForward();
      delay(100);
      turnStop();
      delay(100);
    }
    else if(infraredLeftState == HIGH && infraredRightState == LOW){
      turnLeft();
      delay(50);
      turnStop();
      turnForward();
      delay(100);
      turnStop();
      delay(100);
    }
    else if(infraredLeftState == LOW && infraredRightState == LOW){
      break;
    }
  }
  //forLeRiState();
}

void forRightState(){
  while(1){
    getInfraredState();
    if(infraredLeftState == LOW && infraredRightState == HIGH){
      turnRight();
      delay(50);
      turnStop();
      turnForward();
      delay(100);
      turnStop();
      delay(100);
    }
    else if(infraredLeftState == HIGH && infraredRightState == LOW){
      turnLeft();
      delay(50);
      turnStop();
      turnForward();
      delay(100);
      turnStop();
      delay(100);
    }
    else if(infraredLeftState == LOW && infraredRightState == LOW){
      break;
    }
  }
  forLeRiState();
}

void forLeRiState(){
  turnForward();
  delay(400);
  turnStop();
}

void getInfraredState(){
  infraredLeftState = digitalRead(infraredLeft);
  infraredRightState = digitalRead(infraredRight);
}

/****************************************
 * send message to server
 ****************************************/
void sendBlueMessage(){
  Serial.print(buffer);
}

/****************************************
 * positioning old man 
 * oldJudge()     get and send things
 * getPoint()     get car and old man coordinate 
 * getOldAngle()  next angle
 * turnDir()      turn direction
 * arriveDir()    reach the positon of old man
 ****************************************/
void oldJudge(){
  int dis;
  int dire;
  int nextAngle;
  int nexttime;
//  getPoint();
  while(1){
    if(Serial.available()){
      dire = Serial.read();
      break;
    }
  }
  turnStop();
  delay(100);
  findPower();
  delay(100);
  Serial.print('0');
  while(1){
    if(Serial.available()){
      if(Serial.read() == getThingOk)
        break;
    }
  }
//  motorSpeedRun = 150;
  turnBack();
  delay(1500);
  turnStop();
  delay(100);
//  findPower();
//  delay(15000);
//  turnBack();
//  delay(1000);
//  nextAngle = getOldAngle();
  //Serial.print("old people angle: ");
  //Serial.println(nextAngle);
//  if(nextAngle == -1)
//    return;
//  dis = sqrt(pow((oldX - carX), 2) + pow((oldY - carY), 2));
  //Serial.print("old pelple and car's distance: ");
  //Serial.println(dis);
//  turnDir(nextAngle);
  if(dire == 3){
    turnRight();
    delay(1000);
  }
  else if(dire == 4){
    turnLeft();
    delay(1000);
  }
  turnStop();
  delay(100);
  turnForward();
  delay(2000);
  turnStop();
  delay(100);
//  motorSpeedRun = 110;
  //arriveDir(nextAngle);
}

void arriveDir(int nextAngle){
  turnForward();
  delay(1000);
  turnStop();
  delay(50);
  getAngle();
  if(angle <= nextAngle - 4){
    turnLeft();
    delay(100);
  }
  else if(angle >= nextAngle + 4){
    turnRight();
    delay(100);
  }
  if(messageJudge()){
    return;
  }
}

void turnDir(int nextAngle){
  int runAngle;
  int runT;
  getAngle();
  delay(200);
  if(nextAngle >= 360 - 4){
    if(angle >= 180){
      while(1){
        turnLeft();
        delay(50);
        turnStop();
        delay(100);
        getAngle();
        delay(200);
        if(angle >= 360 - 4 || angle <= 0 + 4){
          turnStop();
          break;
        }
      }
//      runAngle = nextAngle - angle;
//      runT = runTime(runAngle);
//      turnLeft();
//      delay(runT);
//      turnStop();
//      delay(100);
    }
    else{
      while(1){
        turnRight();
        delay(50);
        turnStop();
        delay(100);
        getAngle();
        delay(200);
        if(angle >= 360 - 4 || angle <= 0 + 4){
          turnStop();
          break;
        }
      }
//      runAngle = angle + 360 - nextAngle;
//      runT = runTime(runAngle);
//      turnRight();
//      delay(runT);
//      turnStop();
//      delay(100);
    }
  }
  else if(nextAngle <= 0 + 4){
    if(angle >= 180){
      while(1){
        turnLeft();
        delay(50);
        turnStop();
        delay(100);
        getAngle();
        delay(200);
        if(angle >= 360 - 4 || angle <= 0 + 4){
          turnStop();
          break;
        }
      }
//      runAngle = nextAngle + 360 - angle;
//      runT = runTime(runAngle);
//      turnLeft();
//      delay(runT);
//      turnStop();
//      delay(100);
    }
    else{
      while(1){
        turnRight();
        delay(50);
        turnStop();
        delay(100);
        getAngle();
        delay(200);
        if(angle >= 360 - 4 || angle <= 0 + 4){
          turnStop();
          break;
        }
      }
//      runAngle = angle - nextAngle;
//      runT = runTime(runAngle);
//      turnRight();
//      delay(runT);
//      turnStop();
//      delay(100);
    }
  }
//  else{
//    if(nextAngle >= angle){
//      if(nextAngle - angle >= 180){
//        runAngle = angle + 360 - nextAngle;
//        runT = runTime(runAngle);
//        turnRight();
//        delay(runT);
//        turnStop();
//        delay(100);
//      }
//      else{
//        runAngle = nextAngle - angle;
//        runT = runTime(runAngle);
//        turnLeft();
//        delay(runT);
//        turnStop();
//        delay(100);
//      }
//    }
//    else{
//      if(angle - nextAngle >= 180){
//        runAngle = nextAngle + 360 - angle;
//        runT = runTime(runAngle);
//        turnLeft();
//        delay(runT);
//        turnStop();
//        delay(100);
//      }
//      else{
//        runAngle = angle - nextAngle;
//        runT = runTime(runAngle);
//        turnRight();
//        delay(runT);
//        turnStop();
//        delay(100);
//      }
//    }
//  }
}

int getOldAngle(){
  if(oldX == carX){
    if(oldY > carY){
      return 180;
    }
    else if(oldY == carY){
      return -1;
    }
    else{
      return 0;
    }
  }
  else if(oldY == carY){
    if(oldX > carX){
      return 270;
    }
    else if(oldX == carX){
      return -1;
    } 
    else{
      return 90;
    }
  }
  else if(oldX < carX){
    if(oldY < carY){
      return (180 / 3.14) * atan((oldX - carX) / (oldY - carY));  
    }
    else if(oldY > carY){
      return 180 - (180 / 3.14) * atan((oldX - carX) / (oldY - carY));
    }
  }
  else if(oldX > carX){
    if(oldY < carY){
      return 360- (180 / 3.14) * atan((oldX - carX) / (oldY - carY));
    }
    else if(oldY > carY){
      return 180 + (180 / 3.14) * atan((oldX - carX) / (oldY - carY));
    }
  }
}

void getPoint(){
  carX = carY = 0;
  oldX = oldY = 1;
//  while(1){
//    if(Serial.available()){
//      carX = Serial.read();
//      break;
//    }
//  }
//  while(1){
//    if(Serial.available()){
//      carY = Serial.read();
//      break;
//    }
//  }
//  while(1){
//    if(Serial.available()){
//      oldX = Serial.read();
//      break;
//    }
//  }
//  while(1){
//    if(Serial.available()){
//      oldY = Serial.read();
//      break;
//    }
//  }
}

/****************************************
 * Circulation Body
 ****************************************/
void loop(){
  while(1){
    messageJudge();
  }
}
