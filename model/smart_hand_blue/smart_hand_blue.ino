#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LobotServoController.h>
//#include <NewPing.h>

/****************************************
 * Bluetooth Serial Communication Protocal
 ****************************************/
#define TURN_SERVO     0x00    //control the servo
#define GET_DIFF_COLOR 0x01    //get different color
#define GET_OK         0x02    //get things completed
#define GET_THING      0x03    //get anything

LobotServoController Controller(Serial1);

/****************************************
   * Ultrasonic Model 
 ****************************************/
#define TRIG    9               //ultrasonic pin of TRIG 9
#define ECHO    10              //ultrasonic pin of ECHO 10
#define MAX_DISTANCE 40

//NewPing Sonar(TRIG, ECHO, MAX_DISTANCE);

int distance = 0;               //store the detected distance

/****************************************
 * TSC3200 Model for testing color
 * 
 * S0   S1    Output Frequency Scaling
 * L    L     Power down
 * L    H     2%
 * H    L     20%
 * H    H     100%
 * 
 * S2   S3    Photodiode  Type
 * L    L     Red
 * L    H     Blue
 * H    L     Clear(no filter)
 * H    H     Green
 ****************************************/
 #define S0         4           //signal pin of S0
 #define S1         5           //signal pin of S1
 #define S2         7           //signal pin of S2
 #define S3         6           //signal pin of S3

 #define colorOut   8           //interrupt pin

//color
 #define RED        0        
 #define GREEN      1       
 #define BLUE       2       
 #define NO_COLOR   3       

int color     = NO_COLOR;       //store the detected color
int takeColor = NO_COLOR;       //store the color which one to take
int frequency = 0;              //store the temporary value of RGB

int colorArray[3];              //store teh value of RGB

/****************************************
 * Infrared Model
 ****************************************/
//#define infrared_left     50    //left  infrared pin
#define infrared_right    22    //right infrared pin

/****************************************
 * OLED Model
 ****************************************/
#define OLED_RESET 4

Adafruit_SSD1306 display(OLED_RESET);

#define OLED_SHOW_HEIGHT 16     //define the display height
#define OLED_SHOW_WEIGHT 16     //define the display weight

//Chinese
static const unsigned char PROGMEM str1[] =
{
  //智
  0x20,0x00,0x3E,0x7C,0x48,0x44,0x08,0x44,0xFF,0x44,0x14,0x44,0x22,0x7C,0x40,0x00,
  0x1F,0xF0,0x10,0x10,0x10,0x10,0x1F,0xF0,0x10,0x10,0x10,0x10,0x1F,0xF0,0x10,0x10
};

static const unsigned char PROGMEM str2[] =
{
  //能
  0x10,0x40,0x24,0x44,0x42,0x48,0xFF,0x70,0x01,0x40,0x00,0x42,0x7E,0x42,0x42,0x3E,
  0x42,0x00,0x7E,0x44,0x42,0x48,0x42,0x70,0x7E,0x40,0x42,0x42,0x4A,0x42,0x44,0x3E
};

static const unsigned char PROGMEM str3[] =
{
  //老
  0x02,0x00,0x02,0x08,0x3F,0xD0,0x02,0x20,0x02,0x40,0xFF,0xFE,0x01,0x00,0x02,0x00,
  0x0C,0x10,0x18,0xE0,0x2F,0x00,0x48,0x08,0x88,0x08,0x08,0x08,0x07,0xF8,0x00,0x00
};

static const unsigned char PROGMEM str4[] = 
{
  //人
  0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x02,0x80,0x02,0x80,
  0x04,0x40,0x04,0x40,0x08,0x20,0x08,0x20,0x10,0x10,0x20,0x08,0x40,0x04,0x80,0x02
};

static const unsigned char PROGMEM str5[] =
{
   //助
  0x00,0x20,0x7C,0x20,0x44,0x20,0x44,0x20,0x44,0xFC,0x7C,0x24,0x44,0x24,0x44,0x24,
  0x7C,0x24,0x44,0x24,0x44,0x24,0x44,0x44,0x4E,0x44,0xF0,0x84,0x01,0x28,0x02,0x10
};

static const unsigned char PROGMEM str6[] =
{
  //理
  0x00,0x00,0x01,0xFC,0xFD,0x24,0x11,0x24,0x11,0xFC,0x11,0x24,0x11,0x24,0x7D,0xFC,
  0x10,0x20,0x10,0x20,0x11,0xFC,0x10,0x20,0x1C,0x20,0xE0,0x20,0x43,0xFE,0x00,0x00
};

static const unsigned char PROGMEM str7[] =
{
  //系
  0x00,0xF8,0x3F,0x00,0x04,0x00,0x08,0x20,0x10,0x40,0x3F,0x80,0x01,0x00,0x06,0x10,
  0x18,0x08,0x7F,0xFC,0x01,0x04,0x09,0x20,0x11,0x10,0x21,0x08,0x45,0x04,0x02,0x00
};

static const unsigned char PROGMEM str8[] =
{
  //统
  0x10,0x40,0x10,0x20,0x20,0x20,0x23,0xFE,0x48,0x40,0xF8,0x88,0x11,0x04,0x23,0xFE,
  0x40,0x92,0xF8,0x90,0x40,0x90,0x00,0x90,0x19,0x12,0xE1,0x12,0x42,0x0E,0x04,0x00
};

static const unsigned char PROGMEM str9[] =
{
  //电
  0x01,0x00,0x01,0x00,0x01,0x00,0x3F,0xF8,0x21,0x08,0x21,0x08,0x21,0x08,0x3F,0xF8,
  0x21,0x08,0x21,0x08,0x21,0x08,0x3F,0xF8,0x21,0x0A,0x01,0x02,0x01,0x02,0x00,0xFE
};

static const unsigned char PROGMEM str10[] =
{
  //压
  0x00,0x00,0x3F,0xFE,0x20,0x00,0x20,0x80,0x20,0x80,0x20,0x80,0x20,0x80,0x2F,0xFC,
  0x20,0x80,0x20,0x80,0x20,0x90,0x20,0x88,0x20,0x88,0x40,0x80,0x5F,0xFE,0x80,0x00
};

static const unsigned char PROGMEM str11[] =
{
  //距
  0x00,0x00,0x7D,0xFE,0x45,0x00,0x45,0x00,0x45,0x00,0x7D,0xFC,0x11,0x04,0x11,0x04,
  0x5D,0x04,0x51,0x04,0x51,0xFC,0x51,0x00,0x5D,0x00,0xE1,0x00,0x01,0xFE,0x00,0x00
};

static const unsigned char PROGMEM str12[] =
{
  //离
  0x02,0x00,0x01,0x00,0xFF,0xFE,0x00,0x00,0x14,0x50,0x13,0x90,0x14,0x50,0x1F,0xF0,
  0x01,0x00,0x7F,0xFC,0x42,0x04,0x44,0x44,0x4F,0xE4,0x44,0x24,0x40,0x14,0x40,0x08
};

static const unsigned char PROGMEM str13[] =
{
  //安
  0x02,0x00,0x01,0x00,0x3F,0xFC,0x20,0x04,0x42,0x08,0x02,0x00,0x02,0x00,0xFF,0xFE,
  0x04,0x20,0x08,0x20,0x18,0x40,0x06,0x40,0x01,0x80,0x02,0x60,0x0C,0x10,0x70,0x08
};

static const unsigned char PROGMEM str14[] =
{
  //心
  0x00,0x00,0x02,0x00,0x01,0x00,0x00,0x80,0x00,0x80,0x04,0x00,0x04,0x08,0x24,0x04,
  0x24,0x04,0x24,0x02,0x44,0x02,0x44,0x12,0x84,0x10,0x04,0x10,0x03,0xF0,0x00,0x00
};

static const unsigned char PROGMEM str15[] =
{
  //家
  0x02,0x00,0x01,0x00,0x7F,0xFE,0x40,0x02,0x80,0x04,0x7F,0xFC,0x02,0x00,0x0D,0x08,
  0x71,0x90,0x02,0xA0,0x0C,0xC0,0x71,0xA0,0x06,0x98,0x18,0x86,0xE2,0x80,0x01,0x00
};

static const unsigned char PROGMEM str16[] =
{
  //居
  0x00,0x00,0x3F,0xF8,0x20,0x08,0x20,0x08,0x3F,0xF8,0x20,0x80,0x20,0x80,0x3F,0xFE,
  0x20,0x80,0x20,0x80,0x2F,0xF8,0x28,0x08,0x48,0x08,0x48,0x08,0x8F,0xF8,0x08,0x08
};

/****************************************
 * Servo Model
 ****************************************/
uint8_t  servoNum = 6; 

uint16_t handTime[10]    = {1500,  2000, 1000, 1000, 1500, 1500, 1500, 1500, 1500, 1500};        //running time
uint16_t handId1[10]     = {500,   500,  700,  700,  700,  700,  700,  500,  500,  500};                  //NO.1 servo parameters
uint16_t handId2[10]     = {1500,  1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};        //NO.2 servo parameters
uint16_t handId3[10]     = {1500,  2500, 2500, 2150, 1513, 1962, 1962, 1962, 1962, 1500};        //NO.3 servo parameters
uint16_t handId4[3][10]  = {{500,  500,  500,  780,  565,  1145, 952,  952,  952,  500},
                            {500,  651,  651,  780,  500,  1145, 952,  952,  952,  500},
                            {500,  844,  844,  780,  500,  1145, 952,  952,  952,  500}};               //NO.4 servo parameters
uint16_t handId5[3][10]  = {{1000, 2070, 2070, 1800, 1489, 1640, 1532, 1532, 1532, 1000},
                            {1000, 2134, 2134, 1800, 1360, 1640, 1532, 1532, 1532, 1000},
                            {1000, 2242, 2242, 1800, 1360, 1640, 1532, 1532, 1532, 1000}};      //NO.5 servo parameters
uint16_t handId6         = 1500;                                                                //NO.6 servo parameters

uint8_t handID[] = {1, 2, 3, 4, 5, 6};

/****************************************
 * OLED
 * showWelcome    show the welcome interface
 ****************************************/
void showWelcome(){
  display.setTextSize(4);
  display.setTextColor(WHITE);

  //show the initial interface
  //smart old assistant system
  display.drawBitmap(0,   0, str1, 16, 16, 1);
  display.drawBitmap(16,  0, str2, 16, 16, 1);
  display.drawBitmap(32,  0, str3, 16, 16, 1);
  display.drawBitmap(48,  0, str4, 16, 16, 1);
  display.drawBitmap(0,  16, str5, 16, 16, 1);
  display.drawBitmap(16, 16, str6, 16, 16, 1);
  display.drawBitmap(32, 16, str7, 16, 16, 1);
  display.drawBitmap(48, 16, str8, 16, 16, 1);

  //team name
//  display.drawBitmap(80,  0, str13, 16, 16, 1);
//  display.drawBitmap(96,  0, str14, 16, 16, 1);
//  display.drawBitmap(80, 16, str15, 16, 16, 1);
//  display.drawBitmap(96, 16, str16, 16, 16, 1);
  
  display.display();    //show cached content
}

/****************************************
 * Ultrasonic
 * getDistance    detected the distance
 ****************************************/
void getDistance(){
  float dis;
  unsigned long Time_Echo_us = 0;

  for(int i = 0; i < 6;){
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(50);
    digitalWrite(TRIG, LOW);
  
    Time_Echo_us = pulseIn(ECHO, HIGH); 
    if(Time_Echo_us < 6000 && Time_Echo_us > 1){
      dis = (Time_Echo_us*34/100)/2;
      i++;
//      Serial.print("current distance: ");
//      Serial.println(dis/10, DEC);
    } 
  }
  distance = dis / 10;
 
  /*for(int i = 0; i < 6; i){
      digitalWrite(TRIG, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG, LOW);
      dis = pulseIn(ECHO, HIGH);
  }*/
  
}

/****************************************
 * TSC3200
 * filterColor    filter the color
 * getFrequency   get the value of RGB
 * getColor       get the detected color
 * detectColor    detecting the color
 ****************************************/
void filterColor(int setColor){
  switch(setColor){
    case RED:
      digitalWrite(S2, LOW);
      digitalWrite(S3, LOW);
      break;
    case GREEN:
      digitalWrite(S2, HIGH);
      digitalWrite(S3, HIGH);
      break;
    case BLUE:
      digitalWrite(S2, LOW);
      digitalWrite(S3, HIGH);
      break;
    case NO_COLOR:
      digitalWrite(S2, HIGH);
      digitalWrite(S3, LOW);
      break;
  }
}

void getFrequency(int setColor){
  frequency = pulseIn(colorOut, LOW);               //reading the output frequency
  switch(setColor){
    case RED:
      frequency = map(frequency, 25, 72, 255, 0);   //remaping the value of the frequency to the RGB Model of 0 to 255
      break;
    case GREEN:
      frequency = map(frequency, 30, 90, 255, 0);
      break;
    case BLUE:
      frequency = map(frequency, 25, 70, 255, 0);
      break;
    default:
      break;
  }
  colorArray[setColor] = frequency;                //store the RGB value of color
  delay(100);
}

void getColor(){
  int maxRGB;
  maxRGB = colorArray[0];
  color = RED;
  for(int i = 1; i < 3; i++){
    if(maxRGB < colorArray[i]){
       maxRGB = colorArray[i];
       color = i;
    }
  }
}  

void detectColor(){
  for(int i = 0; i < 3; i++){
    filterColor(i);
    getFrequency(i);
  }
  getColor();
}

/****************************************
 * Servo Control
 * controlServo      control the servo to different angle
 * runAction         running action
 * runColorAction    running action for correct color
 * circleCheck       run a circle for getting things
 * getDiffColor      get things for special color
 * turnBack          turn to origlnal state
 * forCorrectColor   take things for correct color
 ****************************************/
void controlServo(){
  int SerId;
  int turnAngle;
  
  //get the servo ID
  while(1){
    if(Serial2.available()){
      SerId = Serial2.read();
      break;  
    }
  }
  //get the servo angle
  while(1){
    if(Serial2.available()){
      turnAngle = Serial2.read();
      turnAngle = turnAngle * 20 + 500;
      break;
    }
  }
  Controller.moveServo((uint8_t) SerId, (uint16_t) turnAngle, (uint16_t) 500);
}

void getDiffColor(){
  //get color
  while(1){
    if(Serial2.available()){
      takeColor = Serial2.read();
      break;
    }
  }
  while(1){
    if(Serial2.available()){
      if(Serial2.read() == GET_OK){
        break;
      }
    }
  }
  circleCheck();
}

void getAnyColor(){
  while(1){
    if(Serial2.available()){
      if(Serial2.read() == GET_OK){
        break;
      }
    }
  }
  circleCheck();
}

int circleCheck(){
  for(; handId6 < 2400; handId6 += 10){
    Controller.moveServos((uint8_t)6, (uint16_t)100, handID[0], handId1[0], handID[1], handId2[0], handID[2], handId3[0], 
                          handID[3], handId4[0][0], handID[4], handId5[0][0], handID[5], handId6);
    delay(100 + 50);
//    getDistance();
//    delay(100);
//    Serial.println(distance);
    if(judgeInfraredStatus()){
        if(takeColor != NO_COLOR){
//          handId6 += 10;
          if(forCorrectColor() == 0){
            handId6 += 100;
            if(handId6 >= 2400)
              handId6 = 2400;
            continue;
          }
          takeColor = NO_COLOR;
          return 1;
        }
        else{ 
          if(!moveBlock()) 
            continue;
          return 1;
        }
        return 1;
    }
  }
  for(; handId6 > 600; handId6 -= 10){
    Controller.moveServos(servoNum, (uint16_t)100, handID[0], handId1[0], handID[1], handId2[0], handID[2], handId3[0], 
                          handID[3], handId4[0][0], handID[4], handId5[0][0], handID[5], handId6);
    delay(100 + 50);
//    getDistance();
//    delay(100);
//    Serial.println(distance);
    if(judgeInfraredStatus()){
        if(takeColor != NO_COLOR){
//          handId6 -= 10;
          if(forCorrectColor() == 0){
            handId6 -= 100;
            if(handId6 <= 600)
              handId6 = 600;
            continue;
          }
          takeColor = NO_COLOR;
          
          return 1;
        }
        else
        {
          if(!moveBlock()) 
            continue;
          return 1;
        }
        return 1;
    }
  }
  return 0;
}

int forCorrectColor(){
  return  runColorAction(0);
//  getDistance();
//  delay(100);
//  if(distance > 12 && distance < 16){
//    return runColorAction(0);
//  }
//  else if(distance > 15 && distance < 18){
//    return runColorAction(1);
//  }
//  else if(distance > 17 && distance < 20){
//    return runColorAction(2);
//  }
//  return -1;
}

int moveBlock(){
  return runAction(0);
//  getDistance();
//  delay(100);
//  if(distance > 12 && distance < 16){
//    runAction(0);
//  }
//  else if(distance > 15 && distance < 18){
//    runAction(1);
//  }
//  else if(distance > 17 && distance < 20){
//    runAction(2);
//  }
//  return -1;
}

int runColorAction(int flag){
  if(flag < 0 || flag >2){
    //error running flag;
    return 0;  
  }
  for(int i = 0; i < 10; i++){
    if(i >= 6 && i <= 7){
      handId6 = 586;
    }else if(i >= 8){
      handId6 = 1500;  
    }
    Controller.moveServos(servoNum, handTime[i], handID[0], handId1[i], handID[1], handId2[i], handID[2], 
                           handId3[i], handID[3], handId4[flag][i], handID[4], handId5[flag][i], handID[5], handId6); 
    delay(handTime[i] + 100);
    //detect correct color
    if(i == 4){
      detectColor();
      delay(1000);
      if(color != takeColor){
        turnBack(flag); 
        return 0;
      }
    }
  }
  return 1;
}

void turnBack(int flag){
  for(int i = 3; i >=0; i--){
    Controller.moveServos(servoNum, handTime[i], handID[0], handId1[i], handID[1], handId2[i], handID[2], 
                           handId3[i], handID[3], handId4[flag][i], handID[4], handId5[flag][i], handID[5], handId6); 
    delay(handTime[i] + 100);
  }
}

int runAction(int flag){
  if(flag < 0 || flag >2){
    //error running flag;
    return 0;  
  }
  for(int i = 0; i < 10; i++){
    if(i >= 6 && i <= 7){
      handId6 = 586;
    }else if(i >= 8){
      handId6 = 1500;  
    }
    Controller.moveServos(servoNum, handTime[i], handID[0], handId1[i], handID[1], handId2[i], handID[2], 
                           handId3[i], handID[3], handId4[flag][i], handID[4], handId5[flag][i], handID[5], handId6); 
    delay(handTime[i] + 100);
  }
  return 1;
}

/****************************************
 * Bluetooth
 ****************************************/
bool messageJudge(){
  if(Serial2.available()){
    int ch = Serial2.read();
    Serial.println(ch, DEC);
    switch(ch){
      case TURN_SERVO:
        controlServo();
        break;
      case GET_DIFF_COLOR:
        getDiffColor();
        Serial2.print('0');
        break;
      case GET_THING:
        getAnyColor();
        Serial2.print('0');
        break;
      default:
        break;
    }
    return true;
  }
  return false;
}

/****************************************
 * Infrared 
 * judgeInfraredStatus()   //detecting the left and right infrared
 ****************************************/
int judgeInfraredStatus(){
  if(digitalRead(infrared_right) == LOW)
    return 1;
  return 0;
}

/****************************************
 * Battery Voltage
 ****************************************/
int updateBatteryState() {       //update battery voltage state
  static uint32_t Timer = 0;     //static variable for timing

  if (Timer > millis())          //judge running time
    return 0;
  Controller.getBatteryVoltage();//send command for getting battery voltage

  Timer = millis() + 5000;       //run the function after 5 seconds

  return 1;
}

void showOled(){
  float voltage;
  if(!updateBatteryState())
    return;
  display.clearDisplay();
  Controller.receiveHandle();
  voltage = Controller.batteryVoltage/100/10.0+1;
  
  display.setTextSize(4);
  display.setTextColor(WHITE);

  //show the initial interface
  //smart old assistant system
  display.drawBitmap(0,   0, str1, 16, 16, 1);
  display.drawBitmap(16,  0, str2, 16, 16, 1);
  display.drawBitmap(32,  0, str3, 16, 16, 1);
  display.drawBitmap(48,  0, str4, 16, 16, 1);
  display.drawBitmap(64,  0, str5, 16, 16, 1);
  display.drawBitmap(80,  0, str6, 16, 16, 1);
  display.drawBitmap(96,  0, str7, 16, 16, 1);
  display.drawBitmap(112, 0, str8, 16, 16, 1);
  
  display.drawBitmap(0,  16, str9,  16, 16, 1);
  display.drawBitmap(16, 16, str10, 16, 16, 1);

  display.setTextSize(2);
  display.setTextColor(WHITE);
  
  display.setCursor(32, 16);
  display.print(" ");
  display.print(voltage);
  display.print(" v");
  
  display.display();
}

/***************************************
 * Initial Function
 ***************************************/
void setup(){
  Serial2.begin(9600);
  Serial.begin(9600);
  Serial1.begin(9600);

  Controller.runActionGroup(0, 1);            //running 0 action to return initial position
  delay(1000);                                
  Controller.stopActionGroup();               //stop the action

  //for color model
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(colorOut, INPUT);

  //setting frequency-scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  //定义超声波引脚
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  //infrared signal
  pinMode(infrared_right, INPUT);

  //for OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //initialize with the I2C addr
  display.clearDisplay();                     //clear the display
//  showWelcome();  

}

/****************************************
 * Circulation Body
 ****************************************/
void loop(){
  while(1){
    showOled();
//    takeColor = RED;
//    circleCheck();
//    delay(5000);
    messageJudge();
//    getDistance();
//    delay(100);
//    Serial.print("current distance: ");
//    Serial.println(distance);
//    delay(3000);
  }
}


 



 
