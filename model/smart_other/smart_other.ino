#include <my_stepper.h>
#include <OneWire.h>
#include <Wire.h>
#include <IRremote.h>

/****************************************
 * Serial Port Communication Protocal
 ****************************************/
#define IRSEND    0x00  //send infrared signal
#define WATERTEMP 0x01  //get water temperature
#define TURNSTEP  0x02  //turn step motor
#define RAINSTATE 0x03  //rain volume status

/****************************************
 * infrared model
 * instantiate class
 * define pin
 ****************************************/
 IRsend irsend;
 
 #define  irPin 3
 int      khz = 38;    //the fraquency of infrared

/****************************************
 * stepper model
 * setup the pin
 * 64*64 step = 2*PI
 * 1  clockwise rotation
 * -1 anticlockwise rotation
 ****************************************/
 My_stepper stepper(8,9,10,11);
 
 #define PI_2 64*64     //the total steps for turning one circle

 /****************************************
 * rain test model
 * define the model pin
 * rain volumn
 ****************************************/
#define rain_capacity A0  //rain volumn status signal
#define has_rain      7   //the status of rain  pin

bool rainState = false;   //the status of rain(yes or no)
int rainLevel = 300;      //the threshold of rain

/****************************************
 * water temperature model
 * define the pin
 * 4K resistor
 ****************************************/
 OneWire  ds(2);      

 char celsius;     //degress celsius
 int fahrenheit;  //fahrenheit

 //char sendCelsius[2];  //

 int seconds = 0;   //timer1 count

/****************************************
 * Interrupt
 * send heart message
 ****************************************/
unsigned long nowTime;  //current time
unsigned long nextTime; //interrupt time of next

/****************************************
 * buffer of bluetooth
 ****************************************/
byte buffer;            //message buffer

/****************************************
 * stepper model
 * turnDir()       turning stop motor
 * dir             the turning direction of stop motor
 * stepSpeed()     set the speed of stop motor 
 * sped            the speed of stop motor
 ****************************************/
void stepControl(){
  int dir;
  /*while(1){
    if(Serial.available()){
      sped = int(Serial.read());
    }
  }//motor speed*/
  while(1){
    if(Serial.available()){
      dir = int(Serial.read());
      break;
    }
  }//turning direction of motor
  if(dir == 0)
    turnDir(1);
  else
    turnDir(-1);
}
 
void turnDir(int dir){
  stepper.my_step(PI_2/4, dir);
  delay(3000);
}

void stepSpeed(int sped){
  stepper.my_setSpeed(sped);
}

/****************************************
 * rain test model
 * judgeRainLevel()   judge weather reach the threshold
 * judgeRainState()   judge rain status
 * setRainLevel()     set the threshold
 ****************************************/
void rainControl(){
  judgeRainLevel();
  judgeRainState();
}
 
 bool judgeRainLevel(){
  int rain;
  rain = analogRead(rain_capacity);
  if(rain <= rainLevel)
    return true;
  return false;
}

void judgeRainState(){
  rainState = digitalRead(has_rain);
}

void setRainLevel(int level){
  rainLevel = level;
}

/****************************************
 * infrared model send
 * sendInfrared()     send infrared signal
 ****************************************/
void sendInfrared(){
  unsigned int irSignal[] = {9000, 4500, 560,  560,  560,  560,  560,  1690, 560,  560,  560,  560,  560,  560,   560,  560,  560, 560,
                             560,  1690, 560,  1690, 560,  560,  560,  1690, 560,  1690, 560,  1690, 560,  1690,  560,  1690, 560, 560, 
                             560,  560,  560,  560,  560,  1690, 560,  560,  560,  560,  560,  560,  560,  560,   560,  1690, 560, 1690,
                             560,  1690, 560,  560,  560,  1690, 560,  1690, 560,  1690, 560,  1690, 560,  39416, 9000, 2210, 560}; 
                            //AnalysIR Batch Export (IRremote) - RAW
  while(1){
    //Note the approach used to automatically calculate the size of the array.
    irsend.sendRaw(irSignal, sizeof(irSignal) / sizeof(irSignal[0]), khz); 
    delay(2000);    //repeated every 5 seconds
    if(judgeMessage()){
      break;
    }
  }
}

/****************************************
 * waterTemperature()   detect the water temperature
 * calculating the water temperature(degress celsius or fahrenheit)
 * judgeModelType()     detect the type of sensor
 ****************************************/
void waterTemperatur(){
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float temp;

  if ( !ds.search(addr)) {
    //Serial.println("No more addresses.");
    //Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }

  /*Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }*/

  if (OneWire::crc8(addr, 7) != addr[7]) {
      //Serial.println("CRC is not valid!");
      return;
  }
  //Serial.println();

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      //Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44);  // start conversion, use ds.write(0x44,1) with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);  // Read Scratchpad

  //Serial.print("  Data = ");
  //Serial.print(present, HEX);
  //Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
  }
  //Serial.print(" CRC=");
  //Serial.print(OneWire::crc8(data, 8), HEX);
  //Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;        // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3;   // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1;   // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
  temp = (float)raw / 16.0;
  //Serial.print("  Temperature = ");
  celsius = (char)temp;
  Serial.print(celsius);
  //Serial.print(" Celsius, ");
  //Serial.print(fahrenheit);
  //Serial.println(" Fahrenheit");
}

/****************************************
 * timer1 interrupt
 * every seconds
 * timer1Setup()       the setting of timer1
 * ISR()               interrupt function
 ****************************************/
void timer1Setup(){
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;

  OCR1A = 15624;

  TCCR1B |= (1 << WGM12);

  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);

  TIMSK1 |= (1 << OCIE1A);

  interrupts();
}

ISR(TIMER1_COMPA_vect){
  seconds++;
  if(seconds == 3){
    sendMessage();
    seconds = 0;
  }
}

/****************************************
 * initialize the function
 ****************************************/ 
void setup() {
  Serial.begin(9600);

  timer1Setup();

  stepper.my_setSpeed(3000);  //initialize the motor speed 

  pinMode(has_rain, INPUT);

  //initialize the time variable
  nowTime = millis();         
  nextTime = nowTime + 5000;  
}

/****************************************
 * bluetooth communication
 * sendMessage()    send cycle heartbeat message
 * judgeMessage()   judge the message from server
 ****************************************/
void sendMessage(){
  waterTemperatur();
}

bool judgeMessage(){
  if(Serial.available()){
    switch(Serial.read()){
      case IRSEND:
        sendInfrared();
        break;
      case WATERTEMP:
        sendMessage();
        break;
      case TURNSTEP:
        stepControl();
        break;
      case RAINSTATE:
        rainControl();
        break;
      default:
        break;
    }
    return true;
  }
  return false;
}

/*****************************************
 * Circulation Body
 ****************************************/
void loop() {
  while(1){
    judgeMessage();
//    stepper.my_step(PI_2/128, -1);
  }
}
