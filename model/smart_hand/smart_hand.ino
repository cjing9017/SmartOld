#include <LobotServoController.h>
#include <U8glib.h>
//#include <MsTimer2.h>
#include <NewPing.h>

/************************************
 * 超声波模块
 ************************************/
#define TRIG    9         /*超声波TRIG引脚为 9号IO*/
#define ECHO    8         /*超声波ECHO引脚为 8号IO*/

#define MAX_DISTANCE 40   /*最大检测距离为40cm*/

int gDistance;    //全局变量，用于存储超声波测得的距离

/************************************
 * TSC3200颜色检测模块
 ************************************/
#define S0 3    //S0 引脚 在3号IO口
#define S1 4    //S1 引脚 在4号IO口
#define S2 5    //S2 引脚 在5号IO口
#define S3 6    //S3 引脚 在6号IO口

#define OUT 2   //OUT 引脚 在2号引脚 //2号引脚为Arduino !!在Arduino Leonardo 上2号引脚上是1号中断，在UNO 和 mega 2560 上为0号外部中断引脚

//颜色与其对应的代号
#define RED   0     
#define GREEN 1
#define BLUE  2
#define NO_COLOR 4

int get_color = NO_COLOR; //接收串口指令，抓取特定颜色的物体

float g_SF[3];
int g_count = 0;
int g_array[3];
int g_flag = 0;

/************************************\
 * OLED模块
 ************************************/
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_0);  //实例化OLED类

//用到的字模数据
const u8g_fntpgm_uint8_t chinese_test[] U8G_SECTION(".progmem.chinese_test") = {
  0, 16, 16, 0, 254, 0, 0, 0, 0, 0, 161, 165, 0, 14, 254, 0, 0,
  //"电"
  13, 16, 32, 16, 2, 254, 4, 0, 4, 0, 4, 0, 255, 224, 132,
  32, 132, 32, 132, 32, 255, 224, 132, 32, 132, 32, 132, 32, 255, 224, 132,
  40, 4, 8, 4, 8, 3, 248,
  //"压“
  15, 15, 30, 16, 0, 254, 63, 254, 32, 0, 32, 128, 32, 128, 32,
  128, 32, 128, 47, 252, 32, 128, 32, 128, 32, 144, 32, 136, 32, 136, 64,
  128, 95, 254, 128, 0,
  //"距"
  15, 14, 28, 16, 0, 255, 125, 254, 69, 0, 69, 0, 69, 0, 125,
  252, 17, 4, 17, 4, 93, 4, 81, 4, 81, 252, 81, 0, 93, 0, 225,
  0, 1, 254,
  //"离“
  15, 16, 32, 16, 0, 254, 2, 0, 1, 0, 255, 254, 0, 0, 20,
  80, 19, 144, 20, 80, 31, 240, 1, 0, 127, 252, 66, 4, 68, 68, 79,
  228, 68, 36, 64, 20, 64, 8,
  //":"
  2, 8, 8, 16, 7, 0, 192, 192, 0, 0, 0, 0, 192, 192,
};

/************************************
 * 串口
 ************************************/
LobotServoController Controller(Serial1);  //实例化舵机控制板二次开发类,使用1号串口作为通信接口
NewPing Sonar(TRIG, ECHO, MAX_DISTANCE);      //实例化超声波测距类

/************************************
 * 舵机
 ************************************/
uint8_t servoNum = 6;//舵机的数量

uint16_t handTime[10] = {1500, 2000, 1000,  1000,  1500,  1500, 1500, 500, 1500, 1500}; //动作组运行的时间
uint16_t handId1[10]  = {550, 500, 700, 700, 700, 700, 700, 550, 700, 700}; //一号舵机参数
uint16_t handId2[10]  = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; //二号舵机参数
uint16_t handId3[10]  = {1500, 2500, 2500, 2150, 1618, 1962, 1962, 1962, 1962, 1500}; //三号舵机参数
uint16_t handId4[3][10]  = {{500,  672,  672,  780,  500,  1145,  952, 952, 952, 500 },
                           {500, 789, 672, 780, 500, 1145, 952, 952, 952, 500},
                           {500, 737, 672, 780, 500, 1145, 952, 952, 952, 500}}; //四号舵机参数
uint16_t handId5[3][10]  = {{1000, 2091, 2113, 1800, 1425, 1640, 1532, 1532, 1532, 1000},
                           {1000, 2143, 2113, 1800, 1425, 1640, 1532, 1532, 1532, 1000},
                           {1000, 2130, 2113, 1800, 1425, 1640, 1532, 1532, 1532, 1000}}; //五号舵机参数
uint16_t handId6 = 1500; //六号舵机参数

uint8_t actionIndex; // 当前运行的动作组

uint8_t handID[6] = {1, 2, 3, 4, 5, 6};

/************************************
 * OLED
 * draw 显示电压和距离值
 ************************************/
void draw() {                 //OLED绘制
  static uint32_t TimerDraw;  //静态变量，用于计时

  if (TimerDraw > millis())   //如果TimerDraw 小于 运行总毫秒数，返回，否则继续运行此函数
    return;

  u8g.firstPage();
  do {
    u8g.setFont(chinese_test);                //设置字体位chinese_test
    u8g.drawStr( 0, 20, "\xa1\xa2\xa5");      //显示字符串 电压 ：
    u8g.setFont(u8g_font_unifont);            //设置字体为u8g_font_unifont
    u8g.setPrintPos(18 * 2 + 10, 20);         //设置位置为 (46,20)
    u8g.print(Controller.batteryVoltage);     //显示电池电压
    u8g.print(" mv");                         //显示单位"mv"
    u8g.setFont(chinese_test);                //设置字体为chinese_test
    u8g.drawStr( 0, 19 + 20, "\xa3\xa4\xa5"); //显示字符串 距离：
    u8g.setFont(u8g_font_unifont);            //设置字体为u8g_font_unifont
    u8g.setPrintPos(18 * 2 + 10, 19 + 20);    //设置位置为 (46,39)
    u8g.print(gDistance);                     //显示电池电压
    u8g.print(" cm");                         //显示单位"mv"
  } while ( u8g.nextPage());

  TimerDraw = millis() + 500;                   //TimerDraw 赋值为 运行的总毫秒数 + 500，实现500毫秒后再次运行
}

/************************************
 *  LED标识
 ************************************/
bool ledON = true;            //led点亮标识，true时点亮，false熄灭
void ledFlash() {
  static uint32_t Timer;      //定义静态变量Timer， 用于计时
  if (Timer > millis())       //Timer 大于 millis（）（运行的总毫秒数）时返回，
    return;

  //Timer 小于 运行总毫秒数时继续
  if (ledON) {
    digitalWrite(13, HIGH);   //如果点亮标识true，13号IO置高电平,板上LED灯被点亮
    Timer = millis() + 20;    //Timer = 当前运行的总毫秒数 + 20  实现 20毫秒后再次运行
    ledON = false;            //置点亮标识为false
  } else {
    ledON = false;            //如果点亮标识不是true，置点亮标识为false
    digitalWrite(13, LOW);    //置13号IO为低电平，板上LED熄灭
  }
}

/************************************
 *  超声波
 *  getDistance 检测距离
 ************************************/
int getDistance() {       //获得距离
  uint16_t lEchoTime;     //变量 ，用于保存检测到的脉冲高电平时间
  lEchoTime = Sonar.ping_median(6);           //检测6次超声波，排除错误的结果
  int lDistance = Sonar.convert_cm(lEchoTime);//转换检测到的脉冲高电平时间为厘米
  /*float dis;
  for(int i = 0; i < 6; i++){
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    dis = pulseIn(ECHO, HIGH);
  }
  int lDistance = dis / 58;*/
  return lDistance;                           //返回检测到的距离
}

 /************************************
  * 接收串口发回的数据
  * updateBatteryState 更新电池电压状态
  ************************************/
void updateBatteryState() {      
  static uint32_t Timer = 0;     //静态变量Timer，用于计时

  if (Timer > millis())          //如果Timer小于 运行总毫秒数，返回，否则继续运行此函数
    return;
  Controller.getBatteryVoltage();//发送获得电池电压命令

  Timer = millis() + 1000;       //Timer 赋值为 运行的总毫秒数 + 1000，实现1000毫秒后再次运行
}

void updateDistance() {          //更新距离
  //static uint32_t Timer = 0;     //静态变量Timer，用于计时

  //if (Timer > millis())          //如果Timer小于 运行总毫秒数，返回，否则继续运行此函数
    //return;

  gDistance = getDistance();     //获得距离，并将距离保存在gDistance

  //Timer = millis() + 500;        //Timer 赋值为 运行的总毫秒数 + 500，实现500毫秒后再次运行
}

/************************************
 * 运行动作组
 * runAction 根据传入的参数
 * 运行相应的动作组
 * moveBlock 检测判断，拾取物体
 * circleCheck 检测是否有可拾取物体
 ************************************/
void runAction(int flag){
  if(flag < 0 || flag >2){
    //错误的动作编号
    return;  
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
}

int moveBlock() {
  static uint32_t Timer = 0;    //定义静态变量Timer ,用于计时
  static uint8_t  stepx = 0;    //定义静态变量stepx ,用于步骤记录

  //if (Timer > millis())         //如果Timer 小于 运行总毫秒数，返回，否则继续运行此函数
    //return;

  gDistance = getDistance();    //获得距离，并将距离保存在gDistance

  switch (stepx) {              //根据stepx 进行分支
    case 0:
      stepx = 2;
      break;
    case 1:
      static uint8_t count; //用于计数
      if(!Controller.isRunning){  //检测是否有动作组在运行
        if(count < 4){            //多次检测，取最后一次
          if(g_flag > 3){         //满足条件，完成一次颜色检测
            g_flag = 0;           //开始新一轮的检测
            count++;  
          }
        }else{
          count = 0;
          stepx = 2;
        }
      }
      break;
    case 2:
      if (gDistance > 12 && gDistance < 15) {   //如果距离大于9cm下小于15cm
        if (!Controller.isRunning) {            //如果机械手不是在运行动作组
          runAction(0);      //运行0号动作组
          Controller.isRunning = true;
          ledON = true;                         //置亮灯标识为true
          stepx = 3;
        }
      } else {
        stepx = 3;
      }//步骤赋值为1
      break;         //退出switch

    case 3:
      if (gDistance >= 15 && gDistance < 17) {   //如果距离大于16cm下小于21cm
        if (!Controller.isRunning) {            //如果机械手不是在运行动作组
          runAction(1);      //运行1号动作组
          Controller.isRunning = true;
          ledON = true;                         //置亮灯标识为true
          stepx = 4;
        }
      } else {
        stepx = 4;     //步骤赋值为2
      }
      break;         //退出switch

    case 4:
      if (gDistance >= 17 && gDistance < 19) {   //如果距离大于22cm下小于27cm
        if (!Controller.isRunning) {            //如果机械手不是在运行动作组
          runAction(2);      //运行2号动作组
          Controller.isRunning = true;
          ledON = true;                         //置亮灯标识为true
          stepx = 0;
        }
      } else {
        stepx = 0;    //步骤赋值为0，回到第一步
      }
      break;        //退出switch
    default:        //未定义步骤时默认操作，步骤数赋值0重新回到第一步
      stepx = 0;
      break;
  }
  //Timer = millis() + 50;//Timer 赋值为 运行的总毫秒数 + 50，实现50毫秒后再次运行
  if(stepx == 0)
    return 1;
  else
    return 0;
}

int circleCheck(){
  for(; handId6 < 2400; handId6 += 10){
    Controller.moveServos((uint8_t)6, (uint16_t)100, handID[0], handId1[0], handID[1], handId2[0], handID[2], handId3[0], 
                          handID[3], handId4[0], handID[4], handId5[0], handID[5], handId6);
    delay(100 + 50);
    gDistance = getDistance();
    delay(100);
    if(gDistance >= 13 && gDistance <= 18){
      while(!moveBlock());
      Controller.isRunning = false;
      return 1;
    }
  }
  for(; handId6 > 600; handId6 -= 10){
    Controller.moveServos((uint8_t)6, (uint16_t)100, handID[0], handId1[0], handID[1], handId2[0], handID[2], handId3[0], 
                          handID[3], handId4[0], handID[4], handId5[0], handID[5], handId6);
    delay(100 + 50);
    gDistance = getDistance();
    delay(100);
    if(gDistance >= 13 && gDistance <= 18){
      while(!moveBlock());
      Controller.isRunning = false;
      return 1;
    }
  }
  return 0;
}

/************************************
 * TSC3200
 * filterColor 过滤颜色
 * msTimer2CallBack 中断回调函数
 * exCount 外部中断函数
 * getColor 获取检测到的颜色
 ************************************/
/*void filterColor(uint8_t CC) {
  switch (CC) {  //根据颜色，控制S2,S3来选择传感器上滤波器颜色
    case RED:               //红色，S2低，S3低
      digitalWrite(S2, LOW);
      digitalWrite(S3, LOW);
      break; 
    case GREEN:             //绿色，S2高，S3高
      digitalWrite(S2, HIGH);
      digitalWrite(S3, HIGH);
      break;
    case BLUE:              //蓝色，S2低，S3高
      digitalWrite(S2, LOW);
      digitalWrite(S3, HIGH);
      break;
    case NO_COLOR:          //无色，S2高，S3低
      digitalWrite(S2, HIGH);
      digitalWrite(S3, LOW);
      break;
  }
}

void msTimer2CallBack();
void TSC_WB(uint8_t CC) {
  g_count = 0;      //脉冲计数清零
  g_flag++;         //步骤标识+1
  filterColor(CC);  //选择滤波器         
  MsTimer2::stop();       //三个语句用来重置Timer2
  MsTimer2::set(100, msTimer2CallBack);
  MsTimer2::start();
}

void exCount(void) { //外部中断 0号中断函数
  g_count++;         //计数
}

void msTimer2CallBack(void) {
  switch (g_flag)
  {
    case 0:
      TSC_WB(RED);      //选择让红色光线通过滤波器的模式
      break;
    case 1:
      g_array[0] = g_count * 10;  //计算并存储1s内的红光通过滤波器时，因为100ms中断一次所以要乘以10，TCS3200输出的脉冲个数
      TSC_WB(GREEN);    //选择让绿色光线通过滤波器的模式
      break;
    case 2:
      g_array[1] = g_count * 10;  //存储1s内的绿光通过滤波器时，TCS3200输出的脉冲个数
      TSC_WB(BLUE);     //选择让蓝色光线通过滤波器的模式
      break;

    case 3:
      g_array[2] = g_count * 10;   //存储1s内的蓝光通过滤波器时，TCS3200输出的脉冲个数
      TSC_WB(NO_COLOR); //选择无滤波器的模式
      break;
    default:
      g_count = 0;      //计数值清零
      break;
  }
}

uint8_t getColor() { 
  uint8_t colorArray[3];

  //循环计算三原色的值，红绿蓝，对应索引 0,1,2,被保存在数组colorArray中
  for (int i = 0; i < 3; i++) {
    colorArray[i] = g_array[i] * g_SF[i];
  }

  uint8_t temp;     //定义临时变量用于保存颜色数值
  uint8_t CC;        //临时变量用于存储颜色代号
  temp = colorArray[0];     //0号，红色的数值赋值给 temp
  CC = RED;                 //颜色是红色
  if (temp < colorArray[1]) { //如果temp中的数值小于绿色的数值
    temp = colorArray[1];   //将绿色的数值赋值temptemp
    CC = GREEN;             //颜色是绿色
  }
  if (temp < colorArray[2]) { //如果temp中的数值小于蓝色的数值
    temp = colorArray[2];   //将蓝色的数值赋值给temp
    CC = BLUE;              //颜色是蓝色
  }
    for (int i = 0; i < 3; i++)
    Serial.println(int(g_array[i] * g_SF[i]));
  //这样上面就比较出红绿蓝三种颜色中最大的数值，我们就认为数值最大的颜色就是物体的颜色
  //这样就简单地分辨出物体的是红色还是绿色或是蓝色
  return CC; //返回颜色
}*/

/************************************
 * 初始化函数
 ************************************/
void setup() {
  // put your setup code here, to run once
  Serial.begin(9600);                        //初始化0号串口
  Serial1.begin(9600);                       //初始化1号串口
  pinMode(13, OUTPUT);                        //设置板上led 13号IO口为输出

  //定义超声波引脚
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  Controller.runActionGroup(0, 1);            //运行0号动作组， 回初始位置
  delay(1000);                                //延时两秒，简单延时，确保0号动作组运行完毕
  Controller.stopActionGroup();               //停止动作运行，确保停止
  
  pinMode(S0, OUTPUT);   //配置S0引脚为输出
  pinMode(S1, OUTPUT);   //配置S1引脚为输出
  pinMode(S2, OUTPUT);   //配置S2引脚为输出
  pinMode(S3, OUTPUT);   //配置S3引脚为输出
  pinMode(OUT, INPUT);   //配置S4引脚为输出
  
  digitalWrite(S0, LOW); //S0置低，S1置高，颜色传感器内部分频 为 2%
  digitalWrite(S1, HIGH);

  //MsTimer2::set(100, msTimer2CallBack);  //2号定时器每100ms中断1次，中断中回调msTimer2CallBack函数
  //attachInterrupt(1, exCount, RISING);    //配置Arduino0号中断,中断函数exCount,RISING上升沿触发中断
  //MsTimer2::start();                     //2号定时器开始计时

  //delay(2000);
  g_SF[0] = 255.0 / g_array[0];    //红色光比例因子
  g_SF[1] = 255.0 / g_array[1] ;   //绿色光比例因子
  g_SF[2] = 255.0 / g_array[2] ;   //蓝色光比例因子

  Serial.println(g_SF[0], 5);
  Serial.println(g_SF[1], 5);
  Serial.println(g_SF[2], 5);

  //circleCheck();
}

/************************************ 
 * 循环体
 ************************************/
void loop() {
  // put your main code here, to run repeatedly:
  //updateBatteryState();       //更新电池电压状态
  //updateDistance();           //更新距离
  //Controller.receiveHandle(); //接收处理函数，从串口接收缓存中取出数据
  //draw();                     //OLED绘制
//  gDistance = getDistance();
//  delay(50);
//  Serial.println(gDistance);
  //moveBlock();                //移动物体的逻辑实现
  //ledFlash();                 //led闪灯，用于运行状态提示
  circleCheck();
//  delay(5000);
}

