#include <ArduPID.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <String.h>
#include <Ticker.h>


MPU6050 mpu6050(Wire);

/************电机参数宏**************/
#define PWML2 21
#define PWML1 27
#define PWMR2 33
#define PWMR1 32
#define RCODE2 26
#define RCODE1 25
#define LCODE2 22
#define LCODE1 23
/**********************************/

#define LEFT        0x02
#define RIGHT       0x03

ArduPID angPID;

double setpoint = -86;
double input;
double output;
double p = 60.0;
double i = 0;
double d = 0.0;

void moto_init(){  //方向测试用，电机初始化
  pinMode(PWMR1,OUTPUT);
  pinMode(PWMR2,OUTPUT);
  pinMode(PWML1,OUTPUT);
  pinMode(PWML2,OUTPUT);
  /*
  digitalWrite(PWML1,LOW);
  digitalWrite(PWML2,HIGH);
  digitalWrite(PWMR1,HIGH);
  digitalWrite(PWMR2,LOW);
  */
}

/***************** 定时中断参数 *****************/
Ticker timer1;  // 中断函数
int interrupt_time = 50; // 中断时间
int timer_flag=0;               //定时器标志；
 
/***************** 编码器引脚及参数 *****************/
volatile long Rcounter=0; // 右轮脉冲计数  该变量用于存储编码器的值，所以用类型修饰符volatile；
volatile long Lcounter=0; // 左轮脉冲计数  该变量用于存储编码器的值，所以用类型修饰符volatile；
long  Rcountbuff=0;
long  Lcountbuff=0;

int direcL=1;   //电机方向标志
int direcR=1;   //电机方向标志


void moto_pwm_init(){
  ledcSetup(8, 1000, 10);  //设置LEDC通道8频率为1000，分辨率为10位，即占空比可选0~1023
  ledcAttachPin(PWMR1, 8); //设置LEDC通道8在IO上输出
  ledcSetup(9, 1000, 10);  
  ledcAttachPin(PWMR2, 9); 

  ledcSetup(10, 1000, 10);  
  ledcAttachPin(PWML1, 10); 
  ledcSetup(11, 1000, 10);  
  ledcAttachPin(PWML2, 11); 
}

void moto_pwm_set(uint8_t moto, int pwm){
  if(moto==LEFT){
    if(pwm<0){
      direcL=-1;
      ledcWrite(10, -pwm); //设置输出PWM占空比
      ledcWrite(11, 0);
    }
    else{
      direcL=1;
      ledcWrite(11, pwm); //设置输出PWM占空比
      ledcWrite(10, 0);
    }
  }
  else{
    if(pwm<0){
      direcR=-1;
      ledcWrite(9, -pwm); //设置输出PWM占空比
      ledcWrite(8, 0);
    }
    else{
      direcR=1;
      ledcWrite(9, 0);
      ledcWrite(8, pwm); //设置输出PWM占空比
    }
  }
}



void setup() {
  // put your setup code here, to run once:
  moto_init();

  Serial.begin(115200);

  //mpu6050
  Wire.begin(18,19 ,400000);// (sda,scl)Set I2C frequency to 400kHz
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  digitalWrite(PWML1,LOW);
  digitalWrite(PWML2,HIGH);
  digitalWrite(PWMR1,HIGH);
  digitalWrite(PWMR2,LOW);
  delay(1000);

  digitalWrite(PWML2,LOW);
  digitalWrite(PWML1,HIGH);
  digitalWrite(PWMR2,HIGH);
  digitalWrite(PWMR1,LOW);
  delay(1000);
  */


  /*
  Serial.print("R : ");
  Serial.println(Rcounter);
  Serial.print("L : ");
  Serial.println(Lcounter);
  delay(50);
  */

  mpu6050.update();
  input = mpu6050.getAngleX();
  Serial.print(input);
  Serial.print(',');
  input = mpu6050.getAngleY();
  Serial.print(input);
  Serial.print(',');
  input = mpu6050.getAngleZ();
  Serial.print(input);
  Serial.println(',');
  delay(50);
 
}


//定时器中断处理函数,其功能主要为了输出编码器得到的数据
void timerIsr(){
   timer_flag=1;  //定时时间达到标志      
   readEncoder();   // 编码器
}

//编码器输出  
void readEncoder(){
  Rcountbuff=Rcounter;
  Lcountbuff=Lcounter;
//数值清零，重新计数
  //Rcounter = 0;
  //Lcounter = 0;
}


// 编码器计数，中断回调函数
void right_counter_encoder(){
    Rcounter++;
}
void left_counter_encoder(){  // //左轮计数
    Lcounter++;
}
