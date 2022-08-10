/**moto.cpp + moto.h
 * 功能：直流电机控制函数，包括基于ledc库的pwm调速功能和编码器计数值获取功能
 * 说明：接口部分通过宏定义配置，可结合实际使用管脚进行配置
 */


#include "moto.h"

void left_counter_encoder1();//读取左编码器脉冲数
void right_counter_encoder1();//读取右编码器脉冲数
void left_counter_encoder2();//读取左编码器脉冲数
void right_counter_encoder2();//读取右编码器脉冲数

/*使用ESP32LEDC库函数配置为pwm输出模式，16个通道：0-7为高速模式，8-15为低速模式 */
void moto_pwm_init()
{
  ledcSetup(8, 1000, 10);  //设置LEDC通道8频率为1000，分辨率为10位，即占空比可选0~1023
  ledcAttachPin(PWMR1, 8); //绑定通道8输出IO口为PWMR1
  ledcSetup(9, 1000, 10);  //通道9，右边电机反向通道
  ledcAttachPin(PWMR2, 9); 

  ledcSetup(10, 1000, 10);  //通道10,左边电机正向通道
  ledcAttachPin(PWML1, 10); 
  ledcSetup(11, 1000, 10);  //通道11,左边电机反向通道
  ledcAttachPin(PWML2, 11); 
}


/* 根据pwm输出电机转速，适用于驱动芯片DRV8833，正反方向根据pwm正负选择*/
void moto_pwm_set(uint8_t moto, float pwm)
{ 
  pwm = (int)pwm;  //转换为整形数据
  if(moto==LEFT){
    if(pwm<0){
      ledcWrite(10, -pwm); //设置输出PWM占空比
      ledcWrite(11, 0);
    }
    else{
      ledcWrite(11, pwm); //设置输出PWM占空比
      ledcWrite(10, 0);
    }
  }
  else{
    if(pwm<0){
      ledcWrite(9, -pwm); //设置输出PWM占空比
      ledcWrite(8, 0);
    }
    else{
      ledcWrite(9, 0);
      ledcWrite(8, pwm); //设置输出PWM占空比
    }
  }
}


volatile long Rcounter=0; // 右轮脉冲计数  该变量用于存储编码器的值，所以用类型修饰符volatile；
volatile long Lcounter=0; // 左轮脉冲计数  该变量用于存储编码器的值，所以用类型修饰符volatile；

 /***************** 编码器初始化 *****************/
void encoder_init()
{
  pinMode(RCODE1, INPUT);    
  pinMode(LCODE1, INPUT);   
  pinMode(RCODE2, INPUT);    
  pinMode(LCODE2, INPUT);   
  attachInterrupt(RCODE1, right_counter_encoder1, RISING);//设置编码器R上升沿中断
  attachInterrupt(LCODE1, left_counter_encoder1, RISING);//设置编码器L上升沿中断
  attachInterrupt(RCODE2, right_counter_encoder2, RISING);//设置编码器R上升沿中断
  attachInterrupt(LCODE2, left_counter_encoder2, RISING);//设置编码器L上升沿中断  
  interrupts();                      //打开外部中断
  Rcounter = 0;
  Lcounter = 0;
}

/************ 编码器计数，中断回调函数***********/
void right_counter_encoder1()
{
    if(digitalRead(RCODE2) == HIGH){ //判断此时的B相电平，为高则说明是前进
      Rcounter++;  //前进则计数值增加
    }
    else
      Rcounter--; //后退则计数值减少
}

void right_counter_encoder2()  //判断此时的A相电平，为低则说明是前进
{
    if(digitalRead(RCODE1) == LOW){
      Rcounter++;
    }
    else
      Rcounter--;
}

void left_counter_encoder1() // 左轮A相计数
{  
    if(digitalRead(LCODE2) == HIGH){
      Lcounter++;
    }
    else
      Lcounter--;
}


void left_counter_encoder2() // 左轮B相计数
{  
    if(digitalRead(LCODE1) == LOW){
      Lcounter++;
    }
    else
      Lcounter--;
}
  

/************ 编码器读取函数***********/
//编码器输出  
void readEncoder(double* numR, double* numL)
{
  *numR = (double)Rcounter;
  *numL = (double)Lcounter;
  //数值清零，重新计数
  Rcounter = 0;
  Lcounter = 0;
}
