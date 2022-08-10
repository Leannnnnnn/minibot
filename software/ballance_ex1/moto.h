/**moto.cpp + moto.h
 * 功能：直流电机控制函数，包括基于ledc库的pwm调速功能和编码器计数值获取功能
 * 说明：接口部分通过宏定义配置，可结合实际使用管脚进行配置
 */


#ifndef MOTO_H_
#define MOTO_H_

#include<Arduino.h>

/************电机PWM控制接口定义***************/
#define PWML1 21
#define PWML2 27
#define PWMR1 33
#define PWMR2 32
/********************************************/

/************电机编码器接口定义（AB相）**********/
#define LCODE1 26
#define LCODE2 25
#define RCODE1 23
#define RCODE2 22
/********************************************/

/*********运行方向宏定义（直观便于查看）**********/
#define LEFT        0x02
#define RIGHT       0x03

/*使用ESP32LEDC库函数配置为pwm输出模式，16个通道：0-7为高速模式，8-15为低速模式 */
void moto_pwm_init();

/* 根据pwm输出电机转速，适用于驱动芯片DRV8833，正反方向根据pwm正负选择*/
void moto_pwm_set(uint8_t moto, float pwm);

/*编码器初始化*/
void encoder_init();

/*编码器输出*/  
void readEncoder(double* numR, double* numL);

#endif/* MOTO_H_ */
