/**mpu6050.cpp + mpu6050.h
 * 功能：模块初始化，直接读取mpu6050角度
 * 说明：基于MPU6050_tockn库，仅进行了简单封装
 *   
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <Arduino.h>
#include <MPU6050_tockn.h>

void mpu6050_init();//mpu6050初始化
void mpu6050_get_angel(double* getAngel,char Axis); //获取角度，参数例如'X'

#endif /* MPU6050_H_ */
