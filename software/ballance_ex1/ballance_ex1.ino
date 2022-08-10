/* ESP32平衡车miniBot
 *  
 * 平台：Arduino
 * Version: v1.1.0
 * Update: 2022-08-01
 * Author: Leannnus @bilibili.com & oshwhub.com
 * Link: https://oshwhub.com/leannn/minibot
 * 
**/


#include "PID.h"
#include <ArduinoJson.h>
#include <MPU6050_tockn.h>

#include "myBlueTooth.h"
#include "moto.h"
#include "mpu6050.h"

#define DEBUG 1  //串口输出调试信息
#define BTMODE 1 //蓝牙控制模式：0为调参模式，1为运动模式

/*************PID参数定义***********/
ArduPID pid_ang, pid_spd, pid_turn;

/*****角度环****/
double ang_set = -1.0;
double ang_in, ang_out;
double ang_p = 98.0;
double ang_i = 0.0;
double ang_d = 170.0;

/*****速度环*****/
double spd_set = 0.0;
double spdL_in, spdR_in, spdSum, spd_out;
double spd_p = 24.0;
double spd_i = 0.12;
double spd_d = 0.0;


double spd_turn = 0;  //差速转向值


char* recBuff; 
StaticJsonDocument<200> doc;  //JSON缓冲段分配
  
void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  BT_init();//蓝牙设备初始化--"miniBot"
  Serial.println("The device started, now you can pair it with bluetooth!");

  moto_pwm_init();//电机接口初始化

  encoder_init();//编码器接口初始化

  mpu6050_init();//mpu6050初始化

  
  pid_ang.begin(&ang_in, &ang_out, &ang_set, ang_p, ang_i, ang_d);  //初始化角度环PID参数

  pid_spd.begin(&spdSum, &spd_out, &spd_set, spd_p, spd_i, spd_d);  //初始化速度环PID参数
  
  pid_ang.setOutputLimits(-1000.0, 1000.0);  //设置PID输出PWM的幅度限制
  pid_ang.setWindUpLimits(-50.0,50.0);       //设置PID积分限幅

  pid_spd.setOutputLimits(-1000.0, 1000.0);
  pid_spd.setWindUpLimits(-100.0,100.0);

  moto_pwm_set(LEFT, 0);
  moto_pwm_set(RIGHT, 0);

}

void loop() 
{
    static float pwm_out;
    static int BTmode = 0;  //蓝牙运行模式
    static String strBuff;
    static String substr;  //字符串解析
    static uint16_t t=0, dt = 50;  //时间测量
    if((millis() - t)>dt){   //以dt(ms)为周期循环运行

      mpu6050_get_angel(&ang_in,'Y');  //获取Y轴角度值
      readEncoder(&spdL_in, &spdR_in); //获取编码器值

      spdSum = spdL_in + spdR_in;  //左右编码值累加
  
      pid_ang.compute();  //前倾PWM输出负值，后倾输出正值
      pid_spd.compute(); 
    
      pwm_out = -ang_out - spd_out ;  //角度环为负反馈，速度环为正反馈，符号由实际测试得到
                                      //一般来说，设置极性是改变的pid参数的符号，这里直接改变的是输出结果，达到的效果类似
      
      moto_pwm_set(LEFT, (pwm_out+spd_turn));   //差速转向，psd_turn大于0右转，小于0左转
      moto_pwm_set(RIGHT, (pwm_out-spd_turn));

      if(abs(ang_in - ang_set)>40){  //偏角过大时则电机停止转动
          moto_pwm_set(LEFT, 0);
          moto_pwm_set(RIGHT, 0);
          pid_ang.reset();
          pid_spd.reset();
      }
      
#if DEBUG
      pid_ang.debug(&Serial, "angle", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                       PRINT_OUTPUT   | // in the Serial plotter
                                       PRINT_SETPOINT |
                                       PRINT_BIAS     |
                                       PRINT_P        |
                                       PRINT_I        |
                                       PRINT_D);  
#endif
         
/* 蓝牙发送参数指令，使用JSON格式进行打包和解析，字符串格式："{"cmd":1,"data":[p,i,d]}" ,  (p、i、d为常数)
 * cmd=1：角度环pid设置
 * cmd=2：速度环pid设置
 * cmd=3：转向环pid(暂未添加)
 * cmd=4：角度平衡值, 发送格式： "{"cmd":4,"data":[angle]}" , (angle为常数)
 * cmd=5：速度值，同角度
 * cmd=6：转向值，同上
**/
      recBuff = getDataBT();
#if BTMODE == 0 //蓝牙调参模式
      DeserializationError error = deserializeJson(doc, recBuff); //Json字符解析
      if(!error){
        int cmd = doc["cmd"];
        switch(cmd){
          case 1:
            ang_p = doc["data"][0];
            ang_i = doc["data"][1];
            ang_d = doc["data"][2];
            pid_ang.begin(&ang_in, &ang_out, &ang_set, ang_p, ang_i, ang_d);
            sendStringBT("Set angle pid successful!\r\n");
            break;
    
         case 2:
            spd_p = doc["data"][0];
            spd_i = doc["data"][1];
            spd_d = doc["data"][2];
                        
            pid_spd.begin(&spdSum, &spd_out, &spd_set, spd_p, spd_i, spd_d);  //初始化速度环PID参数
            
            sendStringBT("Set speed pid successful!\r\n");
            break;
    
          case 3:
            break;
          case 4:
            ang_set = doc["data"][0];
            sendStringBT("Set angle point successful!\r\n");
            break;
          case 5:
            spd_set = doc["data"][0];
            sendStringBT("Set speed point successful!\r\n");
            break;
          case 6:
            spd_turn = doc["data"][0];
            if(spd_turn>0)
              sendStringBT("Turn right!\r\n");
            if(spd_turn<0)
              sendStringBT("Turn left!\r\n");
          break;
          default:break;
        }    
      }
      
#else /**摇杆控制模式，接收蓝牙数据格式:"{X10Y10}"
        *手机上位机APP见工程链接附件
        *原程序为横屏，此处使用竖屏操作摇杆，因此需要X、Y坐标转换，均取负号
       **/
      strBuff=recBuff;  

      substr = strBuff.substring(strBuff.indexOf("X")+1,strBuff.indexOf("Y"));  //读取速度值:X和Y中间的值
      int temp_spd=0;
      
      temp_spd = -(substr.toInt());  //调节运动速度，取相反值翻转方向
      spd_set = constrain(temp_spd, -20, 20);  //速度限制
      
      Serial.print(substr); //调试输出
      Serial.print(",");
      
      substr = strBuff.substring(strBuff.indexOf("Y")+1,strBuff.indexOf("}"));  //读取转向值
      spd_turn = - substr.toInt();  //调节转向角，同样取相反值
      
      Serial.println(substr);  //调试用
#endif    
      
      t = millis();
    }
}

/*
  char json[100];
  int i =0;
  while(Serial.available() > 0){
    json[i++]=Serial.read();
  }
  deserializeJson(doc, json); //Json字符解析

*/
