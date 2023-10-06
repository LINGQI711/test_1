#include "comm.h"   //传感器数据读取
#include "motor.h"  //电机控制
#include "Adafruit_NeoPixel.h"  //彩色灯珠驱动
#include <math.h>

#define PIN            4
#define NUMPIXELS      2
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800); 

float Kp = 30, Ki = 0.02, Kd = 15;  //PID算法参数
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;  
float previous_error = 0, previous_I = 0;
int   left_motor_speed = 0;
int   right_motor_speed = 0;
int   motor_flag = 0;

void setup() {
    shift_reg_init(); //传感器初始化
    motor_init();     //电机初始化
    pixels.begin();   //彩色LED初始化
    Serial.begin(9600); //配置串口
}

void calculate_pid()//PID算法
{
  P = error;
  I = I + error;
  D = error - previous_error;

  PID_value = (Kp * P)+ (Ki * I) + (Kd * D) ;
  PID_value = constrain(PID_value, -140, 140);
  previous_error = error;
}

int read_ir_values() //计算小车偏移量
{
      // if((sensor.ir_mid==1)&&(sensor.ir_left_1==1)&&(sensor.ir_right_1==1)&&(sensor.ir_right_2==1))
      //  {error=10;
      //  motor_step(0,0);
      //  delay(6000);

      //  }
      if((sensor.ir_mid==1)&&(sensor.ir_left_1==1)&&(sensor.ir_left_2==1)&&(sensor.ir_right_1==1)) //  1 1 1 1 1 1 1 
    {  
        motor_step(0, 0);
        while(1);
        //delay(2000);
        //
    }            
                       //停车
    else if(sensor.ir_left_1==1)     //左转小
        error=2;
     else if(sensor.ir_left_2==1)
        error=6;             //左转小
     else if(sensor.ir_left_2==1&&sensor.ir_left_3==1)//左转大
        error=18;   
    else if(sensor.ir_left_3==1)      //左转大
        error=40;
     else if(sensor.ir_right_1==1)  //右转小 
        error=-2;
     else if (sensor.ir_right_3==1)   //右转大
        error=-40;
     else if(sensor.ir_right_2==1)
        error=-6;//右转小
     else if((sensor.ir_right_2==1)&&(sensor.ir_right_3==1))
        error=-18;//右转大
     else if(sensor.ir_mid==1) //直行
         error=0;
       //  else if((sensor.ir_mid==0)&&(sensor.ir_left_1==0)&&(sensor.ir_left_2==0)&&(sensor.ir_left_3==0)&&(sensor.ir_right_1==0)&&(sensor.ir_right_2==0)&&(sensor.ir_right_3==0))
       //  error=100;
     return error;         
  }

void motor_control()//设置电机速度
{
  
  //计算每个电机的速度
 if(fabs(error-40) < 1e-5){
     left_motor_speed = -(-165+ PID_value);
     right_motor_speed = -165  - PID_value;
 }
 else if(fabs(error+40) < 1e-5){
     left_motor_speed = -165+ PID_value;
     right_motor_speed = 165  + PID_value;
 }
 else {
 left_motor_speed = -225+ PID_value;
 right_motor_speed = -225  - PID_value;
 }
  
  // if(fabs(error-40) < 1e-5){
  //     left_motor_speed = -(-265+ PID_value);
  //     right_motor_speed = -265  - PID_value;
  // }
  // else if(fabs(error+40) < 1e-5){
  //     left_motor_speed = -265+ PID_value;
  //     right_motor_speed = 265  + PID_value;
  // }
  // else {
  // left_motor_speed = -315+ PID_value;
  // right_motor_speed = -315  - PID_value;
  // }

  // if (left_motor_speed > 255) left_motor_speed = 255;
  // if (right_motor_speed > 255) right_motor_speed = 255;

  // if ()

  motor_step(left_motor_speed/2, right_motor_speed/2);
  // motor_step(left_motor_speed/2, right_motor_speed/2);
  // motor_set_PWM(left_motor_speed, right_motor_speed);
}

void led()//控制LED
{
  
    char r0,r1,g0,g1,b0,b1;
    reload_shift_reg(); //刷新传感器数据
    if(sensor.ir_left_3) //左侧第三个(最外面)红外传感器测到黑线或悬空
        g0 = 100; //设定第1个灯珠亮红色
    else
        g0 = 0;
    if(sensor.ir_left_2) r0 = 100; else r0 = 0; //g红色  r绿色  b蓝色
    if(sensor.ir_left_1) b0 = 100; else b0 = 0;
    if(sensor.ir_right_1) b1 = 100; else b1 = 0;
    if(sensor.ir_right_2) r1 = 100; else r1 = 0;
    if(sensor.ir_right_3) g1 = 100; else g1 = 0;
    if(sensor.ir_mid) r0=r1=g0=g1=b0=b1 = 100;
        
    pixels.setPixelColor(0, pixels.Color(r0,g0,b0)); //设定第一个灯珠颜色RGB(0~255)
    pixels.setPixelColor(1, pixels.Color(r1,g1,b1)); //设定第二个灯珠颜色
    pixels.show();  //显示设定好的颜色
}


void loop() {
  while(!(sensor.switcher_back_left || sensor.switcher_back_right))
  {
    
                //Serial.print(left_pulse);
          //Serial.println(right_pulse);
      led(); 
      reload_shift_reg(); //消抖
  } 


      GoStraight(-200);
      while(1)
      {
        led();  
      reload_shift_reg();//刷新寄存器
      read_ir_values(); //刷新转弯角度
      calculate_pid();  //PID计算
      motor_control();  //电机控制

        
      }

}
