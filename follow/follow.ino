
#define sensor_LEFT  26//左外传感器状态 引脚PD_3
#define sensor_left  27//左传感器状态 引脚PE_1
#define sensor_mid   28//中传感器状态 引脚PE_2
#define sensor_right 29//右传感器状态 引脚PE_3
#define sensor_RIGHT 25//右外传感器状态 引脚PD_2

#define motor_r_phb_sensor 10//右轮编码器读取引脚 PA_7 B相
#define motor_r_pha_sensor 9 //右轮编码器读取引脚 PA_6 A相
#define motor_l_pha_sensor 6 //左轮编码器读取引脚 PE_5 B相
#define motor_l_phb_sensor 5 //左轮编码器读取引脚 PE_4 A相

#define pwml 23//左轮PWM输出端口 PD0
#define pwmr 24//右轮PWM输出端口 PD1

#define led1 2//状态指示LED PB_5
#define ringing 7//状态指示蜂鸣器 PB_4

#define button1 35//状态设置切换按钮 PC_6
#define button2 34//状态设置切换按钮 PC_7
#define button3 31//状态设置切换按钮 PF_4

#define KP 7.2 //位置PID比例参数KP 1.2
#define KD 0.6 //位置PID微分参数KD 1.0

#define kpl 1.0 //左轮速度PID比例参数kp 
#define kil 0.8 //左轮速度PID积分参数ki 0.8
#define kdl 0.5 //左轮速度PID微分参数kd 0.6

#define kpr 1.0 //右轮速度PID比例参数kp 
#define kir 0.8 //右轮速度PID积分参数ki 0.8 1.0
#define kdr 0.5 //右轮速度PID微分参数kd 0.6 1.0

#define LED 2      //LED灯PB5
#define BUZZER 7   //蜂鸣器PB4
#define KEY1 35     //PC6
#define KEY2 34     //PC7
#define KEY3 31     //PF4

#define standard_speed 0.272 //标准速度，可设置为变量通过按钮更改

volatile long motor_l = 0;//左轮编码器读取量
long last_motor_l = 0;//确保编码器在变化
volatile long motor_r = 0;//右轮编码器读取量
long last_motor_r = 0;//确保编码器在变化

unsigned long time1 = 0;//记录开发板运行时间保证时间差
unsigned long time2 = 0;//记录开发板运行时间保证时间差


int gray_sensor=0;//灰度传感器状态

float sensor_err = 0,sensor_last_err=0;//PWM位置加权参数
float pwm=0;


float speed_l_err=0,speed_l_last_err=0;//左轮PWM速度加权偏差
float speed_r_err=0,speed_r_last_err=0;//右轮PWM速度加权偏差

float speed_l = 0, speed_r = 0;//左轮速度,右轮速度
float speed_l_l=0,speed_r_r=0;//两轮速度输出值


void read_l_pha_sensor();
void read_l_phb_sensor();
void read_r_pha_sensor();
void read_r_phb_sensor();//编码器读取

int read_sensor();//灰度传感读取

void motor_drv();//电机输出，速度＋位置

void read_speed();//读取电机速度

void setup() 
{
pinMode(sensor_LEFT,INPUT);
pinMode(sensor_left,INPUT);
pinMode(sensor_mid,INPUT);
pinMode(sensor_right,INPUT);
pinMode(sensor_RIGHT,INPUT);

pinMode(motor_l_phb_sensor,INPUT);
pinMode(motor_l_pha_sensor,INPUT);
pinMode(motor_r_phb_sensor,INPUT);
pinMode(motor_r_pha_sensor,INPUT);
pinMode(pwml,OUTPUT);
pinMode(pwmr,OUTPUT);

pinMode(LED,OUTPUT);
pinMode(BUZZER,OUTPUT);

time1 = micros();//初始须记录一次

attachInterrupt(digitalPinToInterrupt(motor_l_phb_sensor),read_l_phb_sensor,CHANGE);
attachInterrupt(digitalPinToInterrupt(motor_l_pha_sensor),read_l_pha_sensor,CHANGE);
attachInterrupt(digitalPinToInterrupt(motor_r_phb_sensor),read_r_phb_sensor,CHANGE);
attachInterrupt(digitalPinToInterrupt(motor_r_pha_sensor),read_r_pha_sensor,CHANGE);

Serial.begin(115200);
Serial1.begin(9600);
Serial2.begin(9600);
}


char g='a';
void loop() 
{
  Serial.println(g);
  Serial.println(speed_l_l);
   if(Serial2.available()>=1)
     {
      g=Serial2.read();
      Serial.println(g);
      Serial.println(speed_l_l);
   if(g=='r')
     {
       read_speed();//得出speed_l和speed_r两个速度
       motor_drv();//驱动电机，分两部分，速度和位置
     } 
if(g=='s')
{
  {
    read_speed();//得出speed_l和speed_r两个速度
    motor_drv();//驱动电机，分两部分，速度和位置
} 
  }
}
}

/* 
正转：Arising触发 a高 b低 Afalling触发 a低b高
     Brising触发 a高 b高 Bfalling触发 a低b低 
     A超前B90°
     
反转：Arising触发 a高b高 Afalling触发a低b低
      Brising触发 a低b高 Bfalling触发a高b低
      A滞后B90°
基于左轮测出 
*/
//单电机，单相，一圈，脉冲计数24 
void read_l_pha_sensor()
{ 
  if(digitalRead(motor_l_pha_sensor)!=digitalRead(motor_l_phb_sensor))
    motor_l+=1;
  else   
    motor_l-=1;
}
void read_l_phb_sensor()
{
 if(digitalRead(motor_l_pha_sensor)==digitalRead(motor_l_phb_sensor))
    motor_l+=1;
  else
    motor_l-=1; 
}
void read_r_pha_sensor()
{ 
  if(digitalRead(motor_r_pha_sensor)==digitalRead(motor_r_phb_sensor))
    motor_r+=1;
  else   
    motor_r-=1;
}
void read_r_phb_sensor()
{
 if(digitalRead(motor_r_pha_sensor)!=digitalRead(motor_r_phb_sensor))
    motor_r+=1;
  else
    motor_r-=1; 
}




int read_sensor()
{/*
  编码器
  
  */
  gray_sensor=(((((((digitalRead(sensor_LEFT)<<1)|digitalRead(sensor_left))<<1)|digitalRead(sensor_mid))<<1)|digitalRead(sensor_right))<<1)|digitalRead(sensor_RIGHT);
  switch(gray_sensor)
  {
    case 0x10:return -10;//10000
    case 0x18:return -8;//11000
    case 0x08:return -7;//01000
    case 0x14:return 0;//10100
    case 0x0C:return -3;//01100
    case 0x04:return 0;//00100
    case 0x06:return 3; //00110
    case 0x02:return 7;//00010
    case 0x03:return 8;//00011
    case 0x01:return 10;//00001

        case 0x1E:return 0;//11100
        case 0x0E:return 0;//01110
        case 0x07:return 0;//00111
    default:return -1;   
   }
}

void motor_drv()
{   
  sensor_err=read_sensor();//位置pid函数
//  pwm=constrain(KP*sensor_err+(sensor_err-sensor_last_err)*KD,(int)(-standard_speed*255*0.05),(int)(standard_speed*255*0.05));//约束pwm不超过标准速度的±5%
  pwm=KP*sensor_err+(sensor_err-sensor_last_err)*KD;
  sensor_last_err = sensor_err;

if(Serial1.available()>=1)
  {if(Serial1.read()==1)
    {
      speed_l_l=0.25+pwm;
      speed_r_r=0.25-pwm;
     
    }
  if(Serial1.read()==0)
 {
      speed_l_l=0.35+pwm;
      speed_r_r=0.35-pwm;
  }
 }

  speed_l_l = speed_l+(speed_l_err*kpl)+(speed_l_err+speed_l_last_err)*kil+(speed_l_err-speed_l_last_err)*kdl;
  speed_r_r = speed_r+(speed_r_err*kpr)+(speed_r_err+speed_r_last_err)*kir+(speed_r_err-speed_r_last_err)*kdr;
  
     if(g=='s')
     speed_l_l=speed_r_r=0;   
  

  speed_l_err=(standard_speed+pwm/255.0 )-speed_l;//左轮pid速度函数,
  //实际左轮加权速度
  analogWrite(pwml,(int)(constrain((map(speed_l_l*100,0,100,0,255)),0,255)));//左轮pwm输出 +pwm
  speed_l_last_err=speed_l_err;
  delay(1);


  speed_r_err=(standard_speed-pwm/255.0)-speed_r;//右轮pid速度函数,
  ////实际右轮加权速度
  analogWrite(pwmr,(int)(constrain((map(speed_r_r*100,0,100,0,255)),0,255)));//右轮PWM输出 -pwm
  speed_r_last_err=speed_r_err;
  delay(1);
  
    Serial.print("speed_l_l:"); 
    Serial.println(speed_l_err); 
    Serial.print("speed_r_r:"); 
    Serial.println(speed_r_err);
}
void read_speed()
{ 
  time2=micros();
  if(time2-time1>20000)
  { 
    time1 = time2;
    speed_l = ((((motor_l-last_motor_l)/1040.)*0.14)/0.02); //车轮周长20cm，车轮一圈脉冲四倍频计数2496，（脉冲差/2496*0.2m）/（20ms=0.02）=speed  
    speed_r = ((((motor_r-last_motor_r)/1040.)*0.14)/0.02);//车轮周长20cm，车轮一圈脉冲四倍频计数2496，（脉冲差/2496*0.2m）/（20ms=0.02）=speed 
    last_motor_l=motor_l;
    last_motor_r=motor_r;
    Serial.print("left:"); 
    Serial.println(speed_l); 
    Serial.print("right:"); 
    Serial.println(speed_r); 
   }  
}