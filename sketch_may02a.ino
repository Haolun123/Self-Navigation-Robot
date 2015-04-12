#include <WProgram.h> 
/*
HMC5883L.h - Header file for the HMC5883L Triple Axis Magnetometer Arduino Library.
Copyright (C) 2011 Love Electronics (loveelectronics.co.uk)

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

 WARNING: THE HMC5883L IS NOT IDENTICAL TO THE HMC5883!
 Datasheet for HMC5883L:
 http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/HMC5883L_3-Axis_Digital_Compass_IC.pdf

*/


#include <inttypes.h>
#include <Wire.h>

#define HMC5883L_Address 0x1E
#define ConfigurationRegisterA 0x00
#define ConfigurationRegisterB 0x01
#define ModeRegister 0x02
#define DataRegisterBegin 0x03

#define Measurement_Continuous 0x00
#define Measurement_SingleShot 0x01
#define Measurement_Idle 0x03

#define ErrorCode_1 "Entered scale was not valid, valid gauss values are: 0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1"
#define ErrorCode_1_Num 1

struct MagnetometerScaled
{
	float XAxis;
	float YAxis;
	float ZAxis;
};

struct MagnetometerRaw
{
	int XAxis;
	int YAxis;
	int ZAxis;
};

class HMC5883L
{
	public:
	  HMC5883L();

	  MagnetometerRaw ReadRawAxis();
	  MagnetometerScaled ReadScaledAxis();
  
	  int SetMeasurementMode(uint8_t mode);
	  int SetScale(float gauss);

	  char* GetErrorText(int errorCode);

	protected:
	  void Write(int address, int byte);
	  uint8_t* Read(int address, int length);

	private:
	  float m_Scale;
};



HMC5883L::HMC5883L()
{
  m_Scale = 1;
}

MagnetometerRaw HMC5883L::ReadRawAxis()
{
  uint8_t* buffer = Read(DataRegisterBegin, 6);
  MagnetometerRaw raw = MagnetometerRaw();
  raw.XAxis = (buffer[0] << 8) | buffer[1];
  raw.ZAxis = (buffer[2] << 8) | buffer[3];
  raw.YAxis = (buffer[4] << 8) | buffer[5];
  return raw;
}

MagnetometerScaled HMC5883L::ReadScaledAxis()
{
  MagnetometerRaw raw = ReadRawAxis();
  MagnetometerScaled scaled = MagnetometerScaled();
  scaled.XAxis = raw.XAxis * m_Scale;
  scaled.ZAxis = raw.ZAxis * m_Scale;
  scaled.YAxis = raw.YAxis * m_Scale;
  return scaled;
}

int HMC5883L::SetScale(float gauss)
{
	uint8_t regValue = 0x00;
	if(gauss == 0.88)
	{
		regValue = 0x00;
		m_Scale = 0.73;
	}
	else if(gauss == 1.3)
	{
		regValue = 0x01;
		m_Scale = 0.92;
	}
	else if(gauss == 1.9)
	{
		regValue = 0x02;
		m_Scale = 1.22;
	}
	else if(gauss == 2.5)
	{
		regValue = 0x03;
		m_Scale = 1.52;
	}
	else if(gauss == 4.0)
	{
		regValue = 0x04;
		m_Scale = 2.27;
	}
	else if(gauss == 4.7)
	{
		regValue = 0x05;
		m_Scale = 2.56;
	}
	else if(gauss == 5.6)
	{
		regValue = 0x06;
		m_Scale = 3.03;
	}
	else if(gauss == 8.1)
	{
		regValue = 0x07;
		m_Scale = 4.35;
	}
	else
		return ErrorCode_1_Num;
	
	// Setting is in the top 3 bits of the register.
	regValue = regValue << 5;
	Write(ConfigurationRegisterB, regValue);
}

int HMC5883L::SetMeasurementMode(uint8_t mode)
{
	Write(ModeRegister, mode);
}

void HMC5883L::Write(int address, int data)
{
  Wire.beginTransmission(HMC5883L_Address);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t* HMC5883L::Read(int address, int length)
{
  Wire.beginTransmission(HMC5883L_Address);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.beginTransmission(HMC5883L_Address);
  Wire.requestFrom(HMC5883L_Address, length);

  uint8_t buffer[length];
  if(Wire.available() == length)
  {
	  for(uint8_t i = 0; i < length; i++)
	  {
		  buffer[i] = Wire.read();
	  }
  }
  Wire.endTransmission();

  return buffer;
}

char* HMC5883L::GetErrorText(int errorCode)
{
	if(ErrorCode_1_Num == 1)
		return ErrorCode_1;
	
	return "Error not defined.";
}

int ir_sensor = A0;
const int echopin=3; // 
const int trigpin=2; // 
int mf1 = 10; // motor1 forward
int mf2 = 13; // motor2 forward
int mb1 = 11;
int mb2 = 12;
int gf = 7;
int gb = 8;
int notdrop = 4;
int pwm1 = 5;
int pwm2 = 6;
int severo = 9; 
int PICKUP = 0;
int navigationsensor = A1;
int countdistance;
float IRrange = 0;
float ULrange = 0;
float Navigation = 0;
void getBothSensor()
{
  int i= 0;
  int countinrange = 0;
  int countoutrange = 0;
  float sumnavigtion = 0;
  float sumrangeIR = 0;
  float sumrangeUL = 0;
  float Navigation = 0;
  while(i<8){
   float sensor_value = analogRead(ir_sensor); //read the sensor value
   float distance_cm = 4800/(sensor_value-20);
   if (abs(distance_cm)>80){
     countoutrange +=1;
   }
   else
  {
    sumrangeIR = sumrangeIR + distance_cm;
    countinrange +=1;
  }  
  float navigation_value = analogRead(navigationsensor); //read the sensor value
  float navigation_cm =9462/(navigation_value - 16.92);
  sumnavigtion = sumnavigtion + navigation_cm;
  digitalWrite(trigpin,LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin,LOW); //
  float distanceUL = pulseIn(echopin,HIGH);//
  distanceUL = distanceUL/58.0;//
  sumrangeUL = sumrangeUL + distanceUL;
  //Serial.println(distanceUL);
  //Serial.println(distance_cm);
  i=i+1;
  delay(60);
  }
  if (countoutrange>3){
    IRrange = 255;
  }
  else
  {
    IRrange = sumrangeIR/countinrange;
  }
  ULrange = sumrangeUL/8;
  Navigation =  sumnavigtion/8;
  //Serial.println(ULrange);
  //Serial.println(IRrange);
}
HMC5883L compass;
float X_max = 0.0;  //X轴最大读数
float X_min = 0.0;  //X轴最小读数
 
float Y_max = 0.0;  //Y轴最大读数
float Y_min = 0.0;  //Y轴最小读数
 
float X_offset = 0.0;  //X轴偏移
float Y_offset = 0.0;  //Y轴偏移
 
void calibRead() //指南针读数
{
        MagnetometerScaled scaled2 = compass.ReadScaledAxis();
 
        if (  (abs(scaled2.XAxis) < 500) && (abs(scaled2.YAxis) < 500) && (abs(scaled2.ZAxis) < 500)) //这行对于我很重要，能过滤掉很多干扰，增加校准的精度。
        {  //下面几行就是读取到每个轴的最大值和最小值，然后算出相应的偏移量
            if(scaled2.XAxis > X_max) {X_max = scaled2.XAxis;}
            if(scaled2.YAxis > Y_max) {Y_max = scaled2.YAxis;}
 
            if(scaled2.XAxis < X_min) {X_min = scaled2.XAxis;}
            if(scaled2.YAxis < Y_min) {Y_min = scaled2.YAxis;}
 
            X_offset = (X_max + X_min) / 2.0;
            Y_offset = (Y_max + Y_min) / 2.0;
        }
}
 
void calibration()
{
        int i = 0;
       /* digitalWrite(mf1, LOW);  // 定义转弯方向，对于我的小车来说是左转，逆时针方向
        digitalWrite(mf2, HIGH);
        digitalWrite(mb1, HIGH);
        digitalWrite(mb2, LOW);*/      
 
        analogWrite(pwm1, 200);  // 启动马达开始转动
        analogWrite(pwm2, 200);
 
        /*while(i < 1500)  // 逆时针方向循环读取XY轴每个方向的值，然后算出相应的偏移量
        {  calibRead();
            delay(10);
            i++;
        }*/
 
        
        digitalWrite(mf2, LOW);  // 
        digitalWrite(mf1, HIGH);
        digitalWrite(mb2, HIGH);
        digitalWrite(mb1, LOW); 
 
        while(i < 1000)  // 再顺时针方向方向旋转测算偏移量
        {  calibRead();
            delay(10);
            i++;
        }
 
        analogWrite(pwm1, 0);  // 停止小车，校准完成
        analogWrite(pwm2, 0);
        delay(500);
}
 
float readCompass()  //得到偏移了就可以读数了
{
        MagnetometerScaled scaled = compass.ReadScaledAxis();
 
        float heading = atan2(scaled.YAxis - Y_offset, scaled.XAxis - X_offset);  // 读数的时候直接减掉偏移量就可以了
 
        //float declinationAngle = 36.36/1000.0;  [这几行是考虑磁偏角的，除非真的是想
        //heading = heading + declinationAngle;    找地理北极，不然对方向判断用处不大]
 
        if(heading < 0)  heading += 2*PI;
        if(heading > 2*PI)  heading -= 2*PI;
 
        float headingDegrees = heading * 200/M_PI;
 
        return headingDegrees;
}
float getverticalangle()
{
  getBothSensor();
  delay(500);
  getBothSensor();
  float ULtemp = ULrange;
  float angletemp = readCompass();
  int countinverse = 0;
  digitalWrite(mf2, LOW);  // 
  digitalWrite(mf1, HIGH);
  digitalWrite(mb2, HIGH);
  digitalWrite(mb1, LOW);
  analogWrite(pwm1,200);
  analogWrite(pwm2,200);
  delay(500);
  analogWrite(pwm1,0);
  analogWrite(pwm2,0);
  int a = 0;
  int b = 0;
  while(countinverse<2){ 
    getBothSensor();
    if(ULrange<=ULtemp){
      ULtemp = ULrange;
      angletemp = readCompass();
      Serial.println(angletemp);
      delay(20);
      analogWrite(pwm1,200);
      analogWrite(pwm2,200);
      delay(500);
      analogWrite(pwm1,0);
      analogWrite(pwm2,0);
      delay(200);
    }
    else{
      if(countinverse==0){ 
      digitalWrite(mf2,HIGH);
      digitalWrite(mf1,LOW);
      digitalWrite(mb2,LOW);
      digitalWrite(mb1,HIGH);
      analogWrite(pwm1,200);
      analogWrite(pwm2,200);}
      else{
      digitalWrite(mf2,LOW);
      digitalWrite(mf1,HIGH);
      digitalWrite(mb2,HIGH);
      digitalWrite(mb1,LOW);
      analogWrite(pwm1,200);
      analogWrite(pwm2,200);
      }
      delay(500);
      analogWrite(pwm1,0);
      analogWrite(pwm2,0);
      countinverse += 1;
      delay(500);
    }
  }
  return angletemp;
}
int NumberOfBall = 0;
/*void SearchObject()
{
  float startingangle;
  digitalWrite(gf,LOW);
  digitalWrite(gb,HIGH);
  analogWrite(severo,16);
  switch(NumberOfBall){
    case 0:
    analogWrite(pwm1,200);
    analogWrite(pwm2,200);
    digitalWrite(mf2,HIGH);
    digitalWrite(mf1,LOW);
    digitalWrite(mb2,LOW);
    digitalWrite(mb1,HIGH);
    delay(1600);
    startingangle = readCompass();
    while(1){
      analogWrite(pwm1,240);
      analogWrite(pwm2,240);
      digitalWrite(mf2,LOW);
      digitalWrite(mf1,LOW);
      digitalWrite(mb1,HIGH);
      digitalWrite(mb2,HIGH);
      delay(1500);
      analogWrite(pwm1,0);
      analogWrite(pwm2,0);
      getBothSensor();
    int leftturns = 0;
    int rightturns = 0;
  while(leftturns<9){
    leftturns += 1;
    analogWrite(pwm1,200);
    analogWrite(pwm2,200);
    digitalWrite(mf2,HIGH);
    digitalWrite(mf1,LOW);
    digitalWrite(mb2,LOW);
    digitalWrite(mb1,HIGH);
    delay(200);
    analogWrite(pwm1,0);
    analogWrite(pwm2,0);
    getBothSensor();
  if(ULrange>60&&IRrange>80)
    {
    digitalWrite(mf2,LOW);
    digitalWrite(mf1,LOW);
    digitalWrite(mb1,HIGH);
    digitalWrite(mb2,HIGH);
    analogWrite(pwm1,150);
    analogWrite(pwm2,150);
    delay(500);
    }
  if(ULrange<20&&IRrange<60){
    if(ULrange<5) 
    {
      digitalWrite(mf2,HIGH);
      digitalWrite(mf1,HIGH);
      digitalWrite(mb1,LOW);
      digitalWrite(mb2,LOW);
      analogWrite(pwm1,90);
      analogWrite(pwm2,90);
      delay(200);
      digitalWrite(mb1,LOW);
      digitalWrite(mf1,LOW);
      digitalWrite(mb2,LOW);
      digitalWrite(mf2,LOW);
      digitalWrite(gf,HIGH);
      digitalWrite(gb,LOW);
      PICKUP = 1;
      break;
    }
   if(ULrange<20&&IRrange>70)
    {
    digitalWrite(mf2,HIGH);
    digitalWrite(mf1,HIGH);
    digitalWrite(mb1,LOW);
    digitalWrite(mb2,LOW);
    analogWrite(pwm1,150);
    analogWrite(pwm2,150);
    delay(200);
    }
  }
  else{   
      digitalWrite(mf2,LOW);
      digitalWrite(mf1,LOW);
      digitalWrite(mb1,HIGH);
      digitalWrite(mb2,HIGH);
    }
  delay(60);
  }
  delay(1000);
      digitalWrite(mb1,LOW);
      digitalWrite(mf1,HIGH);
      digitalWrite(mb2,LOW);
      digitalWrite(mf2,HIGH);
      analogWrite(pwm1,100);
      analogWrite(pwm2,100);
  digitalWrite(mf2,HIGH);
  digitalWrite(mf1,HIGH);
  digitalWrite(mb1,LOW);
  digitalWrite(mb2,LOW);
  analogWrite(pwm1,150);
  analogWrite(pwm2,150);
  countdistance = 0;
  while(PICKUP==1){
      delay(500);
      countdistance++;
      if(countdistance>10){break;}
    
  }  
  digitalWrite(mf2,LOW);
  digitalWrite(mf1,LOW);
  digitalWrite(mb1,LOW);
  digitalWrite(mb2,LOW);
  digitalWrite(gf,LOW);
  digitalWrite(gb,HIGH);
  delay(1000);
  while(1){
  }   
}*/
void Navigatetohome (float verticalangle)
{
  float Xposition = 0;
  float Yposition = 0;
  float ULX1 = 0;
  float ULX2 = 0;
  float ULY1 = 0;
  float ULY2 = 0;
  float NVX1 = 0;
  float NVX2 = 0;
  float NVY1 = 0;
  float NVY2 = 0;
  while(abs(readCompass()-verticalangle)>10){
    digitalWrite(mf2, LOW);  // 
    digitalWrite(mf1, HIGH);
    digitalWrite(mb2, HIGH);
    digitalWrite(mb1, LOW);
    analogWrite(pwm1,200);
    analogWrite(pwm2,200);
    delay(200);
    analogWrite(pwm1,0);
    analogWrite(pwm2,0);
    delay(200);
  }
  float newvertical = getverticalangle();
  Serial.println(newvertical);
  getBothSensor();
  ULX1=ULrange;
  NVY1=Navigation;
  int getpositions = 1;
  while(1){
    digitalWrite(mf2, HIGH);  // 
    digitalWrite(mf1, LOW);
    digitalWrite(mb2, LOW);
    digitalWrite(mb1, HIGH);
    analogWrite(pwm1,200);
    analogWrite(pwm2,200);
    delay(200);
    analogWrite(pwm1,0);
    analogWrite(pwm2,0);
    delay(200);
    Serial.println(readCompass());
    if(abs(readCompass()-(newvertical+90))<10){
      getBothSensor();
      ULY1=ULrange;
      NVX1=Navigation;
      getpositions+=1;
    }
    if(abs(readCompass()-(newvertical+180))<10){
      getBothSensor();
      ULX2=ULrange;
      NVY2=Navigation;
      getpositions+=1;
    }
    if(abs(readCompass()-(newvertical+270))<10){
      getBothSensor();
      ULY2=ULrange;
      NVX2=Navigation;
      getpositions+=1;
    }
    if(getpositions>=4){
      break;
    }
  }
  float X1=(ULX1+NVX1)/2;
  float Y1=(ULY1+NVY1)/2;
  float X2=(ULX2+NVX2)/2;
  float Y2=(ULY2+NVY2)/2;
  
  if (X1<X2){
    Xposition =75-X1;
    if (Y1<Y2){
      Yposition=Y1;
      float homeangle=atan2(Xposition,Yposition);
      while(abs(readCompass()-homeangle)>10){
        digitalWrite(mf2, LOW);  // 
        digitalWrite(mf1, HIGH);
        digitalWrite(mb2, HIGH);
        digitalWrite(mb1, LOW);
        analogWrite(pwm1,200);
        analogWrite(pwm2,200);
        delay(200);
        analogWrite(pwm1,0);
        analogWrite(pwm2,0);
        delay(200);
      }
    }
  
    if (Y1>Y2){
      Yposition=Y2;
      float homeangle=atan2(Xposition,Yposition);
      while(abs(readCompass()-homeangle)>10){
        digitalWrite(mf2, LOW);  // 
        digitalWrite(mf1, HIGH);
        digitalWrite(mb2, HIGH);
        digitalWrite(mb1, LOW);
        analogWrite(pwm1,200);
        analogWrite(pwm2,200);
        delay(200);
        analogWrite(pwm1,0);
        analogWrite(pwm2,0);
        delay(200);
      }
    }
  }
    if (X1>X2){
    Xposition =75-X2;
    if (Y1<Y2){
      Yposition=Y1;
      float homeangle=atan2(Xposition,Yposition);
      while(abs(readCompass()-homeangle)>10){
        digitalWrite(mf2, HIGH);  // 
        digitalWrite(mf1, LOW);
        digitalWrite(mb2, LOW);
        digitalWrite(mb1, HIGH);
        analogWrite(pwm1,200);
        analogWrite(pwm2,200);
        delay(200);
        analogWrite(pwm1,0);
        analogWrite(pwm2,0);
        delay(200);
      }
    }
    if (Y1>Y2){
      Yposition=Y2;
      float homeangle=atan2(Xposition,Yposition);
      while(abs(readCompass()-homeangle)>10){
        digitalWrite(mf2, HIGH);  // 
        digitalWrite(mf1, LOW);
        digitalWrite(mb2, LOW);
        digitalWrite(mb1, HIGH);
        analogWrite(pwm1,200);
        analogWrite(pwm2,200);
        delay(200);
        analogWrite(pwm1,0);
        analogWrite(pwm2,0);
        delay(200);
      }
    }
  }
}
  
void setup()
{
  Serial.begin(9600);  
  pinMode(echopin,INPUT); //
  pinMode(trigpin,OUTPUT);//
  pinMode(mf1,OUTPUT);
  pinMode(mb1,OUTPUT);
  pinMode(mf2,OUTPUT);
  pinMode(mb2,OUTPUT);
  pinMode(gf,OUTPUT);
  pinMode(gb,OUTPUT);
  pinMode(pwm1,OUTPUT);
  pinMode(pwm2,OUTPUT);
  
}
void loop()
{
  calibration();
  float verticalangle = getverticalangle();
  
  delay(3000);
   digitalWrite(mf2, HIGH);  // 
   digitalWrite(mf1, LOW);
   digitalWrite(mb2, LOW);
   digitalWrite(mb1, HIGH);
   analogWrite(pwm1,200);
   analogWrite(pwm2,200);
   delay(1800);
   analogWrite(pwm1,0);
   analogWrite(pwm2,0);
   delay(200);
   int time1=0;
   while(time1<6){
   analogWrite(pwm1,200);
      analogWrite(pwm2,200);
      digitalWrite(mf2,LOW);
      digitalWrite(mf1,LOW);
      digitalWrite(mb1,HIGH);
      digitalWrite(mb2,HIGH);
      delay(500);
      analogWrite(pwm1,0);
      analogWrite(pwm2,0);
      delay(200);
      time1+=1;
   }
      Navigatetohome(verticalangle);
      digitalWrite(mf2,LOW);
      digitalWrite(mf1,LOW);
      digitalWrite(mb1,HIGH);
      digitalWrite(mb2,HIGH);
      analogWrite(pwm1,200);
      analogWrite(pwm2,200);
      delay(1500);
      analogWrite(pwm1,0);
      analogWrite(pwm2,0);
      delay(500);
      Navigatetohome(verticalangle);
      digitalWrite(mf2,LOW);
      digitalWrite(mf1,LOW);
      digitalWrite(mb1,HIGH);
      digitalWrite(mb2,HIGH);
      analogWrite(pwm1,200);
      analogWrite(pwm2,200);
      delay(1500);
      analogWrite(pwm1,0);
      analogWrite(pwm2,0);
      delay(500);
      while(1){}
  //SearchObject();
  
}
