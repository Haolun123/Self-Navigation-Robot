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
#include <Servo.h> 
Servo myservo;
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
int mb1 = 4;
int mb2 = 12;
int gf = 7;
int gb = 8;
int notdrop =11;
int pwm1 = 5;
int pwm2 = 6;
//int pwm3 = 11;
int severo = 9; 
int PICKUP = 0;
int navigationsensor = A1;
int countdistance;
float IRrange = 0;
float ULrange = 0;
float Navigation = 0;
int Minimaldistance(float *distance, int numberoffind){
  int i=1;
  int numberobj=1;
  float temp;
  temp=distance[1];
  while(i<numberoffind){
    i++;
    if(temp>=distance[i]){
      temp=distance[i];
      numberobj=i;
    }
  }
  return numberobj;
}
void getBothSensor()
{
  int i= 0;
  int countinrange = 0;
  int countoutrange = 0;
  float sumnavigtion = 0;
  float sumrangeIR = 0;
  float sumrangeUL = 0;
  float Navigation = 0;
  int j=0;
  while(i<8){
   float sensor_value = analogRead(ir_sensor); //read the sensor value
   float distance_cm = 4800/(sensor_value-20);
   float absdistance = abs(distance_cm);
   if (absdistance>60){
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
  //if((distanceUL<250)&&(distanceUL>2)){
  sumrangeUL = sumrangeUL + distanceUL;
  //j++;
  //}
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
void moveforward(int pwmset1,int pwmset2,int timemove,int timestop){
      digitalWrite(mf2,LOW);
      digitalWrite(mf1,LOW);
      digitalWrite(mb2,HIGH);
      digitalWrite(mb1,HIGH);
      analogWrite(pwm1,pwmset1);
      analogWrite(pwm2,pwmset2);
      delay(timemove);
      analogWrite(pwm1,0);
      analogWrite(pwm2,0);
      delay(timestop);
} 
void movebackward(int pwmset1,int pwmset2,int timemove,int timestop){
      digitalWrite(mf2,HIGH);
      digitalWrite(mf1,HIGH);
      digitalWrite(mb2,LOW);
      digitalWrite(mb1,LOW);
      analogWrite(pwm1,pwmset1);
      analogWrite(pwm2,pwmset2);
      delay(timemove);
      analogWrite(pwm1,0);
      analogWrite(pwm2,0);
      delay(timestop);
}
void turnleft(int pwmset1,int pwmset2,int timemove,int timestop){
      digitalWrite(mf2,HIGH);
      digitalWrite(mf1,LOW);
      digitalWrite(mb2,LOW);
      digitalWrite(mb1,HIGH);
      analogWrite(pwm1,pwmset1);
      analogWrite(pwm2,pwmset2);
      delay(timemove);
      analogWrite(pwm1,0);
      analogWrite(pwm2,0);
      delay(timestop);
}

void turnright(int pwmset1,int pwmset2,int timemove,int timestop){
      digitalWrite(mf2,LOW);
      digitalWrite(mf1,HIGH);
      digitalWrite(mb2,HIGH);
      digitalWrite(mb1,LOW);
      analogWrite(pwm1,pwmset1);
      analogWrite(pwm2,pwmset2);
      delay(timemove);
      analogWrite(pwm1,0);
      analogWrite(pwm2,0);
      delay(timestop);
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
 
        
        
 
        /*while(i < 1500)  // 逆时针方向循环读取XY轴每个方向的值，然后算出相应的偏移量
        {  calibRead();
            delay(10);
            i++;
        }*/
 
        
        digitalWrite(mf2, LOW);  // 
        digitalWrite(mf1, HIGH);
        digitalWrite(mb2, HIGH);
        digitalWrite(mb1, LOW); 
        analogWrite(pwm1,200);
        analogWrite(pwm2,200);
        while(i < 2000)  // 再顺时针方向方向旋转测算偏移量
        {  calibRead();
           //Serial.println(i);
           
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
 
        float headingDegrees = heading * 180/M_PI;
 
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
  analogWrite(pwm1,210);
  analogWrite(pwm2,210);
  delay(200);
  analogWrite(pwm1,0);
  analogWrite(pwm2,0);
  int a = 0;
  int b = 0;
  while(countinverse<2){ 
    getBothSensor();
    if(ULrange<=ULtemp){
      float deltadistance = ULtemp - ULrange;
      if(deltadistance< 15){
      ULtemp = ULrange;
      angletemp = readCompass();
      }
      Serial.println(angletemp);
      delay(20);
      analogWrite(pwm1,210);
      analogWrite(pwm2,210);
      delay(200);
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
      analogWrite(pwm1,210);
      analogWrite(pwm2,210);}
      else{
      digitalWrite(mf2,LOW);
      digitalWrite(mf1,HIGH);
      digitalWrite(mb2,HIGH);
      digitalWrite(mb1,LOW);
      analogWrite(pwm1,210);
      analogWrite(pwm2,210);
      }
      delay(200);
      analogWrite(pwm1,0);
      analogWrite(pwm2,0);
      countinverse += 1;
      delay(200);
    }
  }
  return angletemp;
}
int NumberOfBall = 0;
void SearchObject()
{
  float startingangle;
  //digitalWrite(gf,LOW);
  //digitalWrite(gb,HIGH);
  myservo.writeMicroseconds(300);
  switch(NumberOfBall){
    case 0:
    case 1:
    Serial.println("start");
    //turnleft(200,200,1600,200);
    startingangle = readCompass();
    
    moveforward(200,200,3500,200);    
    while(1){      
      getBothSensor();
      
      if((ULrange<60)&&(IRrange<ULrange)){
        Serial.println(ULrange);
        int countmoveforward =0;
        while(countmoveforward<3){
          moveforward(200,220,200,200);
          countmoveforward++;
        }
      }
      else if(ULrange<30){
        Serial.println("find it!");
        Serial.println(ULrange);
        break;
      }
      else {
    int leftturns = 0;
    int rightturns = 0;
    float getdirbeforescan = readCompass();
    Serial.println(getdirbeforescan);
  while(leftturns<9){
    leftturns += 1;
    turnleft(210,210,200,200);
  }
  int findobject = 0;
  float lastIRrange = 0;
  float presentIRrange = 0;
  getBothSensor();
  lastIRrange = IRrange;
  presentIRrange = IRrange;
  int findobjangle[10];
  int lostobjangle[10];
  float *objectdistance = new float[10];
  float *objectangle=new float[10];
  float ULdistancetemp,angletemp;
  ULdistancetemp=ULrange;
  while(rightturns<18){
    turnright(200,220,200,200);
    rightturns+=1;
    getBothSensor();
    Serial.println("ULrange");
    Serial.println(ULrange);
    lastIRrange = presentIRrange;
    presentIRrange = IRrange;
    Serial.println("lastIR");
    Serial.println(lastIRrange);
    Serial.println("presentIR");
    Serial.println(presentIRrange);
    //if(presentIRrange<20){
      
        if (ULrange<=ULdistancetemp){
          ULdistancetemp = ULrange;
          angletemp = readCompass();
          Serial.println("angletemp");
          Serial.println(angletemp);
        }
       if ((presentIRrange-lastIRrange)<-15){
          findobjangle[findobject+1]=rightturns;
       }
       if((presentIRrange-lastIRrange)>15){
          lostobjangle[findobject+1]=rightturns;
          if(lostobjangle[findobject+1]-findobjangle[findobject+1]<7){
            findobject+=1;
            objectdistance[findobject] = ULdistancetemp;
            objectangle[findobject] = angletemp;
          }
       }
  }
   Serial.println(findobject);
   float finaldistance;
   float finalangle;
  if(findobject>=1){
    int numberobj=Minimaldistance(objectdistance, findobject);
    Serial.println("numberobj");
    Serial.println(numberobj);
    finaldistance = objectdistance[numberobj];      
    finalangle = objectangle[numberobj];
    Serial.println("angle");
    Serial.println(finalangle);
    Serial.println("distance");
    Serial.println(finaldistance);
    if(finaldistance>75){
      Serial.println(">75");
      float testangle = readCompass();
      Serial.println( testangle );
      float deltaangle = abs(getdirbeforescan-testangle);
      while(1){
          
          if (deltaangle<3){
            Serial.println("stop");
            break;
          }
          turnleft(200,200,200,200); 
          testangle = readCompass();
          deltaangle = abs(getdirbeforescan-testangle);
          Serial.println( deltaangle );         
       }
       
    }
    else if(finaldistance>30){
      Serial.println(">30");
      float testangle = readCompass();
      Serial.println( testangle );
      float deltaangle = abs(finalangle-testangle);
      while(1){
          
          if (deltaangle<3){
            Serial.println("stop");
            break;
          }
          turnleft(200,200,200,200); 
          testangle = readCompass();
          deltaangle = abs(finalangle-testangle);
          Serial.println( deltaangle );         
       }
       
    }
    else if(finaldistance<=30) {
      float testangle = readCompass();
      Serial.println( testangle );
      float deltaangle = abs(finalangle-testangle);
      while(1){
          
          if (deltaangle<2){
            Serial.println("stop");
            break;
          }
          turnleft(200,200,200,200); 
          testangle = readCompass();
          deltaangle = abs(finalangle-testangle);
          Serial.println( deltaangle );         
       }
      Serial.println("go to get ball ");
      
    break;
    }
  }
  else {
      Serial.println("don't find");
      float testangle = readCompass();
      Serial.println( testangle );
      float deltaangle = abs(getdirbeforescan-testangle);
      while(1){
          
          if (deltaangle<3){
            Serial.println("stop");
            break;
          }
          turnleft(200,200,200,200); 
          testangle = readCompass();
          deltaangle = abs(getdirbeforescan-testangle);
          Serial.println( deltaangle );         
       }
       
    }
  int countmove=0;
  while(countmove<4){
    moveforward(200,200,200,200);
    countmove++;
    }
   }
  }
 
break;
  
    case 2:
    Serial.println("start");
    //turnleft(200,200,1600,200);
    startingangle = readCompass();
    
    moveforward(200,200,3500,200);    
    while(1){      
      getBothSensor();
      
      if((ULrange<60)&&(IRrange<ULrange)){
        Serial.println(ULrange);
        int countmoveforward =0;
        while(countmoveforward<3){
          moveforward(200,220,200,200);
          countmoveforward++;
        }
      }
      else if(ULrange<30){
        Serial.println("find it!");
        Serial.println(ULrange);
        break;
      }
      else {
    int leftturns = 0;
    int rightturns = 0;
    float getdirbeforescan = readCompass();
    Serial.println(getdirbeforescan);
  while(leftturns<9){
    leftturns += 1;
    turnleft(210,200,200,200);
  }
  int findobject = 0;
  float lastIRrange = 0;
  float presentIRrange = 0;
  getBothSensor();
  lastIRrange = IRrange;
  presentIRrange = IRrange;
  int findobjangle[10];
  int lostobjangle[10];
  float *objectdistance = new float[10];
  float *objectangle=new float[10];
  float ULdistancetemp,angletemp;
  ULdistancetemp=ULrange;
  while(rightturns<18){
    turnright(200,220,200,200);
    rightturns+=1;
    getBothSensor();
    Serial.println("ULrange");
    Serial.println(ULrange);
    lastIRrange = presentIRrange;
    presentIRrange = IRrange;
    Serial.println("lastIR");
    Serial.println(lastIRrange);
    Serial.println("presentIR");
    Serial.println(presentIRrange);
    //if(presentIRrange<20){
      
        if (ULrange<=ULdistancetemp){
          ULdistancetemp = ULrange;
          angletemp = readCompass();
          Serial.println("angletemp");
          Serial.println(angletemp);
        }
       if ((presentIRrange-lastIRrange)<-15){
          findobjangle[findobject+1]=rightturns;
       }
       if((presentIRrange-lastIRrange)>15){
          lostobjangle[findobject+1]=rightturns;
          if(lostobjangle[findobject+1]-findobjangle[findobject+1]<7){
            findobject+=1;
            objectdistance[findobject] = ULdistancetemp;
            objectangle[findobject] = angletemp;
          }
       }
  }
   Serial.println(findobject);
   float finaldistance;
   float finalangle;
  if(findobject>=1){
    int numberobj=Minimaldistance(objectdistance, findobject);
    Serial.println("numberobj");
    Serial.println(numberobj);
    finaldistance = objectdistance[numberobj];      
    finalangle = objectangle[numberobj];
    Serial.println("angle");
    Serial.println(finalangle);
    Serial.println("distance");
    Serial.println(finaldistance);
    if(finaldistance>75){
      Serial.println(">75");
      float testangle = readCompass();
      Serial.println( testangle );
      float deltaangle = abs(getdirbeforescan-testangle);
      while(1){
          
          if (deltaangle<3){
            Serial.println("stop");
            break;
          }
          turnleft(200,200,200,200); 
          testangle = readCompass();
          deltaangle = abs(getdirbeforescan-testangle);
          Serial.println( deltaangle );         
       }
       
    }
    else if(finaldistance>30){
      Serial.println(">30");
      float testangle = readCompass();
      Serial.println( testangle );
      float deltaangle = abs(finalangle-testangle);
      while(1){
          
          if (deltaangle<3){
            Serial.println("stop");
            break;
          }
          turnleft(200,200,200,200); 
          testangle = readCompass();
          deltaangle = abs(finalangle-testangle);
          Serial.println( deltaangle );         
       }
       
    }
    else if(finaldistance<=30) {
      float testangle = readCompass();
      Serial.println( testangle );
      float deltaangle = abs(finalangle-testangle);
      while(1){
          
          if (deltaangle<2){
            Serial.println("stop");
            break;
          }
          turnleft(200,200,200,200); 
          testangle = readCompass();
          deltaangle = abs(finalangle-testangle);
          Serial.println( deltaangle );         
       }
      Serial.println("go to get ball ");
      
    break;
    }
  }
  else {
      Serial.println("don't find");
      float testangle = readCompass();
      Serial.println( testangle );
      float deltaangle = abs(getdirbeforescan-testangle);
      while(1){
          
          if (deltaangle<3){
            Serial.println("stop");
            break;
          }
          turnleft(200,200,200,200); 
          testangle = readCompass();
          deltaangle = abs(getdirbeforescan-testangle);
          Serial.println( deltaangle );         
       }
       
    }
  int countmove=0;
  while(countmove<4){
    moveforward(200,200,200,200);
    countmove++;
    }
   }
  }
 
break;
  }
}

void getobject(){
  while(1)
  {
    
    getBothSensor();
    if((ULrange<30)&&(ULrange>10)){
      Serial.println("7-30");
      int counter_1 = 0;
      while(1){
        if(IRrange<ULrange){
          Serial.println("IR<UL");
        while(counter_1<1){
          moveforward(200,200,200,200);
          counter_1++;
        }
        break;
        }
        else{
          Serial.println("Search IR");
          int counter_2 = 0;
          int counter_3 = 0;
          int in_range = 0;
          int setcounter = 6;
          
          while(1){
          float centerdir = readCompass();
          while(counter_2<setcounter){
            Serial.println("go left to prepare");
            turnleft(210,200,200,200);
            counter_2++;
          }
          int counterright = setcounter*2-1;
          while(counter_3<counterright){
            Serial.println("turnning right");
            turnright(220,220,200,200);
            counter_3++;
            getBothSensor();
            if(IRrange<ULrange){
              in_range = 1;            
              break;
            }
          }
          if(in_range==0){
            float testangle = readCompass();
            float deltaangle = abs(centerdir-testangle);
            while(deltaangle>2){
              turnleft(200,200,200,200);
            }
            setcounter++;
         }
         else
         break;
          }
      }
    }
    }
    
   else if(ULrange<=10){
    //turnleft(200,200,200,200);
    //turnright(200,220,400,200);
    Serial.println("Freedom!");
    break;
  }
  else {
    Serial.println("re-search");
    int countleftmove = 0;
    int countrightmove = 0;
    int findit = 0;
    int firstposition = 0;
    float centerdir = readCompass();
    float ULtemp;
    while(countleftmove<8){
      turnleft(200,200,200,200);
      countleftmove++;
      getBothSensor();
      Serial.println(ULrange);
      if(ULrange<30){
        if(firstposition == 0){
          firstposition = 1;
          ULtemp = ULrange;
        }
        else{
          if (ULrange<=ULtemp){
            ULtemp = ULrange;
          }
          else{
            findit = 1;
            turnright(200,200,200,200);
            break;
          }
      }
    }
    }
    if(findit==0){
      float testangle = readCompass();
      float deltaangle = abs(centerdir-testangle);
      while(deltaangle>3){
        turnright(200,220,200,200);
        testangle = readCompass();
        deltaangle = abs(centerdir-testangle);
      }
      while(countrightmove<12){
      turnright(210,220,200,200);
      countrightmove++;
      getBothSensor();
      if(ULrange<30){
        if(firstposition == 0){
          firstposition = 1;
          ULtemp = ULrange;
        }
        else{
          if (ULrange<=ULtemp){
            ULtemp = ULrange;
          }
          else{
            findit = 1;
            turnleft(200,200,200,200);
            break;
          }
      }
        
      }
    }
    }
  }
  }
  
}

float Navigatetohome ()
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
  float homeangle;
  float newvertical = getverticalangle();
  Serial.println(newvertical);
  getBothSensor();
  ULY1=ULrange+10;
  NVX1=Navigation+5;
  int getpositionone = 0;
  int getpositiontwo = 0;
  int getpositionthree = 0;
  int countonevertical = 0;
  int counttwovertical = 0;
  int countthreevertical = 0;
  float readangle1,readangle2,readangle3;
  while(1){
    turnleft(220,220,200,200);
    countonevertical+=1;
    if(countonevertical>=16){
      readangle1 = getverticalangle();
      getBothSensor();
      ULX2=ULrange+10;
      NVY1=Navigation+5;
      getpositionone=1;
      break;
    }
  }
  while(1){
      turnleft(220,220,200,200);
      counttwovertical+=1;
      if(counttwovertical>=16){
      readangle2 = getverticalangle();
      getBothSensor();
      ULY2=ULrange+10;
      NVX2=Navigation+5;
      getpositiontwo=1;
      break;
    }
  }
  while(1){
    turnleft(220,220,200,200);
    countthreevertical+=1;
    if(countthreevertical>=16){
      readangle3 = getverticalangle();
      getBothSensor();
      ULX1=ULrange+10;
      NVY2=Navigation+5;
      getpositionthree=1;
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
      Yposition=217-Y1;
      homeangle=atan2(Xposition,Yposition);
      int i=0;
      int j=(homeangle+90)/2;
      while(i<j){
        turnright(200,200,200,200);
        i++;
      }
      //homeangle = -homeangle;
     /* float calculateangle,turntoangle;
      if (readangle2>readangle1){
        calculateangle = homeangle*(readangle2-readangle1)/90;
      }
      else if(readangle2<readangle1){
        calculateangle = homeangle*(readangle2-readangle1+360)/90;
      }
        turntoangle = readangle2+calculateangle;
        if(turntoangle<0){
           turntoangle=turntoangle+360;
        }
      float deltaangle=abs(readCompass()-turntoangle);
      while(deltaangle>4){
        turnright(200,200,200,200);
        deltaangle=abs(readCompass()-turntoangle);
      }*/
    }
  
    if (Y1>Y2){
      Yposition=Y2;
      homeangle=atan2(Xposition,Yposition);
      int i=0;
      int j=(homeangle+90)/2;
      while(i<j){
        turnright(200,200,200,200);
        i++;
      }
      //homeangle = -homeangle;
      /*float calculateangle,turntoangle;
      if (readangle2>readangle1){
        calculateangle = homeangle*(readangle2-readangle1)/90;
      }
      else if(readangle2<readangle1){
        calculateangle = homeangle*(readangle2-readangle1+360)/90;
      }
        turntoangle = readangle2+calculateangle;
        if(turntoangle<0){
           turntoangle=turntoangle+360;
        }
      
      float deltaangle=abs(readCompass()-turntoangle);
      while(deltaangle>4){
        turnright(200,200,200,200);
        deltaangle=abs(readCompass()-turntoangle);
      }*/
    }
  }
    if (X1>X2){
    Xposition =75-X2;
    if (Y1<Y2){
      Yposition=217-Y1;
      homeangle=atan2(Xposition,Yposition);
      homeangle = -homeangle;
      int i=0;
      int j=(homeangle+90)/2;
      while(i<j){
        turnright(200,200,200,200);
        i++;  
    }
      /*float calculateangle,turntoangle;
      if (readangle3>readangle2){
        calculateangle = homeangle*(readangle3-readangle2)/90;
      }
      else if(readangle3<readangle2){
        calculateangle = homeangle*(readangle3-readangle2+360)/90;
      }
        turntoangle = readangle2+calculateangle;
        if(turntoangle>360){
           turntoangle=turntoangle-360;
        }
      
      
      float deltaangle=abs(readCompass()-turntoangle);
      while(deltaangle>4){
        turnright(200,200,200,200);
        deltaangle=abs(readCompass()-turntoangle);
      }*/
    }
    if (Y1>Y2){
      Yposition=Y2;
      homeangle=atan2(Xposition,Yposition);
      homeangle = -homeangle;
      int i=0;
      int j=(homeangle+90)/2;
      while(i<j){
        turnright(200,200,200,200);
        i++;  
    }
      /*float calculateangle,turntoangle;
      if (readangle3>readangle2){
        calculateangle = homeangle*(readangle3-readangle2)/90;
      }
      else if(readangle3<readangle2){
        calculateangle = homeangle*(readangle3-readangle2+360)/90;
      }
        turntoangle = readangle2+calculateangle;
        if(turntoangle>360){
           turntoangle=turntoangle-360;
        }
      
      
      float deltaangle=abs(readCompass()-turntoangle);
      while(deltaangle>4){
        turnright(200,200,200,200);
        deltaangle=abs(readCompass()-turntoangle);
      }*/
    }
  }
  return homeangle;
}
  
void setup()
{
  Serial.begin(9600); 
  Wire.begin(); 
  myservo.attach(9);
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
  //pinMode(pwm3,OUTPUT);
  compass.SetScale(1.3); 
  compass.SetMeasurementMode(Measurement_Continuous);
  
}
void loop()
{ 
  //analogWrite(pwm3,255);
  delay(2000);
  myservo.writeMicroseconds(300);
  delay(2000);
  //digitalWrite(gf,LOW);
  //digitalWrite(gb,HIGH);
  calibration();
  float verticalangle = getverticalangle(); 
  int ballnumber = 0; 
  while(ballnumber<3){
    if(ballnumber==0){
      float newvertical = verticalangle+90;
      if(newvertical>360)
      newvertical = newvertical - 360;
      float anglenow = readCompass();
      float deltaangle = abs(anglenow-newvertical);
      while(deltaangle>3){
       turnleft(200,200,200,200);
       anglenow = readCompass();
       deltaangle = abs(anglenow-newvertical);
  }  
    }
    else if (ballnumber == 1){
      float newvertical = getverticalangle();
      newvertical = newvertical+180;
      if(newvertical>360)
      newvertical = newvertical - 360;
      float anglenow = readCompass();
      float deltaangle = abs(anglenow-newvertical);
      while(deltaangle>3){
       turnleft(200,200,200,200);
       anglenow = readCompass();
       deltaangle = abs(anglenow-newvertical);
      }
    }
    else{
      float newvertical = getverticalangle();
      newvertical = newvertical+190;
      if(newvertical>360)
      newvertical = newvertical - 360;
      float anglenow = readCompass();
      float deltaangle = abs(anglenow-newvertical);
      while(deltaangle>3){
       turnleft(200,200,200,200);
       anglenow = readCompass();
       deltaangle = abs(anglenow-newvertical);
  }
    }
  
  SearchObject();
  digitalWrite(gf,LOW);
  digitalWrite(gb,HIGH);
  getobject();
  moveforward(200,200,300,200);
  moveforward(200,200,200,200);
  moveforward(200,200,300,300);
  myservo.writeMicroseconds(2200);
  delay(3000);
  
  digitalWrite(gf,HIGH);
  digitalWrite(gb,LOW);
  delay(2000);
  myservo.writeMicroseconds(300);
  delay(2000);
  int waitingtime = 0;
  int dropvalue;
  while(waitingtime<10){
   dropvalue = digitalRead(notdrop);
   delay(500);
   if(dropvalue==HIGH)
   break;
  }
  if(dropvalue==HIGH){
    movebackward(200,205,500,500);
    digitalWrite(gf,LOW);
    digitalWrite(gb,LOW);
    SearchObject();
  digitalWrite(gf,LOW);
  digitalWrite(gb,HIGH);
  getobject();
  moveforward(200,200,300,200);
  moveforward(200,200,200,200);
  moveforward(200,200,300,300);
  myservo.writeMicroseconds(2200);
  delay(3000);
  
  digitalWrite(gf,HIGH);
  digitalWrite(gb,LOW);
  delay(2000);
  myservo.writeMicroseconds(300);
  delay(2000);
  }
 float homeangle = Navigatetohome();
 int movement=0;
 while(movement<15){
   moveforward(200,205,400,200);
   movement++;
 }
 getBothSensor();
 while(ULrange>5){
   float station = getverticalangle();
   if(homeangle<0){
     int j=homeangle/2;
     int i = 0;
     while(i<j){
       turnleft(200,200,200,500);
     }
   }
   else{
     int j=homeangle/2;
     int i = 0;
     while(i<j){
       turnright(200,200,200,500);
     }
   }
   moveforward(200,205,800,500);
    getBothSensor();
 }
 moveforward(200,205,1400,500);
 digitalWrite(gf,LOW);
 digitalWrite(gb,HIGH);
 ballnumber++;
 movebackward(200,205,1000,200);
 delay(2000);
 digitalWrite(gf,LOW);
 digitalWrite(gb,LOW);
  }
}

