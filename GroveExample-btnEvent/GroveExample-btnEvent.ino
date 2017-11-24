//. Motor driver shield- 2012 Copyright (c) Seeed Technology Inc.
// 
//  Original Author: Jimbo.we
//  Contribution: LG
//  
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

/******************************************************************************/
/*macro definitions of Rotary angle sensor and LED pin*/
#define ROTARY_ANGLE_SENSOR A2
#define ROTARY_ANGLE_SENSOR2 A3
#define ADC_REF 5//reference voltage of ADC is 5v.If the Vcc switch on the seeeduino
         //board switches to 3V3, the ADC_REF should be 3.3
#define GROVE_VCC 5//VCC of the grove interface is normally 5v
#define FULL_ANGLE 300//full value of the rotary angle is 300 degrees

const int buttonPin = 2;

int pinI1=8;//define I1 interface
int pinI2=11;//define I2 interface 
int speedpinA=9;//enable motor A
int pinI3=12;//define I3 interface 
int pinI4=13;//define I4 interface 
int speedpinB=10;//enable motor B
int spead =50;//define the spead of motor
int minspeed = 50;
int lastTime=0;
int currentTime = 0;
int buttonState = 0; 
int mode = 0; 
int currentperiod = 0;
int period = 1000;
void setup()
{
  pinMode(pinI1,OUTPUT);
  pinMode(pinI2,OUTPUT);
  pinMode(speedpinA,OUTPUT);
  pinMode(pinI3,OUTPUT);
  pinMode(pinI4,OUTPUT);
  pinMode(speedpinB,OUTPUT);
  currentTime = millis();
  lastTime = millis();
  Serial.begin(9600);
  pinsInit();
  
   attachInterrupt(0, btnClick, INPUT);//Initialize the intterrupt pin (Arduino digital pin 2)
 // pinMode(buttonPin, INPUT);     
}

void stop()//
{
     digitalWrite(speedpinA,HIGH);// Unenble the pin, to stop the motor. this should be done to avid damaging the motor. 
     digitalWrite(speedpinB,HIGH);
   //  delay(1000);
 
}

int periodF()
{
  int pdegrees;
 pdegrees = getperiod();
  Serial.println(pdegrees);
 period= map(pdegrees, 0, FULL_ANGLE, 4800, 50);
 //period = 4000 - period;
  return period;
}

int speed()
{
  
  int degrees;
  degrees = getDegree();
  spead = map(degrees, 0, FULL_ANGLE, minspeed, 255); 
  return spead;
}

void btnClick()
{
 buttonState = digitalRead(buttonPin);
if (buttonState == HIGH) {     
   mode +=1; 
    if(mode >8)
{
  mode = 0;
}
  
}
}
void loop()
{
    int degrees;
  

  } 
 int tempDegree = getDegree();
if(tempDegree != degrees)
{
  degrees = getDegree();
  spead = map(degrees, 0, FULL_ANGLE, minspeed, 250); 
}
 period = periodF();
 

Serial.println(mode);
  if(mode == 0)
  {
    stop();
  }else if(mode ==7)
{
  pulseACC();
}else if(mode ==2)
{
  ticktock();
}else if(mode ==3)
{
  sineOpposingAB();
}else if(mode ==4)
{
  rampSine();
}else if(mode ==5)
{
  constant();
}
else if(mode ==6)
{
   rampSine();
   pulseACC();
   ticktock();
   rampSine();
   ticktock();
   ticktock();
   sineOpposingAB();
   ticktock();
   ticktock();
   ticktock();
   ticktock();
}
  else if(mode ==1)
{
   steppulse();
}else if(mode ==8)
{
   consta();
}
}
 
void consta()
{
   
  int degrees;
  degrees = getDegree();
 int  speed1 = map(degrees, 0, FULL_ANGLE, 50, 255); 
    analogWrite(speedpinA,speed1);//input a simulation value to set the speed
     analogWrite(speedpinB,speed1);
     digitalWrite(pinI2,LOW);//turn DC Motor A move anticlockwise
     digitalWrite(pinI1,HIGH);
      digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
     digitalWrite(pinI3,HIGH);
  
}

void pinsInit()
{
  pinMode(ROTARY_ANGLE_SENSOR, INPUT);
   pinMode(ROTARY_ANGLE_SENSOR2, INPUT);
}

int getDegree()
{
  int sensor_value = analogRead(ROTARY_ANGLE_SENSOR);
    Serial.println(sensor_value);
  float voltage;
  voltage = (float)sensor_value*ADC_REF/1023;
  float degrees = (voltage*FULL_ANGLE)/GROVE_VCC;
  return degrees;
}

int getperiod()
{
  int sensor_value = analogRead(ROTARY_ANGLE_SENSOR2);
  float voltage;
  voltage = (float)sensor_value*ADC_REF/1023;
  float degrees = (voltage*FULL_ANGLE)/GROVE_VCC;
  return degrees;
}

void pulseACC()
{
    analogWrite(speedpinA,speed());//input a simulation value to set the speed
     analogWrite(speedpinB,speed());
     digitalWrite(pinI2,LOW);//turn DC Motor A move anticlockwise
     digitalWrite(pinI1,HIGH);
      digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
     digitalWrite(pinI3,HIGH);
     delay(periodF()*0.9);
     stop();
     delay(periodF()*0.1);
}

void steppulse()
{
  analogWrite(speedpinA,0);//input a simulation value to set the speed
  analogWrite(speedpinB,0);
  while( true)
  {
  for(int g=1; g < 9; g++)
  {
    int stepsize = (speed()-minspeed)/4;
   

    delay(periodF()*0.9);
    stop();
     delay(periodF()*0.1);
  analogWrite(speedpinA,minspeed +30+ (stepsize * g));//input a simulation value to set the speed
  analogWrite(speedpinB,minspeed + 30+(stepsize * g));
  }
//  for(int g=5; g >3; g--)
//  {
//    buttonState = digitalRead(buttonPin);
//    if (buttonState == HIGH) {     
//       return;
//      } 
//  delay(periodF()/2);
//   stop();
//   delay(40);
//  analogWrite(speedpinA,minspeed + (stepsize * g));//input a simulation value to set the speed
//  analogWrite(speedpinB,minspeed + (stepsize * g));
//  }
  }
}

void constant()
{
  int speed2 = speed() *.50;
  currentperiod =0;
 
  while(currentperiod < periodF())
  {
  int calculatedpwm = speed() * 0.50;
    double x = ((double)currentperiod )/(double)periodF();
    double sinValue = sin(x);
    int pwm = (sinValue*(calculatedpwm));
      Serial.print(sinValue);
       Serial.print(":");
        Serial.println(speed2 + pwm);
  analogWrite(speedpinA,speed2 + pwm);//input a simulation value to set the speed
     analogWrite(speedpinB,speed2 + pwm);
     digitalWrite(pinI2,LOW);//turn DC Motor A move anticlockwise
     digitalWrite(pinI1,HIGH);
      digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
     digitalWrite(pinI3,HIGH);
      delay(10);
      
   currentperiod= 10 +currentperiod;
  }
}

void sineOpposingAB()
{
  int currentperiod = 0;
 
  while((currentperiod) < periodF())
  {
  

   // Serial.println(currentperiod);
    double x = (double)currentperiod/(double)periodF();
    int min2 =minspeed;
   //  Serial.println(x);
    double sinValue = sin(x);
    // Serial.println(sinValue);
    int pwm = (sinValue*(speed() - min2)) + min2;
     //Serial.println(pwm);
     analogWrite(speedpinA,pwm);//input a simulation value to set the speed
     analogWrite(speedpinB,pwm);
     digitalWrite(pinI2,LOW);//turn DC Motor A move anticlockwise
     digitalWrite(pinI1,HIGH);
     digitalWrite(pinI4,HIGH);//turn DC Motor B move anticlockwise
     digitalWrite(pinI3,LOW);
     delay(10);
     currentperiod = currentperiod +10;
  }
 
}
void rampSine()
{

  currentperiod = 0;
 
  while((currentperiod/2) < periodF()*.97)
  {

     
   // Serial.println(currentperiod);
    double x = ((double)currentperiod /2)/(double)periodF();
   //  Serial.println(x);
    double sinValue = sin(x);
    // Serial.println(sinValue);
    int pwm = (sinValue*(speed() - minspeed)) + minspeed;
     //Serial.println(pwm);
     analogWrite(speedpinA,pwm);//input a simulation value to set the speed
     analogWrite(speedpinB,pwm);
     digitalWrite(pinI2,LOW);//turn DC Motor A move anticlockwise
     digitalWrite(pinI1,HIGH);
     digitalWrite(pinI4,HIGH);//turn DC Motor B move anticlockwise
     digitalWrite(pinI3,LOW);
     delay(10);
     currentperiod = currentperiod +10;
  }
      delay(periodF() * 0.03);
}
void ticktock()
{
    delay(periodF()/2);
     analogWrite(speedpinA,speed());//input a simulation value to set the speed
     analogWrite(speedpinB,speed());
     digitalWrite(pinI2,LOW);//turn DC Motor A move anticlockwise
     digitalWrite(pinI1,HIGH);
      digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
     digitalWrite(pinI3,HIGH);
     delay(periodF()/2);

       analogWrite(speedpinA,speed());//input a simulation value to set the speed
     analogWrite(speedpinB,speed());
     digitalWrite(pinI2,HIGH);//turn DC Motor A move anticlockwise
     digitalWrite(pinI1,LOW);
      digitalWrite(pinI4,HIGH);//turn DC Motor B move anticlockwise
     digitalWrite(pinI3,LOW);
    
      
}


