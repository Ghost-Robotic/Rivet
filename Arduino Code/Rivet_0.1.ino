#include<math.h>
#include<Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define lenArm1 15  //cm
#define lenArm2 12  //cm


//COORDINATE RESTRICTIONS
//#define XminRestriction 0
//#define XmaxRestriction 250

//#define YminRestriction 0
//#define YmaxRestriction 200

//#define ZminRestriction -100
//#define ZmaxRestriction 100

//#define EminRestriction 0
//#define EmaxRestriction 5

//DEFINE INPUT PINS
#define XcoordPin A0
#define YcoordPin A3
#define ZcoordPin A1
#define EffectorPin A2

//SET MICROSECONDS FOR SERVO
//#define MINservo0 470  //1375
//#define MAXservo0 2280 

//#define MINservo1 520   //1337.5
//#define MAXservo1 2155

//#define MINservo2 2355 //1467.5
//#define MAXservo2 580

//#define MINservo3 500// 13625
//#define MAXservo3 2225

//#define MINservo4 2225
//#define MAXservo4 500

//#define MINservo5 540
//#define MAXservo5 2400

//#define MINservo6 540
//#define MAXservo6 2400

//#define MINservo7 540
//#define MAXservo7 2400

/////////////////////////////////////////////////////////////
#define SERVO_FREQ 50

#define deadzoneVal 50
#define maxChange 0.5
#define Echange 0.5

#define USMIN  540 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600

unsigned long previousMillis = 0;  
const long interval = 1000;

float joint_0_Angle;
float joint_1_Angle;
float joint_2_Angle;
float joint_3_Angle;
float joint_4_Angle;

int servo0_Ms0;
int servo1_Ms1;
int servo2_Ms1;
int servo3_Ms2;
int servo4_Ms2;
int servo5_Ms4;
int gripper_Ms;

float Servo0Smoothed;
float Servo1Smoothed;
float Servo2Smoothed;
float Servo3Smoothed;
float Servo4Smoothed;

float Servo0Prev = 1375;
float Servo1Prev = 1337.5;
float Servo2Prev = 1467.5;
float Servo3Prev = 13625;
float Servo4Prev = 13625;


float xCoord = 12; //cm
float yCoord = 15; //cm
float zCoord = 0; //cm
float gripperDistance;

float sqX = xCoord*xCoord;
float sqY = yCoord*yCoord;
float sqZ = zCoord*zCoord;

int XpotMiddle;
int YpotMiddle;
int ZpotMiddle;
int EpotMiddle;

int XpotVal;
int YpotVal;
int ZpotVal;
int EpotVal;


//=====================================================================================================================================================================================================================================

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  joystickCalibrate();
}

void joystickCalibrate() {
  XpotMiddle = analogRead(XcoordPin);
  YpotMiddle = analogRead(YcoordPin);
  ZpotMiddle = analogRead(ZcoordPin);
  EpotMiddle = analogRead(EffectorPin);

  Serial.println("");
  Serial.println(XpotMiddle);
  Serial.print(" , ");
  Serial.print(YpotMiddle);
  Serial.print(" , ");
  Serial.print(ZpotMiddle);
  Serial.print(" , "); 
  Serial.print(EpotMiddle);
  Serial.println("READY!"); 
}

//=====================================================================================================================================================================================================================================

void loop() {
  XpotVal = analogRead(A0);
  YpotVal = analogRead(A3);
  ZpotVal = analogRead(A1);
  EpotVal = analogRead(A2);
  delay(10);
  findCoords();
  delay(10);
  findAngles();
  delay(10);
  servoMove();
  delay(5);
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    //displayInfo();
  }
  delay(5);
}


void displayInfo(){
  
  //Serial.println("");
  //Serial.println(joint_0_Angle);
  //Serial.println(joint_1_Angle);
  //Serial.println(joint_2_Angle);
  //Serial.println(joint_3_Angle);
  //Serial.println(joint_4_Angle);

  Serial.println(" ");

  Serial.println("   y  ,      z  ,      x  ,  gripper");
  
  Serial.print(xCoord);
  Serial.print("cm , ");
  Serial.print(yCoord);
  Serial.print("cm , ");
  Serial.print(zCoord);
  Serial.print("cm , ");
  Serial.print(gripperDistance);
  Serial.print("cm");
}

//=====================================================================================================================================================================================================================================

void findCoords()
{
  /*
  XpotVal = analogRead(A0);
  YpotVal = analogRead(A3);
  ZpotVal = analogRead(A1);
  EpotVal = analogRead(A4);
  */
//_____________________________________________
  if (XpotVal > XpotMiddle + deadzoneVal){
    xCoord -= maxChange;
  }
  else if (XpotVal < XpotMiddle - deadzoneVal){
    xCoord += maxChange;
  }
//_____________________________________________
  if (YpotVal > YpotMiddle + deadzoneVal){
    yCoord -= maxChange;
  }
  else if (YpotVal < YpotMiddle - deadzoneVal){
    yCoord += maxChange;
  }
//_____________________________________________
  if (ZpotVal < ZpotMiddle - deadzoneVal){
    zCoord += maxChange;
  }
  else if (ZpotVal > ZpotMiddle + deadzoneVal){
    zCoord -= maxChange;
  }
//_____________________________________________
  if (EpotVal > EpotMiddle + deadzoneVal){
    gripperDistance -= Echange;
  }
  else if (EpotVal < EpotMiddle - deadzoneVal){
    gripperDistance += Echange;
  }
//_____________________________________________
  xCoord = constrain(xCoord, 0, 200);
  yCoord = constrain(yCoord, 0, 200);
  //zCoord = constrain(zCoord, 0, 200);
  gripperDistance = constrain(gripperDistance, 0, 2.82);
  if (ZpotVal < ZpotMiddle - deadzoneVal){
    zCoord += maxChange;
  }
  else if (ZpotVal > ZpotMiddle + deadzoneVal){
    zCoord -= maxChange;
  }
}

//=====================================================================================================================================================================================================================================

void findAngles()
{
  float sqX = xCoord*xCoord;
  float sqY = yCoord*yCoord;
  float sqZ = zCoord*zCoord;

  joint_0_Angle = atan2(zCoord,xCoord);
  joint_0_Angle = joint_0_Angle *180 / M_PI + 90;

  joint_1_Angle = (atan(yCoord/sqrt(sqX+sqZ)) + acos( ( (sqX + sqZ + sqY) +sq(lenArm1) - sq(lenArm2) )/ (2* sqrt(sqX + sqY + sqZ) *lenArm1))) *180 / M_PI;  //  atan(y^2 / x^2) + acos(x^2+y^2 / 2*root(x^2+y^2)*armLen)   *180 * Pi

  joint_2_Angle = (acos( ( sq(lenArm1) + sq(lenArm2) - sqX - sqY - sqZ) / (2*lenArm1*lenArm2)) *180 / M_PI) ;
  //Serial.println(sq(lenArm1) + sq(lenArm2));

  //joint_3_Angle = joint_1_Angle+joint_2_Angle-90;
  //joint_4_Angle = joint_0_Angle;

  if (isnan(joint_0_Angle),isnan(joint_1_Angle),isnan(joint_2_Angle)){
    digitalWrite(12, HIGH);
    Serial.println("STOP");
  }
}

//=====================================================================================================================================================================================================================================

void servoMove()
{
  if (joint_0_Angle<=180, joint_0_Angle>=0, joint_1_Angle<=180, joint_1_Angle>=0, joint_2_Angle<=180, joint_2_Angle>=0) {
    servo0_Ms0 = map(joint_0_Angle, 0, 180, 2280, 470);  
    servo1_Ms1 = map(joint_1_Angle, 0, 180, 520, 2155);
    servo2_Ms1 = map(joint_1_Angle, 0, 180, 2355, 580); //Inverse
    servo3_Ms2 = map(joint_2_Angle, 0, 180, 500, 2225);
    servo4_Ms2 = map(joint_2_Angle, 0, 180, 2225, 500); //Inverse
    servo5_Ms4 = map(joint_0_Angle, 0, 180, 540, 2400);
    gripper_Ms  = map(gripperDistance, 0, 2.82, 825, 1300);  //Emin= 0, Emax= 80
/*
    Servo0Smoothed = (servo0_Ms0 * 0.05) + (Servo0Prev*0.95);
    Servo1Smoothed = (servo1_Ms1 * 0.05) + (Servo1Prev*0.95);
    Servo2Smoothed = (servo2_Ms1 * 0.05) + (Servo2Prev*0.95);
    Servo3Smoothed = (servo3_Ms2 * 0.05) + (Servo3Prev*0.95);
    Servo4Smoothed = (servo4_Ms2 * 0.05) + (Servo4Prev*0.95);

    //Serial.println(servo0_Ms0);
   // Serial.println(servo1_Ms1);
    //Serial.println(servo2_Ms1);
    //Serial.println(servo3_Ms2);

    Servo0Prev = Servo0Smoothed;
    Servo1Prev = Servo1Smoothed;
    Servo2Prev = Servo2Smoothed;
    Servo3Prev = Servo3Smoothed;
    Servo4Prev = Servo4Smoothed;
*/
    //Serial.println("Smoothed");
   // Serial.println(Servo0Smoothed);
    //Serial.println(Servo1Smoothed);
    //Serial.println(Servo2Smoothed);
    //Serial.println(Servo3Smoothed);
    //Serial.println("");
    delay(5);

    pwm.writeMicroseconds(4, servo0_Ms0);
    pwm.writeMicroseconds(5, servo1_Ms1); 
    pwm.writeMicroseconds(6, servo2_Ms1);
    pwm.writeMicroseconds(7, servo4_Ms2);
    delay(30);
    pwm.writeMicroseconds(3, servo3_Ms2);
    pwm.writeMicroseconds(0, servo5_Ms4);
    pwm.writeMicroseconds(2, gripper_Ms);
  }
}
