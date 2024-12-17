#include <Arduino.h>
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

// pin allocations
const int scl = D1, sda = D2, led = D3, interrupt = D4, button = D5;

const unsigned long gesturePeriod = 3000;
unsigned long gestureCheckStart;

bool checkGestures = false;

// MPU6050 related stuff
const int accelRange = 4, gyroRange = 250;
const float accelScale = 32767 / accelRange, gyroScale = 32767 / gyroRange;

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// for readability sake
float &yaw = ypr[0];
float &pitch = ypr[1];
float &roll = ypr[2];
int16_t &x = aaReal.x;
int16_t &y = aaReal.y;
int16_t &z = aaReal.z;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

unsigned long lastScreenRefresh = 0;


// function declarations
void updateAccelGyro();
void reportValues();


class Gesture {
  protected : long startGoal;
  protected : long moveGoal;
  protected : long startLimit;
  protected : unsigned long moveLimit;
  protected : unsigned long endGoal;
  protected : unsigned long endLimit;
  unsigned long lastMove;
  long trueTime;
  long falseTime;
  bool started;
  bool moving;
  public : String command;

  public : virtual bool StartStage() = 0;
  public : virtual bool Movement() = 0;
  public : virtual bool EndStage() = 0;

  Gesture(long startGoal, long moveGoal, long startLimit, unsigned long moveLimit,
    unsigned long endGoal, unsigned long endLimit, const String& command)
    : startGoal(startGoal), moveGoal(moveGoal), startLimit(startLimit),
    moveLimit(moveLimit), endGoal(endGoal), endLimit(endLimit), command(command),
    lastMove(0), trueTime(0), falseTime(0), started(false), moving(false) {
    }
  bool CheckMovement(){
    if(!started){
      if(StartStage()){
        trueTime += (millis() - lastMove);
        if(trueTime > startGoal){
          started = true;
          falseTime = 0;
          trueTime = 0;
          Serial.println("detected start");
          }
        }
      else{
        falseTime += (millis() - lastMove);
        if(falseTime > startLimit){
          falseTime = 0;
          trueTime = 0;
          }
        }
      lastMove = millis();
    }
    else if(!moving){
      if(Movement()){
        trueTime += (millis() - lastMove);
        if(trueTime > moveGoal){
          moving = true;
          falseTime = 0;
          trueTime = 0;
          Serial.println("detected move");
          }
        }
      else{
        falseTime += (millis() - lastMove);
        if(falseTime > moveLimit){
          falseTime = 0;
          trueTime = 0;
          started = false;
          }
        }
      lastMove = millis();
    }
    else {
      if(EndStage()){
        trueTime += (millis() - lastMove);
        if(trueTime > endGoal){
          falseTime = 0;
          trueTime = 0;
          started = false;
          moving = false;
          return true;
          }
        }
      else{
        falseTime += (millis() - lastMove);
        if(falseTime > endLimit){
          falseTime = 0;
          trueTime = 0;
          started = false;
          moving = false;
          }
        }
      lastMove = millis();
    }
    return false;
  }
};

class RightGesture : public Gesture{
  public : bool StartStage() override {
   if (abs((roll) > 140 && yaw > -40) &&  yaw < 40){
    if(abs(aaReal.x / accelScale) < 0.2  && abs(aaReal.y / accelScale) < 0.2  && abs(aaReal.z / accelScale) < 0.2){
      return true;
    }
   }
   return false;
  }
  public : bool Movement() override {
    if (abs((roll) > 140 && yaw > -40) &&  yaw < 40){
      if(abs(aaReal.x / accelScale) < 0.4  && abs(aaReal.y / accelScale) < 0.4  && abs(aaReal.z / accelScale) > 0.4){
      return true;
      }
    }
    return false;
  }
  public : bool EndStage() override {
    if (abs((roll) > 140 && yaw > -40) &&  yaw < 40){
      if(abs(aaReal.x / accelScale) < 0.2  && abs(aaReal.y / accelScale) < 0.2  && abs(aaReal.z / accelScale) < 0.2){
      return true;
      }
    }
    return false;
  }
  RightGesture() : Gesture(500, 60, 2000, 4000, 300, 3000, "Right") {
  }
}; 

class LeftGesture : public Gesture{
  public : bool StartStage() override {
   if (((roll) < -30 && (roll) > -90 && yaw > -50) &&  yaw < 50){
    if(abs(aaReal.x / accelScale) < 0.4  && abs(aaReal.y / accelScale) < 0.4  && abs(aaReal.z / accelScale) < 0.4){
      return true;
    }
   }
   return false;
  }
  public : bool Movement() override {
   if (((roll) < -50 && (roll) > -180 && yaw > -50) &&  yaw < 50){
    if(abs(aaReal.x / accelScale) < 0.5  && abs(aaReal.y / accelScale) < 0.5  && abs(aaReal.z / accelScale) < 0.5){
      return true;
    }
   }
    return false;
  }
  public : bool EndStage() override {
  if (((roll) < 180 && (roll) > 100 && yaw > -50) &&  yaw < 50){
    if(abs(aaReal.x / accelScale) < 0.4  && abs(aaReal.y / accelScale) < 0.4  && abs(aaReal.z / accelScale) < 0.4){
      return true;
    }
   }
    return false;
  }
  LeftGesture() : Gesture(500, 60, 2000, 4000, 300, 3000, "Left") {
  }
};  

class DownGesture : public Gesture{
  public : bool StartStage() override {
   if (((roll) < -30 && (roll) > -90 && yaw > -50) &&  yaw < 50){
    if(abs(aaReal.x / accelScale) < 0.4  && abs(aaReal.y / accelScale) < 0.4  && abs(aaReal.z / accelScale) < 0.4){
      return true;
    }
   }
   return false;
  }
  public : bool Movement() override {
   if (((roll) < -50 && (roll) > -180 && yaw > -50) &&  yaw < 50){
    if(abs(aaReal.x / accelScale) < 0.5  && abs(aaReal.y / accelScale) < 0.5  && abs(aaReal.z / accelScale) < 0.5){
      return true;
    }
   }
    return false;
  }
  public : bool EndStage() override {
  if (((roll) < 180 && (roll) > 100 && yaw > -50) &&  yaw < 50){
    if(abs(aaReal.x / accelScale) < 0.4  && abs(aaReal.y / accelScale) < 0.4  && abs(aaReal.z / accelScale) < 0.4){
      return true;
    }
   }
    return false;
  }
  DownGesture() : Gesture(500, 60, 2000, 4000, 300, 3000, "Down") {
  }
};  



Gesture* gestures[3];

void setup() {
  Wire.begin();
  Wire.setClock(100000);
  mpu.initialize();
  Serial.begin(115200);
  
  gestures[0] = new LeftGesture();
  gestures[1] = new RightGesture();
  gestures[2] = new DownGesture();

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(interrupt, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(interrupt));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(interrupt), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
}

void loop() {
  
    // if programming failed, don't try to do anything
    if (!dmpReady){
      return;
    }

    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      // convert to degrees
      yaw *= 180/M_PI;
      pitch *= 180/M_PI;
      roll *= 180/M_PI;

      // apply scaling
      x /= accelScale;
      y /= accelScale;
      z /= accelScale;
      //reportValues();
    }

    if(!checkGestures) {
      Serial.println("startGesture done");
      checkGestures = true;
      gestureCheckStart = millis();
    }
    if(checkGestures){
      for(int i = 0 ; i < 2; i++){
        if (gestures[i]->CheckMovement()){
          String command = gestures[i]->command;
          Serial.println(command);
        }
      }
      if(millis() - gestureCheckStart >= gesturePeriod){
        checkGestures = false;
      }
    }


}

void reportValues(){ 

  Serial.print(x); Serial.print(",");
  Serial.print(y); Serial.print(",");
  Serial.print(z); Serial.print(",");

  Serial.print(yaw); Serial.print(",");
  Serial.print(pitch); Serial.print(",");
  Serial.print(roll); Serial.println();

}