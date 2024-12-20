#include <Arduino.h>
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <arduino-timer.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>


// pin allocations
const int scl = D1, sda = D2, led = D6, interrupt = D7;

// EEPROM addresses
const int eepromSSID = 0, eepromPassword = 64, eepromServer = 128, eepromPort = 192, hasValues = 193;

const unsigned long LEDFlashTime = 150;

const unsigned long gesturePeriod = 3000;
unsigned long gestureCheckStart;

bool checkGestures = false, enableReporting = false;

// MPU6050 related stuff
const int accelRange = 4, gyroRange = 250;
const float accelScale = 32767 / accelRange, gyroScale = 32767 / gyroRange;

MPU6050 mpu;
Timer timer = timer_create_default(); // create a timer with default settings

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
float x, y, z;

// MPU interrupt vars
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

unsigned long lastScreenRefresh = 0;

// network related stuff
String ssid;
String password;
String servername;
int port = 18472;
WiFiClient client;

const int maxConnectionAttempts = 5;
int connectionAttempts = 0;


// function declarations
void firstSetup();
void connectToWiFi();
void connectToServer();
void updateAccelGyro();
void reportValues();
void startGestureCheck();
void processSerialCommands();
void storeStringInEEPROM(String, int);
void processIncomingMessages();
void sendCommandOverNetwork(String);
String readStringInEEPROM(int);
bool yawBetween(float, float);
bool pitchBetween(float, float);
bool rollBetween(float, float);
bool turnOffLED(void *){
  digitalWrite(led, LOW);
  return true;
}


class Gesture {
  bool started;
  bool moving;
  public : String command;

  public : virtual bool StartStage() = 0;
  public : virtual bool MoveStage() = 0;
  public : virtual bool EndStage() {
    return yawBetween(-30, 30) && pitchBetween(-30, 30) && rollBetween(-30, 30);
  }

  Gesture(const String& command) : command(command),
     started(false){
    }
  bool CheckMovement(){
    if(!started){
      if(StartStage()){
        started = true;
        Serial.print(F("detected start: "));
        Serial.println(command);
      }
    }
    else if (!moving){
      if (MoveStage()){
        moving = true;
        Serial.print(F("detected movement: "));
        Serial.println(command);
      }
    }
    else {
      if (!MoveStage()){
        moving = false;
        started = false;
        Serial.print(F("movement aborted: "));
        Serial.println(command);
      }
      else if(EndStage()){
          started = false;
          moving = false;
          return true;
      }
    }
    
    return false;
  }
};

class RightGesture : public Gesture{
  public : bool StartStage() override {
   return (yawBetween(-135, -45) && pitchBetween(-30, 30));
  }
  public : bool MoveStage() override {
    return (yawBetween(-50, 30) && pitchBetween(-30, 30));
  }
  RightGesture() : Gesture("Right") {
  }
}; 

class LeftGesture : public Gesture{
  public : bool StartStage() override {
   return (yawBetween(45, 135) && pitchBetween(-30, 30));
  }
  public : bool MoveStage() override {
    return (yawBetween(-30, 50) && pitchBetween(-30, 30));
  }
  LeftGesture() : Gesture("Left") {
  }
};  

class DownGesture : public Gesture{
  public : bool StartStage() override {
    return (pitchBetween(45, 135) && yawBetween(-30, 30));
  }
  public : bool MoveStage() override {
    return (pitchBetween(-30, 50) && yawBetween(-30, 30));
  }
  public : bool EndStage() override {
    return pitchBetween(-30, 30) && yawBetween(-30, 30);
  }
  DownGesture() : Gesture("Down") {
  }
};  



Gesture* gestures[3];

void startGestureCheck(){
  gestureCheckStart = millis();
  checkGestures = true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(sda, scl);
  EEPROM.begin(512);
  pinMode(interrupt, INPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  bool reset = EEPROM.read(hasValues);

  Serial.println(reset);

  if (reset == 0){
    Serial.println(F("First boot, starting setup"));

    firstSetup();

    Serial.println(F("Setup complete"));
    
    EEPROM.put(hasValues, 1);
    EEPROM.commit();
  }
  else{
    ssid = readStringInEEPROM(eepromSSID);
    password = readStringInEEPROM(eepromPassword);
    servername = readStringInEEPROM(eepromServer);
    EEPROM.get(eepromPort, port);
  }
  
  gestures[0] = new LeftGesture();
  gestures[1] = new RightGesture();
  gestures[2] = new DownGesture();

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  if (mpu.testConnection()){
    Serial.println(F("MPU6050 connection successful"));
  }
  else{
    Serial.println(F("MPU6050 connection failed"));
    digitalWrite(led, HIGH);
    while(1);
  }

  // set sensitivity of the MPU
  mpu.setFullScaleGyroRange(gyroRange);
  mpu.setFullScaleAccelRange(accelRange);

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
      Serial.print(interrupt);
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(interrupt), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Starting Loop!"));
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

  timer.tick();

  // if not connected to wifi, attempt to connect
  if (WiFi.status() != WL_CONNECTED){
    connectToWiFi();
  }

  // if not connected to server, attempt to connect
  if (!client.connected()){
    connectToServer();
  }

  processSerialCommands();
  processIncomingMessages();

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
    x = aaReal.x / accelScale;
    y = aaReal.y / accelScale;
    z = aaReal.z / accelScale;

    if (enableReporting){
      reportValues();
    }
  }

  if(checkGestures){
    for(int i = 0 ; i < 3; i++){
      if (gestures[i]->CheckMovement()){
        String command = gestures[i]->command;
        digitalWrite(led, HIGH);
        timer.in(LEDFlashTime, turnOffLED);
        Serial.println(command);
        sendCommandOverNetwork(command);
        
        checkGestures = false;
        Serial.println(F("Gesture check complete"));
      }
    }
    if(millis() - gestureCheckStart >= gesturePeriod){
      checkGestures = false;
      Serial.println(F("Gesture check complete"));
    }
  }
}

void connectToWiFi(){
  Serial.println(F("Connecting to WiFi"));
  WiFi.begin(ssid.c_str(), password.c_str());
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(F("."));
  }
  Serial.println(F("Connected to WiFi"));
  Serial.print(F("Local IP: ")); Serial.println(WiFi.localIP());
}

void connectToServer(){
  if (connectionAttempts >= maxConnectionAttempts) return;

  Serial.print(F("Connecting to server at: ")); Serial.println(servername);
  if (!client.connect(servername, 18472)){
    Serial.println(F("Connection failed"));
    connectionAttempts++;
    return;
  }
  Serial.println(F("Connected to server"));
  connectionAttempts = 0;
}

void sendCommandOverNetwork(String command){
  if (!client.connected()){
    connectToServer();
  }
  client.println(command);
}

void processIncomingMessages(){
  if (!client.connected()){
    connectToServer();
  }
  if (!client.available()){
    return;
  }

  // decode the incoming message
  unsigned long processStart = micros();
  String command = "";

  while (client.available() > 0){
    command += char(client.read());
  }

  // process the command
  if (command == "start"){
    startGestureCheck();
  } 

  // send an acknowledgement and the time taken to process the command
  unsigned long time = micros() - processStart;

  command.trim();
  Serial.print(F("Received command: ")); Serial.println(command);
  String ack = "Received command: \"" + command + "\" | Time to process: " + String(time) + "µs";
  sendCommandOverNetwork(ack); // send an acknowledgement
  
  
}

void storeStringInEEPROM(String str, int start){
  if (str.length() > 64){
    Serial.println(F("String too long, max is 64 characters"));
    return;
  }
  for (int i = 0; i < str.length(); i++){
    EEPROM.put(start + i, str[i]);
  }
  for (int i = str.length(); i < 64; i++){
    EEPROM.put(start + i, ' ');
  }
  EEPROM.commit();
}

String readStringInEEPROM(int start){
  String str = "";
  for (int i = start; i < start + 64; i++){
    str += char(EEPROM.read(i));
  }
  str.trim();

  return str;
}

void firstSetup(){
  Serial.println(F("Please enter the SSID of your network"));
  while(Serial.available() == 0);
  String tempssid = Serial.readStringUntil('\n');

  Serial.println(F("Please enter the password of your network"));
  while(Serial.available() == 0);
  String temppassword = Serial.readStringUntil('\n');

  Serial.println(F("Please enter the IP address or url of the host server"));
  while(Serial.available() == 0);
  String tempservername = Serial.readStringUntil('\n');

  Serial.println(F("Please enter the port number of the host server"));
  while(Serial.available() == 0);
  String tempport = Serial.readStringUntil('\n');

  Serial.println(F("these are the values you entered: "));
  Serial.print(F("SSID: ")); Serial.println(tempssid);
  Serial.print(F("Password: ")); Serial.println(temppassword);
  Serial.print(F("Server address: ")); Serial.println(tempservername);
  Serial.print(F("Port: ")); Serial.println(tempport);
  Serial.println(F("If these are correct, type 'y' to save them to EEPROM, else type 'n' to re-enter them"));

  while(Serial.available() == 0);
  String response = Serial.readStringUntil('\n');
  if (response == "y"){
    ssid = tempssid;
    password = temppassword;
    servername = tempservername;
    storeStringInEEPROM(ssid, eepromSSID);
    storeStringInEEPROM(password, eepromPassword);
    storeStringInEEPROM(servername, eepromServer);
    EEPROM.put(eepromPort, tempport.toInt());

    EEPROM.commit();
  }
  else firstSetup();
}

void processSerialCommands(){
  if(Serial.available() > 0){
    String command = Serial.readStringUntil('\n');
    if(command == "start"){
      startGestureCheck();
    }
    else if (command.startsWith("set-server "))
    {
      command.remove(0, 11);
      command.trim();
      Serial.print(F("Setting reporting server to: "));
      Serial.println(command);

      servername = command;
      storeStringInEEPROM(servername, eepromServer);
      connectionAttempts = 0;
      connectToServer();
    }
    else if (command.startsWith("set-ssid "))
    {
      // set the SSID
      command.remove(0, 9);
      command.trim();
      Serial.print(F("Setting SSID to: "));
      Serial.println(command);

      ssid = command;
      storeStringInEEPROM(ssid, eepromSSID);
      connectionAttempts = 0;
      connectToWiFi();
      connectToServer();
    }
    else if (command.startsWith("set-password "))
    {
      // set the password
      command.remove(0, 13);
      command.trim();
      Serial.print(F("Setting password to: "));
      Serial.println(command);

      password = command;
      storeStringInEEPROM(password, eepromPassword);
      connectionAttempts = 0;
      connectToWiFi();
      connectToServer();
    }

    else if (command.startsWith("set-port "))
    {
      // set the port
      command.remove(0, 9);
      command.trim();
      if (!command.toInt()){
        Serial.println(F("Port must be a number"));
        return;
      }

      Serial.print(F("Setting port to: "));
      Serial.println(command);

      port = command.toInt();
      EEPROM.put(eepromPort, port);
      EEPROM.commit();
      connectionAttempts = 0;
      connectToServer();
    }

    else if (command.startsWith("send "))
    {
      command.remove(0, 5);
      command.trim();
      Serial.print(F("Sending command: "));
      Serial.println(command);
      sendCommandOverNetwork(command);
    }

    else if (command.startsWith("view-stored-values")){
      Serial.print(F("SSID: ")); Serial.println(readStringInEEPROM(eepromSSID));
      Serial.print(F("Password: ")); Serial.println(readStringInEEPROM(eepromPassword));
      Serial.print(F("Server: ")); Serial.println(readStringInEEPROM(eepromServer));
      Serial.print(F("Reset flag (0 = First Boot, 1 = Normal): ")); Serial.println(EEPROM.read(hasValues));
    }
    else if (command == "toggle-report")
    {
      enableReporting = !enableReporting;
      Serial.print(F("Reporting is now "));
      Serial.println(enableReporting ? "enabled" : "disabled");
    }

    else if (command == "sim"){
      Serial.println(F("Simulating gesture start"));
      startGestureCheck();
    }

    else if (command == "reset")
    {
      Serial.println(F("Resetting EEPROM"));
      for (int i = 0; i < 200; i++)
      {
        EEPROM.put(i, 0);
      }
      EEPROM.put(hasValues, 0);
      EEPROM.commit();
    }
  }
}

void reportValues(){ 

  Serial.print(x); Serial.print(F(","));
  Serial.print(y); Serial.print(F(","));
  Serial.print(z); Serial.print(F(","));

  Serial.print(yaw); Serial.print(F(","));
  Serial.print(pitch); Serial.print(F(","));
  Serial.print(roll); Serial.println();

}

bool yawBetween(float min, float max){
  return yaw > min && yaw < max;
}

bool pitchBetween(float min, float max){
  return pitch > min && pitch < max;
}

bool rollBetween(float min, float max){
  return roll > min && roll < max;
}
