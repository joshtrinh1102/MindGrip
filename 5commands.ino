#include <ESP32Servo.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>

/*
  ESP32 Dual Servo Winch — BLE Edition
  - GPIO 12: servo LEFT
  - GPIO 33: servo RIGHT
  - GPIO 19 -> GND: CLOSE button
  - GPIO 21 -> GND: OPEN button
  - BLE UART: ESP32_Winch
  - Serial: 115200 baud
*/

#define PIN_SERVO_LEFT  12
#define PIN_SERVO_RIGHT 33
#define PIN_CLOSE 19
#define PIN_OPEN  21
#define BLE_NAME "ESP32_Winch"

// BLE UUIDs (Nordic UART Service)
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

Servo servoLeft_;
Servo servoRight_;

float posDeg=90, targetDeg=90;
float r_mm=10.0f, stepMM=5.0f, holdVelDeg=0.0f;
float velDegPerS=0.0f;
bool invertDir=false;
int LIM_MIN=30, LIM_MAX=150, HOME_DEG=90;

unsigned long lastTickMs=0, lastEdgeMs=0;
bool prevClose=HIGH, prevOpen=HIGH;

char lineBuf[96], bleBuf[96];
uint8_t lineLen=0, bleLen=0;

float clampF(float v, float lo, float hi) {
  return v<lo?lo:v>hi?hi:v;
}

int clampI(int v, int lo, int hi) {
  return v<lo?lo:v>hi?hi:v;
}

float mmToDeg(float mm) {
  return (mm*180.0f)/(3.14159265f*r_mm);
}

float applyDir(float ddeg) {
  return invertDir?-ddeg:ddeg;
}

void writeServoDeg(float deg) {
  deg=clampF(deg,LIM_MIN,LIM_MAX);
  posDeg=deg;
  int degInt=(int)(deg+0.5f);
  // Write to BOTH servos simultaneously
  servoLeft_.write(degInt);
  servoRight_.write(degInt);
}

void setTarget(float newTargetDeg) {
  newTargetDeg=clampF(newTargetDeg,LIM_MIN,LIM_MAX);
  targetDeg=newTargetDeg;
  velDegPerS=(targetDeg>posDeg)?360.0f:-360.0f;
}

void startCreep(bool closing) {
  velDegPerS=closing?+holdVelDeg:-holdVelDeg;
}

void stopCreep() {
  velDegPerS=0.0f;
}

void motionTick() {
  unsigned long now=millis();
  if(now-lastTickMs<10) return;
  float dt=(now-lastTickMs)/1000.0f;
  lastTickMs=now;

  if(velDegPerS==0.0f) return;

  float next=posDeg+velDegPerS*dt;

  bool isDiscrete=(fabs(fabs(velDegPerS)-360)<1e-3f);
  if(isDiscrete) {
    if((velDegPerS>0 && next>=targetDeg)||(velDegPerS<0 && next<=targetDeg)) {
      next=targetDeg;
      velDegPerS=0.0f;
    }
  }

  if(next<=LIM_MIN){next=LIM_MIN;velDegPerS=0.0f;}
  if(next>=LIM_MAX){next=LIM_MAX;velDegPerS=0.0f;}

  writeServoDeg(next);
}

void sendBLE(const char* s) {
  if(deviceConnected) {
    pTxCharacteristic->setValue((uint8_t*)s, strlen(s));
    pTxCharacteristic->notify();
    delay(10);
  }
}

void out(const char* s) {
  Serial.println(s);
  sendBLE(s);
}

void doHome() {
  setTarget(HOME_DEG);
  out("ok");
}

void doCloseMM(float mm) {
  float d=applyDir(mmToDeg(mm));
  setTarget(posDeg-d);
  out("ok");
}

void doOpenMM(float mm) {
  float d=applyDir(mmToDeg(mm));
  setTarget(posDeg+d);
  out("ok");
}

void printStatus() {
  char buf[128];
  sprintf(buf,"pos:%.1f r:%.1f step:%.1f lim:%d-%d dual:ON",
    posDeg,r_mm,stepMM,LIM_MIN,LIM_MAX);
  out(buf);
}

void processCmd(char *cmd) {
  char *a1=NULL,*a2=NULL;
  for(uint8_t i=0;cmd[i];i++){
    if(cmd[i]==' '){
      cmd[i]='\0';
      a1=&cmd[i+1];
      break;
    }
  }
  if(a1){
    for(uint8_t i=0;a1[i];i++){
      if(a1[i]==' '){
        a1[i]='\0';
        a2=&a1[i+1];
        break;
      }
    }
  }

  // Command 1: PUSH (winds UP - same as close)
  if(!strcmp(cmd,"push")){
    float mm=(a1&&*a1)?atof(a1):stepMM;
    doCloseMM(mm);
  }
  // Command 2: PULL (lets DOWN - same as open)
  else if(!strcmp(cmd,"pull")){
    float mm=(a1&&*a1)?atof(a1):stepMM;
    doOpenMM(mm);
  }
  // Command 3: DROP (quick release to minimum limit)
  else if(!strcmp(cmd,"drop")){
    setTarget(LIM_MIN);
    out("ok");
  }
  // Command 4: LIFT (raise to maximum limit)
  else if(!strcmp(cmd,"lift")){
    setTarget(LIM_MAX);
    out("ok");
  }
  // Command 5: HOLD (return to center/home position)
  else if(!strcmp(cmd,"hold")){
    setTarget(HOME_DEG);
    out("ok");
  }
  // Legacy commands (still work)
  else if(!strcmp(cmd,"c")||!strcmp(cmd,"close")){
    float mm=(a1&&*a1)?atof(a1):stepMM;
    doCloseMM(mm);
  }
  else if(!strcmp(cmd,"o")||!strcmp(cmd,"open")){
    float mm=(a1&&*a1)?atof(a1):stepMM;
    doOpenMM(mm);
  }
  else if(!strcmp(cmd,"h")||!strcmp(cmd,"home")){
    doHome();
  }
  else if(!strcmp(cmd,"r")||!strcmp(cmd,"radius")){
    if(a1){
      r_mm=fmaxf(1.0f,atof(a1));
      out("ok");
    }
  }
  else if(!strcmp(cmd,"l")||!strcmp(cmd,"limits")){
    if(a1&&a2){
      LIM_MIN=clampI(atoi(a1),0,180);
      LIM_MAX=clampI(atoi(a2),0,180);
      if(LIM_MIN>LIM_MAX){
        int t=LIM_MIN;
        LIM_MIN=LIM_MAX;
        LIM_MAX=t;
      }
      out("ok");
    }
  }
  else if(!strcmp(cmd,"i")||!strcmp(cmd,"invert")){
    if(a1){
      invertDir=(atoi(a1)!=0);
      out("ok");
    }
  }
  else if(!strcmp(cmd,"?")||!strcmp(cmd,"status")||!strcmp(cmd,"ping")){
    printStatus();
  }
  else if(!strcmp(cmd,"s")||!strcmp(cmd,"step")){
    if(a1){
      stepMM=fmaxf(0.5f,atof(a1));
      out("ok");
    }
  }
  else if(!strcmp(cmd,"holdspeed")){
    if(a1){
      float mmps=fmaxf(1.0f,atof(a1));
      holdVelDeg=mmToDeg(mmps);
      out("ok");
    }
  }
  else{
    out("err");
  }
}

void handleSerialInput() {
  while(Serial.available()) {
    char c=Serial.read();
    if(c=='\r') continue;
    if(c=='\n') {
      lineBuf[lineLen]='\0';
      if(lineLen) processCmd(lineBuf);
      lineLen=0;
    } else {
      if(lineLen<95) lineBuf[lineLen++]=c;
    }
  }
}

bool edge(uint8_t pin, bool &prev) {
  bool v=digitalRead(pin);
  if(v!=prev) {
    unsigned long now=millis();
    if(now-lastEdgeMs>=20) {
      prev=v;
      lastEdgeMs=now;
      return true;
    }
  }
  return false;
}

void handleButtons() {
  if(edge(PIN_CLOSE,prevClose) && prevClose==LOW) doCloseMM(stepMM);
  if(edge(PIN_OPEN,prevOpen) && prevOpen==LOW) doOpenMM(stepMM);

  bool closeHeld=(digitalRead(PIN_CLOSE)==LOW);
  bool openHeld=(digitalRead(PIN_OPEN)==LOW);

  if(closeHeld && !openHeld) {
    startCreep(true);
  } else if(openHeld && !closeHeld) {
    startCreep(false);
  } else {
    bool isDiscrete=(fabs(fabs(velDegPerS)-360)<1e-3f);
    if(!isDiscrete) stopCreep();
  }
}

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("BLE Connected");
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("BLE Disconnected");
  }
};

// BLE Characteristic Callbacks
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue().c_str();

    if (rxValue.length() > 0) {
      for (int i = 0; i < rxValue.length(); i++) {
        char c = rxValue[i];
        if(c=='\r') continue;
        if(c=='\n') {
          bleBuf[bleLen]='\0';
          if(bleLen) processCmd(bleBuf);
          bleLen=0;
        } else {
          if(bleLen<95) bleBuf[bleLen++]=c;
        }
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize BLE
  BLEDevice::init(BLE_NAME);
  
  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristics
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.print("BLE: ");
  Serial.println(BLE_NAME);
  Serial.println("Waiting for connection...");

  // Initialize GPIO and Servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  pinMode(PIN_CLOSE,INPUT_PULLUP);
  pinMode(PIN_OPEN,INPUT_PULLUP);

  holdVelDeg=mmToDeg(25.0f);

  // Attach BOTH servos
  servoLeft_.setPeriodHertz(50);
  servoLeft_.attach(PIN_SERVO_LEFT,500,2500);
  
  servoRight_.setPeriodHertz(50);
  servoRight_.attach(PIN_SERVO_RIGHT,500,2500);

  posDeg=targetDeg=HOME_DEG;
  servoLeft_.write(HOME_DEG);
  servoRight_.write(HOME_DEG);
  lastTickMs=millis();

  Serial.println("ready - DUAL SERVO MODE");
  printStatus();
}

void loop() {
  handleSerialInput();
  handleButtons();
  motionTick();

  // Handle BLE disconnection/reconnection
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Advertising...");
    oldDeviceConnected = deviceConnected;
  }
  
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  yield();
}