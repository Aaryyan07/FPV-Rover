#include <IBusBM.h>
#include <Servo.h>

const int MAX_SPEED_ROVER      = 255; 

const int MAX_SPEED_ARM_BASE   = 200;  
const int MAX_SPEED_ARM_LOWER  = 255;  
const int MAX_SPEED_ARM_UPPER  = 255;  
const int MAX_SPEED_ARM_DIGGER = 255;  
const int MAX_SPEED_ARM_CLIPPER= 200;  
const int MAX_SPEED_ARM_LIFT   = 255;  

const int SERVO_MIN_ANGLE      = 0;
const int SERVO_MAX_ANGLE      = 180;

const int LEFT_ENA = 2;   const int LEFT_IN1 = 49; const int LEFT_IN2 = 47; 
const int LEFT_IN3 = 45;  const int LEFT_IN4 = 43; const int LEFT_ENB = 3;

const int RIGHT_ENA = 10; const int RIGHT_IN1 = 37; const int RIGHT_IN2 = 35; 
const int RIGHT_IN3 = 33; const int RIGHT_IN4 = 31; const int RIGHT_ENB = 11;

const int SERVO_PIN = 9; 

const int D1_PWMA = 12;   const int D1_AIN1 = 46; const int D1_AIN2 = 44; 
const int D1_PWMB = 13;   const int D1_BIN1 = 48; const int D1_BIN2 = 50; 

const int D2_PWMA = 4;   const int D2_AIN1 = 28; const int D2_AIN2 = 26; 
const int D2_PWMB = 5;   const int D2_BIN1 = 30; const int D2_BIN2 = 32; 

const int D3_PWMA = 7;   const int D3_AIN1 = 38; const int D3_AIN2 = 36; 
const int D3_PWMB = 6;   const int D3_BIN1 = 40; const int D3_BIN2 = 42; 

const int STDBY1 = 22;
const int STDBY2 = 23;

const int CH_ROVER_ROLL = 0;    
const int CH_ROVER_PITCH = 1;   
const int CH_ROVER_SERVO = 4;   
const int CH_ROVER_LIFT = 3;    
const int CH_ROVER_EMERGENCY = 9;   

const int CH_ARM_LOWER = 0;     
const int CH_ARM_UPPER = 1;     
const int CH_ARM_SAFETY = 2;    
const int CH_ARM_BASE = 3;      
const int CH_ARM_DIGGER = 4;    
const int CH_ARM_CLIPPER = 5;   

const int CH_ARM_CALIBRATE = 7;    
const int CH_ARM_RESET_POS = 6;    

IBusBM IBusRover; 
IBusBM IBusArm;   
Servo myServo;    

int globalLeftSpeed = 0;
int globalRightSpeed = 0;

const int CALIBRATION_SPEED = 200; 

unsigned long lastDebugTime = 0;

unsigned long previousTime = 0;
float deltaTime = 0.0;

float posLeftA = 0.0, posLeftB = 0.0;
float posRightA = 0.0, posRightB = 0.0;
float posBase = 0.0, posDigger = 0.0, posClipper = 0.0;
float posLowerArm = 0.0, posUpperArm = 0.0, posLift = 0.0;

const float SPEED_TO_DIST = 0.5; 

const float CALIBRATION_TOLERANCE = 5.0;
const int SIGNAL_TIMEOUT = 1000; 
unsigned long lastValidSignalTime = 0;

void displayFullSystemDebug();
int mapSpeed(int rcVal, int maxLimit);

void setup() {
  Serial.begin(115200);
  previousTime = millis();
  
  pinMode(LEFT_ENA, OUTPUT);  pinMode(LEFT_IN1, OUTPUT);  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_IN3, OUTPUT);  pinMode(LEFT_IN4, OUTPUT);  pinMode(LEFT_ENB, OUTPUT);
  pinMode(RIGHT_ENA, OUTPUT); pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT); pinMode(RIGHT_IN4, OUTPUT); pinMode(RIGHT_ENB, OUTPUT);

  pinMode(D1_PWMA, OUTPUT); pinMode(D1_AIN1, OUTPUT); pinMode(D1_AIN2, OUTPUT);
  pinMode(D1_PWMB, OUTPUT); pinMode(D1_BIN1, OUTPUT); pinMode(D1_BIN2, OUTPUT);
  pinMode(D2_PWMA, OUTPUT); pinMode(D2_AIN1, OUTPUT); pinMode(D2_AIN2, OUTPUT);
  pinMode(D2_PWMB, OUTPUT); pinMode(D2_BIN1, OUTPUT); pinMode(D2_BIN2, OUTPUT);
  pinMode(D3_PWMA, OUTPUT); pinMode(D3_AIN1, OUTPUT); pinMode(D3_AIN2, OUTPUT);
  pinMode(D3_PWMB, OUTPUT); pinMode(D3_BIN1, OUTPUT); pinMode(D3_BIN2, OUTPUT);

  pinMode(STDBY1, OUTPUT); digitalWrite(STDBY1, HIGH);
  pinMode(STDBY2, OUTPUT); digitalWrite(STDBY2, HIGH);

  myServo.attach(SERVO_PIN);
  
  IBusRover.begin(Serial1); 
  IBusArm.begin(Serial2);   

  Serial.println("SYSTEM READY. FULL DEBUG MODE + SPEED LIMITS ACTIVE.");
  lastValidSignalTime = millis();
}

void loop() {
  
  unsigned long currentTime = millis();
  deltaTime = (currentTime - previousTime) / 1000.0; 
  previousTime = currentTime;
  
  int emergencySignal = IBusRover.readChannel(CH_ROVER_EMERGENCY);
  int calibrateSignal = IBusArm.readChannel(CH_ARM_CALIBRATE);
  int resetPosSignal = IBusArm.readChannel(CH_ARM_RESET_POS);
  
  bool roverSignalValid = (IBusRover.readChannel(CH_ROVER_ROLL) >= 900 && IBusRover.readChannel(CH_ROVER_ROLL) <= 2100);
  bool armSignalValid = (IBusArm.readChannel(CH_ARM_LOWER) >= 900 && IBusArm.readChannel(CH_ARM_LOWER) <= 2100);
  
  if (roverSignalValid || armSignalValid) {
    lastValidSignalTime = millis();
  }
  
  if (emergencySignal > 1900 || (millis() - lastValidSignalTime > SIGNAL_TIMEOUT)) {
    setL298N(LEFT_ENA, LEFT_IN1, LEFT_IN2, 0);
    setL298N(LEFT_ENB, LEFT_IN3, LEFT_IN4, 0);
    setL298N(RIGHT_ENA, RIGHT_IN1, RIGHT_IN2, 0);
    setL298N(RIGHT_ENB, RIGHT_IN3, RIGHT_IN4, 0);
    
    moveArmMotor(D3_PWMA, D3_AIN1, D3_AIN2, 0);
    moveArmMotor(D1_PWMA, D1_AIN1, D1_AIN2, 0);
    moveArmMotor(D1_PWMB, D1_BIN1, D1_BIN2, 0);
    moveArmMotor(D2_PWMA, D2_AIN1, D2_AIN2, 0);
    moveArmMotor(D2_PWMB, D2_BIN1, D2_BIN2, 0);
    moveArmMotor(D3_PWMB, D3_BIN1, D3_BIN2, 0);
    
    if (millis() - lastDebugTime > 500) {
      if (emergencySignal > 1900) Serial.println("!!! EMERGENCY STOP ACTIVATED !!!");
      else Serial.println("!!! SIGNAL LOST - MOTORS STOPPED !!!");
      lastDebugTime = millis();
    }
    return; 
  }
  
  if (resetPosSignal > 1900) {
    posLeftA = 0.0; posLeftB = 0.0; posRightA = 0.0; posRightB = 0.0;
    posBase = 0.0; posDigger = 0.0; posClipper = 0.0;
    posLowerArm = 0.0; posUpperArm = 0.0; posLift = 0.0;
    
    if (millis() - lastDebugTime > 1000) {
       Serial.println("--- POSITIONS RESET TO 0.0 ---");
       lastDebugTime = millis();
    }
    return;
  }
  
  if (calibrateSignal > 1900) {
    myServo.write(SERVO_MIN_ANGLE); 
    setL298N(LEFT_ENA, LEFT_IN1, LEFT_IN2, 0);
    setL298N(LEFT_ENB, LEFT_IN3, LEFT_IN4, 0);
    setL298N(RIGHT_ENA, RIGHT_IN1, RIGHT_IN2, 0);
    setL298N(RIGHT_ENB, RIGHT_IN3, RIGHT_IN4, 0);

    int speedBase = calculateCalibrationSpeed(posBase);
    int speedDigger = calculateCalibrationSpeed(posDigger);
    int speedClipper = calculateCalibrationSpeed(posClipper);
    int speedLowerArm = calculateCalibrationSpeed(posLowerArm);
    int speedUpperArm = calculateCalibrationSpeed(posUpperArm);
    int speedLift = calculateCalibrationSpeed(posLift);

    moveArmMotor(D3_PWMA, D3_AIN1, D3_AIN2, speedBase);
    moveArmMotor(D1_PWMA, D1_AIN1, D1_AIN2, speedDigger);
    moveArmMotor(D1_PWMB, D1_BIN1, D1_BIN2, speedClipper);
    moveArmMotor(D2_PWMA, D2_AIN1, D2_AIN2, speedLowerArm);
    moveArmMotor(D2_PWMB, D2_BIN1, D2_BIN2, speedUpperArm);
    moveArmMotor(D3_PWMB, D3_BIN1, D3_BIN2, speedLift);

    updatePositions(0, 0, 0, 0, speedBase, speedDigger, speedClipper, speedLowerArm, speedUpperArm, speedLift);
    
    if (millis() - lastDebugTime > 250) { 
       displayFullSystemDebug();
       lastDebugTime = millis();
    }
    return;
  }

  int rawSafety = IBusArm.readChannel(CH_ARM_SAFETY); 
  int rawBase   = IBusArm.readChannel(CH_ARM_BASE);   
  
  int validBaseVal = 1500;
  if (rawSafety < 1100 && rawSafety > 800) { 
      validBaseVal = rawBase;
  }

  int rawLower   = IBusArm.readChannel(CH_ARM_LOWER);   
  int rawUpper   = IBusArm.readChannel(CH_ARM_UPPER);   
  int rawDigger  = IBusArm.readChannel(CH_ARM_DIGGER);  
  int rawClipper = IBusArm.readChannel(CH_ARM_CLIPPER); 

  int rawRoll   = IBusRover.readChannel(CH_ROVER_ROLL); 
  int rawPitch  = IBusRover.readChannel(CH_ROVER_PITCH);
  int rawServo  = IBusRover.readChannel(CH_ROVER_SERVO);
  int rawLift   = IBusRover.readChannel(CH_ROVER_LIFT); 

  if (millis() - lastDebugTime > 250) { 
    displayFullSystemDebug();
    lastDebugTime = millis();
  }

  if (rawServo >= 1000) {
      int sAngle = map(rawServo, 1000, 2000, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
      myServo.write(sAngle);
  }
  
  int throttle = mapPulse(rawPitch);
  int steering = mapPulse(rawRoll);
  
  int leftMotorSpeed = throttle + steering;
  int rightMotorSpeed = throttle - steering;

  globalLeftSpeed = constrain(leftMotorSpeed, -MAX_SPEED_ROVER, MAX_SPEED_ROVER);
  globalRightSpeed = constrain(rightMotorSpeed, -MAX_SPEED_ROVER, MAX_SPEED_ROVER);

  setL298N(LEFT_ENA, LEFT_IN1, LEFT_IN2, globalLeftSpeed);
  setL298N(LEFT_ENB, LEFT_IN3, LEFT_IN4, globalLeftSpeed);
  setL298N(RIGHT_ENA, RIGHT_IN1, RIGHT_IN2, globalRightSpeed);
  setL298N(RIGHT_ENB, RIGHT_IN3, RIGHT_IN4, globalRightSpeed);

  int speedBase     = mapSpeed(validBaseVal, MAX_SPEED_ARM_BASE);
  int speedDigger   = mapSpeed(rawDigger, MAX_SPEED_ARM_DIGGER);
  int speedClipper  = mapSpeed(rawClipper, MAX_SPEED_ARM_CLIPPER);
  int speedLower    = mapSpeed(rawLower, MAX_SPEED_ARM_LOWER);
  int speedUpper    = mapSpeed(rawUpper, MAX_SPEED_ARM_UPPER);
  int speedLift     = mapSpeed(rawLift, MAX_SPEED_ARM_LIFT);

  moveArmMotor(D3_PWMA, D3_AIN1, D3_AIN2, speedBase); 
  moveArmMotor(D1_PWMA, D1_AIN1, D1_AIN2, speedDigger);    
  moveArmMotor(D1_PWMB, D1_BIN1, D1_BIN2, speedClipper);   
  moveArmMotor(D2_PWMA, D2_AIN1, D2_AIN2, speedLower);     
  moveArmMotor(D2_PWMB, D2_BIN1, D2_BIN2, speedUpper);     
  moveArmMotor(D3_PWMB, D3_BIN1, D3_BIN2, speedLift);      

  updatePositions(globalLeftSpeed, globalLeftSpeed, globalRightSpeed, globalRightSpeed,
                  speedBase, speedDigger, speedClipper, speedLower, speedUpper, speedLift);

  delay(10); 
}

void moveArmMotor(int pwm, int in1, int in2, int speed) {
  if (abs(speed) < 10) {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW); analogWrite(pwm, 0);
  } else if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW); analogWrite(pwm, speed);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH); analogWrite(pwm, abs(speed));
  }
}

int mapSpeed(int rcVal, int maxLimit) {
  if (rcVal < 800 || rcVal > 2200) return 0;
  
  int speed = map(rcVal, 1000, 2000, -maxLimit, maxLimit);
  
  if (abs(speed) < 35) speed = 0; 
  return speed;
}

int mapPulse(int val) {
  if (val < 900 || val > 2100) return 0;
  int out = map(val, 1000, 2000, -255, 255);
  return (abs(out) < 25) ? 0 : out;
}

void setL298N(int en, int in1, int in2, int speed) {
  if (abs(speed) < 20) {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW); analogWrite(en, 0);
  } else if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW); analogWrite(en, speed);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH); analogWrite(en, abs(speed));
  }
}

void updatePositions(int sLA, int sLB, int sRA, int sRB,
                     int sBase, int sDig, int sClip, int sLow, int sUp, int sLift) {
  posLeftA += sLA * deltaTime * SPEED_TO_DIST;
  posLeftB += sLB * deltaTime * SPEED_TO_DIST;
  posRightA += sRA * deltaTime * SPEED_TO_DIST;
  posRightB += sRB * deltaTime * SPEED_TO_DIST;
  posBase += sBase * deltaTime * SPEED_TO_DIST;
  posDigger += sDig * deltaTime * SPEED_TO_DIST;
  posClipper += sClip * deltaTime * SPEED_TO_DIST;
  posLowerArm += sLow * deltaTime * SPEED_TO_DIST;
  posUpperArm += sUp * deltaTime * SPEED_TO_DIST;
  posLift += sLift * deltaTime * SPEED_TO_DIST;
}

int calculateCalibrationSpeed(float currentPos) {
  if (abs(currentPos) < CALIBRATION_TOLERANCE) return 0;
  int direction = (currentPos > 0) ? -1 : 1;
  float distance = abs(currentPos);
  int speed = constrain((int)(distance * 2), 50, CALIBRATION_SPEED);
  return direction * speed;
}

void displayFullSystemDebug() {
  
  Serial.print("ROVER Rx [Ch1-10]: ");
  for(int i=0; i<10; i++) {
     Serial.print(IBusRover.readChannel(i));
     Serial.print("  ");
  }
  Serial.println(); 

  Serial.print("ARM   Rx [Ch1-10]: ");
  for(int i=0; i<10; i++) {
     Serial.print(IBusArm.readChannel(i));
     Serial.print("  ");
  }
  Serial.println(); 

  Serial.print("POS | Base:"); Serial.print(posBase, 1);
  Serial.print(" Low:"); Serial.print(posLowerArm, 1);
  Serial.print(" Up:"); Serial.print(posUpperArm, 1);
  Serial.print(" Dig:"); Serial.print(posDigger, 1);
  Serial.print(" Clip:"); Serial.print(posClipper, 1);
  Serial.print(" Lift:"); Serial.print(posLift, 1);
  
  Serial.println("\n-------------------------------------------------------------");
}