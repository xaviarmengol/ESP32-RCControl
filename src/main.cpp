// Based on https://github.com/TheDIYGuy999/MRSC_Adapter_3Pin_Servo

const float codeVersion = 0.2; // Software revision

//
// =======================================================================================================
// LIBRARIES
// =======================================================================================================
//

#include <Arduino.h>
#include <Wire.h> 
#include <ESP32Servo.h>

#include "blynkConf.h"
#include "AccOrientation.h"
#include "wifiUtils.h"
#include "configure_rmt.h" // RMT (instead of exceptions)
#include "SharedVar.h"
#include "filtreentrada.hpp"
#include "PinOut.h"
#include "MediumFilter.h"


// Create Servo objects
Servo servoSteering;
Servo servoVelocity;

// Servo limits (initial value = center position = 90° / 1500uSec)
// Limits are adjusted during the first steering operation!

// Always move the steering wheel inside its entire range after switching on the car
byte limSteeringL = 90, limSteeringR = 90; // usually 45 - 135° (90° is the servo center position)
int limuSecL = 1500, limuSecR = 1500; // usually 1000 - 2000 uSec (1500 uSec is in the servo center position)


// DANGER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// Move velocity: CALIBRATE BY HAND
// CenterVel depends if 50%-50% or 30%-70%
int limuSecVelFordward = 1976, limuSecVelReverse = 980, uSecVelCenter=1470; // usually 1000 - 2000 uSec
int uSecVelDeadBand = 10;


// MRSC gain
//int mrscGain = 90; // 80 This MRSC gain applies, if you don't have an adjustment pot and don't call readInputs()
double headingMultiplier = 1.0; // adjust until front wheels stay in parallel with the ground, if the car is swiveled around


unsigned long lastTimeDebug=0;

unsigned long debugPeriod=5000;

// Debug main

unsigned long lastTimeDebugMain=0;
unsigned long periodDebugMain=2000;

// MPU library

AccOrientation mpu;

// Enable System

bool enableSystemInput = false;

// Debug

template <class T>
void debug (String text, T value) {
    Serial.print(text);
    Serial.print(":");
    Serial.println(value);
}

// Filtered inputs

FiltreEntradaDigital inputFiltered[4];

// Global variable to share bw tasks

SharedVar<int> gInp[4];
SharedVar<int> mrscGain;


bool wifiEnabled = false;

// State Machine

bool stateChangingEnable = false;
unsigned long initTimeInStateChangingEnable=0;
bool enableRemote = true;

// Medium Filters

MediumFilter<int> steering(1500);
MediumFilter<int> vel(1500);

// Tasks definition

TaskHandle_t handTskWifi;
void tskWifi (void *pvParameters);

TaskHandle_t handTskLoop;
void tskLoop (void *pvParameters);



///////////////////////////////////////////////////////////////////////////////////

void setup() {

    Serial.begin(115200);

    // Configure inputs
    pinMode(INVERSE_MPU_DIRECTION, INPUT_PULLUP);
    pinMode(GAIN_POT, INPUT);

    // Activate servo signal input pullup resistors
    pinMode(INPUT_STEERING, INPUT_PULLUP);
    pinMode(INPUT_VEL, INPUT_PULLUP);

    // Configure enable
    pinMode(ENABLE_CONTROL, OUTPUT);
    pinMode(ENABLE_CONTROL_INPUT, INPUT_PULLUP);

    // Configure buttons and filter
    pinMode(INPUT_1, INPUT_PULLUP);
    pinMode(INPUT_2, INPUT_PULLUP);
    pinMode(INPUT_3, INPUT_PULLUP);
    pinMode(INPUT_4, INPUT_PULLUP);

    inputFiltered[0].configura(INPUT_1, 50);
    inputFiltered[1].configura(INPUT_2, 50);
    inputFiltered[2].configura(INPUT_3, 50);
    inputFiltered[3].configura(INPUT_4, 50);
    
    // Servo pins
    servoSteering.attach(OUTPUT_STEERING);
    servoSteering.write((limSteeringL + limSteeringR) / 2); // Servo to center position

    servoVelocity.attach(OUTPUT_VEL);
    servoVelocity.writeMicroseconds(uSecVelCenter);

    // MPU 6050 accelerometer / gyro setup

    mpu.begin();

    // RTM Driver (ESP32). Can be used to read any pulse.

    rmtInit();

    // State Machine

    gMode.set(3);
    enableSystem.set(1);

    xTaskCreateUniversal(tskWifi, "TaskWifi", 10000, NULL, 1, &handTskWifi, 0);
    delay(500);

    xTaskCreateUniversal(tskLoop, "TaskLoop", 10000, NULL, 1, &handTskLoop, 1);
    delay(500);

    debug<int>("Ready to Run!", 1);
}


void detectSteeringRange() {
  // input signal calibration (for center point only)
  int steeringuSec = rmtValuePWM(0);
  if (steeringuSec > 500 && steeringuSec < limuSecL) limuSecL = steeringuSec; // around 1000uS
  if (steeringuSec < 2500 && steeringuSec > limuSecR) limuSecR = steeringuSec; // around 2000uS

  // output signal calibration
  int servoAngle = rmtMapConstrain(0, 45, 135); // The range usually is 45 to 135° (+/- 45°)
  if (servoAngle > 20 && servoAngle < limSteeringL) limSteeringL = servoAngle;
  if (servoAngle < 160 && servoAngle > limSteeringR) limSteeringR = servoAngle;
}


void readInputs() {
    mrscGain.set(map(analogRead(GAIN_POT), 0, 1024*4, 0, 100));

    for(int i=0; i<4; i++){
        gInp[i].set(inputFiltered[i].value());    
    }
}


void emergencyControl() {
    servoSteering.writeMicroseconds(limuSecL); // Turn All Left (to avoid going fordward)
    servoVelocity.writeMicroseconds(uSecVelCenter); // Stop motor
}


void controlMrsc() {

    int steeringAngle;
    long gyroFeedback;
    int mrscGainValue = mrscGain.get();

    // Read sensor data
    mpu.update();

    // Compute steering compensation overlay
    int steeringPWMFiltered = steering.filter(rmtValuePWM(0));
    int turnRateSetPoint = map(steeringPWMFiltered, limuSecL, limuSecR, -50, 50); 
    int steering = abs(turnRateSetPoint);
    int gain = map(steering, 0, 50, mrscGainValue, (mrscGainValue / 5)); // MRSC gain around center position is 5 times more!

    // Compute velocity Set Point in Eng.Units
    int velPWMFiltered = vel.filter(rmtValuePWM(1));
    int uSecVel = velPWMFiltered;
    if (uSecVel < uSecVelCenter) uSecVel = uSecVelCenter; // If breaking (or reverse?), velSetPoint = 0;
    int velSetPoint = map(uSecVel, uSecVelCenter, limuSecVelFordward, 0, 100);

    if (steering < 8 && mrscGainValue > 85) { // Straight run @ high gain, "heading hold" mode -------------
        gyroFeedback = mpu.yawAngleStab * headingMultiplier; // degrees

    }

    else { // cornering or low gain, correction depending on yaw rate in °/s --------------------------
        gyroFeedback = mpu.yawRateStab * 50; // degrees/s * speed (always 50%)
        mpu.resetYawStab();
    }

    steeringAngle = turnRateSetPoint - (gyroFeedback * gain / 100);

    // TODO
    // Alternativa !!! amb calcul teòric
    //theoreticalRatio = V * tg(alfa) / l

    //steeringAngle = turnRateSetPoint + ((theoreticalRatio - mpuInversedRatio * static_cast<int>(mpu.yawRateStab)) * gain) / 100

    steeringAngle = constrain(steeringAngle, -50, 50); // range = -50 to 50

    // Control steering servo
    int outputWrite = map(steeringAngle, -50, 50, limSteeringL, limSteeringR);
    servoSteering.write(outputWrite); // 45 - 135°

    // Control velocity. DANGER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    servoVelocity.writeMicroseconds(rmtValuePWM(1));

    if ((millis() - lastTimeDebug)>debugPeriod) {
        debug<int>("turnRateSetPoint", turnRateSetPoint);
        debug<int>("gyroFeedback", gyroFeedback);
        debug<int>("steering", steering);
        debug<int>("gain", gain);
        debug<int>("output", outputWrite);
        Serial.println("-----");

        lastTimeDebug = millis();
    }

    gValuePWMRot.set(rmtValuePWM(0)); 
    gValuePWMVel.set(rmtValuePWM(1));

}

void controlOff() {
    servoSteering.writeMicroseconds((limuSecL + limuSecR)/2); // Turn All Left (to avoid going fordward)
    servoVelocity.writeMicroseconds(uSecVelCenter); // Stop motor
}

void controlManual() {
    servoSteering.writeMicroseconds(gManTurn.get());
    servoVelocity.writeMicroseconds(gManVel.get());

}

void controlDirect() {
    servoSteering.writeMicroseconds(rmtValuePWM(0)); 
    servoVelocity.writeMicroseconds(rmtValuePWM(1));
}

void enableManagment() {

    // Enable system control

    // TODO: Refactor

    if (gValuePWMVel.get() < 1100 && !stateChangingEnable) {
        stateChangingEnable = true;
        initTimeInStateChangingEnable = millis();
    } else if (stateChangingEnable && gValuePWMVel.get() < 1100 ){
        if ((millis() - initTimeInStateChangingEnable) > 2000) {
            enableRemote = !enableRemote;
            stateChangingEnable = false;
        }
    } else if (stateChangingEnable && gValuePWMVel.get() >= 1100) {
        stateChangingEnable = false;
    }

    // TODO: Hardware filter

    if (digitalRead(ENABLE_CONTROL_INPUT) == 1) {
        enableSystemInput = true;
    } else {
        enableSystemInput = false;
    }
    
    if (enableSystem.get() && enableSystemInput && enableRemote) {
        digitalWrite(ENABLE_CONTROL, LOW);
    } else {
        digitalWrite(ENABLE_CONTROL, HIGH);
    }

    // Filter input update

    for(int i=0; i<4; i++){
        inputFiltered[i].update();         
    }

}


// =======================================================================================================
// MAIN LOOP
// =======================================================================================================


void loop() {

}


//
// =======================================================================================================
// TASK LOOP
// =======================================================================================================
//


void tskLoop(void *pvParameters){

    // SETUP or the task
    Serial.println("Task Loop on core: " + String(xPortGetCoreID()));



    // Timers Loop Task

    TickType_t lastTimeTaskLoop;
    TickType_t periodTaskLoop = pdMS_TO_TICKS(10);

    while(true) {
        
        readInputs(); // Read pots and switches

        detectSteeringRange(); // Detect the steering input signal range

        if (gMode.get() == 1) {
            controlOff();
        } else if (gMode.get() == 2) {
            controlDirect();
        } else if (gMode.get() == 3) {
            controlMrsc();
        } else if (gMode.get() == 4) {
            controlManual();
        } 

        vTaskDelayUntil(&lastTimeTaskLoop, periodTaskLoop);
    }

}


//
// =======================================================================================================
// TASK WIFI
// =======================================================================================================
//

void tskWifi(void *pvParameters){

    // SETUP or the task
    Serial.println("Task Wifi on core: " + String(xPortGetCoreID()));

    if (wifiEnabled) InitWifi();
    if (wifiEnabled) Blynk.config(auth);

    TickType_t lastTimeDebugWifi;
    TickType_t periodDebugWifi = pdMS_TO_TICKS(2000);

    unsigned long lastTimeBlynk=0;
    unsigned long periodBlynk=500;

    while(true){

        for(int i=0; i<4; i++){
            Serial.println(gInp[i].get());    
        }
        Serial.println(mrscGain.get());


        if ((millis() - lastTimeBlynk) > periodBlynk) {

            gValuePWMRot.set(rmtValuePWM(0));
            gValuePWMVel.set(rmtValuePWM(1));

            if (wifiEnabled) Blynk.run();

            lastTimeBlynk = millis();
        }

        vTaskDelayUntil(&lastTimeDebugWifi, periodDebugWifi);
    }

}