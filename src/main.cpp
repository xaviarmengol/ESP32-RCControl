// Based on https://github.com/TheDIYGuy999/MRSC_Adapter_3Pin_Servo

const float codeVersion = 0.2; // Software revision

//
// =======================================================================================================
// LIBRARIES
// =======================================================================================================
//

#include <Arduino.h>
#include <Wire.h> 
///#include <Servo.h> // Servo library
#include <ESP32Servo.h>
#include "AccOrientation.h"

// TEST RMT (instead of exceptions)

#include "configure_rmt.h"

//
// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES
// =======================================================================================================
//

// Create Servo objects
Servo servoSteering;
Servo servoVelocity;

// Servo limits (initial value = center position = 90° / 1500uSec)
// Limits are adjusted during the first steering operation!

// Always move the steering wheel inside its entire range after switching on the car
byte limSteeringL = 90, limSteeringR = 90; // usually 45 - 135° (90° is the servo center position)
int limuSecL = 1500, limuSecR = 1500; // usually 1000 - 2000 uSec (1500 uSec is in the servo center position)


// DANGER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// In case of full Auto in Velocity, receiver should be configured WITHOUT REVERSE!!!!!

// Move velocity: CALIBRATE BY HAND

// CenterVel depends if 50%-50% or 30%-70%
int limuSecVelFordward = 1976, limuSecVelReverse = 980, uSecVelCenter=1470; // usually 1000 - 2000 uSec

bool emergencyStop=false;

// MRSC gain
byte mrscGain = 90; // 80 This MRSC gain applies, if you don't have an adjustment pot and don't call readInputs()
double headingMultiplier = 1.0; // adjust until front wheels stay in parallel with the ground, if the car is swiveled around

int generalMrscGain = 1; // 0 Disabled 1 Enabled

// Switch states
boolean mpuInversed = false;


#define INPUT_STEERING 4
#define INPUT_VEL 16

#define OUTPUT_STEERING 12
#define OUTPUT_VEL 14

#define GAIN_POT 1
#define INVERSE_MPU_DIRECTION 2


unsigned long lastTimeDebug=0;
unsigned long lastTimeDebugLoop=0;

unsigned long debugPeriod=5000;

// New MPU library

AccOrientation mpu;


// Debug
template <class T>
void debug (String text, T value) {
    Serial.print(text);
    Serial.print(":");
    Serial.println(value);
}


void setup() {

    Serial.begin(115200);

    // Configure inputs
    pinMode(INVERSE_MPU_DIRECTION, INPUT_PULLUP);
    pinMode(GAIN_POT, INPUT);

    // Activate servo signal input pullup resistors
    pinMode(INPUT_STEERING, INPUT_PULLUP);
    pinMode(INPUT_VEL, INPUT_PULLUP);

    // Servo pins
    servoSteering.attach(OUTPUT_STEERING);
    servoSteering.write((limSteeringL + limSteeringR) / 2); // Servo to center position

    servoVelocity.attach(OUTPUT_VEL);
    servoVelocity.writeMicroseconds(uSecVelCenter);

    // MPU 6050 accelerometer / gyro setup

    mpu.begin();

    // RTM Driver (ESP32). Can be used to read any pulse.

    rmtInit();

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
  mrscGain = map(analogRead(GAIN_POT), 0, 255, 0, 100);
  mpuInversed = !digitalRead(INVERSE_MPU_DIRECTION);
}


void emergencyControl() {
    servoSteering.writeMicroseconds(limuSecL); // Turn All Left (to avoid going fordward)
    servoVelocity.writeMicroseconds(uSecVelCenter); // Stop motor
}


void mrsc() {

    int steeringAngle;
    long gyroFeedback;

    // Read sensor data
    mpu.update();

    // Compute steering compensation overlay
    int turnRateSetPoint = map(rmtValuePWM(0), limuSecL, limuSecR, -50, 50);  // turnRateSetPoint = steering angle (1000 to 2000us) = -50 to 50
    int steering = abs(turnRateSetPoint); // this value is required to compute the gain later on and is always positive
    int gain = map(steering, 0, 50, mrscGain, (mrscGain / 5)); // MRSC gain around center position is 5 times more!

    // Compute velocity Set Point in Eng.Units
    int uSecVel = rmtValuePWM(1);
    if (uSecVel < uSecVelCenter) uSecVel = uSecVelCenter; // If breaking (or reverse?), velSetPoint = 0;
    int velSetPoint = map(uSecVel, uSecVelCenter, limuSecVelFordward, 0, 100);

    if (steering < 5 && mrscGain > 85) { // Straight run @ high gain, "heading hold" mode -------------
        gyroFeedback = mpu.yawAngleStab * headingMultiplier; // degrees

    }

    else { // cornering or low gain, correction depending on yaw rate in °/s --------------------------
        gyroFeedback = mpu.yawRateStab * 50; // degrees/s * speed (always 50%)
        mpu.resetYawStab();
    }


    if (mpuInversed) steeringAngle = turnRateSetPoint + (gyroFeedback * gain / 100) * generalMrscGain;  // Compensation depending on the pot value
    else steeringAngle = turnRateSetPoint - (gyroFeedback * gain / 100) * generalMrscGain;

    int mpuInversedRatio = 1;
    if (mpuInversed) mpuInversedRatio = -1;

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

}

//
// =======================================================================================================
// MAIN LOOP
// =======================================================================================================
//

void loop() {

    //readInputs(); // Read pots and switches

    detectSteeringRange(); // Detect the steering input signal range

    if (!emergencyStop) {
        mrsc(); // Do stability control calculations
    } else {
        emergencyControl();
    }


    if ((millis() - lastTimeDebugLoop) > 500) {

        debug<uint32_t>("Rot", rmtValuePWM(0));
        debug<uint32_t>("Vel", rmtValuePWM(1));
        lastTimeDebugLoop = millis();
    }

}
