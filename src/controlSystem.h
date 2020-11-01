
#include <Arduino.h>
#include "AccOrientation.hpp"

#include <ESP32Servo.h>
#include "globalVars.h"

void controlMrsc(AccOrientation& mpu, Servo& servoSteering, Servo& servoVelocity) {


    // Get global variables and parameters

    int mrscGainValue = gMrscGain.get();
    int steeringPWM = gValuePWMRot.get();
    int velPWM = gValuePWMVel.get();

    paramServo_t paramServoSteering = gParamServoSteering.get();
    paramServo_t paramServoVel = gParamServoVel.get();

    // Start control algorithm

    // Compute steering compensation overlay
    int turnRateSetPoint = map(steeringPWM, paramServoSteering.pwmLimInf, paramServoSteering.pwmLimSup, -50, 50); 
    int steering = abs(turnRateSetPoint);
    int gain = map(steering, 0, 50, mrscGainValue, (mrscGainValue / 5)); // MRSC gain around center position is 5 times more!

    // Compute velocity Set Point in Eng.Units
    int uSecVel = velPWM;
    if (uSecVel < paramServoVel.pwmCenter) uSecVel = paramServoVel.pwmCenter; // If breaking (or reverse?), velSetPoint = 0;
    int velSetPoint = map(uSecVel, paramServoVel.pwmCenter, paramServoVel.pwmLimSup, 0, 100);

    long gyroFeedback;

    if (steering < 8 && mrscGainValue > 85) { // Straight run @ high gain, "heading hold" mode -------------
        gyroFeedback = mpu.yawAngleStab; // degrees
    }

    else { // cornering or low gain, correction depending on yaw rate in °/s --------------------------
        gyroFeedback = mpu.yawRateStab * 50; // degrees/s * speed (always 50%)
        mpu.resetYawStab();
    }

    int steeringAngle = turnRateSetPoint - (gyroFeedback * gain / 100);

    // TODO
    // Alternativa !!! amb calcul teòric
    //theoreticalRatio = V * tg(alfa) / l

    //steeringAngle = turnRateSetPoint + ((theoreticalRatio - mpuInversedRatio * static_cast<int>(mpu.yawRateStab)) * gain) / 100

    steeringAngle = constrain(steeringAngle, -50, 50); // range = -50 to 50

    // Control steering servo
    int outputSteeringPWM = map(steeringAngle, -50, 50, paramServoSteering.pwmLimInf, paramServoSteering.pwmLimSup);
    servoSteering.writeMicroseconds(outputSteeringPWM); // 45 - 135°

    // Control velocity. DANGER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    servoVelocity.writeMicroseconds(velPWM);

    /* REMOVE

    if ((millis() - lastTimeDebug)>debugPeriod) {
        debug<int>("turnRateSetPoint", turnRateSetPoint);
        debug<int>("gyroFeedback", gyroFeedback);
        debug<int>("steering", steering);
        debug<int>("gain", gain);
        debug<int>("output", outputWrite);
        Serial.println("-----");

        lastTimeDebug = millis();
    }

    */



}

void controlOff(Servo& servoSteering, Servo& servoVelocity) {
    servoSteering.writeMicroseconds(gParamServoSteering.get().pwmCenter); // Turn All Left (to avoid going fordward)
    servoVelocity.writeMicroseconds(gParamServoVel.get().pwmCenter); // Stop motor
}

void controlManual(Servo& servoSteering, Servo& servoVelocity) {
    servoSteering.writeMicroseconds(gManTurn.get());
    servoVelocity.writeMicroseconds(gManVel.get());
}

void controlDirect(Servo& servoSteering, Servo& servoVelocity) {
    servoSteering.writeMicroseconds(gValuePWMRot.get()); 
    servoVelocity.writeMicroseconds(gValuePWMVel.get());
}

