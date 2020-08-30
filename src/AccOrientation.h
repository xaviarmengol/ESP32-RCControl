// Info: https://mjwhite8119.github.io/Robots/mpu6050

#pragma once

/* =========================================================================
NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
depends on the MPU-6050's INT pin being connected to the Arduino's
external interrupt #0 pin. 
========================================================================= */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define INTERRUPT_PIN 5 

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}


class AccOrientation {

private:
    MPU6050 mpu; // class default I2C address is 0x68

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
    VectorFloat gravity;    // [x, y, z]            gravity vector

    int accFullScale = 0;

    float _yprReference=0.0;

public:
    AccOrientation(/* args */);
    void begin();
    void update();
    ~AccOrientation();
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements

    VectorInt16 gy;         // [x, y, z]            gyro sensor measurements

    // Special Stabilizer

    double yawAngleStab = 0.0;
    double yawRateStab = 0.0;
    void resetYawStab();

    // Print utils

    void printV(VectorInt16 v, std::string name);
    void printV(VectorFloat v, std::string name);
    void printV(Quaternion q, std::string name);
};

AccOrientation::AccOrientation(/* args */)
{
}

AccOrientation::~AccOrientation()
{
}

void AccOrientation::begin(){
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity.
    // If autocalibration done, manual is not used.

    mpu.setXGyroOffset(-552);
    mpu.setYGyroOffset(-54);
    mpu.setZGyroOffset(-33);

    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {

        // 0 = +/- 2g
        // 1 = +/- 4g
        // 2 = +/- 8g
        // 3 = +/- 16g
        accFullScale = 0;

        // TODO: Calibration is NOT working with FullScaleFactor diferent than 0
        mpu.setFullScaleAccelRange(accFullScale);

        // * 0 = +/- 250 degrees/sec
        // * 1 = +/- 500 degrees/sec
        // * 2 = +/- 1000 degrees/sec
        // * 3 = +/- 2000 degrees/sec

        mpu.setFullScaleGyroRange(0);

        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println();
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        resetYawStab();

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

void AccOrientation::update() {

    // if programming failed, don't try to do anything
    if (!dmpReady) {
        Serial.println("Sensor not ready or begin not called");
        return;
    }
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        mpu.dmpGetAccel(&aa, fifoBuffer);
        
        mpu.dmpGetGyro(&gy, fifoBuffer);

        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

        /*
        printV(aa, "acc ");
        printV(aaReal, "accR");
        printV(aaWorld, "accW");
        */

        // Compute yawAngleStab and yawRateStab (For stabilizer algorithm)

        yawRateStab = gy.z / 131.0; // (250.0 / 32750.0);

        yawAngleStab = ypr[0] * RAD_TO_DEG - _yprReference;
        if (yawAngleStab < -180.0 ) yawAngleStab += 360.0;
        else if (yawAngleStab > 180.0) yawAngleStab -= 360.0;
    }

}

void AccOrientation::resetYawStab() {
    _yprReference = ypr[0] * RAD_TO_DEG;
}

void AccOrientation::printV(VectorInt16 v, std::string name){
    Serial.print(name.c_str());
    Serial.print("=(");
    Serial.print(v.x);
    Serial.print(" ");
    Serial.print(v.y);
    Serial.print(" ");
    Serial.print(v.z);
    Serial.println(")");
}
void AccOrientation::printV(VectorFloat v, std::string name){
    Serial.print(name.c_str());
    Serial.print("=(");
    Serial.print(v.x);
    Serial.print(" ");
    Serial.print(v.y);
    Serial.print(" ");
    Serial.print(v.z);
    Serial.println(")");
}
void AccOrientation::printV(Quaternion q, std::string name){
    Serial.print(name.c_str());
    Serial.print("=(");
    Serial.print(q.w);
    Serial.print(" ");
    Serial.print(q.x);
    Serial.print(" ");
    Serial.print(q.y);
    Serial.print(" ");
    Serial.print(q.z);
    Serial.println(")");

}