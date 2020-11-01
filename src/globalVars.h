#pragma once

#include "SharedVar.hpp"

/////////////////////////////////////////////////////////////////////////////////////
// Parameters definition. They should be shared between tasks and also saved in Flash

// Parameters Servo

struct paramServo_t {
    int pwmLimInf;
    int pwmCenter;
    int pwmLimSup;

    bool operator==(const paramServo_t& other) const {
        return (pwmLimInf == other.pwmLimInf && 
                pwmCenter == other.pwmCenter && 
                pwmLimSup == other.pwmLimSup);
    }

    void println(String name = "") {
        String output = String(name + " - Inf: " + String(pwmLimInf) + " Cent: " + String(pwmCenter) + " Sup: " + String(pwmLimSup));
        Serial.println(output);
    }

};

SharedVar<paramServo_t> gParamServoSteering; 
SharedVar<paramServo_t> gParamServoVel; 

/////////////////////////////////////////////////////////////////////////////////////
// Global variables. Only shared between tasks

SharedVar<int> gInp[4];
SharedVar<int> gMrscGain;

SharedVar<int> gManVel;
SharedVar<int> gManTurn;
SharedVar<int> gMode; 
SharedVar<int> gValuePWMRot;
SharedVar<int> gValuePWMVel;
SharedVar<bool> gEnableSystem;