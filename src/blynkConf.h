#include <BlynkSimpleEsp32_SSL.h>
#include "SharedVar.h"

SharedVar<int> gManVel;
SharedVar<int> gManTurn;
SharedVar<int> gMode; 
SharedVar<int> gValuePWMRot;
SharedVar<int> gValuePWMVel;
SharedVar<bool> enableSystem;

BLYNK_WRITE(V0) { // Joystick
    gManTurn.set(param.asInt());
}

BLYNK_WRITE(V1) { // Joystick
    gManVel.set(param.asInt());
}

BLYNK_WRITE(V2) { // Mode
    gMode.set(param.asInt());
}

BLYNK_READ(V3) { // PWM ROT
    Blynk.virtualWrite(V3, gValuePWMRot.get());
}

BLYNK_READ(V4) { // PWM VEL
    Blynk.virtualWrite(V4, gValuePWMVel.get());
}

BLYNK_WRITE(V5) { // Enable Button

    enableSystem.set(param.asInt() == 1);

}

