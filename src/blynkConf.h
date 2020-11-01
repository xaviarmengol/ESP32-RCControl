#include <BlynkSimpleEsp32_SSL.h>
#include "globalVars.h"

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

    gEnableSystem.set(param.asInt() == 1);

}

