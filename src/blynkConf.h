#include <BlynkSimpleEsp32_SSL.h>

int gManVel;
int gManTurn;
int gMode = 3; // Auto Control
int gValuePWMRot = 0;
int gValuePWMVel = 0;
bool enableSystem=true;

BLYNK_WRITE(V0) { // Joystick
    gManTurn = param.asInt();
}

BLYNK_WRITE(V1) { // Joystick
    gManVel = param.asInt();
}

BLYNK_WRITE(V2) { // Mode
    gMode = param.asInt();
}

BLYNK_READ(V3) { // PWM ROT
    Blynk.virtualWrite(V3, gValuePWMRot);
}

BLYNK_READ(V4) { // PWM VEL
    Blynk.virtualWrite(V4, gValuePWMVel);
}

BLYNK_WRITE(V5) { // Emergency
    int value = param.asInt();

    if (value == 1) {
        enableSystem = true;
    } else {
        enableSystem = false;
    }
}

