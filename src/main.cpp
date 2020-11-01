// Based on https://github.com/TheDIYGuy999/MRSC_Adapter_3Pin_Servo

const float codeVersion = 0.2; // Software revision

#include <Arduino.h>
#include <Wire.h> 
#include <ESP32Servo.h>

#include <Adafruit_I2CDevice.h> //??

// Support Classes
#include "AccOrientation.hpp"
#include "Filtreentrada.hpp"
#include "MediumFilter.hpp"
#include "ParametersMgmt.hpp"
#include "ParametersMgmtShared.hpp"
#include "SharedVar.hpp"

// Global information
#include "globalVars.h"
#include "pinOut.h"

#include "configure_rmt.h" // RMT (instead of exceptions)
#include "controlSystem.h"

#include "blynkConf.h"
#include "wifiUtils.h"

// Includes for GUI

#include "Screen.h"
#include "ButtonInfoBar.h"
#include "StatusBar.h"
#include "Button.h"
#include "WidgetMosaicComp.h"
#include "AppScreen.h"
#include "UpDownButton.h"
#include "Logscreen.h"
#include "ParamButton.h"

#include "GUIHal.h"

GUIHal GUI(128, 64);

// Debug

template <class T>
void debug (String text, T value) {
    Serial.print(text);
    Serial.print(":");
    Serial.println(value);
}


// Enable management
// TODO: Create a specific class

bool stateChangingEnable = false;
unsigned long initTimeInStateChangingEnable=0;
bool enableRemote = true;


// Tasks definition

TaskHandle_t handTskControl;
void tskControl (void *pvParameters);

TaskHandle_t handTskWifi;
void tskWifi (void *pvParameters);

TaskHandle_t handTskGui;
void tskGui (void *pvParameters);

TaskHandle_t handTskParam;
void tskParam (void *pvParameters);

void pinModeConfig () {
    
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

}

void detectSteeringRange() {
    // input signal calibration (for center point only)
    int steeringuSec = gValuePWMRot.get();

    paramServo_t paramSteering = gParamServoSteering.get();

    if (steeringuSec > 500 && steeringuSec < paramSteering.pwmLimInf) {
        paramSteering.pwmLimInf = steeringuSec;
        gParamServoSteering.set(paramSteering); // around 1000uS
    }

    if (steeringuSec < 2500 && steeringuSec > paramSteering.pwmLimSup) {
        paramSteering.pwmLimSup = steeringuSec;
        gParamServoSteering.set(paramSteering);  // around 2000uS
    }

}


void enableManagment() {

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

    bool enableSystemInput;

    if (digitalRead(ENABLE_CONTROL_INPUT) == 1) {
        enableSystemInput = true;
    } else {
        enableSystemInput = false;
    }
    
    if (gEnableSystem.get() && enableSystemInput && enableRemote) {
        digitalWrite(ENABLE_CONTROL, LOW);
    } else {
        digitalWrite(ENABLE_CONTROL, HIGH);
    }

}

///////////////////////////////////////////////////////////////////////////////////

void setup() {

    Serial.begin(115200);

    pinModeConfig();

    // RTM Driver (ESP32). Can be used to read any pulse.

    rmtInit();

    // State Machine and variables

    gMode.set(3);
    gEnableSystem.set(1);
    gMrscGain.set(50);
    
    // Task creation

    xTaskCreateUniversal(tskParam, "TaskParam", 10000, NULL, 3, &handTskParam, 0);
    delay(500);

    xTaskCreateUniversal(tskControl, "TaskControl", 10000, NULL, 1, &handTskControl, 1);
    delay(500);

    xTaskCreateUniversal(tskWifi, "TaskWifi", 10000, NULL, 2, &handTskWifi, 0);
    delay(500);

    xTaskCreateUniversal(tskGui, "TaskGui", 10000, NULL, 1, &handTskGui, 0);
    delay(500);
    

    debug<int>("Ready to Run!", 1);
}

// =======================================================================================================
// MAIN LOOP
// =======================================================================================================


void loop() {
    delay(10000);
}


//
// =======================================================================================================
// TASK LOOP
// =======================================================================================================
//


void tskControl(void *pvParameters){

    // SETUP or the task
    Serial.println("Task Loop on core: " + String(xPortGetCoreID()));
    TickType_t lastTimeTask=0;
    TickType_t periodTask = pdMS_TO_TICKS(10);

    // Create Servo objects
    Servo servoSteering;
    Servo servoVelocity;

    // MPU library
    bool mpuActive = true;

    AccOrientation mpu;
    if (mpuActive) mpu.begin();

    // Servo pins

    servoSteering.attach(OUTPUT_STEERING);
    servoSteering.write(gParamServoSteering.get().pwmCenter); // Servo to center position

    servoVelocity.attach(OUTPUT_VEL);
    servoVelocity.writeMicroseconds(gParamServoVel.get().pwmCenter);


    // Medium Filters

    MediumFilter<int> steeringMediumFilter(1500);
    MediumFilter<int> velMediumFilter(1500);

    steeringMediumFilter.enableFilter(true);
    velMediumFilter.enableFilter(true);

    while(true) {
        
        // Move values from interrups (RMT) to global variables
        // Can be done in any fast task

        gValuePWMRot.set(steeringMediumFilter.filter(rmtValuePWM(0))); 
        gValuePWMVel.set(velMediumFilter.filter(rmtValuePWM(1)));

        // Read pots

        // TODO: SOLVE POTENCIOMETER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        //gMrscGain.set(map(analogRead(GAIN_POT), 0, 1024*4, 0, 100));

        //detectSteeringRange(); // Detect the steering input signal range

        if (mpuActive) mpu.update();

        // Working Modes

        int mode = gMode.get();

        enableManagment();

        if (mode == 1) {
            controlOff(servoSteering, servoVelocity);
        } else if (mode == 2) {
            controlDirect(servoSteering, servoVelocity);
        } else if (mode == 3) {
            if (mpuActive) controlMrsc(mpu, servoSteering, servoVelocity);
            else controlOff(servoSteering, servoVelocity);
        } else if (mode == 4) {
            controlManual(servoSteering, servoVelocity);
        } 

        vTaskDelayUntil(&lastTimeTask, periodTask);
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
    TickType_t lastTimeDebug=0;
    TickType_t periodDebug = pdMS_TO_TICKS(50);

    bool wifiEnabled = true;

    if (wifiEnabled) InitWifi();
    if (wifiEnabled) Blynk.config(auth);

    while(true){

        if (wifiEnabled) Blynk.run();

        vTaskDelayUntil(&lastTimeDebug, periodDebug);
    }

}



//
// =======================================================================================================
// TASK Gui
// =======================================================================================================
//


void tskGui(void *pvParameters){

    // SETUP or the task
    Serial.println("Task Gui on core: " + String(xPortGetCoreID()));
    TickType_t lastTime=0;
    TickType_t period = pdMS_TO_TICKS(100);

    // Declare Gui variables

    paramServo_t paramServoSteering = gParamServoSteering.get();
    paramServo_t paramServoVel = gParamServoVel.get();

    int pwmSteering = gValuePWMRot.get();
    int pwmVel = gValuePWMVel.get();
    
    bool inputVariables[4];

    // Begin GUI

    GUI.Lcd.begin();
    
    //using namespace Codingfield::UI;
    Codingfield::UI::AppScreen* screen;
    Codingfield::UI::StatusBar* topBar;
    Codingfield::UI::ButtonInfoBar* bottomBar;
    Codingfield::UI::WidgetMosaicComp* mosaic;

    Codingfield::UI::ParamButton* paramMinServoSteering;
    Codingfield::UI::ParamButton* paramCenterServoSteering;
    Codingfield::UI::ParamButton* paramMaxServoSteering;

    Codingfield::UI::ParamButton* paramMinServoVel;
    Codingfield::UI::ParamButton* paramCenterServoVel;
    Codingfield::UI::ParamButton* paramMaxServoVel;

    Codingfield::UI::Widget* focus;


    // Instanciate and configure all widgets

    topBar = new Codingfield::UI::StatusBar();
    topBar->SetUptime(0);
    topBar->SetWifiStatus(Codingfield::UI::StatusBar::WifiStatuses::No_signal);

    bottomBar = new Codingfield::UI::ButtonInfoBar();
    bottomBar->SetButtonAText("<");
    bottomBar->SetButtonBText("SELECT");
    bottomBar->SetButtonCText(">");

    mosaic = new Codingfield::UI::WidgetMosaicComp(nullptr, 3, 2, topBar, bottomBar);
    mosaic->SetBackgroundColor(GUI.Lcd.COLOR_BLACK);
    mosaic->SetTextColor(GUI.Lcd.COLOR_WHITE);
    mosaic->SetTitle("Adj Servos");
    mosaic->SetTextSize(1);

    screen = new Codingfield::UI::AppScreen(Codingfield::UI::Size(128, 64), GUI.Lcd.COLOR_BLACK, topBar, bottomBar, mosaic);

    // Servo Steering Parameter

    paramMinServoSteering = new Codingfield::UI::ParamButton(mosaic, &pwmSteering, &(paramServoSteering.pwmLimInf)); 
    paramMinServoSteering->SetBackgroundColor(GUI.Lcd.COLOR_WHITE);
    paramMinServoSteering->SetTextColor(GUI.Lcd.COLOR_BLACK);
    paramMinServoSteering->SetTitle("mS");

    paramCenterServoSteering = new Codingfield::UI::ParamButton(mosaic, &pwmSteering, &(paramServoSteering.pwmCenter)); 
    paramCenterServoSteering->SetBackgroundColor(GUI.Lcd.COLOR_WHITE);
    paramCenterServoSteering->SetTextColor(GUI.Lcd.COLOR_BLACK);
    paramCenterServoSteering->SetTitle("cS");

    paramMaxServoSteering = new Codingfield::UI::ParamButton(mosaic, &pwmSteering, &(paramServoSteering.pwmLimSup)); 
    paramMaxServoSteering->SetBackgroundColor(GUI.Lcd.COLOR_WHITE);
    paramMaxServoSteering->SetTextColor(GUI.Lcd.COLOR_BLACK);
    paramMaxServoSteering->SetTitle("xS");

    // Servo Vel Parameter

    paramMinServoVel = new Codingfield::UI::ParamButton(mosaic, &pwmVel, &(paramServoVel.pwmLimInf)); 
    paramMinServoVel->SetBackgroundColor(GUI.Lcd.COLOR_WHITE);
    paramMinServoVel->SetTextColor(GUI.Lcd.COLOR_BLACK);
    paramMinServoVel->SetTitle("mV");

    paramCenterServoVel = new Codingfield::UI::ParamButton(mosaic, &pwmVel, &(paramServoVel.pwmCenter)); 
    paramCenterServoVel->SetBackgroundColor(GUI.Lcd.COLOR_WHITE);
    paramCenterServoVel->SetTextColor(GUI.Lcd.COLOR_BLACK);
    paramCenterServoVel->SetTitle("cV");

    paramMaxServoVel = new Codingfield::UI::ParamButton(mosaic, &pwmVel, &(paramServoVel.pwmLimSup)); 
    paramMaxServoVel->SetBackgroundColor(GUI.Lcd.COLOR_WHITE);
    paramMaxServoVel->SetTextColor(GUI.Lcd.COLOR_BLACK);
    paramMaxServoVel->SetTitle("xV");

    // Give the focus to the main screen
    focus = mosaic;

    // Draw the screen and all its children
    //screen->Draw();

    // INPUTS GUI

    FiltreEntradaDigital inputFiltered[4];

    // GUI Buttons definition

    GUI.BtnA.defineVariable(&inputVariables[0]);
    GUI.BtnB.defineVariable(&inputVariables[1]);
    GUI.BtnC.defineVariable(&inputVariables[2]);

    const int milisFilter = 50;
    inputFiltered[0].config(INPUT_1, milisFilter);
    inputFiltered[1].config(INPUT_2, milisFilter);
    inputFiltered[2].config(INPUT_3, milisFilter);
    inputFiltered[3].config(INPUT_4, milisFilter);


    while(true){
        
        // Move from Globals to GUI Variables

        paramServoSteering = gParamServoSteering.get();
        paramServoVel = gParamServoVel.get();
        pwmSteering = gValuePWMRot.get();
        pwmVel = gValuePWMVel.get();

        // Move from Phisical to GUI Variables

        for(int i=0; i<3; i++) {
            inputVariables[i] = inputFiltered[i].value();    
        }

        screen->Draw(focus);
        GUI.Lcd.update();

        // Move Phisical values to Globals

        for(int i=0; i<4; i++){
            gInp[i].set(inputFiltered[i].value());    
        }

        // Move from GUI Variables to Globals (only if they can be modified)

        gParamServoSteering.set(paramServoSteering);
        gParamServoVel.set(paramServoVel);

        // Phisical inputs filter update

        for(int i=0; i<4; i++){
            inputFiltered[i].update();         
        }
        
        vTaskDelayUntil(&lastTime, period);
    }

}



//
// =======================================================================================================
// TASK PARAM
// =======================================================================================================
//

void tskParam(void *pvParameters){

    // SETUP or the task
    Serial.println("Task Param on core: " + String(xPortGetCoreID()));
    TickType_t lastTime=0;
    TickType_t period = pdMS_TO_TICKS(1000);

    // Parameters management to be load/saved flash

    ParametersMgmtShared<paramServo_t> parSteeringMgmt("servosStering");
    ParametersMgmtShared<paramServo_t> parVelMgmt("servosVeloc");
    
    // Load servo parameters Steering

    paramServo_t paramServoSteeringDefault;
    paramServoSteeringDefault.pwmLimInf = 1000;
    paramServoSteeringDefault.pwmCenter = 1000;
    paramServoSteeringDefault.pwmLimSup = 1000;

    parSteeringMgmt.load(gParamServoSteering, paramServoSteeringDefault);

    gParamServoSteering.get().println("Load gParamServoSteering");

    // Load servo parameters Vel

    paramServo_t paramServoVelDefault;// = {980, 1470, 1976};
    paramServoVelDefault.pwmLimInf = 980;
    paramServoVelDefault.pwmCenter = 1470;
    paramServoVelDefault.pwmLimSup = 1976;

    parVelMgmt.load(gParamServoVel, paramServoVelDefault);
    gParamServoVel.get().println("Load gParamServoVel");

    while(true){

        // Saving parameter if have changed in any task
        
        parSteeringMgmt.update(gParamServoSteering);
        parVelMgmt.update(gParamServoVel);

        gParamServoSteering.get().println("Inside While gParamServoSteer");
        gParamServoVel.get().println("Inside While gParamServoVel");

        Serial.print("PWM Rot:"); Serial.println(gValuePWMRot.get());
        Serial.print("PWM Vel:"); Serial.println(gValuePWMVel.get());
        
        vTaskDelayUntil(&lastTime, period);
    }

}
