#ifndef FILTREENTRADA_HPP_INCLUDED
#define FILTREENTRADA__HPP_INCLUDED

#include "Arduino.h"

class FiltreEntrada { // Pure virtual. Necessita ser derivada

protected:

    int _inputPin;
    int _llindar;
    unsigned long _minMillis;

    bool _estat=false;
    bool _ultimEstat=false;
    bool _estatFlanc;

    int _valorRaw;
    bool _valor;
    bool _ultimValor=false;
    float _valorAcum=0.0;

    unsigned long _ultimaConmutacioMillis;
    unsigned long _deltaMillis;

    //bool _touchToBool(uint16_t entradaTouch, uint16_t llindar);
    virtual bool _rawToBool() = 0;

    FiltreEntrada(); // Constructor protected per evitar que sigui instanciat directament

public:
    
    virtual void configura(int iPin, unsigned long minMillis);
    int getValorRaw();
    void update();
    bool value(); // estat del boto
    bool hasPressed(); // flanc pujada
};

// Filtre entrada per a pulsadors tàctils

class FiltreEntradaTouch : public FiltreEntrada {

public:

    FiltreEntradaTouch() {};

    void configura(int iPin, unsigned long minMillis, int llindar) {
        FiltreEntrada::configura(iPin, minMillis);
        _llindar = llindar;
    }

    bool _rawToBool () {
        _valorRaw = touchRead(_inputPin);
        return(_touchToBool(_valorRaw, _llindar));
    }    

    bool _touchToBool(uint16_t entradaTouch, uint16_t llindar){
        return(entradaTouch <= llindar);
    }

};

// Filtre entrada per entrades digitals

class FiltreEntradaDigital : public FiltreEntrada {

public:

    FiltreEntradaDigital() {};

    void configura(int iPin, unsigned long minMillis) {
        FiltreEntrada::configura(iPin, minMillis);
        pinMode(iPin, INPUT_PULLUP); // Pullup -> signal to 0V
    }

    bool _rawToBool () {
        _valorRaw = 0;
        return(digitalRead(_inputPin));
    }    

};

#endif