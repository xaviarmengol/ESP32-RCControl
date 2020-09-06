#include "Arduino.h"
#include "filtreentrada.hpp"


// Clase Filtre Entrada ///////////////


FiltreEntrada::FiltreEntrada(){

}

void FiltreEntrada::configura(int iPin, unsigned long minMillis) {
    _inputPin = iPin;
    _minMillis = minMillis;
    _llindar = 0;
}


void FiltreEntrada::update(){

    _valor = _rawToBool();

    if (_valor != _ultimValor){ // en cas de canvi, reseteja comptador
        _ultimaConmutacioMillis = millis();
    }

    _deltaMillis = millis() - _ultimaConmutacioMillis;

    if (_deltaMillis > _minMillis){
       // _estat = ((_valor/(float)_deltaMillis)>0.5)? 1 : 0; // comparem si ha estat a 1 més de la meitat del temps
       _estat = _valor;
    }

    // Flanc pujada només
    if ((_ultimEstat !=_estat) && (_estat == true)) _estatFlanc = true;
    else _estatFlanc = false;

    _ultimValor = _valor;
    _ultimEstat = _estat;
    
}

int FiltreEntrada::getValorRaw(){
    return (_valorRaw);
}


bool FiltreEntrada::value(){
    return(_estat);
}

bool FiltreEntrada::hasPressed(){
    if (_estatFlanc) {
        Serial.print("Apretat : ");
        Serial.println(getValorRaw());
    }
    return(_estatFlanc);
}

