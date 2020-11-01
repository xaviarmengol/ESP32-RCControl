#pragma once
#include <Preferences.h>
//#include "SharedVar.hpp" // Optional: To overload load and update

Preferences preferences;

template <class paramStruct>

class ParametersMgmt {

    public:

        ParametersMgmt(std::string nameParam) {
            _nameParam = nameParam;
        }


        bool load (paramStruct& loadParam, paramStruct defaultParam) {
            bool allOK=false;

            _setDefaultParameters(defaultParam);
            allOK = _loadParameters(_param);

            if (!allOK) { 
                _writeDefaultParameters();
                _param = defaultParam;
            }

            loadParam = _param;

            return(allOK);
        }

        // Overloaded SharedVar
        /*
        bool load (SharedVar<paramStruct>& loadSharedParam, paramStruct defaultParam) {
            bool allOK;
            paramStruct param;
            allOK = load(param, defaultParam);
            loadSharedParam.set(param);
            return(allOK);
        }
        */

        // To be updated everytime that a parameter change, or cyclically.

        void update (const paramStruct& param) {       

            if (param == _lastStoredParameters) {
                // No update to avoid writing to flash unnecessarily
            } else {
                _forceUpdate(param);
            }
        }

        // To be used ONLY to ensure that the parameters are well saved

        void _forceUpdate (const paramStruct& param) {

            _writeParameters(param);
            Serial.println("Caution: Flash Write!");

        }


    private:

        std::string _nameParam;
        paramStruct _param;

        paramStruct _lastStoredParameters = paramStruct(); // Initialize to 0
        paramStruct _defaultParameters = paramStruct(); // Initialize to 0

        void _setDefaultParameters (const paramStruct& param) {
            _defaultParameters = param;
        }

        void _writeParameters (const paramStruct& param) {

            preferences.begin(_nameParam.c_str(), false);
            preferences.putBytes(_nameParam.c_str(), &param, sizeof(param));
            _lastStoredParameters = param;
            preferences.end();

        }

        void _writeDefaultParameters () {
            _writeParameters(_defaultParameters);
        }


        bool _loadParameters (paramStruct& param) {

            bool allOK=false;

            preferences.begin(_nameParam.c_str(), true);
            size_t lenParam = preferences.getBytesLength(_nameParam.c_str());
            char buffer[lenParam];
            
            if (lenParam == sizeof(param)) {
                preferences.getBytes(_nameParam.c_str(), buffer, lenParam);
                memcpy(&param, buffer, lenParam); // Copy buffer to param
                _lastStoredParameters = param; // Copy to compare if something changes
                allOK = true;
            } else {
                Serial.println("Error Parameters: Parameters size NOT equal to structure size.");
                allOK = false;
            }

            preferences.end();
            return(allOK);
        }

};


