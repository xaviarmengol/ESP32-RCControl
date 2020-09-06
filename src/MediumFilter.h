#pragma once

// Medium filter

template <class T>
class MediumFilter{
    public:
        MediumFilter(T initValue) {
            for(int i=0; i<3; i++){
                _values[i] = static_cast<T>(initValue);                
            }
        };

        T feedAndReturn(T value) {
            T returnValue;

            // Dischard last value and feed the pipe
            _values[2] = _values[1];
            _values[1] = _values[0];
            _values[0] = value;

            if (_values[0] < _values[1] && _values[1] <= _values[2]) {
                returnValue = _values[1];
            } else if (_values[0] < _values[2] && _values[2] <= _values[1]) {
                returnValue = _values[2];
            } else {
                returnValue = _values[0];
            }

            if (_enableFilter) return (returnValue);
            else return (value);
        };

        void enable (bool enableFlag) {
            _enableFilter = enableFlag;
        }
    
    private:
        T _values[3];
        bool _enableFilter=true;
};

