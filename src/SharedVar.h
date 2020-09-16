#pragma once

#include <Arduino.h>

template <class T>

class SharedVar {
    public:
        SharedVar() {
            // Only 1 position because we are interested in real time value
            _queue = xQueueCreate( 1 , sizeof(T));
            _allocated = (_queue != NULL);

            if(!_allocated) Serial.println("Variable can not be allocated. Check memory allocation.");
                
        };

        void set (T value) {
            if (_allocated) {
                xQueueOverwrite(_queue, &value);
                _lastUpdateMillis = millis();
            } else {
                Serial.println("Variable not allocated in initialitzation. Check memory allocation.");
            }
        }

        T get () {
            int bufferWithValue;
            T value;//[1];
            if (_allocated) {
                bufferWithValue = (xQueuePeek(_queue, &value, 0) == pdPASS);

                if (bufferWithValue) return(value);//[0]);
                else return(0);
                
            } else {
                Serial.println("Variable not allocated in initialitzation. Check memory allocation.");
                return(0);
            }
        }

        unsigned long getLastUpdate() {
            return(_lastUpdateMillis);
        }

    private:
        QueueHandle_t _queue;
        bool _allocated=false;
        unsigned long _lastUpdateMillis=0;
};