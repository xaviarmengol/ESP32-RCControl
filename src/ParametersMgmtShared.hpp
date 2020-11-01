#pragma once
#include "ParametersMgmt.hpp"
#include "SharedVar.hpp" 

template <class paramStruct>
class ParametersMgmtShared : public ParametersMgmt <paramStruct> {

    public:

        ParametersMgmtShared (std::string nameParameter) : ParametersMgmt<paramStruct>(nameParameter) {}

        bool load (SharedVar<paramStruct>& loadSharedParam, paramStruct defaultParam) {
            bool allOK;
            paramStruct param;
            allOK = ParametersMgmt<paramStruct>::load(param, defaultParam);
            loadSharedParam.set(param);
            return(allOK);
        }

        void update (SharedVar<paramStruct>& sharedParam) {
            paramStruct param;
            param = sharedParam.get();
            ParametersMgmt<paramStruct>::update(param);
        }

};


