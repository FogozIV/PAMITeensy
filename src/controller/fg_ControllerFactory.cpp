//
// Created by fogoz on 28/07/2025.
//

#include <controller/ControllerFactory.h>

#include "controller/SamsonController.h"
#include "controller/SimpleTripleBasicController.h"

#define ControllerTypesDeserializer \
    CTD(TRIPLE_BASIC, SimpleTripleBasicController)\
    CTD(SAMSON, SamsonController) \
    CTD(BASIC_SPEED, BasicSpeedController)



namespace ControllerFactory {
    std::map<ControllerType, Deserializer> deserializers;
    bool registered = false;

    void registerTypes() {
        if (registered)
            return;
        registered = true;
#define CTD(enum_name, class_name)\
        deserializers[enum_name] = [](std::shared_ptr<BaseRobot> robot, const JsonVariant& json){\
            return class_name::deserialize_as_T<class_name>(robot, json);\
        };

        ControllerTypesDeserializer
    }

    Deserializer getDeserializer(ControllerType type) {
        registerTypes();
        bool contains = deserializers.count(type);
        if (contains) {
            return deserializers[type];
        }
        return deserializers[ControllerType::TRIPLE_BASIC];
    }

    return_type getFromJson(std::shared_ptr<BaseRobot> robot, const JsonVariant& json) {
        if (json["type"].is<BasicControllerType::ControllerType>()) {
            return getDeserializer(json["type"].as<ControllerType>())(robot, json);
        }
        return getDeserializer(ControllerType::TRIPLE_BASIC)(robot, json);

    }

    bool isTypeCastableTo(ControllerType to_cast, ControllerType caster) {
        switch(caster){
            case BASE:
                return true;
            default:
                break;
        }
        return false;
    }


}