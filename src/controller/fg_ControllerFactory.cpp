//
// Created by fogoz on 28/07/2025.
//

#include <controller/ControllerFactory.h>
#include "controller/SimpleTripleBasicController.h"

namespace ControllerFactory {
    std::map<ControllerType, Deserializer> deserializers;
    bool registered = false;

    void registerTypes() {
        if (registered)
            return;
        registered = true;
        deserializers[ControllerType::TRIPLE_BASIC] = [](std::shared_ptr<BaseRobot> robot, const JsonVariant& json){
            return SimpleTripleBasicController::deserialize_as_T<SimpleTripleBasicController>(robot, json);
        };
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