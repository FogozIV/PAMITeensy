//
// Created by fogoz on 28/07/2025.
//

#include <controller/ControllerFactory.h>


namespace ControllerFactory {
    std::map<ControllerType, Deserializer> deserializers;
    bool registered = false;

    void registerTypes() {
        if (registered)
            return;
        registered = true;

    }

    Deserializer getDeserializer(ControllerType type) {
        registerTypes();
    }

    return_type getFromJson(std::shared_ptr<BaseRobot> robot, const JsonVariant& json) {

    }

    bool isTypeCastableTo(ControllerType to_cast, ControllerType caster) {

    }


}