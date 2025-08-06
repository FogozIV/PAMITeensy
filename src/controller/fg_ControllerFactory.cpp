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
            case BASIC_SPEED: {
                switch (to_cast) {
                    case TRIPLE_BASIC:
                        return false;
                    case BASE:
                        return false;
                    case BASIC_SPEED:
                        return true;
                    case SAMSON:
                        return true;
                }
            }
            default:
                break;
        }
        return false;
    }
    template<typename T>
    std::shared_ptr<T> castTo(std::shared_ptr<BaseController> controller, ControllerType caster) {
        if (isTypeCastableTo(controller->getType(), caster)) {
            return std::static_pointer_cast<T>(controller);
        }
        return nullptr;
    }
    template<typename T>
    std::shared_ptr<T> upgradeTo(std::shared_ptr<BaseRobot> robot,std::shared_ptr<BaseController> controller) {
        return std::make_shared<T>(robot, controller);
    }


}