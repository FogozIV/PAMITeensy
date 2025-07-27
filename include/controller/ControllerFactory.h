//
// Created by fogoz on 28/07/2025.
//

#ifndef CONTROLLERFACTORY_H
#define CONTROLLERFACTORY_H
#include <map>
#include <memory>
#include "ArduinoJson.h"
#include "BaseController.h"
#include "ControllerTypes.h"

class BaseRobot;

namespace ControllerFactory {
    using return_type = std::shared_ptr<BaseController>;
    using Deserializer = std::function<return_type(std::shared_ptr<BaseRobot> robot, const JsonVariant& json)>;
    extern std::map<ControllerType, Deserializer> deserializers;
    extern bool registered;

    void registerTypes();

    Deserializer getDeserializer(ControllerType type);

    return_type getFromJson(std::shared_ptr<BaseRobot> robot, const JsonVariant& json);

    bool isTypeCastableTo(ControllerType to_cast, ControllerType caster);

    template<class Caster>
    std::shared_ptr<Caster> castToController(std::shared_ptr<BaseController> controller, ControllerType caster) {
        if (isTypeCastableTo(controller->getType(), caster)) {
            return std::static_pointer_cast<Caster>(controller);
        }
        return nullptr;
    }


};


#endif //CONTROLLERFACTORY_H
