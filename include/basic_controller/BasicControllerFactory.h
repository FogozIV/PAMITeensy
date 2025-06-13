//
// Created by fogoz on 09/06/2025.
//

#ifndef BASICCONTROLLERFACTORY_H
#define BASICCONTROLLERFACTORY_H
#include <map>
#include <memory>
#include <basic_controller/PID.h>
#include "BasicController.h"


namespace BasicControllerDeserialisation {
    using return_type = std::shared_ptr<BasicController>;
    using Deserializer = std::function<return_type(std::shared_ptr<BaseRobot> robot, const JsonVariant& json)>;
    extern std::map<BasicControllerType::ControllerType, Deserializer> deserializers;
    extern bool registered;

    void registerTypes();

    Deserializer getDeserializer(BasicControllerType::ControllerType type);

    return_type getFromJson(std::shared_ptr<BaseRobot> robot, const JsonVariant& json);

    bool isTypeCastableTo(BasicControllerType::ControllerType to_cast, BasicControllerType::ControllerType caster);

    template<class Caster>
    std::shared_ptr<Caster> castToController(std::shared_ptr<BasicController> controller, BasicControllerType::ControllerType caster) {
        if (isTypeCastableTo(controller->getType(), caster)) {
            return std::static_pointer_cast<Caster>(controller);
        }
        return nullptr;
    }

    std::shared_ptr<PID> castToPID(std::shared_ptr<BasicController> controller);


}



#endif //BASICCONTROLLERFACTORY_H
