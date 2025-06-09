//
// Created by fogoz on 09/06/2025.
//

#ifndef BASICCONTROLLERFACTORY_H
#define BASICCONTROLLERFACTORY_H
#include <map>
#include <memory>

#include "BasicController.h"


namespace BasicControllerDeserialisation {
    using return_type = std::shared_ptr<BasicController>;
    using Deserializer = std::function<return_type(std::shared_ptr<BaseRobot> robot, const JsonVariant& json)>;
    extern std::map<BasicControllerType::ControllerType, Deserializer> deserializers;
    extern bool registered;

    void registerTypes();

    Deserializer getDeserializer(BasicControllerType::ControllerType type);

    return_type getFromJson(std::shared_ptr<BaseRobot> robot, const JsonVariant& json);


}



#endif //BASICCONTROLLERFACTORY_H
