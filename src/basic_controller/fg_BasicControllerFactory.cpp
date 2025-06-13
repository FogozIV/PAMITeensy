//
// Created by fogoz on 09/06/2025.
//

#include <basic_controller/BasicControllerFactory.h>

#include "basic_controller/PID.h"
#include "basic_controller/PIDFilteredD.h"
#include "basic_controller/PIDSpeedFeedForward.h"

namespace BasicControllerDeserialisation {
   std::map<BasicControllerType::ControllerType, Deserializer> deserializers;
   bool registered = false;
   void registerTypes() {
      if (registered) {
         return;
      }
      registered = true;
      deserializers[BasicControllerType::ControllerType::PID] = [](std::shared_ptr<BaseRobot> robot, const JsonVariant& json) {
         return PID::deserialize_as_T<PID>(robot, json);
      };
      deserializers[BasicControllerType::ControllerType::PIDSpeedFeedForward] = [](std::shared_ptr<BaseRobot> robot, const JsonVariant& json) {
         return PIDSpeedFeedForward::deserialize_as_T<PIDSpeedFeedForward>(robot, json);
      };

      deserializers[BasicControllerType::ControllerType::PIDFilteredD] = [](std::shared_ptr<BaseRobot> robot, const JsonVariant& json) {
         return PIDFilteredD::deserialize_as_T<PIDFilteredD>(robot, json);
      };
   }

   Deserializer getDeserializer(BasicControllerType::ControllerType type) {
      registerTypes();
      bool contains = deserializers.count(type);
      if (contains) {
         return deserializers[type];
      }
      return deserializers[BasicControllerType::ControllerType::PID];
   }

   return_type getFromJson(std::shared_ptr<BaseRobot> robot, const JsonVariant &json) {
      if (json["type"].is<BasicControllerType::ControllerType>()) {
         return getDeserializer(json["type"].as<BasicControllerType::ControllerType>())(robot, json);
      }
      return getDeserializer(BasicControllerType::ControllerType::PID)(robot, json);
   }

   bool isTypeCastableTo(BasicControllerType::ControllerType to_cast, BasicControllerType::ControllerType caster) {
      switch (caster) {
         case BasicControllerType::PID:
            switch (to_cast) {
               case BasicControllerType::PID:
               case BasicControllerType::PIDSpeedFeedForward:
               case BasicControllerType::PIDFilteredD:
                  return true;
               default:
                  break;
            }
            break;
         default:
            break;
      }
      return false;
   }

   std::shared_ptr<PID> castToPID(std::shared_ptr<BasicController> controller) {
      if (isTypeCastableTo(controller->getType(), BasicControllerType::PID)) {
         return std::static_pointer_cast<PID>(controller);
      }
      return nullptr;
   }


}
