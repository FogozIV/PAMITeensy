//
// Created by fogoz on 28/07/2025.
//

#ifndef SERIALIZABLECLASS_H
#define SERIALIZABLECLASS_H

#define SET_JSON_ADVANCED(name, source) json[#name] = (source)->name

#define GET_AND_CHECK_JSON(var_name, name, type) \
if(json[#name].is<type>()){\
var_name->name = json[#name].as<type>(); \
}

#define SET_JSON(name) SET_JSON_ADVANCED(name, this)

class BaseRobot;
class CommandParser;

template<class Type, class BaseType>
class Serializable {
protected:
    Type type = static_cast<Type>(0);
public:
    virtual ~Serializable() = default;

    virtual Type getType() {
        return type;
    }

    virtual void serialize(JsonObject json) = 0;

    virtual std::shared_ptr<BaseType> deserialize(std::shared_ptr<BaseRobot> robot, JsonVariant& json)  = 0;

    virtual void registerCommands(CommandParser& parser, const char* name) = 0;

    virtual void unregisterCommands(CommandParser& parser, const char* name) = 0;
};

#endif //SERIALIZABLECLASS_H
