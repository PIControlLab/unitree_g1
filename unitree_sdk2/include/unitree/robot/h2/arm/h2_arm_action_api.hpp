#pragma once

#include <cstdint>
#include <string>

#include <unitree/common/json/jsonize.hpp>

namespace unitree {
namespace robot {
namespace h2 {

const std::string ARM_ACTION_SERVICE_NAME = "arm";
const std::string ARM_ACTION_API_VERSION = "1.0.0.14";

const int32_t ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION = 7106;
const int32_t ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST = 7107;
const int32_t ROBOT_API_ID_ARM_ACTION_EXECUTE_CUSTOM_ACTION = 7108;
const int32_t ROBOT_API_ID_ARM_ACTION_STOP_CUSTOM_ACTION = 7113;

class JsonizeArmActionCommand : public common::Jsonize {
 public:
  void fromJson(common::JsonMap& json) override {
    common::FromJson(json["action_id"], action_id);
  }

  void toJson(common::JsonMap& json) const override {
    common::ToJson(action_id, json["action_id"]);
  }

  int32_t action_id = 0;
};

class JsonizeArmActionName : public common::Jsonize {
 public:
  void fromJson(common::JsonMap& json) override {
    common::FromJson(json["action_name"], action_name);
  }

  void toJson(common::JsonMap& json) const override {
    common::ToJson(action_name, json["action_name"]);
  }

  std::string action_name;
};

}  // namespace h2
}  // namespace robot
}  // namespace unitree
