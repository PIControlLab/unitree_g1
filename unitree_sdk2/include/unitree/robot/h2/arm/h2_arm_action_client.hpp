#pragma once

#include <cstdint>
#include <map>
#include <string>

#include <unitree/robot/client/client.hpp>

#include "h2_arm_action_api.hpp"

namespace unitree {
namespace robot {
namespace h2 {

/**
 * @brief Client for requesting preset H2 upper-body actions.
 *
 * The matching robot-side "arm" service generates the trajectory and sends it
 * through rt/arm_sdk. This client sends action requests over RPC and does not
 * generate joint trajectories itself.
 */
class H2ArmActionClient : public Client {
 public:
  H2ArmActionClient() : Client(ARM_ACTION_SERVICE_NAME, false) {}

  void Init() {
    SetApiVersion(ARM_ACTION_API_VERSION);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_ARM_ACTION_EXECUTE_CUSTOM_ACTION);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_ARM_ACTION_STOP_CUSTOM_ACTION);
  }

  int32_t ExecuteAction(int32_t action_id) {
    JsonizeArmActionCommand command;
    command.action_id = action_id;

    std::string data;
    const std::string parameter = common::ToJsonString(command);
    return Call(ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION, parameter, data);
  }

  int32_t ExecuteAction(const std::string& action_name) {
    JsonizeArmActionName command;
    command.action_name = action_name;

    std::string data;
    const std::string parameter = common::ToJsonString(command);
    return Call(ROBOT_API_ID_ARM_ACTION_EXECUTE_CUSTOM_ACTION, parameter, data);
  }

  int32_t StopCustomAction() {
    std::string parameter;
    std::string data;
    return Call(ROBOT_API_ID_ARM_ACTION_STOP_CUSTOM_ACTION, parameter, data);
  }

  int32_t GetActionList(std::string& data) {
    std::string parameter;
    return Call(ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST, parameter, data);
  }

  // This is a local list known by the client, not a list queried from the robot.
  inline static const std::map<std::string, int32_t> action_map = {
      {"release arm", 99},   {"two-hand kiss", 11},
      {"left kiss", 12},     {"hands up", 15},
      {"clap", 17},          {"high five", 18},
      {"hug", 19},           {"heart", 20},
      {"right heart", 21},   {"reject", 22},
      {"right hand up", 23}, {"x-ray", 24},
      {"face wave", 25},     {"high wave", 26},
      {"shake hand", 27},
  };
};

}  // namespace h2
}  // namespace robot
}  // namespace unitree
