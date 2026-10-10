/**
 * @file r1_arm_action_example.cpp
 * @brief This example demonstrates how to use the R1 Arm Action Client to
 *        execute predefined arm actions and recorded custom actions.
 *
 * The onboard arm action service provides a set of preset upper-body
 * interaction actions, as well as playback of actions recorded through the
 * teaching feature. This example shows how to call that service over DDS RPC:
 * get the action list, execute a preset action, execute a custom action and
 * stop a custom action.
 *
 * Usage:
 *   r1_arm_action_example <networkInterface> --list
 *   r1_arm_action_example <networkInterface> --id <action_id>
 *   r1_arm_action_example <networkInterface> --name <action_name>
 *   r1_arm_action_example <networkInterface> --stop
 *
 * Attention: do not replace this client with G1's g1_arm_action_client.hpp.
 *            Although both share the same service name and API IDs, R1 has a
 *            different action ID list and a larger set of error codes.
 */

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/client/client.hpp>

#include <iostream>
#include <string>

namespace unitree {
namespace robot {
namespace r1 {

/* Service name, matching the onboard arm action service.
   Maps to the topics rt/api/arm/request and rt/api/arm/response. */
const std::string ARM_ACTION_SERVICE_NAME = "arm";

/* API version. The onboard service does not verify this field at present;
   it is set only to stay consistent with the SDK convention. */
const std::string ARM_ACTION_API_VERSION = "1.0.0.0";

/* API id */
const int32_t ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION = 7106;         // Execute a preset action
const int32_t ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST = 7107;        // Get the action list
const int32_t ROBOT_API_ID_ARM_ACTION_EXECUTE_CUSTOM_ACTION = 7108;  // Execute a custom action
const int32_t ROBOT_API_ID_ARM_ACTION_STOP_CUSTOM_ACTION = 7113;     // Stop the custom action

/* Error code */
const int32_t ARM_ACTION_ERR_COMMON = 7399;             // Internal error on the server side
const int32_t ARM_ACTION_ERR_ARMSDK_OCCUPIED = 7400;    // rt/arm_sdk is occupied, or an action is running
const int32_t ARM_ACTION_ERR_HOLDING = 7401;            // The arm is holding, release it first
const int32_t ARM_ACTION_ERR_INVALID_ACTION_ID = 7402;  // Invalid action id
const int32_t ARM_ACTION_ERR_LOAD_ACTION_FILE = 7403;   // Failed to load the action file / custom action not found
const int32_t ARM_ACTION_ERR_INVALID_FSM_ID = 7404;     // The action cannot be triggered in the current fsm state
const int32_t ARM_ACTION_ERR_ACTION_FILE_EXIST = 7405;  // A custom action with the same name already exists
const int32_t ARM_ACTION_ERR_LOW_BATTERY = 7406;        // Battery too low
const int32_t ARM_ACTION_ERR_MOTOR = 7407;              // Motor in error state

/**
 * @brief Return the description of an error code
 */
inline std::string ArmActionErrorDesc(int32_t code) {
  switch (code) {
    case ARM_ACTION_ERR_COMMON:
      return "Internal error on the server side.";
    case ARM_ACTION_ERR_ARMSDK_OCCUPIED:
      return "The topic rt/arm_sdk is occupied, or another action is running.";
    case ARM_ACTION_ERR_HOLDING:
      return "The arm is holding the pose of the last action. "
             "Expecting release action (99) or the same last action id.";
    case ARM_ACTION_ERR_INVALID_ACTION_ID:
      return "Invalid action id. Use --list to check the supported actions.";
    case ARM_ACTION_ERR_LOAD_ACTION_FILE:
      return "Failed to load the action file, or the custom action does not exist.";
    case ARM_ACTION_ERR_INVALID_FSM_ID:
      return "The action cannot be triggered in the current fsm state. "
             "Make sure the robot is running the built-in controller, not in debug mode.";
    case ARM_ACTION_ERR_ACTION_FILE_EXIST:
      return "A custom action with the same name already exists.";
    case ARM_ACTION_ERR_LOW_BATTERY:
      return "Battery too low, the request is rejected.";
    case ARM_ACTION_ERR_MOTOR:
      return "Some motor is in an error state, the request is rejected.";
    default:
      return "Unknown error.";
  }
}

/**
 * @brief R1 arm action client
 *
 * The current state of the arm is published on the rt/arm/action/state topic:
 * {
 *   "holding": false,      # Whether to hold the pose after the action ends;
 *                          # will release after a maximum of 20 seconds
 *   "id": 99,              # Current action id; always 100 for custom action
 *                          # playback, and -1 while teaching is in progress
 *   "name": "release_arm"  # Current action name
 * }
 */
class R1ArmActionClient : public Client {
 public:
  R1ArmActionClient() : Client(ARM_ACTION_SERVICE_NAME, false) {}
  ~R1ArmActionClient() {}

  /*Init*/
  void Init() {
    SetApiVersion(ARM_ACTION_API_VERSION);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_ARM_ACTION_EXECUTE_CUSTOM_ACTION);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_ARM_ACTION_STOP_CUSTOM_ACTION);
  }

  /*API Call*/
  /**
   * @brief Execute a preset action, indexed by id
   *
   * Blocking call: it returns only after the action has finished playing, so
   * the timeout must be longer than the action duration.
   * Some actions hold the pose of the last keyframe after completion; send
   * id = 99 or the same id to release.
   */
  int32_t ExecuteAction(int32_t action_id) {
    std::string parameter, data;
    parameter = R"({"action_id":)" + std::to_string(action_id) + R"(})";
    return Call(ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION, parameter, data);
  }

  /**
   * @brief Execute a custom teach action, indexed by name (case sensitive)
   *
   * Non-blocking call: it returns immediately while the action plays in the
   * background. The arm is released automatically once playback finishes.
   */
  int32_t ExecuteAction(const std::string &action_name) {
    std::string parameter, data;
    parameter = R"({"action_name":")" + action_name + R"("})";
    return Call(ROBOT_API_ID_ARM_ACTION_EXECUTE_CUSTOM_ACTION, parameter, data);
  }

  /**
   * @brief Stop the custom action being played; the arm returns to its
   *        initial pose
   */
  int32_t StopCustomAction() {
    std::string parameter, data;
    return Call(ROBOT_API_ID_ARM_ACTION_STOP_CUSTOM_ACTION, parameter, data);
  }

  /**
   * @brief List the preset actions supported by the current firmware, together
   *        with the recorded custom actions
   *
   * `data` is a JSON array whose first element is the preset action list and
   * whose second element is the custom action list:
   * [[{"id":99,"name":"release_arm"}], [{"name":"my_action","time":12.3}]]
   */
  int32_t GetActionList(std::string &data) {
    std::string parameter;
    return Call(ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST, parameter, data);
  }
};

}  // namespace r1
}  // namespace robot
}  // namespace unitree

using namespace unitree::robot;

static void PrintUsage(const char *prog) {
  std::cout << "Usage:\n"
            << "  " << prog << " <networkInterface> --list              list the available actions\n"
            << "  " << prog << " <networkInterface> --id <action_id>    execute a preset action\n"
            << "  " << prog << " <networkInterface> --name <name>       execute a custom action\n"
            << "  " << prog << " <networkInterface> --stop              stop the custom action\n"
            << "\nExample:\n"
            << "  " << prog << " eth0 --id 27      # shake hand\n"
            << "  " << prog << " eth0 --id 99      # release the arm\n";
}

int main(int argc, char const *argv[]) {
  std::cout << " --- Unitree Robotics --- \n";
  std::cout << "     R1 Arm Action Example      \n\n";

  if (argc < 3) {
    PrintUsage(argv[0]);
    return 0;
  }

  const std::string network_interface = argv[1];
  const std::string command = argv[2];

  // The network interface used for DDS communication must be provided.
  ChannelFactory::Instance()->Init(0, network_interface);

  auto client = std::make_shared<r1::R1ArmActionClient>();
  client->Init();
  // Attention: preset actions are blocking calls and the custom ones may be
  // even longer, so the timeout has to be larger than the action duration.
  client->SetTimeout(10.f);

  int32_t ret = 0;

  if (command == "--list") {
    std::string action_list;
    ret = client->GetActionList(action_list);
    if (ret == 0) {
      std::cout << "Available actions:\n" << action_list << std::endl;
    }
  } else if (command == "--id") {
    if (argc < 4) {
      PrintUsage(argv[0]);
      return 0;
    }
    int32_t action_id = std::stoi(argv[3]);
    ret = client->ExecuteAction(action_id);
    if (ret == 0 && action_id != 99) {
      std::cout << "Action finished. If it holds the pose afterwards, "
                   "send --id 99 to release the arm."
                << std::endl;
    }
  } else if (command == "--name") {
    if (argc < 4) {
      PrintUsage(argv[0]);
      return 0;
    }
    // Non-blocking: returns immediately while the action plays on the robot.
    ret = client->ExecuteAction(std::string(argv[3]));
  } else if (command == "--stop") {
    ret = client->StopCustomAction();
  } else {
    PrintUsage(argv[0]);
    return 0;
  }

  if (ret != 0) {
    std::cerr << "Request failed, error code: " << ret << " - "
              << r1::ArmActionErrorDesc(ret) << std::endl;
    return ret;
  }

  return 0;
}
