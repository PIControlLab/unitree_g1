/**
 * @file h2_arm_action_example.cpp
 * @brief Execute a preset H2 arm action through the robot-side arm service.
 */

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include <boost/program_options.hpp>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/h2/arm/h2_arm_action_client.hpp>
#include <unitree/robot/h2/arm/h2_arm_action_error.hpp>

namespace po = boost::program_options;
using unitree::robot::ChannelFactory;
using namespace unitree::robot::h2;

int main(int argc, const char** argv) {
  po::options_description options("Unitree H2 Arm Action Example");
  options.add_options()
      ("help,h", "show help message")
      ("network,n", po::value<std::string>()->default_value(""),
       "DDS network interface")
      ("list,l", "query the robot for all supported actions")
      ("id,i", po::value<int32_t>(),
       "preset action ID to execute; 0 lists actions and 99 releases the arms")
      ("name", po::value<std::string>(), "custom action name to execute")
      ("stop", "stop the current custom action");

  po::variables_map args;
  try {
    po::store(po::parse_command_line(argc, argv, options), args);
    po::notify(args);
  } catch (const po::error& error) {
    std::cerr << "Argument error: " << error.what() << "\n\n" << options << '\n';
    return 2;
  }

  if (argc < 2 || args.count("help")) {
    std::cout << options << '\n';
    return 0;
  }

  if (!args.count("list") && !args.count("id") && !args.count("name") &&
      !args.count("stop")) {
    std::cerr << "No action selected. Use --id, --list, --name, or --stop.\n\n"
              << options << '\n';
    return 2;
  }

  ChannelFactory::Instance()->Init(0, args["network"].as<std::string>());
  auto client = std::make_shared<H2ArmActionClient>();
  client->Init();
  client->SetTimeout(10.0f);

  int32_t ret = 0;
  if (args.count("list") ||
      (args.count("id") && args["id"].as<int32_t>() == 0)) {
    std::string action_list;
    ret = client->GetActionList(action_list);
    if (ret == 0) {
      std::cout << "Available actions:\n" << action_list << std::endl;
    }
  } else if (args.count("id")) {
    const int32_t action_id = args["id"].as<int32_t>();
    std::cout << "Requesting H2 arm action " << action_id << "..." << std::endl;
    ret = client->ExecuteAction(action_id);
    if (ret == 0) {
      std::cout << "Arm action " << action_id << " completed successfully."
                << std::endl;
    }
  } else if (args.count("name")) {
    const std::string action_name = args["name"].as<std::string>();
    std::cout << "Requesting H2 custom arm action '" << action_name << "'..."
              << std::endl;
    ret = client->ExecuteAction(action_name);
    if (ret == 0) {
      std::cout << "Custom arm action request accepted." << std::endl;
    }
  } else if (args.count("stop")) {
    ret = client->StopCustomAction();
    if (ret == 0) {
      std::cout << "Stop request accepted." << std::endl;
    }
  }

  if (ret == 0) {
    return 0;
  }

  std::cerr << "Arm action request failed, error code: " << ret << '\n';
  switch (ret) {
    case UT_ROBOT_ARM_ACTION_ERR_ARMSDK:
      std::cerr << UT_ROBOT_ARM_ACTION_ERR_ARMSDK_DESC << '\n';
      break;
    case UT_ROBOT_ARM_ACTION_ERR_HOLDING:
      std::cerr << UT_ROBOT_ARM_ACTION_ERR_HOLDING_DESC << '\n';
      break;
    case UT_ROBOT_ARM_ACTION_ERR_INVALID_ACTION_ID:
      std::cerr << UT_ROBOT_ARM_ACTION_ERR_INVALID_ACTION_ID_DESC << '\n';
      break;
    case UT_ROBOT_ARM_ACTION_ERR_INVALID_FSM_ID:
      std::cerr << UT_ROBOT_ARM_ACTION_ERR_INVALID_FSM_ID_DESC << '\n';
      break;
    default:
      std::cerr << "Check the DDS interface, robot-side arm service, API "
                   "compatibility, and request parameters.\n";
      break;
  }
  return 1;
}
