#include "FSM/CtrlFSM.h"
#include "FSM/FSMState.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "State_Velocity.h"
#include "State_Mimic.h"
#include "State_UFO.h"
#include "FSM/State_WBC.h"
#include "UnitreeJoystickBridge.h"
#include <memory>

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
LowCmd_t::MsgType FSMState::command_msg{};
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

void init_fsm_state()
{
    // g1_ctrl publishes lowcmd directly over DDS.
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();

    // GUI launches with stdin=/dev/null; only enable terminal keyboard control
    // for a direct, interactive g1_ctrl launch.
    if (isatty(STDIN_FILENO)) {
        FSMState::keyboard = std::make_shared<Keyboard>();
    }

    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     G1-23dof Controller \n";

    int domain_id = param::config["domain_id"] ? param::config["domain_id"].as<int>() : 0;
    unitree::robot::ChannelFactory::Instance()->Init(domain_id, vm["network"].as<std::string>());

    init_fsm_state();

    int mode_machine = param::config["mode_machine"] ? param::config["mode_machine"].as<int>() : 4;
    FSMState::lowcmd->msg_.mode_machine() = mode_machine;
    if(!FSMState::lowcmd->check_mode_machine(FSMState::lowstate)) {
        spdlog::critical("Unmatched robot type.");
        exit(-1);
    }

    // The SDK worker has not been signaled to publish yet.
    FSMState::command_msg = FSMState::lowcmd->msg_;

    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    std::cout << "Press [L2 + Up] to enter FixStand mode.\n";
    std::cout << "And then press [R1 + X] to start controlling the robot.\n";

    test::remote_joystick = std::make_unique<UnitreeJoystickBridge>(
        "/dev/shm/unitree_joystick_bridge.json", 0.2);

    while (true) {
        sleep(1);
    }

    return 0;
}
