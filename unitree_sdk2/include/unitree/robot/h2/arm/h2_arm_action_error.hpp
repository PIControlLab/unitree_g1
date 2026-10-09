#pragma once

#include <unitree/common/decl.hpp>

namespace unitree {
namespace robot {
namespace h2 {

UT_DECL_ERR(UT_ROBOT_ARM_ACTION_ERR_ARMSDK, 7400,
            "The topic rt/arm_sdk is occupied.")
UT_DECL_ERR(UT_ROBOT_ARM_ACTION_ERR_HOLDING, 7401,
            "The arm is holding. Send action 99 or repeat the last action ID to release it.")
UT_DECL_ERR(UT_ROBOT_ARM_ACTION_ERR_INVALID_ACTION_ID, 7402,
            "Invalid arm action ID.")
UT_DECL_ERR(UT_ROBOT_ARM_ACTION_ERR_INVALID_FSM_ID, 7404,
            "The current FSM does not support arm actions.")

}  // namespace h2
}  // namespace robot
}  // namespace unitree
