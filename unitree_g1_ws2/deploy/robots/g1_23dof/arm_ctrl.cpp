/**
 * G1 右臂独立控制器 — DDS 直连，无 ROS 2 依赖
 *
 * 摇杆速度 → 位置阻抗积分 → DDS LowCmd (motor 22-26)
 * 用法: ./arm_ctrl [--network eth0]
 * 控制: echo '{"x":0.1, "y":0.0, "z":0.0}' > /dev/shm/arm_vel.json
 */

#include "Types.h"
#include "param.h"
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <thread>

using namespace std::chrono;
using json = nlohmann::json;

constexpr double PERIOD   = 0.02;
constexpr double DEADZONE = 0.005;
constexpr double TIMEOUT  = 0.3;
constexpr int    NUM_MOTORS = 35;
constexpr double PosStopF = 2.146e9;

struct MotorDef { int idx; const char* name; double kp,kd,ki,tau_lim; };
static const MotorDef ARM_MOTORS[] = {
    {22, "shld_pitch", 60, 1.5, 3.0, 100.0},
    {23, "shld_roll",  60, 1.5, 3.0, 100.0},
    {24, "shld_yaw",   60, 1.5, 3.0, 100.0},
    {25, "elbow",      60, 1.5, 1.0,  50.0},
    {26, "wrist_roll", 60, 1.5, 3.0,  25.0},
};
constexpr int NUM_ARM = sizeof(ARM_MOTORS)/sizeof(ARM_MOTORS[0]);

struct CartMap { const char* axis; int idx; double gain; };
static const CartMap CART_MAP[] = {
    {"y", 22, -2.0},
    {"x", 23, -2.0},
    {"z", 25, -2.0},
};

struct Joint {
    int idx; double kp,kd,ki,tau_limit,max_delta=0.01;
    double q_target=0,q_hold=0,q=0,dq=0,i_err=0,last_cmd=0;
    bool was_active=false;

    void on_cmd(double vel, double now) {
        double delta = std::clamp(vel * PERIOD, -max_delta, max_delta);
        q_target += delta; last_cmd = now;
    }
    bool active(double now) const { return last_cmd>0 && (now-last_cmd)<TIMEOUT; }
};

struct ArmVel { double x=0,y=0,z=0; };
ArmVel read_vel(const std::string& path="/dev/shm/arm_vel.json") {
    ArmVel v;
    std::ifstream f(path); if(!f.good()) return v;
    try {
        auto j = json::parse(f);
        v.x=j.value("x",0.0); v.y=j.value("y",0.0); v.z=j.value("z",0.0);
    } catch(...) {}
    return v;
}

int main(int argc, char** argv) {
    auto vm = param::helper(argc, argv);
    int domain_id = param::config["domain_id"] ? param::config["domain_id"].as<int>() : 0;
    unitree::robot::ChannelFactory::Instance()->Init(domain_id, vm["network"].as<std::string>());

    auto lowcmd  = std::make_unique<LowCmd_t>();
    auto lowstate = std::make_shared<LowState_t>();

    spdlog::info("Arm Ctrl: waiting for robot...");
    lowstate->wait_for_connection();
    spdlog::info("Arm Ctrl: connected.");

    Joint joints[NUM_ARM];
    for (int i=0;i<NUM_ARM;++i) {
        auto& m=ARM_MOTORS[i];
        joints[i]={m.idx,m.kp,m.kd,m.ki,m.tau_lim};
    }

    // init LowCmd: all motors = enable + q far
    lowcmd->msg_.mode_machine()=4;
    for (int i=0;i<NUM_MOTORS;++i) {
        lowcmd->msg_.motor_cmd()[i].mode()=1;
        lowcmd->msg_.motor_cmd()[i].q()=PosStopF;
        lowcmd->msg_.motor_cmd()[i].dq()=0;
        lowcmd->msg_.motor_cmd()[i].kp()=0;
        lowcmd->msg_.motor_cmd()[i].kd()=0;
        lowcmd->msg_.motor_cmd()[i].tau()=0;
    }

    spdlog::info("Arm Ctrl ready | {} joints @{}Hz", NUM_ARM, int(1.0/PERIOD));
    spdlog::info("  echo '{{\"x\":0.1}}' > /dev/shm/arm_vel.json");

    bool ready=false;
    auto t_next=steady_clock::now();
    int cnt=0;

    while (true) {
        t_next+=microseconds(int(PERIOD*1e6)); ++cnt;
        lowstate->update();
        double now=duration<double>(steady_clock::now().time_since_epoch()).count();

        if (!ready) {
            for (int i=0;i<NUM_ARM;++i) {
                joints[i].q=lowstate->msg_.motor_state()[joints[i].idx].q();
                joints[i].dq=lowstate->msg_.motor_state()[joints[i].idx].dq();
                joints[i].q_hold=joints[i].q_target=joints[i].q;
            }
            ready=true;
            spdlog::info("Arm Ctrl: first state, holding");
        } else {
            for (int i=0;i<NUM_ARM;++i) {
                joints[i].q=lowstate->msg_.motor_state()[joints[i].idx].q();
                joints[i].dq=lowstate->msg_.motor_state()[joints[i].idx].dq();
            }
        }

        // read joystick
        auto vel=read_vel();
        double mag=std::sqrt(vel.x*vel.x+vel.y*vel.y+vel.z*vel.z);
        if (mag>DEADZONE) {
            for (auto& cm:CART_MAP) {
                double v=(cm.axis[0]=='x')?vel.x:(cm.axis[0]=='y')?vel.y:vel.z;
                double dq=cm.gain*v;
                if (std::abs(dq)>DEADZONE)
                    for (int i=0;i<NUM_ARM;++i)
                        if (joints[i].idx==cm.idx) joints[i].on_cmd(dq,now);
            }
        }

        // apply & publish
        for (int i=0;i<NUM_ARM;++i) {
            auto& j=joints[i];
            auto& m=lowcmd->msg_.motor_cmd()[j.idx];
            bool act=j.active(now);
            if (act) {
                if (!j.was_active) j.q_target=j.q;
                m.q()=j.q_target;
            } else {
                if (j.was_active) {j.q_hold=j.q;j.q_target=j.q;}
                m.q()=j.q_hold;
            }
            double err=m.q()-j.q;
            j.i_err+=j.ki*err*PERIOD;
            j.i_err=std::clamp(j.i_err,-j.tau_limit,j.tau_limit);
            m.mode()=0x01; m.dq()=0; m.kp()=j.kp; m.kd()=j.kd; m.tau()=j.i_err;
            j.was_active=act;
        }
        lowcmd->unlockAndPublish();

        if (cnt%50==0) {
            std::string s;
            for (int i=0;i<NUM_ARM;++i) {
                char b[48]; snprintf(b,sizeof(b),"%s:%c:%.2f ",
                    ARM_MOTORS[i].name,joints[i].active(now)?'A':'L',joints[i].q);
                s+=b;
            }
            if (mag>DEADZONE) spdlog::info("{}| v:{:.2f},{:.2f},{:.2f}",s,vel.x,vel.y,vel.z);
            else spdlog::info("{}",s);
        }

        auto delay=t_next-steady_clock::now();
        if (delay>microseconds(0)) std::this_thread::sleep_for(delay);
    }
}
