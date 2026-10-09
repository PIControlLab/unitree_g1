#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <boost/program_options.hpp>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/h2/common/terminations.hpp>

namespace po = boost::program_options;
using unitree::robot::ChannelFactory;
using unitree::robot::ChannelSubscriber;
using unitree_hg::msg::dds_::LowState_;

int main(int argc, char** argv) {
  po::options_description options("Unitree H2 termination functions testing.");
  options.add_options()
      ("network,n", po::value<std::string>()->default_value(""),
       "dds network interface");
  po::variables_map args;
  po::store(po::parse_command_line(argc, argv, options), args);
  po::notify(args);
  std::cout << options << std::endl;

  ChannelFactory::Instance()->Init(0, args["network"].as<std::string>());

  auto lowstate_subscriber =
      std::make_shared<ChannelSubscriber<LowState_>>("rt/lowstate");
  LowState_ lowstate;
  lowstate_subscriber->InitChannel([&lowstate](const void* message) {
    lowstate = *static_cast<const LowState_*>(message);
  });

  std::cout << "Checking terminations..." << std::endl;
  while (true) {
    if (unitree::robot::h2::bad_orientation(lowstate, 1.0f)) {
      std::cout << "Bad orientation detected!" << std::endl;
    }
    if (unitree::robot::h2::lost_connection(lowstate_subscriber, 1000)) {
      std::cout << "Lost connection!" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
