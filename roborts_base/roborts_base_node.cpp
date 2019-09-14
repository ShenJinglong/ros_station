#include "gimbal/gimbal.h"
#include "chassis/chassis.h"
#include "referee_system/referee_system.h"
#include "roborts_base_config.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "roborts_base_node");
    ros::NodeHandle nh;
    roborts_base::Config config;
    config.GetParam(&nh);
    ROS_INFO("%s", config.serial_port.c_str());
    auto handle = std::make_shared<roborts_sdk::Handle>(config.serial_port);
    if (!handle->Init()) return 1;

    roborts_base::Chassis chassis(handle);
    roborts_base::Gimbal gimbal(handle);
    roborts_base::RefereeSystem referee_system(handle);

    while (ros::ok()) {
        handle->Spin();
        ros::spinOnce();
        usleep(1000);
    }
}