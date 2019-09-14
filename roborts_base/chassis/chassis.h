#ifndef ROBORTS_BASE_CHASSIS_H
#define ROBORTS_BASE_CHASSIS_H

#include "../roborts_sdk/sdk.h"
#include "../ros_dep.h"
#include "roborts_msgs/current_mode.h"

namespace roborts_base {

class Chassis {
public:
    Chassis(std::shared_ptr<roborts_sdk::Handle> handle);
    ~Chassis();
private:
    void SDK_Init();
    void ROS_Init();
    void ChassisInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_info> chassis_info);
    void UWBInfoCallback(const std::shared_ptr<roborts_sdk::cmd_uwb_info> uwb_info);
    void ChassisSpeedCtrlCallback(const geometry_msgs::Twist::ConstPtr &vel);
    void ChassisSpeedAccCtrlCallback(const roborts_msgs::TwistAccel::ConstPtr &vel_acc);
    void ModeCtrlCallback(const roborts_msgs::current_mode::ConstPtr &msg);

    std::shared_ptr<roborts_sdk::Handle> handle_;
    std::shared_ptr<roborts_sdk::Client<roborts_sdk::cmd_version_id, roborts_sdk::cmd_version_id>> version_client_;

    std::thread heartbeat_thread_;
    std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_heartbeat>> heartbeat_pub_;

    std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_chassis_speed>> chassis_speed_pub_;
    std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_chassis_spd_acc>> chassis_spd_acc_pub_;

    ros::NodeHandle ros_nh_;
    ros::Subscriber ros_sub_cmd_chassis_vel_;
    ros::Subscriber ros_sub_cmd_chassis_vel_acc_;

    ros::Subscriber ros_sub_mode_;
    ros::Publisher ros_odom_pub_;
    ros::Publisher ros_uwb_pub_;

    geometry_msgs::TransformStamped odom_tf_;
    tf::TransformBroadcaster tf_broadcaster_;
    nav_msgs::Odometry odom_;
    geometry_msgs::PoseStamped uwb_data_;
    bool catwalk;
    ros::Time stamp;
};

}


#endif