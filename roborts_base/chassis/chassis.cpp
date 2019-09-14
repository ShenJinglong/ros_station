#include "chassis.h"
#include "../roborts_sdk/sdk.h"

namespace roborts_base {

Chassis::Chassis(std::shared_ptr<roborts_sdk::Handle> handle): handle_(handle) {
    SDK_Init();
    ROS_Init();
}

Chassis::~Chassis() {
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }
}

void Chassis::SDK_Init() {
    version_client_ = handle_->CreateClient<roborts_sdk::cmd_version_id, roborts_sdk::cmd_version_id>(UNIVERSAL_CMD_SET, CMD_REPORT_VERSION,
                                            MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
    roborts_sdk::cmd_version_id version_cmd;
    version_cmd.version_id = 0;
    auto version = std::make_shared<roborts_sdk::cmd_version_id>(version_cmd);
    version_client_->AsyncSendRequest(version,
                                    [](roborts_sdk::Client<roborts_sdk::cmd_version_id, roborts_sdk::cmd_version_id>::SharedFuture future) {
                                        ROS_INFO("Chassis Firmware Version: %d.%d.%d.%d",
                                            int(future.get()->version_id>>24&0xFF),
                                            int(future.get()->version_id>>16&0xFF),
                                            int(future.get()->version_id>>8&0xFF),
                                            int(future.get()->version_id&0xFF));
                                    });

    handle_->CreateSubscriber<roborts_sdk::cmd_chassis_info>(CHASSIS_CMD_SET, CMD_PUSH_CHASSIS_INFO,
                                                            CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                            std::bind(&Chassis::ChassisInfoCallback, this, std::placeholders::_1));
    handle_->CreateSubscriber<roborts_sdk::cmd_uwb_info>(COMPATIBLE_CMD_SET, CMD_PUSH_UWB_INFO,
                                                            CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                            std::bind(&Chassis::UWBInfoCallback, this, std::placeholders::_1));

    chassis_speed_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_speed>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_SPEED,
                                                                                    MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
    chassis_spd_acc_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_spd_acc>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_SPD_ACC,
                                                                                        MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);

    heartbeat_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_heartbeat>(UNIVERSAL_CMD_SET, CMD_HEARTBEAT,
                                                                           MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
    heartbeat_thread_ = std::thread([this] {
                                        roborts_sdk::cmd_heartbeat heartbeat;
                                        heartbeat.heartbeat=0;
                                        while (ros::ok()) {
                                            heartbeat_pub_->Publish(heartbeat);
                                            std::this_thread::sleep_for(std::chrono::milliseconds(300));
                                        }
                                    });
}

void Chassis::ROS_Init() {
    //ros publisher
    ros_odom_pub_ = ros_nh_.advertise<nav_msgs::Odometry>("odom", 30);
    ros_uwb_pub_ = ros_nh_.advertise<geometry_msgs::PoseStamped>("uwb", 30);
    //ros subscriber
    ros_sub_cmd_chassis_vel_ = ros_nh_.subscribe("cmd_vel", 1, &Chassis::ChassisSpeedCtrlCallback, this);
    ros_sub_cmd_chassis_vel_acc_ = ros_nh_.subscribe("cmd_vel_acc", 1, &Chassis::ChassisSpeedAccCtrlCallback, this);
    ros_sub_mode_ = ros_nh_.subscribe("current_mode", 1, &Chassis::ModeCtrlCallback, this);

    //ros_message_init
    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "base_link";

    odom_tf_.header.frame_id = "odom";
    odom_tf_.child_frame_id = "base_link";

    uwb_data_.header.frame_id = "uwb";
}

void Chassis::ChassisInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_info> chassis_info) {
    ros::Time current_time = ros::Time::now();
    odom_.header.stamp = current_time;
    odom_.pose.pose.position.x = chassis_info->position_x_mm / 1000.0;
    odom_.pose.pose.position.y = chassis_info->position_y_mm / 1000.0;
    odom_.pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(chassis_info->gyro_angle / 1800.0 * M_PI);
    odom_.pose.pose.orientation = q;
    odom_.twist.twist.linear.x = chassis_info->v_x_mm / 1000.0;
    odom_.twist.twist.linear.y = chassis_info->v_y_mm / 1000.0;
    odom_.twist.twist.angular.z = chassis_info->gyro_rate / 1800.0 * M_PI;
    ros_odom_pub_.publish(odom_);
    //std::cout<<"twist_vw:"<<chassis_info->twist_wz<<std::endl;
    odom_tf_.header.stamp = current_time;
    odom_tf_.transform.translation.x = chassis_info->position_x_mm / 1000.;
    odom_tf_.transform.translation.y = chassis_info->position_y_mm / 1000.;
    odom_tf_.transform.translation.z = 0.0;
    odom_tf_.transform.rotation = q;
    tf_broadcaster_.sendTransform(odom_tf_);
}

void Chassis::UWBInfoCallback(const std::shared_ptr<roborts_sdk::cmd_uwb_info> uwb_info) {
    uwb_data_.header.stamp = ros::Time::now();
    uwb_data_.pose.position.x = ((double)uwb_info->x)/100.0;
    uwb_data_.pose.position.y = ((double)uwb_info->y)/100.0;
    uwb_data_.pose.position.z = 0;
    uwb_data_.pose.orientation = tf::createQuaternionMsgFromYaw(uwb_info->yaw/ 180.0 * M_PI);
    ros_uwb_pub_.publish(uwb_data_);
}

void Chassis::ChassisSpeedCtrlCallback(const geometry_msgs::Twist::ConstPtr &vel) {
    roborts_sdk::cmd_chassis_speed chassis_speed;
    chassis_speed.vx = vel->linear.x * 1000;
    chassis_speed.vy = vel->linear.y * 1000;
    chassis_speed.vw = vel->angular.z * 1800.0 / M_PI;
    chassis_speed.rotate_x_offset = 0;
    chassis_speed.rotate_y_offset = 0;
    // chassis_speed.mode=0;
    ROS_INFO("vx: %d, vy: %d, vw: %d", chassis_speed.vx, chassis_speed.vy, chassis_speed.vw);
    chassis_speed_pub_->Publish(chassis_speed);
}

void Chassis::ChassisSpeedAccCtrlCallback(const roborts_msgs::TwistAccel::ConstPtr &vel_acc) {
	std::cout << "catwalk:" << catwalk << std::endl;
	std::cout << "time:" << (ros::Time::now() - stamp < ros::Duration(1.0)) << std::endl;
	if (!(catwalk && (ros::Time::now() - stamp < ros::Duration(1.0)))) {
        roborts_sdk::cmd_chassis_spd_acc chassis_spd_acc;
        chassis_spd_acc.vx = vel_acc->twist.linear.x * 1000;
        chassis_spd_acc.vy = vel_acc->twist.linear.y * 1000;
        chassis_spd_acc.vw = vel_acc->twist.angular.z * 1800.0 / M_PI;
        chassis_spd_acc.ax = vel_acc->accel.linear.x * 1000;
        chassis_spd_acc.ay = vel_acc->accel.linear.y * 1000;
        chassis_spd_acc.wz = vel_acc->accel.angular.z * 1800.0 / M_PI;
        chassis_spd_acc.rotate_x_offset = 0;
        chassis_spd_acc.rotate_y_offset = 0;
        //chassis_spd_acc.mode = 0;
        chassis_spd_acc_pub_->Publish(chassis_spd_acc);
	}
}

void Chassis::ModeCtrlCallback(const roborts_msgs::current_mode::ConstPtr &msg) {
 	catwalk = msg->catwalk;
	stamp = msg->header.stamp;	
}

}
