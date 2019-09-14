#ifndef ROBORTS_BASE_GIMBAL_H
#define ROBORTS_BASE_GIMBAL_H
#include "../roborts_sdk/sdk.h"
#include "../ros_dep.h"

namespace roborts_base {
/**
 * @brief ROS API for gimbal module
 */
class Gimbal {
public:
    /**
     * @brief Constructor of gimbal including initialization of sdk and ROS
     * @param handle handler of sdk
     */
    Gimbal(std::shared_ptr<roborts_sdk::Handle> handle);
    /**
     * @brief Destructor of gimbal
     */
    ~Gimbal();
private:
    /**
     * @brief Initialization of sdk
     */
    void SDK_Init();
    /**
     * @brief Initialization of ROS
     */
    void ROS_Init();

    /**
     * @brief Gimbal information callback in sdk
     * @param gimbal_info Gimbal information
     */
    void GimbalInfoCallback(const std::shared_ptr<roborts_sdk::cmd_gimbal_info> gimbal_info);
    /**
     * @brief Gimbal angle control callback in ROS
     * @param msg Gimbal angle control data
     */
    void GimbalAngleCtrlCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg);

    /**
     * @brief Gimbal mode set service callback in ROS
     * @param req Gimbal mode set as request
     * @param res Mode set result as response
     * @return True if success
     */
    bool SetGimbalModeService(roborts_msgs::GimbalMode::Request &req,
                                roborts_msgs::GimbalMode::Response &res);
    /**
     * @brief Control friction wheel service callback in ROS
     * @param req Friction wheel control data as request
     * @param res Control result as response
     * @return True if success
     */
    bool CtrlFricWheelService(roborts_msgs::FricWhl::Request &req,
                                roborts_msgs::FricWhl::Response &res);
    /**
     * @brief Control shoot service callback in ROS
     * @param req Shoot control data as request
     * @param res Control result as response
     * @return True if success
     */
    bool CtrlShootService(roborts_msgs::ShootCmd::Request &req,
                            roborts_msgs::ShootCmd::Response &res);

    //! sdk handler
    std::shared_ptr<roborts_sdk::Handle> handle_;
    //! sdk version client
    std::shared_ptr<roborts_sdk::Client<roborts_sdk::cmd_version_id,
                                        roborts_sdk::cmd_version_id>> verison_client_;

    //! sdk heartbeat thread
    std::thread heartbeat_thread_;
    //! sdk publisher for heartbeat
    std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_heartbeat>> heartbeat_pub_;


    //! sdk publisher for gimbal angle control
    std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_gimbal_angle>>     gimbal_angle_pub_;
    //! sdk publisher for gimbal mode set
    std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::gimbal_mode_e>>        gimbal_mode_pub_;
    //! sdk publisher for frcition wheel control
    std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_fric_wheel_speed>> fric_wheel_pub_;
    //! sdk publisher for gimbal shoot control
    std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_shoot_info>>       gimbal_shoot_pub_;

    //! ros node handler
    ros::NodeHandle    ros_nh_;
    //! ros subscriber for gimbal angle control
    ros::Subscriber    ros_sub_cmd_gimbal_angle_;
    //! ros service server for gimbal mode set
    ros::ServiceServer ros_gimbal_mode_srv_;
    //! ros service server for friction wheel control
    ros::ServiceServer ros_ctrl_fric_wheel_srv_;
    //! ros service server for gimbal shoot control
    ros::ServiceServer ros_ctrl_shoot_srv_;

    ros::Publisher ros_pub_gimbal_angle_;
    ros::Publisher ros_bullet_pub_;

    roborts_msgs::GimbalAngle gimbal_data;
    //! ros gimbal tf
    geometry_msgs::TransformStamped gimbal_tf_;
    //! ros gimbal tf broadcaster
    tf::TransformBroadcaster        tf_broadcaster_;
};

}

#endif //ROBORTS_BASE_GIMBAL_H
