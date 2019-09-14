#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "base_test_node");
    ros::NodeHandle nh;

    ros::Publisher vel_ctrl = nh.advertise<geometry_msgs::Twist>("cmd_vel", 30);

    ros::Rate rate(10);
    int send_counter = 0;
    while (ros::ok()) {
        geometry_msgs::Twist vel_ctrl_msg;
        vel_ctrl_msg.linear.x = 0.3;
        vel_ctrl_msg.linear.y = 0;
        vel_ctrl_msg.linear.z = 0;
        vel_ctrl_msg.angular.x = 0;
        vel_ctrl_msg.angular.y = 0;
        vel_ctrl_msg.angular.z = 0;
        vel_ctrl.publish(vel_ctrl_msg);
        rate.sleep();
        ++send_counter;
        if (send_counter == 50) break;
    }
    return 0;
}