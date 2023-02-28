#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "dynamic_reconfigure/server.h"
#include "xbox/paramConfig.h"
#include "geometry_msgs/Twist.h"
using namespace std;


class controller {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_cmd_vel_;
    dynamic_reconfigure::Server<xbox::paramConfig> server;
    int axis_Lx, axis_Ly, axis_Rx, axis_Ry, axis_Lt, axis_Rt;
    int max_v;
    double max_w;

    void joy_callback(sensor_msgs::Joy::ConstPtr joy_data);
    void reconfig_param(xbox::paramConfig& config);
public:
    controller(ros::NodeHandle& nh);
    ~controller();
};

controller::controller(ros::NodeHandle& nh) {
    nh_ = nh;
    axis_Lx = nh_.param<int>("axis_Lx", 0);
    axis_Ly = nh_.param<int>("axis_Ly", 1);
    axis_Rx = nh_.param<int>("axis_Rx", 2);
    axis_Ry = nh_.param<int>("axis_Ry", 3);
    axis_Lt = nh_.param<int>("axis_Lt", 5);
    axis_Rt = nh_.param<int>("axis_Rt", 4);
    server.setCallback(std::bind(&controller::reconfig_param, this, std::placeholders::_1));
    sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10,
        std::bind(&controller::joy_callback, this, std::placeholders::_1));
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

controller::~controller() {}

void controller::joy_callback(sensor_msgs::Joy::ConstPtr joy_data) {
    double Lx, Ly, Rx, Ry, Lt, Rt;
    Lx = joy_data->axes[axis_Lx];
    Ly = joy_data->axes[axis_Ly];
    Rx = joy_data->axes[axis_Rx];
    Ry = joy_data->axes[axis_Ry];
    Lt = joy_data->axes[axis_Lt];
    Rt = joy_data->axes[axis_Rt];
    ROS_INFO("\nLx: %lf Ly: %lf\nRx: %lf Ry: %lf\nLt: %lf Rt: %lf", Lx, Ly, Rx, Ry, Lt, Rt);
    // nh_.getParam("max_v", max_v);
    // nh_.getParam("max_w", max_w);
    double vx, vy, wz;
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = Ly * max_v;
    cmd_vel.linear.y = Lx * max_v;
    cmd_vel.angular.z = Rx * max_w;
    pub_cmd_vel_.publish(cmd_vel);
}

void controller::reconfig_param(xbox::paramConfig& config) {
    max_v = config.V_max;
    max_w = config.W_max;
    ROS_INFO("max_v: %d max_w: %.3f", max_v, max_w);
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "xbox_controller");
    ros::NodeHandle nh;
    ROS_INFO("xbox_controller started!");
    controller mycontorller(nh);
    ros::spin();
    return 0;
}

