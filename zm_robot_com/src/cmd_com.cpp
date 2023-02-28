#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

class cmdCom {
private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_;
    serial::Serial ser_;
    char data[100];
    void cmdCallback(const geometry_msgs::TwistConstPtr& msg);
public:
    std::string port;
    int baudrate;
    cmdCom(ros::NodeHandle& nh);
    ~cmdCom();
};

cmdCom::cmdCom(ros::NodeHandle& nh):nh_(nh) {
    ROS_INFO("cmd_com started");
    nh_.param<std::string>("port", port, "/dev/ttyUSB0");
    nh_.param<int>("baudrate", baudrate, 115200);
    while (ros::ok() && !ser_.isOpen()) {
        try { // 设置串口属性，并打开串口 
            ser_.setPort(port);
            ser_.setBaudrate(baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(to);
            ser_.open();
        }
        catch (serial::IOException& e) {
            ROS_ERROR_STREAM("Unable to open port");
        }
        // 检测串口是否已经打开，并给出提示信息 
        if (ser_.isOpen()) {
            ROS_INFO_STREAM("Serial Port initialized");
        }
        else {
            ROS_ERROR_STREAM("Unable to open port");
        }
        ros::Duration(1).sleep();
    }




    cmd_sub_ = nh_.subscribe("/cmd_vel", 1, &cmdCom::cmdCallback, this);
}

cmdCom::~cmdCom() {}

void cmdCom::cmdCallback(const geometry_msgs::TwistConstPtr& msg) {
    int x = int(msg->linear.x * 100);
    int y = int(msg->linear.y * 100);
    double z = msg->angular.z;

    snprintf(data, sizeof(data), "vel|0|% .3d|% .3d|% .2f|", x, y, z);
    ser_.write(data);
    ROS_INFO_STREAM(data);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_com");
    ros::NodeHandle nh("~");
    cmdCom cmd_com(nh);
    ros::spin();
    return 0;
}
