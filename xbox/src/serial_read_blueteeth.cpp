#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "xbox/motor_msg.h"
#include "xbox/four_motor_msg.h"

using namespace std;

serial::Serial ser_blueteeth_;

class serial_read {
private:
    ros::NodeHandle nh_;
    ros::Publisher four_motor_pub;
    ros::Timer timer_;
    int idx;
    double rpm_real, rpm_set, out, pout, iout, dout;
    unsigned char recdata[1000];
    xbox::four_motor_msg four_motor;

    void serial_recieve();
public:
    serial_read(ros::NodeHandle& nh);
    ~serial_read();
};

serial_read::serial_read(ros::NodeHandle& nh) {
    nh_ = nh;
    four_motor_pub = nh_.advertise<xbox::four_motor_msg>("motor_data", 10);
    timer_ = nh.createTimer(ros::Duration(0.05), std::bind(&serial_read::serial_recieve, this));
    idx = 1;
}

serial_read::~serial_read() {}

void serial_read::serial_recieve() {
    size_t n = ser_blueteeth_.available();
    char restrict[100] = "/*%d,%lf,%lf,%lf,%lf,%lf,%lf*/";
    if (n != 0) {
        n = ser_blueteeth_.read(recdata, n);
        sscanf((const char*)recdata, restrict, &idx, &rpm_real, &rpm_set, &out, &pout, &iout, &dout);
        four_motor.motor[idx].rpm_real = rpm_real;
        four_motor.motor[idx].rpm_set = rpm_set;
        four_motor.motor[idx].out = out;
        four_motor.motor[idx].pout = pout;
        four_motor.motor[idx].iout = iout;
        four_motor.motor[idx].dout = dout;
    }
    four_motor_pub.publish(four_motor);
    // ROS_INFO("Recieve data: /*%d,%lf,%lf,%lf,%lf,%lf,%lf*/", idx,
    //     four_motor.motor[idx].rpm_real, four_motor.motor[idx].rpm_set,
    //     four_motor.motor[idx].out, four_motor.motor[idx].pout,
    //     four_motor.motor[idx].iout, four_motor.motor[idx].dout);
    // ROS_INFO("%s, size %d", recdata, n);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "serial_read_blueteeth");
    ros::NodeHandle nh;


    ROS_INFO("xbox_controller started!");
    try {
        ser_blueteeth_.setPort("/dev/rfcomm0");
        // ser.setPort("/dev/ttyUSB0");
        ser_blueteeth_.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(3000);
        ser_blueteeth_.setTimeout(to);
        ser_blueteeth_.open();
    }
    catch (serial::IOException& e) {
        ROS_ERROR("failed to open serial");
        return -1;
    }

    if (ser_blueteeth_.isOpen()) {
        ROS_INFO("serial initilized");
    }
    else {
        ROS_ERROR("serial is not open");
        return -1;
    }
    serial_read mycontorller(nh);
    ros::spin();
    return 0;
}

