#include <sbus_wrapper/sbus_wrapper.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "sbus_wrapper_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    sbus_wrapper::SbusWrapper wrapper(nh, pnh, true);

    ros::spin();
}