#include "cnuav_control/autopilot.h"
#include "cnuav_control/mpc/mpc.h"

#include <ros/ros.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "cnuav_five");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    const std::string controller_config = "/home/tianbot/multi_uav/src/cnuav_ros/cnuav_control/params/mpc_controller5.yaml";
    cnuav::Controller *mpc_controller = new cnuav::MPCController(controller_config);

    const std::string autopilot_config = "/home/tianbot/multi_uav/src/cnuav_ros/cnuav_control/params/autopilot5.yaml";
    cnuav::Autopilot autopilot(nh, pnh, autopilot_config);

    autopilot.setController(mpc_controller);

    ros::spin();

    return 0;
}