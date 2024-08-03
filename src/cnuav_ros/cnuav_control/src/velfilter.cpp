#include "cnuav_control/velfilter.h"


namespace cnuav {

    VelFilter::VelFilter()
        : intervars_(3), binit_(false),last_v_(Eigen::Vector3f::Zero()) {}

    Eigen::Vector3f VelFilter::calVel(geometry_msgs::PoseStamped &pose) {

        if (!binit_) {
            last_pose_ = pose;
            binit_ = true;
            return Eigen::Vector3f::Zero();
        }

        double dt = (pose.header.stamp - last_pose_.header.stamp).toSec();

        if (dt < 1e-3) {
            return last_v_;
        }

        double vx = (pose.pose.position.x - last_pose_.pose.position.x) / dt;
        double vy = (pose.pose.position.y - last_pose_.pose.position.y) / dt;
        double vz = (pose.pose.position.z - last_pose_.pose.position.z) / dt;


        double vx_filtered = updateAxis(0, vx);
        double vy_filtered = updateAxis(1, vy);
        double vz_filtered = updateAxis(2, vz);
        shiftInterVars();

        Eigen::Vector3f v(vx_filtered, vy_filtered, vz_filtered);

        last_pose_ = pose;
        last_v_ = v;

        return v;
    }

    double VelFilter::updateAxis(int i, double x) {
        intervars_[i].interv1 = 0.3333 * x + 1.4803e-16 * intervars_[i].interv2[1];
        intervars_[i].interv2[2] = intervars_[i].interv1 - 0.3333 * intervars_[i].interv2[0];
        intervars_[i].interv3 = intervars_[i].interv2[2] + 2 * intervars_[i].interv2[1];
        intervars_[i].interv4 = intervars_[i].interv3 + intervars_[i].interv2[0];
        intervars_[i].interv5[1] = 0.5 * intervars_[i].interv4 + 5.5511e-17 * intervars_[i].interv5[0];
        double y = intervars_[i].interv5[1] + intervars_[i].interv5[0];

        return y;
    }

    void VelFilter::shiftInterVars() {
        for (int i = 0; i < 3; i++) {
            intervars_[i].interv2[0] = intervars_[i].interv2[1];
            intervars_[i].interv2[1] = intervars_[i].interv2[2];
            intervars_[i].interv5[0] = intervars_[i].interv5[1];
        }
    }
}// namespace mpc