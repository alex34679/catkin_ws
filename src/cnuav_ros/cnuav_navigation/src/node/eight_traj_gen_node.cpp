#include <ros/ros.h>

#include <quadrotor_common/trajectory.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>

void GenerateCollisionFreeTrajectory(quadrotor_common::Trajectory &trajectory);

int main(int argc, char **argv) {
    ros::init(argc, argv, "eight_traj_gen_node");

    ros::NodeHandle nh;
    ros::Publisher trajectory_pub = nh.advertise<quadrotor_msgs::Trajectory>("hummingbird/trajectory", 1);

    //pause 5 second waiting for taking off
    ros::Duration(5).sleep();

    quadrotor_common::Trajectory trajectory;
    GenerateCollisionFreeTrajectory(trajectory);

    /// deal with bug
    auto point = &trajectory.points.back();
    point->position = Eigen::Vector3d(0, 0, 0.5);
    point->orientation =  Eigen::Quaterniond::Identity();
    point->velocity = Eigen::Vector3d::Zero();

    quadrotor_msgs::Trajectory msg = trajectory.toRosMessage();
    trajectory_pub.publish(msg);

    exit(0);
}

void GenerateCollisionFreeTrajectory(quadrotor_common::Trajectory &trajectory) {

    double kExecLoopRate_ = 120.0;

    const double max_vel = 2.0;
    const double max_thrust = 12.0;
    const double max_roll_pitch_rate = 1.0;

    std::vector<Eigen::Vector3d> way_points;
    way_points.push_back(Eigen::Vector3d(0.0, 0.0, 0.5));
    way_points.push_back(Eigen::Vector3d(1.0, 2.0, 1.0));
    way_points.push_back(Eigen::Vector3d(2.0, 0.0, 1.5));
    way_points.push_back(Eigen::Vector3d(1.0, -2.0, 1.0));
    way_points.push_back(Eigen::Vector3d(0.0, 0.0, 0.5));
    way_points.push_back(Eigen::Vector3d(-1.0, 2.0, 1.0));
    way_points.push_back(Eigen::Vector3d(-2.0, 0.0, 1.5));
    way_points.push_back(Eigen::Vector3d(-1.0, -2.0, 1.0));
    way_points.push_back(Eigen::Vector3d(0, 0, 0.5));


    Eigen::VectorXd initial_ring_segment_times =
            Eigen::VectorXd::Ones(int(way_points.size()));
    polynomial_trajectories::PolynomialTrajectorySettings
            ring_trajectory_settings;
    ring_trajectory_settings.continuity_order = 4;
    Eigen::VectorXd minimization_weights(5);
    minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;
    ring_trajectory_settings.minimization_weights = minimization_weights;
    ring_trajectory_settings.polynomial_order = 11;
    ring_trajectory_settings.way_points = way_points;

    trajectory = trajectory_generation_helper::
            polynomials::generateMinimumSnapRingTrajectoryWithSegmentRefinement(
                    initial_ring_segment_times, ring_trajectory_settings, max_vel,
                    max_thrust, max_roll_pitch_rate, kExecLoopRate_);
}