#include <geometry_msgs/Point.h>
#include <quadrotor_common/trajectory.h>
#include <ros/ros.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>

void GenerateCollisionFreeTrajectory(quadrotor_common::Trajectory &trajectory);

int main(int argc, char **argv) {
    ros::init(argc, argv, "collision_free_node");

    ros::NodeHandle nh;
    ros::Publisher trajectory_pub = nh.advertise<quadrotor_msgs::Trajectory>("hummingbird/trajectory", 1);

    //pause 5 second waiting for taking off
    ros::Duration(5).sleep();

    quadrotor_common::Trajectory trajectory;
    GenerateCollisionFreeTrajectory(trajectory);

    /// deal with bug
    trajectory.points.pop_back();

    quadrotor_msgs::Trajectory msg = trajectory.toRosMessage();
    trajectory_pub.publish(msg);

    exit(0);
}

void GenerateCollisionFreeTrajectory(quadrotor_common::Trajectory &trajectory) {

    double kExecLoopRate_ = 120.0;

    const double max_vel = 1.0;
    const double max_thrust = 11.0;
    const double max_roll_pitch_rate = 1.0;

    float epsilon = 0.05;

    std::vector<Eigen::Vector3d> way_points;
    way_points.push_back(Eigen::Vector3d(-1.5, 0.37 + epsilon, 0.5));
    //way_points.push_back(Eigen::Vector3d(-1.0, 0.5 + epsilon, 0.5));
    //way_points.push_back(Eigen::Vector3d(-0.5, 0.37 + +epsilon, 0.5));
    way_points.push_back(Eigen::Vector3d(0.0, 0.0, 0.5));
    //way_points.push_back(Eigen::Vector3d(0.5, -0.37 - epsilon, 0.5));
    //way_points.push_back(Eigen::Vector3d(1.0, -0.5 - epsilon, 0.5));
    way_points.push_back(Eigen::Vector3d(1.5, -0.37 - epsilon, 0.5));

    Eigen::VectorXd initial_segment_times =
            Eigen::VectorXd::Ones(int(way_points.size())+1);
    polynomial_trajectories::PolynomialTrajectorySettings
            trajectory_settings;
    trajectory_settings.continuity_order = 4;
    Eigen::VectorXd minimization_weights(5);
    minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;
    trajectory_settings.minimization_weights = minimization_weights;
    trajectory_settings.polynomial_order = 11;
    trajectory_settings.way_points = way_points;

    quadrotor_common::TrajectoryPoint start_state;
    start_state.position = Eigen::Vector3d(-2, 0, 0.5);
    quadrotor_common::TrajectoryPoint end_state;
    end_state.position = Eigen::Vector3d(2, 0, 0.5);

    trajectory = trajectory_generation_helper::
            polynomials::generateMinimumSnapTrajectoryWithSegmentRefinement(
                    initial_segment_times, start_state, end_state, trajectory_settings, max_vel,
                    max_thrust, max_roll_pitch_rate, kExecLoopRate_);
}