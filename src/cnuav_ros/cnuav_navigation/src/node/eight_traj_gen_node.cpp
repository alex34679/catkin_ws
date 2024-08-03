// #include <ros/ros.h>

// #include <quadrotor_common/trajectory.h>
// #include <trajectory_generation_helper/polynomial_trajectory_helper.h>

// void GenerateCollisionFreeTrajectory(quadrotor_common::Trajectory &trajectory);

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "eight_traj_gen_node");
//     ROS_INFO("Initializing eight_traj_gen_node...");

//     ros::NodeHandle nh;
//     ros::Publisher trajectory_pub = nh.advertise<quadrotor_msgs::Trajectory>("/hummingbird/trajectory", 1);

//     // Pause 5 seconds waiting for taking off
//     ROS_INFO("Pausing for 5 seconds to wait for takeoff...");
//     ros::Duration(5).sleep();

//     quadrotor_common::Trajectory trajectory;
//     ROS_INFO("Generating trajectory...");
//     GenerateCollisionFreeTrajectory(trajectory);

//     // Deal with bug
//     ROS_INFO("Dealing with the bug by modifying the last trajectory point...");
//     if (!trajectory.points.empty()) {
//         auto point = &trajectory.points.back();
//         point->position = Eigen::Vector3d(0, 0, 0.5);
//         point->orientation = Eigen::Quaterniond::Identity();
//         point->velocity = Eigen::Vector3d::Zero();
//         ROS_INFO("Modified the last point: position (0, 0, 0.5), orientation (Identity), velocity (Zero)");
//     } else {
//         ROS_WARN("The trajectory is empty, no point to modify.");
//     }

//     quadrotor_msgs::Trajectory msg = trajectory.toRosMessage();
//     ROS_INFO("Publishing trajectory...");
//     trajectory_pub.publish(msg);

//     ROS_INFO("Node exiting...");
//     return 0;
// }
// void GenerateCollisionFreeTrajectory(quadrotor_common::Trajectory &trajectory) {

//     double kExecLoopRate_ = 120.0;

//     const double max_vel = 1.0;
//     const double max_thrust = 10.0;
//     const double max_roll_pitch_rate = 1.5;

//     std::vector<Eigen::Vector3d> way_points;
//     way_points.push_back(Eigen::Vector3d(0.0, 0.0, 0.5));
//     way_points.push_back(Eigen::Vector3d(1.0, 2.0, 1.0));
//     way_points.push_back(Eigen::Vector3d(2.0, 0.0, 1.5));
//     way_points.push_back(Eigen::Vector3d(1.0, -2.0, 1.0));
//     way_points.push_back(Eigen::Vector3d(0.0, 0.0, 0.5));
//     way_points.push_back(Eigen::Vector3d(-1.0, 2.0, 1.0));
//     way_points.push_back(Eigen::Vector3d(-2.0, 0.0, 1.5));
//     way_points.push_back(Eigen::Vector3d(-1.0, -2.0, 1.0));
//     way_points.push_back(Eigen::Vector3d(0, 0, 0.5));


//     Eigen::VectorXd initial_ring_segment_times =
//             Eigen::VectorXd::Ones(int(way_points.size()));
//     polynomial_trajectories::PolynomialTrajectorySettings
//             ring_trajectory_settings;
//     ring_trajectory_settings.continuity_order = 4;
//     Eigen::VectorXd minimization_weights(5);
//     minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;
//     ring_trajectory_settings.minimization_weights = minimization_weights;
//     ring_trajectory_settings.polynomial_order = 11;
//     ring_trajectory_settings.way_points = way_points;

//     trajectory = trajectory_generation_helper::
//             polynomials::generateMinimumSnapRingTrajectoryWithSegmentRefinement(
//                     initial_ring_segment_times, ring_trajectory_settings, max_vel,
//                     max_thrust, max_roll_pitch_rate, kExecLoopRate_);
// }








#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <quadrotor_common/trajectory.h>
#include <quadrotor_msgs/Trajectory.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include <Eigen/Dense>
#include <vector>
#include <nav_msgs/Odometry.h>


// 全局变量来存储初始位置和路标点
geometry_msgs::PoseStamped initial_pose;
geometry_msgs::PoseStamped final_pose;
std::vector<geometry_msgs::PoseStamped> waypoints;
bool initial_pose_received = false;
bool waypoints_received = false;

bool if_sim;

void GenerateCollisionFreeTrajectory(quadrotor_common::Trajectory &trajectory);

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    // initial_pose = *msg;
    initial_pose_received = true;
    ROS_INFO("Initial pose received.");
    // if (!waypoints.empty()) {
    //     final_pose = waypoints.back();  // 获取最后一个元素
    //     waypoints.pop_back();           // 删除最后一个元素
    // } 


}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    waypoints.push_back(*msg);
    waypoints_received = true;
    ROS_INFO("Goal received.");
}

void odometryCallback_sim(const nav_msgs::Odometry::ConstPtr& msg){
    initial_pose.pose.position.x = msg->pose.pose.position.x;
    initial_pose.pose.position.y = msg->pose.pose.position.y;
    initial_pose.pose.position.z = msg->pose.pose.position.z;
}

void odometryCallback_exp(const geometry_msgs::PoseStampedConstPtr& msg)
{
    initial_pose.pose.position.x = msg->pose.position.x;
    initial_pose.pose.position.y = msg->pose.position.y;
    initial_pose.pose.position.z = msg->pose.position.z;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "eight_traj_gen_node");
    ROS_INFO("Initializing eight_traj_gen_node...");

    ros::NodeHandle nh("~"); 
    nh.getParam("mode", if_sim);


    ros::NodeHandle nh_;
    ros::Publisher trajectory_pub = nh_.advertise<quadrotor_msgs::Trajectory>("traj", 1);
    // ros::Publisher trajectory_pub = nh.advertise<quadrotor_msgs::Trajectory>("/multi3/trajectory", 1);

    ros::Subscriber initial_pose_sub = nh_.subscribe("/initialpose", 1, initialPoseCallback);
    ros::Subscriber goal_sub = nh_.subscribe("/move_base_simple/goal", 1, goalCallback);

    // ros::Subscriber odom_sub_ = nh.subscribe("/vrpn_client_node/multi3/pose", 10, odometryCallback);
    ros::Subscriber odom_sub;
    if (if_sim) {
        odom_sub = nh_.subscribe("pose", 10, odometryCallback_sim);
    } else {
        odom_sub = nh_.subscribe("pose", 10, odometryCallback_exp);
    }

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        // 如果收到初始位置和至少一个路标点，生成并发布路径
        if (initial_pose_received && waypoints_received) {
            ROS_INFO("Generating trajectory...");

            quadrotor_common::Trajectory trajectory;
            GenerateCollisionFreeTrajectory(trajectory);


            if (!trajectory.points.empty()) {
                auto point = &trajectory.points.back();
                // point->position = Eigen::Vector3d(0, 0, 0.5);
                point->orientation = Eigen::Quaterniond::Identity();
                point->velocity = Eigen::Vector3d::Zero();
                ROS_INFO("Modified the last point: position (0, 0, 0.5), orientation (Identity), velocity (Zero)");
            } else {
                ROS_WARN("The trajectory is empty, no point to modify.");
            }

            quadrotor_msgs::Trajectory msg = trajectory.toRosMessage();
            ROS_INFO("Publishing trajectory...");
            trajectory_pub.publish(msg);

            // 重置标志位
            initial_pose_received = false;
            waypoints_received = false;
            waypoints.clear();
        }

        rate.sleep();
    }

    ROS_INFO("Node exiting...");
    return 0;
}

void GenerateCollisionFreeTrajectory(quadrotor_common::Trajectory &trajectory) {
    double kExecLoopRate_ = 120.0;

    const double max_vel = 0.5;
    const double max_thrust = 10.0;
    const double max_roll_pitch_rate = 2.5;

    std::vector<Eigen::Vector3d> way_points;
    // way_points.push_back(Eigen::Vector3d(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z));

    for (const auto &wp : waypoints) {
        way_points.push_back(Eigen::Vector3d(wp.pose.position.x, wp.pose.position.y, wp.pose.position.z+0.5));
    }

    // Eigen::VectorXd initial_ring_segment_times = Eigen::VectorXd::Ones(int(way_points.size()));
    // polynomial_trajectories::PolynomialTrajectorySettings ring_trajectory_settings;
    // ring_trajectory_settings.continuity_order = 4;
    // Eigen::VectorXd minimization_weights(5);
    // minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;
    // ring_trajectory_settings.minimization_weights = minimization_weights;
    // ring_trajectory_settings.polynomial_order = 11;
    // ring_trajectory_settings.way_points = way_points;

    // trajectory = trajectory_generation_helper::polynomials::generateMinimumSnapRingTrajectoryWithSegmentRefinement(
    //     initial_ring_segment_times, ring_trajectory_settings, max_vel, max_thrust, max_roll_pitch_rate, kExecLoopRate_);


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
    start_state.position = Eigen::Vector3d(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z);
    start_state.orientation = Eigen::Quaterniond::Identity();
    start_state.velocity = Eigen::Vector3d::Zero();
    quadrotor_common::TrajectoryPoint end_state;
    end_state.position = Eigen::Vector3d(waypoints.back().pose.position.x, waypoints.back().pose.position.y, waypoints.back().pose.position.z+0.8);
    end_state.orientation = Eigen::Quaterniond::Identity();
    end_state.velocity = Eigen::Vector3d::Zero();

    trajectory = trajectory_generation_helper::
            polynomials::generateMinimumSnapTrajectoryWithSegmentRefinement(
                    initial_segment_times, start_state, end_state, trajectory_settings, max_vel,
                    max_thrust, max_roll_pitch_rate, kExecLoopRate_);


}


