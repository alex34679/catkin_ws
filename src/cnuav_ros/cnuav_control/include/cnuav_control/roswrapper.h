#ifndef SRC_ROSWRAPPER_H
#define SRC_ROSWRAPPER_H

#ifdef EXPERIMENT
#include "cnuav_control/velfilter.h"
#endif


#include <iostream>
#include "cnuav_control/mpc/mpc_params.h"
#include "cnuav_control/common.h"
#include "cnuav_control/status.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/Trajectory.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <ros/ros.h>
#include <random> // 引入随机数生成器相关的头文件
#include <geometry_msgs/PoseArray.h>

namespace cnuav {


        struct GateState {
                std::string name;
                geometry_msgs::Pose pose;
                };


    class ROSWrapper {

    public:
        ROSWrapper() = default;

        ROSWrapper(ros::NodeHandle &nh, ros::NodeHandle &pnh);

        ~ROSWrapper() = default;

        /**
         * return joy control command
         * @return
         */
        u_char getStatus();

        std::vector<quadrotor_msgs::TrajectoryPoint> getTrajectory() const;

        Eigen::Vector3f getTargetPosition() const;

        Eigen::Matrix<float, STATE_DIM, 1> getInitState();

        Eigen::MatrixXf getObstacleStates();

        std::vector<GateState> getgateState();
        

        Eigen::Quaterniond calculateQuaternion(const Eigen::Vector3d& cur_path, double fai);

        Eigen::Vector2f setModelState(const std::string& model_name);


        void pubCmdThrustRates(const Eigen::Ref<const Eigen::Matrix<float, CONTROL_DIM, 1>> &cmd_thrust_rates);

        /// send disarm message to rpg_rotors_interface or sbus_interface
        void pubArmMsg(bool barm);

        

    private:

        void initROSInterface();

        void trajCallback(const quadrotor_msgs::Trajectory::ConstPtr &msg);

        void publishTrajectoryPoints();

        //void pathCallback(const nav_msgs::PathConstPtr &msg);

        void pointCallback(const geometry_msgs::PointConstPtr &msg);

        void joyCallback(const sensor_msgs::JoyConstPtr &msg);

        bool startsWith(const std::string& str, const std::string& prefix);

#ifdef SIMULATION
        void poseCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void gateStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
#elif EXPERIMENT
        void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
        

#endif

        void obstacleCallback(const std_msgs::Float32MultiArrayConstPtr &msg);
       

#ifdef SIMULATION
        /// [p_x p_y p_z v_x v_y v_z]^T
        nav_msgs::Odometry odometry_;
        
        std::vector<GateState> gate_states;

        ros::ServiceClient gate_client;


        
#elif EXPERIMENT
        geometry_msgs::PoseStamped pose_;
        VelFilter vel_filter_;
#endif

        std::vector<quadrotor_msgs::TrajectoryPoint> traj_points_;

        Eigen::Vector3f point_;

        /// joy | point | traj | circle | hover | disarm | land | takeoff
        u_char flag_;

        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Publisher cmdPub_;

        ros::Publisher armPub_;

        ros::Publisher pose_array_pub;

        ros::Subscriber traj_sub_;
        // TODO:
        // ros::Subscriber path_sub_;
        ros::Subscriber point_sub_;
        ros::Subscriber joy_sub_;

        ros::Subscriber pose_sub_;
        ros::Subscriber gate_sub_;
    };
}// namespace cnuav

#endif//SRC_ROSWRAPPER_H