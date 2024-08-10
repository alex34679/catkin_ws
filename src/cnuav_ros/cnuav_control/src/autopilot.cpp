#include "cnuav_control/autopilot.h"
#include <yaml-cpp/yaml.h>

namespace cnuav {

    Autopilot::Autopilot(ros::NodeHandle &nh, ros::NodeHandle &pnh, const std::string &filename)
        : nh_(nh), pnh_(pnh),
          wrapper_(new ROSWrapper(nh, pnh)),
          status_(Disarm) {

        error_integral_x_ = 0.0;
        error_integral_y_ = 0.0;

        loadParams(filename);

        if (enable_log_) {
            fout_.open(log_filename_);
            fout_ << "time,costs\n";
        }

        prefix_ = "[" + nh_.getNamespace() + "] ";
        mainloopTimer_ = nh_.createTimer(ros::Duration(1.0 / frequency_), &Autopilot::mainLoop, this);

        base_controller_params_.loadParameters(pnh);

        pnh.getParam("mpc", if_mpc);

        if(if_mpc){
            ROS_INFO_ONCE("now control mod is MPC !!!");
        }
        else{
            ROS_INFO_ONCE("now control mod is position control !!!");
        }
    }

    bool Autopilot::loadParams(const std::string &filename) {

        bool bSuccess = true;

        YAML::Node node = YAML::LoadFile(filename);

        assert(!node.IsNull());

        if (node["frequency"]) {
            frequency_ = node["frequency"].as<float>();
        } else {
            bSuccess = false;
        }

        if (node["verbose"]) {
            verbose_ = node["verbose"].as<bool>();
        } else {
            bSuccess = false;
        }

        if (node["enable_log"]) {
            enable_log_ = node["enable_log"].as<bool>();
        } else {
            bSuccess = false;
        }

        if (enable_log_) {
            if (node["log_filename"]) {
                log_filename_ = node["log_filename"].as<std::string>();
                std::cout << log_filename_ << std::endl;
            } else {
                bSuccess = false;
            }
        }

        if (node["enable_integral_compensation"]) {
            b_integral_compensation_ = node["enable_integral_compensation"].as<bool>();
        } else {
            bSuccess = false;
        }

        if (b_integral_compensation_) {

            if (node["max_error_integral"]) {
                max_error_integral_ = node["max_error_integral"].as<float>();
            } else {
                bSuccess = false;
            }

            if (node["k_integral_compensation"]) {
                k_integral_compensation_ = node["k_integral_compensation"].as<float>();
            } else {
                bSuccess = false;
            }
        }

        if (node["takeoff_height"]) {
            takeoff_height_ = node["takeoff_height"].as<float>();
        } else {
            bSuccess = false;
        }

        if (node["takeoff_velocity"]) {
            takeoff_velocity_ = node["takeoff_velocity"].as<float>();
        } else {
            bSuccess = false;
        }

        if (node["land_velocity"]) {
            land_velocity_ = node["land_velocity"].as<float>();
        } else {
            bSuccess = false;
        }

        if (node["circle_center"]) {
            std::vector<float> v = node["circle_center"].as<std::vector<float>>();
            assert(v.size() == 3);
            circle_center_ = Eigen::Vector3f(v.data());
        } else {
            bSuccess = false;
        }

        if (node["circle_velocity"]) {
            circle_velocity_ = node["circle_velocity"].as<float>();
        } else {
            bSuccess = false;
        }

        if (node["circle_radius"]) {
            circle_radius_ = node["circle_radius"].as<float>();
            assert(circle_radius_ > 0);
        } else {
            bSuccess = false;
        }

        if (node["circle_height"]) {
            circle_height_ = node["circle_height"].as<float>();
            assert(circle_height_ > 0);
        } else {
            bSuccess = false;
        }

        if (node["std_velocity"]) {
            std_velocity_ = node["std_velocity"].as<float>();
            assert(std_velocity_ > 0);
        } else {
            bSuccess = false;
        }

        if (node["std_acceleration"]) {
            std_acceleration_ = node["std_acceleration"].as<float>();
            assert(std_acceleration_ > 0);
        } else {
            bSuccess = false;
        }

        if (node["circle_warmup_time"]) {
            circle_warmup_time_ = node["circle_warmup_time"].as<float>();
             if (std_velocity_ / std_acceleration_ > circle_warmup_time_) {
                    circle_warmup_time_ = std_velocity_ / std_acceleration_;
                 }
        } else {
            bSuccess = false;
        }
        if (node["slow_down_velocity"]) {
            slow_down_velocity_ = node["slow_down_velocity"].as<float>();
        } else {
            bSuccess = false;
        }
        if (node["slow_down_time"]) {
            slow_down_time_ = node["slow_down_time"].as<float>();
            if ((circle_velocity_ - slow_down_velocity_) / std_acceleration_ > slow_down_time_) {
                slow_down_time_ = (circle_velocity_ - slow_down_velocity_) / std_acceleration_;
            }
        } else {
            bSuccess = false;
        }

        if (node["id"]) {
            quad_id_ = node["id"].as<int>();
        } else {
            bSuccess = false;
        }

        return bSuccess;
    }

    void Autopilot::setController(Controller *controller) {
        controller_ = controller;
    }

    void Autopilot::mainLoop(const ros::TimerEvent &event) {

        current_state_ = wrapper_->getInitState();

        #ifdef SIMULATION
        current_gate_states_ = wrapper_->getgateState();
        for (const auto& gate_state : current_gate_states_) {
                // ROS_INFO_ONCE("Gate name: %s", gate_state.name.c_str());
                // ROS_INFO("Position - x: %f, y: %f, z: %f",
                //         gate_state.pose.position.x,
                //         gate_state.pose.position.y,
                //         gate_state.pose.position.z);
                // ROS_INFO("Orientation - x: %f, y: %f, z: %f, w: %f",
                //         gate_state.pose.orientation.x,
                //         gate_state.pose.orientation.y,
                //         gate_state.pose.orientation.z,
                //         gate_state.pose.orientation.w);
            }

        #endif

        ros::Time current_time = ros::Time::now();

        /// 更新指令
        u_char flag = wrapper_->getStatus();
        if (flag) {
            if (!switchStatus(flag)) {
                ROS_WARN_COND(verbose_, "Status Switch Fail");
            }
        }

        /// disarm: do nothing
        if (status_ == Disarm) {
            if (verbose_) {
                ROS_INFO_THROTTLE(1.0, "%sDisarm", prefix_.c_str());
            }
            return;
        }

        /// if task is going to finish, status will be switched to next stage
        if (status_ != Hover && taskFinished()) {
            ROS_INFO_STREAM_COND(verbose_, prefix_ + "Task Finish");
            if (status_ == Land) {
                ROS_INFO_STREAM_COND(verbose_, prefix_ + "Disarm");
                disarmDrone();
                status_ = Disarm;
                ros::shutdown();
                exit(EXIT_SUCCESS);
            } else {
                ROS_INFO_STREAM_COND(verbose_, prefix_ + "Hover");
                status_ = Hover;
                quadrotor_msgs::TrajectoryPoint hover_point = traj_points_.back();
                Eigen::Vector3f hover_position;
                hover_position << hover_point.pose.position.x,
                        hover_point.pose.position.y,
                        hover_point.pose.position.z;

                genTrajectoryHover(hover_position);
            }
        }

        controller_->setInitState(current_state_);
        
        ros::Duration time_from_start = current_time - start_time_;

        int N = controller_->getTimestep();
        Eigen::MatrixXf ref_states = Eigen::MatrixXf::Zero(STATE_DIM, N);

        calRefStates(time_from_start, ref_states);

        float dt = controller_->getDt();
        ros::Duration t = time_from_start + ros::Duration(dt);
        auto ref_point = getTrajectoryPoint(t);

        quadrotor_common::TrajectoryPoint reference_trajectory_;
        reference_trajectory_.position.x() = ref_point.pose.position.x;
        reference_trajectory_.position.y() = ref_point.pose.position.y; 
        reference_trajectory_.position.z() = ref_point.pose.position.z; 

        reference_trajectory_.velocity.x() = ref_point.velocity.linear.x;
        reference_trajectory_.velocity.y() = ref_point.velocity.linear.y;
        reference_trajectory_.velocity.z() = ref_point.velocity.linear.z;

        reference_trajectory_.acceleration.x() = ref_point.acceleration.linear.x;
        reference_trajectory_.acceleration.y() = ref_point.acceleration.linear.y;
        reference_trajectory_.acceleration.z() = ref_point.acceleration.linear.z;
        
        quadrotor_common::QuadStateEstimate state_estimate;
        /// [ p_x p_y p_z v_x v_y v_z q_w q_x q_y q_z ]  false
        state_estimate.position.x() = current_state_[0];
        state_estimate.position.y() = current_state_[1];
        state_estimate.position.z() = current_state_[2];

        state_estimate.orientation.w() = current_state_[3];
        state_estimate.orientation.x() = current_state_[4];
        state_estimate.orientation.y() = current_state_[5];
        state_estimate.orientation.z() = current_state_[6];

        state_estimate.velocity.x() = current_state_[7];
        state_estimate.velocity.y() = current_state_[8];
        state_estimate.velocity.z() = current_state_[9];


        cmd = pos_controller.run(state_estimate, reference_trajectory_,
                                 base_controller_params_);


        controller_->setRefStates(ref_states);

        controller_->preControlProcess();

        clock_t start = clock();

        int success = controller_->computeControl();

        /// time statistics variables
        clock_t end = clock();
        float time = 1000.0 * float(end - start) / CLOCKS_PER_SEC;
        
        if(success){
            // ROS_INFO("Elapsed time: %.2f ms", time);
        }
        else{
            ROS_INFO("fail Elapsed time: %.2f ms", time);
        }


        if (success) {
            Eigen::Matrix<float, CONTROL_DIM, 1> inputs = controller_->getInputs();



                // 使用匿名函数打印矩阵
                auto printEigenMatrix = [](const Eigen::Matrix<float, CONTROL_DIM, 1>& matrix) {
                    std::ostringstream oss;
                    oss << "inputs_: [";
                    for (int i = 0; i < CONTROL_DIM; ++i) {
                        oss << matrix(i);
                        if (i < CONTROL_DIM - 1) {
                            oss << ", ";
                        }
                    }
                    oss << "]";
                    ROS_INFO("%s", oss.str().c_str());
                };

                // 调用匿名函数打印矩阵
                // printEigenMatrix(inputs_);

            bool valid = true;
            for (int i = 0; i < CONTROL_DIM; i++) {
                if (isinf(inputs(i)) || isnan(inputs(i))) {
                    valid = false;
                }
            }
            if (valid) {
                inputs_ = inputs;
            } else {
                ROS_WARN("Control command invalid!");
                controller_->reset();
            }

        } else {
            ROS_WARN("Controller fail!");
            //controller_->reset();
        }

#ifdef EXPERIMENT
        inputs_ += calSteadyCompensation();
#endif

        if(if_mpc){
            wrapper_->pubCmdThrustRates(inputs_);
        }
        else{
            quadrotor_msgs::ControlCommand ros_cmd = cmd.toRosMessage();
            wrapper_->pubCmd(ros_cmd);
        }


        controller_->postControlProcess();
    }

    bool Autopilot::taskFinished() const {

        if (status_ == Disarm || status_ == Circle||status_ == Slowdown) {
            return false;
        }

        ROS_ASSERT(!traj_points_.empty());

        if (ros::Time::now() - start_time_ > finish_duration_ + ros::Duration(1.0)) {
            return true;
        }

        /// if current time exceeds scheduled time and state is close to end position, return true
        const quadrotor_msgs::TrajectoryPoint endpoint = traj_points_.back();
        if (ros::Time::now() - start_time_ > endpoint.time_from_start) {
            float distance = sqrtf(powf(current_state_[0] - endpoint.pose.position.x, 2) +
                                   powf(current_state_[1] - endpoint.pose.position.y, 2) +
                                   powf(current_state_[2] - endpoint.pose.position.z, 2));
            if (distance < 5e-2) {
                return true;
            }
        }
        return false;
    }

    bool Autopilot::switchStatus(u_char flag) {

        /// disarm
        if (flag & 1 << 2) {
            status_ = Disarm;
            ROS_INFO_STREAM_COND(verbose_, prefix_ + "Force Disarm");
            return true;
        }

        /// force hover
        if (status_ != Disarm && flag & 1 << 3) {
            // if(status_== Circle && circle_velocity_ >=0.5 && (ros::Time::now()-start_time_).toSec() > 0.5)
            // {   
            //     status_ = Circle;
            //     start_time_ = ros::Time::now();
            //     circle_velocity_-=0.1;
            //     // circle_warmup_time_= 0;
            //     circle_start_rad_ = atan2(current_state_[1] , current_state_[0]);
            //     circle_start_radius_ = sqrt(current_state_[0] * current_state_[0] + current_state_[1] * current_state_[1]);
            //     circle_start_height_ = current_state_[2];
            //     ROS_INFO_STREAM_COND(verbose_, prefix_ + "Slow down");
            //     return true;
            // }
            // sqrt(current_state_[7] * current_state_[7] + current_state_[8] * current_state_[8])

            if(status_== Circle &&  circle_velocity_ >slow_down_velocity_)
            {   
                status_ = Slowdown;
                start_time_ = ros::Time::now();
                circle_start_rad_ = atan2(current_state_[1] , current_state_[0]);
                circle_start_radius_ = circle_radius_;
                circle_start_height_ = circle_height_;
                ROS_INFO_STREAM_COND(verbose_, prefix_ + "Slow down");
                return true;
            }
        
        
            else{
            Eigen::Vector3f last_point;
            last_point << traj_points_.back().pose.position.x,
                    traj_points_.back().pose.position.y,
                    traj_points_.back().pose.position.z;

            Eigen::Vector3f current_point = current_state_.topRows(3);
            if ((last_point - current_point).norm() < 1e-1) {
                genTrajectoryHover(last_point);
            } else {
                genTrajectoryHover(current_point);
            }
            status_ = Hover;
            ROS_INFO_STREAM_COND(verbose_, prefix_ + "Force Hover");
            return true;
            }
        }

        /// receive takeoff command
        if (status_ == Disarm && (flag & 1)) {

            status_ = TakeOff;

            /// send disarm message to rpg_rotors_interface or sbus_interface
            armDrone();

            Eigen::Vector3f takeoff_position(current_state_[0], current_state_[1], takeoff_height_);
            genTrajectory(current_state_, takeoff_position, takeoff_velocity_);
            ROS_INFO_STREAM_COND(verbose_, prefix_ + "Take Off");
            return true;
        }

        /// receive land command
        if (status_ == Hover && (flag & 1 << 1)) {
            status_ = Land;
            Eigen::Vector3f land_position(current_state_[0], current_state_[1], 0.0);
            genTrajectory(current_state_, land_position, land_velocity_);
            ROS_INFO_STREAM_COND(verbose_, prefix_ + "Land");
            return true;
        }

        /// receive circle command
        if (status_ == Hover && (flag & 1 << 4)) {
            status_ = Circle;
            start_time_ = ros::Time::now();
            circle_start_rad_ = atan2(current_state_[1] , current_state_[0]);
            circle_start_radius_ = sqrt(current_state_[0] * current_state_[0] + current_state_[1] * current_state_[1]);
            circle_start_height_ = current_state_[2];
            ROS_INFO_STREAM_COND(verbose_, prefix_ + "Circle");
            return true;
        }

        /// receive trajectory command
        if (status_ == Hover && (flag & 1 << 5)) {
            status_ = Trajectory;
            traj_points_ = wrapper_->getTrajectory();
            start_time_ = ros::Time::now();
            finish_duration_ = traj_points_.back().time_from_start;
            ROS_INFO_STREAM_COND(verbose_, prefix_ + "now : Trajectory");
            return true;
        }

                /// receive trajectory command
        if (status_ == Trajectory && (flag & 1 << 5)) {
            status_ = Trajectory;
            traj_points_ = wrapper_->getTrajectory();
            start_time_ = ros::Time::now();
            finish_duration_ = traj_points_.back().time_from_start;
            ROS_INFO_STREAM_COND(verbose_, prefix_ + "now : Trajectory");
            return true;
        }

        /// receive point to point command
        // if (status_ == Hover && (flag & 1 << 6)) {
        //     status_ = Trajectory;
        //     Eigen::Vector3f point = wrapper_->getTargetPosition();
        //     genTrajectory(current_state_, point, std_velocity_);
        //     ROS_INFO_COND(verbose_, "%sGo to Point %6.2f %6.2f %6.2f", prefix_.c_str(), point[0], point[1], point[2]);
        //     return true;
        // }

        //根据指令，自动到门前
        if (status_ == Hover && (flag & 1 << 7)) {

            Eigen::Vector3f point;

            #ifdef SIMULATION
            // Eigen::Vector2f point_temp = wrapper_->setModelState("gate0");
            // for (auto& gate_state : current_gate_states_) {
            //     if (gate_state.name == "gate0") {
            //         gate_state.pose.position.x = point_temp[0];
            //         gate_state.pose.position.y = point_temp[1];
            //     }
            // }
            //在和gazebo环境交互后返回gazebo中的位置
            Eigen::Vector2f point_2 = calculateNearestPoint(current_state_, current_gate_states_);
            point << point_2[0], point_2[1], takeoff_height_;
            genTrajectory(current_state_, point, std_velocity_);
            ROS_INFO_STREAM_COND(verbose_, prefix_ + "Ready_Crossgate");
            #endif
            return true;


            

            // //改变无人机圆轨迹参数
            // raw_radius = circle_radius_;
            // raw_vel = circle_velocity_;

            // circle_radius_ = 1.0;
            // circle_velocity_ = 0.5;

            // //变换位置
            // if (quad_id_ == 1){
            //     status_ = Trajectory;
            //     local_rad = atan2(current_state_[1], current_state_[0]);
            //     target_rad = local_rad + 0.55;
            //     float target_radius = 1.0;

            //     Eigen::Vector3f point;
            //     point << target_radius*cos(target_rad), target_radius*sin(target_rad), 0.75;
            //     genTrajectory(current_state_, point, std_velocity_);
            // }
            // else if (quad_id_ == 3)
            // {
            //     status_ = Trajectory;
            //     local_rad = atan2(current_state_[1], current_state_[0]);
            //     target_rad = local_rad - 0.55;
            //     float target_radius = 1.0;
                
            //     Eigen::Vector3f point;
            //     point << target_radius*cos(target_rad), target_radius*sin(target_rad), 0.75;
            //     genTrajectory(current_state_, point, std_velocity_);

            // }

            // else if (quad_id_ == 2)
            // {
            //     Eigen::Vector3f last_point;
            //     last_point << traj_points_.back().pose.position.x,
            //             traj_points_.back().pose.position.y,
            //             traj_points_.back().pose.position.z;

            //     Eigen::Vector3f current_point = current_state_.topRows(3);
            //     if ((last_point - current_point).norm() < 1e-1) {
            //         genTrajectoryHover(last_point);
            //     } else {
            //         genTrajectoryHover(current_point);
            //     }
            //     status_ = Hover;
            // }
            // return true;
        }


        if (status_ == Hover && (flag & 1 << 6)) {
            
            Eigen::Vector3f point;

            #ifdef SIMULATION
            Eigen::Vector2f point_2 = calculateCrossgatePoint(current_state_,current_gate_states_);
            point << point_2[0], point_2[1], takeoff_height_;
            genTrajectory(current_state_, point, std_velocity_);
            ROS_INFO_STREAM_COND(verbose_, prefix_ + "Crossgate");   
            #endif

            return  true; 
            // // 恢复原有参数
            // circle_radius_ = raw_radius;
            // circle_velocity_ = raw_vel;

            // //变换位置
            // if (quad_id_ == 1){
            //     status_ = Trajectory;
            //     local_rad = atan2(current_state_[1], current_state_[0]);
            //     target_rad = local_rad - 0.55;

            //     Eigen::Vector3f point;
            //     point << circle_radius_*cos(target_rad), circle_radius_*sin(target_rad), 0.75;
            //     genTrajectory(current_state_, point, std_velocity_);
            // }
            // else if (quad_id_ == 3)
            // {
            //     status_ = Trajectory;
            //     local_rad = atan2(current_state_[1], current_state_[0]);
            //     target_rad = local_rad + 0.55;
                
            //     Eigen::Vector3f point;
            //     point << circle_radius_*cos(target_rad), circle_radius_*sin(target_rad), 0.75;
            //     genTrajectory(current_state_, point, std_velocity_);
            // }
            // else if (quad_id_ == 2)
            // {
            //     Eigen::Vector3f last_point;
            //     last_point << traj_points_.back().pose.position.x,
            //             traj_points_.back().pose.position.y,
            //             traj_points_.back().pose.position.z;

            //     Eigen::Vector3f current_point = current_state_.topRows(3);
            //     if ((last_point - current_point).norm() < 1e-1) {
            //         genTrajectoryHover(last_point);
            //     } else {
            //         genTrajectoryHover(current_point);
            //     }
            //     status_ = Hover;
            // }
            
            // return true;
        }

        if (status_ == Hover && (flag & 1 << 10)) {

        Eigen::Vector3f point;

        #ifdef SIMULATION
        Eigen::Vector2f point_2 = calculateCrossgatePoint(current_state_,current_gate_states_);
        point << point_2[0], point_2[1], takeoff_height_;
        genTrajectory(current_state_, point, std_velocity_);
        ROS_INFO_STREAM_COND(verbose_, prefix_ + "Crossgate");   
        #endif

        return  true; 
        
        }  

        return false;
    }




    void Autopilot::armDrone() {
        wrapper_->pubArmMsg(true);
    }

    void Autopilot::disarmDrone() {
        wrapper_->pubArmMsg(false);
    }

    void Autopilot::genTrajectory(const Eigen::Ref<const Eigen::VectorXf> &start_position, const Eigen::Ref<const Eigen::VectorXf> &end_position, float velocity) {

        float dx = end_position[0] - start_position[0];
        float dy = end_position[1] - start_position[1];
        float dz = end_position[2] - start_position[2];
        float distance = sqrtf(powf(dx, 2) + powf(dy, 2) + powf(dz, 2));

        float t1, t2, t3;
        if (distance > velocity * velocity / std_acceleration_) {
            t1 = t3 = velocity / std_acceleration_;
            t2 = (distance - velocity * velocity / std_acceleration_) / velocity;
        } else {
            t1 = t3 = sqrtf(distance / std_acceleration_);
            t2 = 0;
        }

        float vx = velocity * dx / distance;
        float vy = velocity * dy / distance;
        float vz = velocity * dz / distance;

        float ax = std_acceleration_ * dx / distance;
        float ay = std_acceleration_ * dy / distance;
        float az = std_acceleration_ * dz / distance;

        /// round up to the integral to guarantee the final state is in the trajectory
        float dt = controller_->getDt();
        int N = ceil((t1 + t2 + t3) / dt);
        traj_points_.resize(N);

        for (int i = 0; i < N - 1; i++) {
            quadrotor_msgs::TrajectoryPoint &traj_point = traj_points_[i];

            float t = i * dt;
            traj_point.time_from_start = ros::Duration(i * dt);
            if (t < t1) {
                traj_point.pose.position.x = start_position[0] + 1.0 / 2 * ax * t * t;
                traj_point.pose.position.y = start_position[1] + 1.0 / 2 * ay * t * t;
                traj_point.pose.position.z = start_position[2] + 1.0 / 2 * az * t * t;
                traj_point.velocity.linear.x = ax * t;
                traj_point.velocity.linear.y = ay * t;
                traj_point.velocity.linear.z = az * t;
                traj_point.pose.orientation.w = 1.0;
                traj_point.pose.orientation.x = 0.0;
                traj_point.pose.orientation.y = 0.0;
                traj_point.pose.orientation.z = 0.0;
            } else if (t < t2) {
                traj_point.pose.position.x = start_position[0] + 1.0 / 2 * ax * t1 * t1 + vx * (t - t1);
                traj_point.pose.position.y = start_position[1] + 1.0 / 2 * ay * t1 * t1 + vy * (t - t1);
                traj_point.pose.position.z = start_position[2] + 1.0 / 2 * az * t1 * t1 + vz * (t - t1);
                traj_point.velocity.linear.x = vx;
                traj_point.velocity.linear.y = vy;
                traj_point.velocity.linear.z = vz;
                traj_point.pose.orientation.w = 1.0;
                traj_point.pose.orientation.x = 0.0;
                traj_point.pose.orientation.y = 0.0;
                traj_point.pose.orientation.z = 0.0;
            } else {
                traj_point.pose.position.x = end_position[0] - 1.0 / 2 * ax * (t1 + t2 + t3 - t) * (t1 + t2 + t3 - t);
                traj_point.pose.position.y = end_position[1] - 1.0 / 2 * ay * (t1 + t2 + t3 - t) * (t1 + t2 + t3 - t);
                traj_point.pose.position.z = end_position[2] - 1.0 / 2 * az * (t1 + t2 + t3 - t) * (t1 + t2 + t3 - t);
                traj_point.velocity.linear.x = ax * (t1 + t2 + t3 - t);
                traj_point.velocity.linear.y = ay * (t1 + t2 + t3 - t);
                traj_point.velocity.linear.z = az * (t1 + t2 + t3 - t);
                traj_point.pose.orientation.w = 1.0;
                traj_point.pose.orientation.x = 0.0;
                traj_point.pose.orientation.y = 0.0;
                traj_point.pose.orientation.z = 0.0;
            }
        }

        // final state
        quadrotor_msgs::TrajectoryPoint &last_point = traj_points_.back();
        last_point.time_from_start = ros::Duration(t1 + t2 + t3);
        last_point.pose.position.x = end_position[0];
        last_point.pose.position.y = end_position[1];
        last_point.pose.position.z = end_position[2];
        last_point.velocity.linear.x = 0.0;
        last_point.velocity.linear.y = 0.0;
        last_point.velocity.linear.z = 0.0;
        last_point.pose.orientation.w = 1.0;
        last_point.pose.orientation.x = 0.0;
        last_point.pose.orientation.y = 0.0;
        last_point.pose.orientation.z = 0.0;

        start_time_ = ros::Time::now();
        finish_duration_ = ros::Duration(t1 + t2 + t3);
    }

    void Autopilot::genTrajectoryHover(const Eigen::Ref<Eigen::VectorXf> &hover_position) {

        ROS_INFO("%sHover: %6.2f%6.2f%6.2f", prefix_.c_str(), hover_position(0), hover_position(1), hover_position(2));

        traj_points_.resize(1);
        /// final state
        quadrotor_msgs::TrajectoryPoint &last_point = traj_points_.back();
        last_point.time_from_start = ros::Duration(0.0);
        last_point.pose.position.x = hover_position(0);
        last_point.pose.position.y = hover_position(1);
        last_point.pose.position.z = hover_position(2);
        last_point.velocity.linear.x = 0.0;
        last_point.velocity.linear.y = 0.0;
        last_point.velocity.linear.z = 0.0;
        last_point.pose.orientation.w = 1.0;
        last_point.pose.orientation.x = 0.0;
        last_point.pose.orientation.y = 0.0;
        last_point.pose.orientation.z = 0.0;

        start_time_ = ros::Time::now();
        finish_duration_ = ros::Duration(0.0);
    }

    //轨迹点插值
    quadrotor_msgs::TrajectoryPoint Autopilot::interpolate(const quadrotor_msgs::TrajectoryPoint &p0,
                                                           const quadrotor_msgs::TrajectoryPoint &p1, float ratio) const {

        if (ratio <= 0.0) {
            return p0;
        }
        if (ratio >= 1.0) {
            return p1;
        }

        quadrotor_msgs::TrajectoryPoint p_inter;

        /// timestamp
        p_inter.time_from_start = p0.time_from_start + ros::Duration(
                                                               ratio * (p1.time_from_start - p0.time_from_start).toSec());

        p_inter.pose.position.x = (1 - ratio) * p0.pose.position.x + ratio * p1.pose.position.x;
        p_inter.pose.position.y = (1 - ratio) * p0.pose.position.y + ratio * p1.pose.position.y;
        p_inter.pose.position.z = (1 - ratio) * p0.pose.position.z + ratio * p1.pose.position.z;

        p_inter.velocity.linear.x = (1 - ratio) * p0.velocity.linear.x + ratio * p1.velocity.linear.x;
        p_inter.velocity.linear.y = (1 - ratio) * p0.velocity.linear.y + ratio * p1.velocity.linear.y;
        p_inter.velocity.linear.z = (1 - ratio) * p0.velocity.linear.z + ratio * p1.velocity.linear.z;

        p_inter.pose.orientation.w = (1 - ratio) * p0.pose.orientation.w + ratio * p1.pose.orientation.w;
        p_inter.pose.orientation.x = (1 - ratio) * p0.pose.orientation.x + ratio * p1.pose.orientation.x;
        p_inter.pose.orientation.y = (1 - ratio) * p0.pose.orientation.y + ratio * p1.pose.orientation.y;
        p_inter.pose.orientation.z = (1 - ratio) * p0.pose.orientation.z + ratio * p1.pose.orientation.z;

        /// normalized
        float norm = sqrt(p_inter.pose.orientation.w * p_inter.pose.orientation.w + p_inter.pose.orientation.x * p_inter.pose.orientation.x + p_inter.pose.orientation.y * p_inter.pose.orientation.y + p_inter.pose.orientation.z * p_inter.pose.orientation.z);
        assert(norm > 1e-1);
        p_inter.pose.orientation.w /= norm;
        p_inter.pose.orientation.x /= norm;
        p_inter.pose.orientation.y /= norm;
        p_inter.pose.orientation.z /= norm;

        return p_inter;
    }

    quadrotor_msgs::TrajectoryPoint Autopilot::getSlowdownPoint(const ros::Duration &duration) const {
        float  t = duration.toSec();

        quadrotor_msgs::TrajectoryPoint point;
        point.time_from_start = duration;

        float angular_acc = (circle_velocity_ - slow_down_velocity_) / circle_radius_ / slow_down_time_;
        float angular_vel;
        float rad;
        float radius;
        float height;

        if (t < slow_down_time_) {
            angular_vel = circle_velocity_ / circle_radius_ - angular_acc * t;
            rad = circle_start_rad_ - 0.5 * angular_acc * t * t +  circle_velocity_ / circle_radius_ * t;
            radius = circle_radius_;
            height = circle_height_;
        } else {
            angular_vel = slow_down_velocity_ / circle_radius_;
            rad = circle_start_rad_ - 0.5 * angular_acc * slow_down_time_ * slow_down_time_ + circle_velocity_ / circle_radius_*slow_down_time_+angular_vel * (t - slow_down_time_);
            radius = circle_radius_;
            height = circle_height_;
        }

       
        point.pose.position.x = radius * cos(rad);
        point.pose.position.y = radius * sin(rad);
        point.pose.position.z = height;

        point.velocity.linear.x = radius * angular_vel * -sin(rad);
        point.velocity.linear.y = radius * angular_vel * cos(rad);
        point.velocity.linear.z = 0.0;

        point.pose.orientation.w = 1.0;
        point.pose.orientation.x = 0.0;
        point.pose.orientation.y = 0.0;
        point.pose.orientation.z = 0.0;

        // ROS_INFO("t: %f, angular_acc: %f, angular_vel: %f, rad: %f, radius: %f, height: %f, slow_down_time, %f", t, angular_acc, angular_vel, rad, radius, height,slow_down_time_);


        return point;
    }

quadrotor_msgs::TrajectoryPoint Autopilot::getCirclePoint(const ros::Duration &duration) const {

    float t = duration.toSec();
    quadrotor_msgs::TrajectoryPoint point;
    point.time_from_start = duration;

    float angular_acc = circle_velocity_ / circle_radius_ / circle_warmup_time_;
    float total_circle_time = 2 * M_PI * circle_radius_ / circle_velocity_; // 一圈的时间
    float total_time_two_circles = 2 * total_circle_time; // 两圈的时间
    
    // 计算减速时间
    float deceleration = (circle_velocity_ - slow_down_velocity_) / slow_down_time_;

    if (t < circle_warmup_time_) {
        float augular_vel = angular_acc * t;
        float rad = circle_start_rad_ + 1.0 / 2 * angular_acc * t * t;
        float radius = circle_start_radius_ + (circle_radius_ - circle_start_radius_) * t / circle_warmup_time_;
        float height = circle_start_height_ + (circle_height_ - circle_start_height_) * t / circle_warmup_time_;

        point.pose.position.x = radius * cos(rad);
        point.pose.position.y = radius * sin(rad);
        point.pose.position.z = height;
        point.velocity.linear.x = (circle_radius_ - circle_start_radius_) / circle_warmup_time_ * cos(rad) + radius * (-sin(rad) * augular_vel);
        point.velocity.linear.y = (circle_radius_ - circle_start_radius_) / circle_warmup_time_ * sin(rad) + radius * cos(rad) * augular_vel;
        point.velocity.linear.z = (circle_height_ - circle_start_height_) / circle_warmup_time_;

        point.acceleration.linear.x = -radius * angular_acc * sin(rad);
        point.acceleration.linear.y = radius * angular_acc * cos(rad);
        point.acceleration.linear.z = 0.0;

        Eigen::Vector3d now_a(point.acceleration.linear.x, point.acceleration.linear.y, point.acceleration.linear.z);

        Eigen::Quaterniond res = wrapper_->calculateQuaternion(now_a, 0);

        point.pose.orientation.w = res.w();
        point.pose.orientation.x = res.x();
        point.pose.orientation.y = res.y();
        point.pose.orientation.z = res.z();

        // point.pose.orientation.w = 1.0;
        // point.pose.orientation.x = 0.0;
        // point.pose.orientation.y = 0.0;
        // point.pose.orientation.z = 0.0;

    } else if (t < total_time_two_circles) {
        float augular_vel = circle_velocity_ / circle_radius_;
        float rad = circle_start_rad_ + 1.0 / 2 * angular_acc * circle_warmup_time_ * circle_warmup_time_ + augular_vel * (t - circle_warmup_time_);
        float radius = circle_radius_;
        float height = circle_height_;

        point.pose.position.x = radius * cos(rad);
        point.pose.position.y = radius * sin(rad);
        point.pose.position.z = height;
        point.velocity.linear.x = radius * (-sin(rad) * augular_vel);
        point.velocity.linear.y = radius * cos(rad) * augular_vel;
        point.velocity.linear.z = 0.0;

        point.acceleration.linear.x = -radius * augular_vel * augular_vel * cos(rad);
        point.acceleration.linear.y = -radius * augular_vel * augular_vel * sin(rad);
        point.acceleration.linear.z = 0.0;


        Eigen::Vector3d now_a(point.acceleration.linear.x, point.acceleration.linear.y, point.acceleration.linear.z);

        Eigen::Quaterniond res = wrapper_->calculateQuaternion(now_a, 0);

        point.pose.orientation.w = res.w();
        point.pose.orientation.x = res.x();
        point.pose.orientation.y = res.y();
        point.pose.orientation.z = res.z();

        // point.pose.orientation.w = 1.0;
        // point.pose.orientation.x = 0.0;
        // point.pose.orientation.y = 0.0;
        // point.pose.orientation.z = 0.0;

    } 
    else if (t < total_time_two_circles + slow_down_time_) {
        float elapsed_time = t - total_time_two_circles;
        float current_velocity = circle_velocity_ - deceleration * elapsed_time;
        float angular_vel = current_velocity / circle_radius_;
        
        float total_elapsed_time = total_time_two_circles + elapsed_time;
        float rad = circle_start_rad_ + 0.5 * angular_acc * circle_warmup_time_ * circle_warmup_time_ 
                    + (circle_velocity_ / circle_radius_) * (total_time_two_circles - circle_warmup_time_)
                    + circle_velocity_ * elapsed_time - 0.5 * deceleration * elapsed_time * elapsed_time;

        point.pose.position.x = circle_radius_ * cos(rad);
        point.pose.position.y = circle_radius_ * sin(rad);
        point.pose.position.z = circle_height_;
        point.velocity.linear.x = circle_radius_ * (-sin(rad) * angular_vel);
        point.velocity.linear.y = circle_radius_ * cos(rad) * angular_vel;
        point.velocity.linear.z = 0.0;

        point.acceleration.linear.x = -circle_radius_ * deceleration * sin(rad);
        point.acceleration.linear.y = circle_radius_ * deceleration * cos(rad);
        point.acceleration.linear.z = 0.0;


        Eigen::Vector3d now_a(point.acceleration.linear.x, point.acceleration.linear.y, point.acceleration.linear.z);

        Eigen::Quaterniond res = wrapper_->calculateQuaternion(now_a, 0);

        point.pose.orientation.w = res.w();
        point.pose.orientation.x = res.x();
        point.pose.orientation.y = res.y();
        point.pose.orientation.z = res.z();
        // point.pose.orientation.w = 1.0;
        // point.pose.orientation.x = 0.0;
        // point.pose.orientation.y = 0.0;
        // point.pose.orientation.z = 0.0;

    } else {
        // 保持在slow_down_velocity_，继续圆周飞行
        float elapsed_time = t - total_time_two_circles - slow_down_time_;
        float angular_vel = slow_down_velocity_ / circle_radius_;
        float rad = circle_start_rad_ + 0.5 * angular_acc * circle_warmup_time_ * circle_warmup_time_ 
                    + (circle_velocity_ / circle_radius_) * (total_time_two_circles - circle_warmup_time_)
                    + circle_velocity_ * slow_down_time_ - 0.5 * deceleration * slow_down_time_ *  slow_down_time_
                    + slow_down_velocity_ / circle_radius_ * elapsed_time;

        point.pose.position.x = circle_radius_ * cos(rad);
        point.pose.position.y = circle_radius_ * sin(rad);
        point.pose.position.z = circle_height_;
        point.velocity.linear.x = circle_radius_ * (-sin(rad) * angular_vel);
        point.velocity.linear.y = circle_radius_ * cos(rad) * angular_vel;
        point.velocity.linear.z = 0.0;

        point.acceleration.linear.x = 0.0;
        point.acceleration.linear.y = 0.0;
        point.acceleration.linear.z = 0.0;

        point.pose.orientation.w = 1.0;
        point.pose.orientation.x = 0.0;
        point.pose.orientation.y = 0.0;
        point.pose.orientation.z = 0.0;
    }

    return point;
}



    quadrotor_msgs::TrajectoryPoint Autopilot::getTrajectoryPoint(const ros::Duration &duration) const {

        if (duration < traj_points_.front().time_from_start) {
            return traj_points_.front();
        }

        if (duration > traj_points_.back().time_from_start) {
            return traj_points_.back();
        }

        /// Find points p0 and p1 such that
        /// p0.time_from_start <= time_from_start <= p1.time_from_start
        std::vector<quadrotor_msgs::TrajectoryPoint>::const_iterator p1;
        for (p1 = traj_points_.begin(); p1 != traj_points_.end(); p1++) {
            if (p1->time_from_start > duration) {
                break;
            }
        }
        auto p0 = std::prev(p1);
        const float ratio = (duration - p0->time_from_start).toSec() / (p1->time_from_start - p0->time_from_start).toSec();

        return interpolate(*p0, *p1, ratio);
    }

    void Autopilot::calRefStates(const ros::Duration &time_from_start, Eigen::Ref<Eigen::Matrix<float, STATE_DIM, Eigen::Dynamic>> ref_states) const {

        int N = ref_states.cols();

        float dt = controller_->getDt();

        for (int i = 0; i < N; i++) {

            ros::Duration t = time_from_start + ros::Duration(i * dt);

            quadrotor_msgs::TrajectoryPoint point;
            if (status_ == Circle) {
                point = getCirclePoint(t);
                // point = getTrajectoryPoint(t);
            } else if(status_ == Slowdown) {
                point = getSlowdownPoint(t);

            }
            else if(status_ == Trajectory){
            point = getTrajectoryPoint(t);
            }
            else{

                point = getTrajectoryPoint(t);
                
            }

            ref_states(0, i) = point.pose.position.x;
            ref_states(1, i) = point.pose.position.y;
            ref_states(2, i) = point.pose.position.z;
            ref_states(3, i) = point.pose.orientation.w;
            ref_states(4, i) = point.pose.orientation.x;
            ref_states(5, i) = point.pose.orientation.y;
            ref_states(6, i) = point.pose.orientation.z;
            ref_states(7, i) = point.velocity.linear.x;
            ref_states(8, i) = point.velocity.linear.y;
            ref_states(9, i) = point.velocity.linear.z;


            //     // 打印每个轨迹点的信息
            // ROS_INFO("Trajectory point : x = %f, y = %f, z = %f",
            //          ref_states(3, 0), ref_states(4, 0), ref_states(5, 0));
        }
    }

    Eigen::Matrix<float, CONTROL_DIM, 1> Autopilot::calSteadyCompensation() {

        if (!b_integral_compensation_) {
            return Eigen::Matrix<float, CONTROL_DIM, 1>::Zero();
        }

        /// if state is not hover, do not update integral error
        if (status_ != Hover) {
            Eigen::Matrix<float, CONTROL_DIM, 1> input_compensation;
            input_compensation << 0,
                    -error_integral_y_,
                    error_integral_x_,
                    0;
            return input_compensation;
        }

        quadrotor_msgs::TrajectoryPoint hover_point = traj_points_.front();

        float dx = hover_point.pose.position.x - current_state_.x();
        float dy = hover_point.pose.position.y - current_state_.y();

        error_integral_x_ += k_integral_compensation_ * dx;

        if (abs(error_integral_x_) > abs(max_error_integral_)) {
            error_integral_x_ = std::copysign(max_error_integral_, error_integral_x_);
        }

        error_integral_y_ += k_integral_compensation_ * dy;

        if (abs(error_integral_y_) > abs(max_error_integral_)) {
            error_integral_y_ = std::copysign(max_error_integral_, error_integral_y_);
        }

        Eigen::Matrix<float, CONTROL_DIM, 1> input_compensation;
        input_compensation << 0,
                -error_integral_y_,
                error_integral_x_,
                0;

        return input_compensation;
    }



    Eigen::Vector2f Autopilot::calculateNearestPoint(const Eigen::Matrix<float, STATE_DIM, 1> current_state, const std::vector<GateState>& gate_states)
    {
        Eigen::Vector3f point;
        float current_x = current_state[0];
        float current_y = current_state[1];
        
        float closest_distance = std::numeric_limits<float>::max();
        float closest_x = 0.0f;
        float closest_y = 0.0f; 

        for (const auto& gate_state : gate_states) {
        float gate_x = gate_state.pose.position.x;
        float gate_y = gate_state.pose.position.y;
        tf2::Quaternion q(gate_state.pose.orientation.x, gate_state.pose.orientation.y, gate_state.pose.orientation.z, gate_state.pose.orientation.w);
        double gate_roll,gate_pitch,gate_yaw; // 从门的姿态中获取方向（例如四元数转换为欧拉角）
        tf2::Matrix3x3(q).getRPY(gate_roll, gate_roll,gate_yaw);
        gate_yaw-=M_PI/2;

        // 计算正前方和正后方各点的坐标
        float offset = 0.5f; // 偏移量，单位为米
        float front_x = gate_x + offset * std::cos(gate_yaw);
        float front_y = gate_y + offset * std::sin(gate_yaw);
        float back_x = gate_x - offset * std::cos(gate_yaw);
        float back_y = gate_y - offset * std::sin(gate_yaw);

        // 计算当前状态到正前方和正后方各点的距离
        float dist_to_front = distance(current_x, current_y, front_x, front_y);
        float dist_to_back = distance(current_x, current_y, back_x, back_y);

        // 找到距离最近的点
        if (dist_to_front < closest_distance) {
            closest_distance = dist_to_front;
            closest_x = front_x;
            closest_y = front_y;
        }
        if (dist_to_back < closest_distance) {
            closest_distance = dist_to_back;
            closest_x = back_x;
            closest_y = back_y;
        }
    }
        return Eigen::Vector2f(closest_x, closest_y);//门的位置，第几个门
    }

    
//计算当前与门的距离，且该如何穿
    Eigen::Vector2f Autopilot::calculateCrossgatePoint(const Eigen::Matrix<float, STATE_DIM, 1> current_state, const std::vector<GateState>& gate_states)
    {       
                Eigen::Vector3f point;
        float current_x = current_state[0];
        float current_y = current_state[1];
        
        float closest_distance = std::numeric_limits<float>::max();
        float target_x = 0.0f;
        float target_y = 0.0f; 

        for (const auto& gate_state : gate_states) {
        float gate_x = gate_state.pose.position.x;
        float gate_y = gate_state.pose.position.y;


        float dist_temp = distance(current_x, current_y, gate_x, gate_y);

            if (dist_temp < closest_distance) {
            closest_distance = dist_temp;
            target_x = current_x+2*(gate_x- current_x);
            target_y = current_y+2*(gate_y- current_y);
        }

        }
        //后续注意这里目前的逻辑是穿最近的门但不一定是原来想穿的门


        return Eigen::Vector2f(target_x, target_y);

    }


    float Autopilot::distance(float x1, float y1, float x2, float y2) 
    {

        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    }






}// namespace cnuav