#include "cnuav_control/roswrapper.h"

#include <tf2/LinearMath/Quaternion.h>

namespace cnuav {

    ROSWrapper::ROSWrapper(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : nh_(nh), pnh_(pnh),
          flag_(0), point_(new float[3]) {
        initROSInterface();
    }

    u_char ROSWrapper::getStatus() {

        u_char flag = flag_;

        // clear flags
        flag_ &= 0;

        return flag;
    }


    std::vector<quadrotor_msgs::TrajectoryPoint> ROSWrapper::getTrajectory() const {
        return traj_points_;
    }

    Eigen::Vector3f ROSWrapper::getTargetPosition() const {
        return point_;
    }


    std::vector<GateState> ROSWrapper::getgateState() {

        
#ifdef SIMULATION
         return gate_states;
       
#elif EXPERIMENT

#endif
//这里实际实验没有返回值后续注意修改
    }


#ifdef SIMULATION
    Eigen::Vector2f ROSWrapper::setModelState(const std::string& model_name) {

        // 准备设置模型状态的服务消息
        gazebo_msgs::SetModelState set_model_state;
        set_model_state.request.model_state.model_name = model_name;

        geometry_msgs::Pose pose;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis(-4.0, 4.0);

        pose.position.x = dis(gen);
        pose.position.y = dis(gen);
        set_model_state.request.model_state.pose = pose;


        geometry_msgs::Twist twist;
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;
        set_model_state.request.model_state.twist = twist;
        
        if (gate_client.call(set_model_state)) {
            ROS_INFO("Model state set successfully");
        } else {
            ROS_ERROR("Failed to set model state");
        }
        return Eigen::Vector2f(pose.position.x, pose.position.y);

    }
#endif

//获取当前状态
    Eigen::Matrix<float, STATE_DIM, 1> ROSWrapper::getInitState() {

        Eigen::Matrix<float, STATE_DIM, 1> current_state;
#ifdef SIMULATION
        current_state << odometry_.pose.pose.position.x,
                odometry_.pose.pose.position.y,
                odometry_.pose.pose.position.z,
                odometry_.pose.pose.orientation.w,
                odometry_.pose.pose.orientation.x,
                odometry_.pose.pose.orientation.y,
                odometry_.pose.pose.orientation.z,
                odometry_.twist.twist.linear.x,
                odometry_.twist.twist.linear.y,
                odometry_.twist.twist.linear.z;

        
#elif EXPERIMENT

        Eigen::Quaternionf q(pose_.pose.orientation.w, pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z);
        q.normalized();
        if (q.w() < 0) {
            q.w() *= -1;
            q.x() *= -1;
            q.y() *= -1;
            q.z() *= -1;
        }

        float yaw_comp = 0.0 * M_PI / 180;
        float pitch_comp = 0.0 * M_PI / 180;
        float roll_comp = 0.0 * M_PI / 180;

        Eigen::Vector3f eulerAngle(yaw_comp, pitch_comp, roll_comp);
        Eigen::Quaternionf q_comp = Eigen::AngleAxisf(eulerAngle(2), Eigen::Vector3f::UnitX()) *
                                    Eigen::AngleAxisf(eulerAngle(1), Eigen::Vector3f::UnitY()) *
                                    Eigen::AngleAxisf(eulerAngle(0), Eigen::Vector3f::UnitZ());

        q = q_comp * q;


        Eigen::Vector3f v = vel_filter_.calVel(pose_);
        current_state << pose_.pose.position.x,
                pose_.pose.position.y,
                pose_.pose.position.z,
                q.w(),
                q.x(),
                q.y(),
                q.z(),
                v[0],
                v[1],
                v[2];

#endif
        return current_state;
    }

    void ROSWrapper::pubCmdThrustRates(const Eigen::Ref<const Eigen::Matrix<float, CONTROL_DIM, 1>> &cmd_thrust_rates) {

        quadrotor_msgs::ControlCommand cmd;
        cmd.header.frame_id = "world";
        static int cnt = 0;
        cmd.header.seq = cnt++;
        cmd.header.stamp = ros::Time::now();
        cmd.control_mode = 2;// body rates
        cmd.armed = true;
        cmd.collective_thrust = cmd_thrust_rates(0);

#ifdef SIMULATION
        cmd.bodyrates.x = cmd_thrust_rates(1);
        cmd.bodyrates.y = cmd_thrust_rates(2);
        cmd.bodyrates.z = cmd_thrust_rates(3);
#elif EXPERIMENT
        cmd.bodyrates.x = cmd_thrust_rates(1);
        cmd.bodyrates.y = cmd_thrust_rates(2);
        cmd.bodyrates.z = cmd_thrust_rates(3);
#endif

        cmdPub_.publish(cmd);
    }

    void ROSWrapper::pubArmMsg(bool barm) {

        std_msgs::Bool msg;
        msg.data = barm;

        for (int i = 0; i < 5; i++) {
            armPub_.publish(msg);
            ros::Duration(0.1).sleep();
        }
    }

    void ROSWrapper::initROSInterface() {

        cmdPub_ = nh_.advertise<quadrotor_msgs::ControlCommand>("command", 10);

        armPub_ = nh_.advertise<std_msgs::Bool>("arm", 10);

        traj_sub_ = nh_.subscribe("trajectory", 10, &ROSWrapper::trajCallback, this);

        //path_sub_ = nh_.subscribe("path", 10, &ROSWrapper::pathCallback, this);

        point_sub_ = nh_.subscribe("point", 10, &ROSWrapper::pointCallback, this);
        
        // joy_sub_ = nh_.subscribe("/joy", 10, &ROSWrapper::joyCallback, this);


        joy_sub_ = nh_.subscribe("/joy", 10, &ROSWrapper::joyCallback, this);

        pose_sub_ = nh_.subscribe("pose", 100, &ROSWrapper::poseCallback, this);
        
        #ifdef SIMULATION

        gate_sub_ = nh_.subscribe("/gazebo/model_states", 100, &ROSWrapper::gateStatesCallback, this);
        

        gate_client = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        
        #endif
    }

    void ROSWrapper::trajCallback(const quadrotor_msgs::Trajectory::ConstPtr &msg) {

        ROS_INFO("Receive Trajectory Topic");

        auto first_point = msg->points.front();

#ifdef SIMULATION
        float distance = sqrt(pow(odometry_.pose.pose.position.x - first_point.pose.position.x, 2) +
                              pow(odometry_.pose.pose.position.y - first_point.pose.position.y, 2) +
                              pow(odometry_.pose.pose.position.z - first_point.pose.position.z, 2));

#elif EXPERIMENT
        float distance = sqrt(pow(pose_.pose.position.x - first_point.pose.position.x, 2) +
                              pow(pose_.pose.position.y - first_point.pose.position.y, 2) +
                              pow(pose_.pose.position.z - first_point.pose.position.z, 2));
#endif

        if (distance > 5e-1) {
            ROS_WARN("Trajectory Start Position is Far away from Current Position");
            return;
        }

        traj_points_ = msg->points;

        flag_ |= 1 << 5;
        // flag_ |= 1 << 4;
    }


    /*void ROSWrapper::pathCallback(const nav_msgs::PathConstPtr &msg) {
        ROS_INFO_ONCE("Receive Path Topic");
        // TODO: implementation
    }*/

    void ROSWrapper::pointCallback(const geometry_msgs::PointConstPtr &msg) {

        ROS_INFO("Receive Point Topic");
        point_ << msg->x,
                msg->y,
                msg->z;
        flag_ |= 1 << 6;
    }


    void ROSWrapper::joyCallback(const sensor_msgs::JoyConstPtr &msg) {

        ROS_INFO_ONCE("Receive Joy Topic");

        //         Y(3)[hover]
        // X(2)[disarm]   B(1)[land]
        //         A(0)[takeoff]
        if (msg->buttons[0]) {
            flag_ |= 1;
        }
        if (msg->buttons[1]) {
            flag_ |= 1 << 1;
        }
        if (msg->buttons[2]) {
            flag_ |= 1 << 2;
        }
        if (msg->buttons[3]) {
            flag_ |= 1 << 3;
        }
        if (msg->buttons[5]) {
            flag_ |= 1 << 4;
        }


        if (msg->buttons[6]) {
            flag_ |= 1 << 7;
        }
        //占用了point模式的标志位
        if (msg->buttons[7]) {
            flag_ |= 1 << 6;
        }

        if (msg->buttons[10]) {
            flag_ |= 1 << 10;
        }
        
    }




#ifdef SIMULATION    /*void ROSWrapper::pathCallback(const nav_msgs::PathConstPtr &msg) {
        ROS_INFO_ONCE("Receive Path Topic");
        // TODO: implementation
    }*/

    void ROSWrapper::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {

        ROS_INFO_ONCE("Receive Gazebo Pose Topic");

        odometry_ = *msg;
    }

    void ROSWrapper::gateStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        gate_states.clear();
        // 在回调函数中处理模型状态信息
        for (int i = 0; i < msg->name.size(); ++i) {
            if (startsWith(msg->name[i], "gate")) { // 检查模型名称是否以"gate"开头
                GateState gate_state;
                gate_state.name = msg->name[i];
                gate_state.pose = msg->pose[i]; 
                gate_states.push_back(gate_state);

                // 打印模型的位置和姿态信息
                // ROS_INFO_ONCE("Receive gate %d States Topic", i);
                // ROS_INFO("Model %s: Position(x=%f, y=%f, z=%f), Orientation(x=%f, y=%f, z=%f, w=%f)",
                // msg->name[i].c_str(),
                // msg->pose[i].position.x,
                // msg->pose[i].position.y,
                // msg->pose[i].position.z,
                // msg->pose[i].orientation.x,
                // msg->pose[i].orientation.y,
                // msg->pose[i].orientation.z,
                // msg->pose[i].orientation.w);
            }
        }
    }





#elif EXPERIMENT
    void ROSWrapper::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg) {

        ROS_INFO_ONCE("Receive Mocap Pose Topic");

        pose_ = *msg;
    }
    


#endif

    bool ROSWrapper::startsWith(const std::string& str, const std::string& prefix) {
    return str.compare(0, prefix.size(), prefix) == 0;
    }
    

}// namespace cnuav