#include <ros/ros.h>
#include <quadrotor_msgs/Trajectory.h>
#include <quadrotor_common/trajectory.h>
#include <quadrotor_msgs/TrajectoryPoint.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>


Eigen::Vector3f circle_center_;
float circle_radius_;
float circle_velocity_;
float circle_height_;
float circle_warmup_time_;
float circle_start_rad_ = 0;
float circle_start_radius_;
float circle_start_height_;

float slow_down_velocity_;
float slow_down_time_;
float lin_acc_ = 0.2;
double warm_up_speed_ = 0.2;

bool initial_pose_received = false;

geometry_msgs::PoseStamped initial_pose;

quadrotor_common::Trajectory trajectory;

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    // initial_pose = *msg;
    initial_pose_received = true;
    ROS_INFO("Initial pose received.");


}

Eigen::Quaterniond calculateQuaternion(const Eigen::Vector3d& cur_path, double fai) {  
    const double g = 9.8;    
    Eigen::Vector3d t = cur_path.head<3>(); // 使用Eigen的head方法获取前三个元素  
    t[2] += g; // 更新z分量  

    // 归一化t向量得到Z_b  
    Eigen::Vector3d Z_b = t.normalized();  
    // std::cout << "Z_b:\n" << Z_b << std::endl; 
    // 计算X_c  
    Eigen::Vector3d X_c(cos(fai), sin(fai), 0);  

    // 计算Y_b  
    Eigen::Vector3d Y_b = Z_b.cross(X_c).normalized();  
    // std::cout << "Y_b:\n" << Y_b << std::endl;
    // 计算X_b  
    Eigen::Vector3d X_b = Y_b.cross(Z_b);  
    
    // std::cout << "X_b:\n" << X_b << std::endl;
    // // 构造旋转矩阵  
    Eigen::Matrix3d rota_matrix;  
    rota_matrix << X_b, Y_b, Z_b;   

    Eigen::Quaterniond quaternion(rota_matrix);

    return quaternion;  
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




// quadrotor_common::TrajectoryPoint getCirclePoint(float time_second) {

//     float t = time_second;
//     quadrotor_common::TrajectoryPoint point;
//     point.time_from_start =  ros::Duration(time_second);

//     float angular_acc = circle_velocity_ / circle_radius_ / circle_warmup_time_;
//     float total_circle_time = 2 * M_PI * circle_radius_ / circle_velocity_; // 一圈的时间
//     float total_time_two_circles = 2 * total_circle_time; // 两圈的时间
    
//     // 计算减速时间
//     float deceleration = (circle_velocity_ - slow_down_velocity_) / slow_down_time_;

//     if (t < circle_warmup_time_) {
//         float augular_vel = angular_acc * t;
//         float rad = circle_start_rad_ + 1.0 / 2 * angular_acc * t * t;
//         float radius = circle_start_radius_ + (circle_radius_ - circle_start_radius_) * t / circle_warmup_time_;
//         float height = circle_start_height_ + (circle_height_ - circle_start_height_) * t / circle_warmup_time_;

//         point.position = Eigen::Vector3d(radius * cos(rad),
//                                          radius * sin(rad),
//                                          height);
        

//         point.velocity = Eigen::Vector3d((circle_radius_ - circle_start_radius_) / circle_warmup_time_ * cos(rad) + radius * (-sin(rad) * augular_vel),
//                                          (circle_radius_ - circle_start_radius_) / circle_warmup_time_ * sin(rad) + radius * cos(rad) * augular_vel,
//                                          (circle_height_ - circle_start_height_) / circle_warmup_time_);


//         point.acceleration = Eigen::Vector3d(-radius * angular_acc * sin(rad),
//                                     radius * angular_acc * cos(rad),
//                                     0.0);

//         Eigen::Quaterniond res = calculateQuaternion(point.acceleration, 0);

//         point.orientation = res;

//     } else if (t < 10*total_time_two_circles) {
//         float augular_vel = circle_velocity_ / circle_radius_;
//         float rad = circle_start_rad_ + 1.0 / 2 * angular_acc * circle_warmup_time_ * circle_warmup_time_ + augular_vel * (t - circle_warmup_time_);
//         float radius = circle_radius_;
//         float height = circle_height_;

//         point.position = Eigen::Vector3d(
//                                         radius * cos(rad),
//                                         radius * sin(rad),
//                                         height
//         );

//         point.velocity = Eigen::Vector3d(
//                                 radius * (-sin(rad) * augular_vel),
//                                 radius * cos(rad) * augular_vel,
//                                 0.0
//         );  

//         point.acceleration = Eigen::Vector3d(
//                                             -radius * augular_vel * augular_vel * cos(rad),
//                                             -radius * augular_vel * augular_vel * sin(rad),
//                                             0.0
//         );  

//         // Eigen::Vector3d now_a(point.acceleration.linear.x, point.acceleration.linear.y, point.acceleration.linear.z);

//         Eigen::Quaterniond res = calculateQuaternion(point.acceleration, 0);

//         point.orientation = res;

//     } 
//     else if (t < total_time_two_circles + slow_down_time_) {
//         float elapsed_time = t - total_time_two_circles;
//         float current_velocity = circle_velocity_ - deceleration * elapsed_time;
//         float angular_vel = current_velocity / circle_radius_;
        
//         float total_elapsed_time = total_time_two_circles + elapsed_time;
//         float rad = circle_start_rad_ + 0.5 * angular_acc * circle_warmup_time_ * circle_warmup_time_ 
//                     + (circle_velocity_ / circle_radius_) * (total_time_two_circles - circle_warmup_time_)
//                     + circle_velocity_ * elapsed_time - 0.5 * deceleration * elapsed_time * elapsed_time;

//         point.position = Eigen::Vector3d(
//                                     circle_radius_ * cos(rad),
//                                     circle_radius_ * sin(rad),
//                                     circle_height_
//         );  

//         point.velocity = Eigen::Vector3d(
//                                     circle_radius_ * (-sin(rad) * angular_vel),
//                                     circle_radius_ * cos(rad) * angular_vel,
//                                     0.0
//         );  


//         point.acceleration = Eigen::Vector3d(
//                                     -circle_radius_ * deceleration * sin(rad),
//                                     circle_radius_ * deceleration * cos(rad),
//                                     0.0
//         );  

//         Eigen::Quaterniond res = calculateQuaternion(point.acceleration, 0);

//         point.orientation = res;

//     } else {
//         // 保持在slow_down_velocity_，继续圆周飞行
//         float elapsed_time = t - total_time_two_circles - slow_down_time_;
//         float angular_vel = slow_down_velocity_ / circle_radius_;
//         float rad = circle_start_rad_ + 0.5 * angular_acc * circle_warmup_time_ * circle_warmup_time_ 
//                     + (circle_velocity_ / circle_radius_) * (total_time_two_circles - circle_warmup_time_)
//                     + circle_velocity_ * slow_down_time_ - 0.5 * deceleration * slow_down_time_ *  slow_down_time_
//                     + slow_down_velocity_ / circle_radius_ * elapsed_time;

//         point.position = Eigen::Vector3d(
//                                     circle_radius_ * cos(rad),
//                                     circle_radius_ * sin(rad),
//                                     circle_height_
//         );  

//         point.velocity = Eigen::Vector3d(
//                                     circle_radius_ * (-sin(rad) * angular_vel),
//                                     circle_radius_ * cos(rad) * angular_vel,
//                                     0.0
//         );  


//         point.acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);


//         point.orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
//     }

//     return point;
// }


quadrotor_common::TrajectoryPoint getCirclePoint(float time_second) {

    float t = time_second;
    quadrotor_common::TrajectoryPoint point;
    quadrotor_common::TrajectoryPoint last_point = trajectory.points.back();
    point.time_from_start =  ros::Duration(time_second);

    float angular_acc = circle_velocity_ / circle_radius_ / circle_warmup_time_;
    float total_circle_time = 2 * M_PI * circle_radius_ / circle_velocity_; // 一圈的时间
    float total_time_two_circles = 2 * total_circle_time; // 两圈的时间
    
    Eigen::Vector3d start_pos(initial_pose.pose.position.x, initial_pose.pose.position.y , initial_pose.pose.position.z);
    Eigen::Vector3d target_pos(circle_radius_,0,circle_height_);
    // 计算减速时间
    float deceleration = (circle_velocity_ - slow_down_velocity_) / slow_down_time_;

    float circle_warmup_time_ = (target_pos - start_pos).norm() / warm_up_speed_;
    float speed_up_time_ = (circle_velocity_ - warm_up_speed_)  / lin_acc_;
    if (t < circle_warmup_time_) {

        double total_distance = (target_pos - start_pos).norm();
        double progress = t / circle_warmup_time_;

        point.position = start_pos + progress * (target_pos - start_pos); 
        point.velocity = (target_pos - start_pos).normalized() * warm_up_speed_;
        point.acceleration = Eigen::Vector3d(0.0,0.0,0.0);

        Eigen::Quaterniond res = calculateQuaternion(point.acceleration, 0);

        point.orientation = res;

    } else if (t < circle_warmup_time_ + speed_up_time_) {

        Eigen::Vector3d last_position = last_point.position;
        Eigen::Vector3d last_velocity = last_point.velocity;

        // 计算当前线速度
        double delta_time = t - last_point.time_from_start.toSec();
        double current_speed = last_velocity.norm() + lin_acc_ * delta_time;


    // 根据当前线速度计算角速度
        double angular_velocity = current_speed / circle_radius_;

        // 计算新的角度（相对于上一个点）
        double rad = atan2(last_position.y(), last_position.x()) + angular_velocity * delta_time;

        // 计算新位置
        point.position = Eigen::Vector3d(
            circle_radius_ * cos(rad),
            circle_radius_ * sin(rad),
            circle_height_
        );

        // 计算新速度
        point.velocity = Eigen::Vector3d(
            -circle_radius_ * sin(rad) * angular_velocity,
            circle_radius_ * cos(rad) * angular_velocity,
            0.0
        );

        // 计算新加速度
        point.acceleration = Eigen::Vector3d(
            -circle_radius_ * (angular_velocity * angular_velocity * cos(rad)),
            -circle_radius_ * (angular_velocity * angular_velocity * sin(rad)),
            0.0
        );

        // Eigen::Vector3d now_a(point.acceleration.linear.x, point.acceleration.linear.y, point.acceleration.linear.z);

        Eigen::Quaterniond res = calculateQuaternion(point.acceleration, 0);

        point.orientation = res;

    } 
    else {
        // 恒定线速度，计算角速度
        double angular_velocity = circle_velocity_ / circle_radius_;

        // 计算当前角度，假设上一个角度已经计算好
        double last_angle = atan2(last_point.position.y(), last_point.position.x());
        double rad = last_angle + angular_velocity * (t - last_point.time_from_start.toSec());

        // 计算新位置
        point.position = Eigen::Vector3d(
            circle_radius_ * cos(rad),
            circle_radius_ * sin(rad),
            circle_height_
        );

        // 计算新速度
        point.velocity  = Eigen::Vector3d(
            -circle_radius_ * sin(rad) * angular_velocity,
            circle_radius_ * cos(rad) * angular_velocity,
            0.0
        );

        // 匀速圆周运动的加速度是向心加速度
        point.acceleration = Eigen::Vector3d(
            -circle_radius_ * angular_velocity * angular_velocity * cos(rad),
            -circle_radius_ * angular_velocity * angular_velocity * sin(rad),
            0.0
        );

        Eigen::Quaterniond res = calculateQuaternion(point.acceleration, 0);

        point.orientation = res;

    }

    return point;
}




int main(int argc, char** argv) {


    ros::init(argc, argv, "circle_trajectory_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Publisher trajectory_pub = nh.advertise<quadrotor_msgs::Trajectory>("traj", 1);
    ros::Subscriber initial_pose_sub = nh.subscribe("/initialpose", 1, initialPoseCallback);

    bool if_sim;
    pnh.getParam("mode", if_sim);
    ros::Subscriber odom_sub;
    if (if_sim) {
        odom_sub = nh.subscribe("pose", 10, odometryCallback_sim);
    } else {
        odom_sub = nh.subscribe("pose", 10, odometryCallback_exp);
    }

    if (!pnh.getParam("circle_velocity", circle_velocity_))  {  
        ROS_ERROR("Failed to get 'circle_velocity' from parameter server. Using default value 0.0.");  
        circle_velocity_ = 0.0; // 或者设置一个合理的默认值  
    }  else  {  
        ROS_INFO("circle_velocity: %f", circle_velocity_);  
    }  
  
    if (!pnh.getParam("circle_radius", circle_radius_))  {  
        ROS_ERROR("Failed to get 'circle_radius' from parameter server. Using default value 0.0.");  
        circle_radius_ = 0.0; // 或者设置一个合理的默认值  
    }  else  {  
        ROS_INFO("circle_radius: %f", circle_radius_);  
    }  
  
    if (!pnh.getParam("circle_height", circle_height_))  {  
        ROS_ERROR("Failed to get 'circle_height' from parameter server. Using default value 0.0.");  
        circle_height_ = 0.0; // 或者设置一个合理的默认值  
    }  else  {  
        ROS_INFO("circle_height: %f", circle_height_);  
    }  
  
    if (!pnh.getParam("circle_warmup_time", circle_warmup_time_))  {  
        ROS_ERROR("Failed to get 'circle_warmup_time' from parameter server. Using default value 0.0.");  
        circle_warmup_time_ = 0.0; // 或者设置一个合理的默认值  
    }  else  {  
        ROS_INFO("circle_warmup_time: %f", circle_warmup_time_);  
    }  
  
    if (!pnh.getParam("slow_down_time", slow_down_time_))  {  
        ROS_ERROR("Failed to get 'slow_down_time' from parameter server. Using default value 0.0.");  
        slow_down_time_ = 0.0; // 或者设置一个合理的默认值  
    }  else  {  
        ROS_INFO("slow_down_time: %f", slow_down_time_);  
    }  
  
    if (!pnh.getParam("slow_down_velocity", slow_down_velocity_))  {  
        ROS_ERROR("Failed to get 'slow_down_velocity' from parameter server. Using default value 0.0.");  
        slow_down_velocity_ = 0.0; // 或者设置一个合理的默认值  
    }  else  {  
        ROS_INFO("slow_down_velocity: %f", slow_down_velocity_);  
    }  


    if (!pnh.getParam("takeoff_height", circle_start_height_))  {  
        ROS_ERROR("Failed to get 'circle_start_height_' from parameter server. Using default value 0.0.");  
        circle_start_height_ = 0.0; // 或者设置一个合理的默认值  
    }  else  {  
        ROS_INFO("circle_start_height_: %f", circle_start_height_);  
    }  

    std::vector<double> circle_center_vector;
    if (pnh.getParam("circle_center", circle_center_vector)) {
        if (circle_center_vector.size() == 3) {
            circle_center_ = Eigen::Vector3f(circle_center_vector[0], circle_center_vector[1], circle_center_vector[2]);
        } else {
            ROS_ERROR("circle_center should have exactly 3 elements");
        }
    }

    



   ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        // 如果收到初始位置和至少一个路标点，生成并发布路径
        if (initial_pose_received) {
            ROS_INFO("Generating trajectory...");

            float total_circle_time = 2 * M_PI * circle_radius_ / circle_velocity_ + slow_down_time_;
            float slow_circle_time = 0 * M_PI * circle_radius_ / slow_down_velocity_;
            float dt = 0.01;
            for(float nt = 0; nt < 3*total_circle_time + slow_circle_time; nt+=dt){
                auto point = getCirclePoint(nt);
                trajectory.points.push_back(point);
            }
            ROS_INFO("trajectory size %d...", trajectory.points.size());

            quadrotor_msgs::Trajectory pub_msg = trajectory.toRosMessage();
            ROS_INFO("Publishing trajectory...");
            trajectory_pub.publish(pub_msg);


            // 重置标志位
            initial_pose_received = false;

            trajectory.points.clear();
        }

        rate.sleep();
    }
    return 0;
}