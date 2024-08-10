#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <quadrotor_msgs/Trajectory.h>

class OdometryVisualizer {
public:
    OdometryVisualizer();
    OdometryVisualizer(const ros::NodeHandle& nh,const ros::NodeHandle& pnh);
    void odometryCallback_sim(const nav_msgs::Odometry::ConstPtr& msg);
    void odometryCallback_exp(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void trajCallback(const quadrotor_msgs::Trajectory::ConstPtr& traj_msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void trajectoryCallback(const quadrotor_msgs::Trajectory::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void showNowPos(const geometry_msgs::Point& p);
    void setColor(std_msgs::ColorRGBA& color, int index, int total);
    void hslToRgb(float h, float s, float l, float& r, float& g, float& b);
    float hue2rgb(float p, float q, float t) ;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    bool mode_;
    std::string mesh_resource;
    std::string quad_name;
    ros::Subscriber odom_sub_;
    ros::Subscriber traj_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher traj_marker_pub_;
    ros::Publisher meshPub;
    visualization_msgs::Marker marker_;
    visualization_msgs::Marker traj_marker_;
    visualization_msgs::Marker meshROS;

     double color_r, color_g, color_b, color_a;
};

#endif // VISUALIZATION_H



