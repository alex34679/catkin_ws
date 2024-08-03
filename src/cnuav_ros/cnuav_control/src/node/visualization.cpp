#include "cnuav_control/visualization.h"


OdometryVisualizer::OdometryVisualizer(const ros::NodeHandle& nh, const std::string& quad_name, const bool mode)
    : nh_(nh), mode_(mode)
{
    std::string topic_name = "pose";
    std::string traj_topic_name = "traj";
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &OdometryVisualizer::goalCallback, this);
    if (mode_) {
        ROS_INFO("now mod is sim !!!");
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(topic_name, 10, 
                                [this](const nav_msgs::Odometry::ConstPtr& msg) {
                                    this->odometryCallback_sim(msg);
                                });
    } else {
        ROS_INFO("now mod is exp !!!");
        odom_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(topic_name, 10, 
                                [this](const geometry_msgs::PoseStampedConstPtr& msg) {
                                    this->odometryCallback_exp(msg);
                                });
    }

    traj_sub_ = nh_.subscribe("traj", 1, &OdometryVisualizer::trajCallback, this);

    // std::string marker_topic = "/" + quad_name + "/visualization_marker";
    std::string marker_topic = "marker";
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic, 10);

    marker_.header.frame_id = "world";
    marker_.ns = quad_name;
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::LINE_STRIP;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.scale.x = 0.05;  // 轨迹线宽度
    marker_.pose.orientation.x = 0;
    marker_.pose.orientation.y = 0;
    marker_.pose.orientation.z = 0;
    marker_.pose.orientation.w = 1;

    // 设置颜色
    setColor(marker_.color, 1, 4);


    traj_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("traj_marker", 1);

    traj_marker_.header.frame_id = "world";
    traj_marker_.ns = quad_name;
    traj_marker_.id = 1;
    traj_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    traj_marker_.action = visualization_msgs::Marker::ADD;
    traj_marker_.scale.x = 0.06;  // 轨迹线宽度
    traj_marker_.pose.orientation.x = 0;
    traj_marker_.pose.orientation.y = 0;
    traj_marker_.pose.orientation.z = 0;
    traj_marker_.pose.orientation.w = 1.0;


    setColor(traj_marker_.color, 2,4);
}


void OdometryVisualizer::odometryCallback_sim(const nav_msgs::Odometry::ConstPtr& msg)
{
    // ROS_INFO("CALLBACK");
    marker_.header.stamp = ros::Time::now();
    geometry_msgs::Point point;
    point.x = msg->pose.pose.position.x;
    point.y = msg->pose.pose.position.y;
    point.z = msg->pose.pose.position.z;
    marker_.points.push_back(point);
    marker_pub_.publish(marker_);
}

void OdometryVisualizer::odometryCallback_exp(const geometry_msgs::PoseStampedConstPtr& msg)
{
    marker_.header.stamp = ros::Time::now();
    geometry_msgs::Point point;
    point.x = msg->pose.position.x;
    point.y = msg->pose.position.y;
    point.z = msg->pose.position.z;
    marker_.points.push_back(point);
    marker_pub_.publish(marker_);
}

void OdometryVisualizer::setColor(std_msgs::ColorRGBA& color, int index, int total)
{
    float hue = static_cast<float>(index) / total;  // 分布均匀的色调
    float saturation = 1.0;
    float lightness = 0.5;
    
    // 将 HSL 转换为 RGB
    float r, g, b;
    hslToRgb(hue, saturation, lightness, r, g, b);

    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1.0;
}

float OdometryVisualizer::hue2rgb(float p, float q, float t) {
    if (t < 0.0) t += 1.0;
    if (t > 1.0) t -= 1.0;
    if (t < 1.0 / 6.0) return p + (q - p) * 6.0 * t;
    if (t < 1.0 / 2.0) return q;
    if (t < 2.0 / 3.0) return p + (q - p) * (2.0 / 3.0 - t) * 6.0;
    return p;
}

void OdometryVisualizer::hslToRgb(float h, float s, float l, float& r, float& g, float& b)
{
    if (s == 0.0)
    {
        r = g = b = l;  // 灰色
    }
    else
    {
        float q = l < 0.5 ? l * (1.0 + s) : l + s - l * s;
        float p = 2.0 * l - q;
        r = hue2rgb(p, q, h + 1.0 / 3.0);
        g = hue2rgb(p, q, h);
        b = hue2rgb(p, q, h - 1.0 / 3.0);
    }
}



void OdometryVisualizer::trajCallback(const quadrotor_msgs::Trajectory::ConstPtr& traj_msg) {
    visualization_msgs::Marker traj_marker;
    traj_marker.header.frame_id = "world";  // 确保坐标系名正确
    traj_marker.header.stamp = ros::Time::now();
    traj_marker.ns = "trajectory";
    traj_marker.id = 1;  // 确保唯一标识符与其他标记区分
    traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
    traj_marker.action = visualization_msgs::Marker::ADD;
    traj_marker.scale.x = 0.05;  // 轨迹线宽度
    traj_marker.pose.orientation.x = 0;
    traj_marker.pose.orientation.y = 0;
    traj_marker.pose.orientation.z = 0;
    traj_marker.pose.orientation.w = 1.0;

    traj_marker.header.stamp = ros::Time::now();

    setColor(traj_marker.color, 2, 4);

    // Add trajectory points to the marker
    for (const auto& traj_point : traj_msg->points) {
        geometry_msgs::Point p;
        p.x = traj_point.pose.position.x;
        p.y = traj_point.pose.position.y;
        p.z = traj_point.pose.position.z;
        traj_marker.points.push_back(p);
    }

    // Publish the marker for visualization in RViz
    traj_marker_pub_.publish(traj_marker);
}


void OdometryVisualizer::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    marker_.points.clear();
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_visualizer");
    ros::NodeHandle nh("~"); 

    std::string quad_name;
    nh.getParam("quad_name", quad_name);

    bool mode;
    nh.getParam("mode", mode);
    
    ros::NodeHandle nh_global;
    OdometryVisualizer visualizer(nh_global, quad_name, mode);

    // ros::spin();



    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);

    ros::Rate rate(10.0);
    while (nh.ok()) {
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}



