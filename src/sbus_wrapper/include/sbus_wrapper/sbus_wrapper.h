#ifndef SRC_SBUS_WRAPPER_H
#define SRC_SBUS_WRAPPER_H

#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace sbus_wrapper {

    class SbusWrapper {
    public:
        SbusWrapper() = delete;

        SbusWrapper(ros::NodeHandle &nh, ros::NodeHandle &pnh, bool proctection = true);

        ~SbusWrapper() = default;

    private:
        /**
         * initialize parameter from ROS parameter server
         */
        void initParams();

        /**
         * init serial port
         */
        void initSerial();

        /**
         * send message via serial port
         */
        void sendData(const std::vector<uint8_t> &frame) const;

        /**
         * main loop
         * @param event
         */
        void timerCallback(const ros::TimerEvent &event);

        /**
         *  package rc channels to non-inverted sbus signal
         * @param channels channels signal
         */
        void cvtChannels2Frame(const std::vector<int> &channels, std::vector<uint8_t> &frame) const;

        /**
         * generate standard remote controller channels
         * @param msg control command
         * @param channels 16 channel signals, here we only need the first 5 channels which stand for A(Roll) E(Pitch) T(Throttle) R(Yaw) and ARM respectively
         */
        void genNormalizedChannels(const quadrotor_msgs::ControlCommand &msg, std::vector<int> &channels) const;

        /**
         * amplitude limiting funtion
         * @param lowerbound
         * @param val
         * @param upperboud
         * @return
         */
        template<class T>
        inline T limit(T lowerbound, T val, T upperboud) const;

        void cmdCallback(const quadrotor_msgs::ControlCommandConstPtr &msg);

        void armCallback(const std_msgs::BoolConstPtr &msg);

        void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

        // main loop frequency
        double freq_;

        // if drone is set to arm
        bool barm_;

        double max_roll_rate_;
        double max_pitch_rate_;
        double max_yaw_rate_;
        double max_thrust_;
        double drone_mass_;
        double thrust_k_;
        double thrust_b_;

        std::string serial_port_;
        int serial_fd_;

        std::vector<uint8_t> disarm_frame_;

        quadrotor_msgs::ControlCommand command_;
        bool bcmdupdate_;

        bool b_protection_;

        bool brecpose_;

        ros::Time stamp_;

        static const uint32_t CHANNEL_MAX;
        static const uint32_t CHANNEL_SIZE;
        static const uint8_t BYTE_SIZE;
        static const uint8_t SBUS_FRAME_SIZE;
        static const uint8_t SBUS_FRAME_BEGIN_BYTE;
        static const uint8_t SBUS_NORMAL_CHANS;
        static const int SBUS_CHAN_CENTER;
        static const uint8_t SBUS_CHAN_BITS;
        static const uint8_t SBUS_CH_BITS;

        ros::NodeHandle nh_;
        // private node handle
        ros::NodeHandle pnh_;

        ros::Timer timer_;

        ros::Subscriber cmd_sub_;
        ros::Subscriber arm_sub_;
        ros::Subscriber emgy_sub_;
        ros::Subscriber pose_sub_;
    };


}// namespace sbus_wrapper

#endif//SRC_SBUS_WRAPPER_H
