#ifndef SRC_AUTOPILOT_H
#define SRC_AUTOPILOT_H

#include "cnuav_control/common.h"
#include "cnuav_control/controller.h"
#include "cnuav_control/roswrapper.h"
#include "cnuav_control/status.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>

namespace cnuav {


        
    class Autopilot {

    public:
        Autopilot() = delete;

        Autopilot(ros::NodeHandle &nh, ros::NodeHandle &pnh, const std::string &filename);

        ~Autopilot() = default;

        bool loadParams(const std::string &filename);

        void setController(Controller *controller);

        void mainLoop(const ros::TimerEvent &event);

        

    private:
        /**
         * @brief check whether trajectory following process has finished
         * @return
         */
        bool taskFinished() const;

        /**
         * check whether the status can be switched to new status
         * @param status new status
         * @return
         */
        bool switchStatus(u_char flag);

        void armDrone();

        void disarmDrone();

        /**
         * generate new uniform acceleration trajectory
         * @param start_position initial position
         * @param end_position end position
         * @param velocity velocity
         */
        void genTrajectory(const Eigen::Ref<const Eigen::VectorXf> &start_position, const Eigen::Ref<const Eigen::VectorXf> &end_position, float velocity);

        /**
         * single hover position trajectory
         * @param hover_position hover position
         */
        void genTrajectoryHover(const Eigen::Ref<Eigen::VectorXf> &hover_position);

        /**
         * WARNING: implement function which interpolates position and velocity only !!!
         * @param p0 previous TrajectoryPoint
         * @param p1 next TrajectoryPoint
         * @param ratio ratio
         * @return target TrajectoryPoint
         */
        quadrotor_msgs::TrajectoryPoint interpolate(const quadrotor_msgs::TrajectoryPoint &p0, const quadrotor_msgs::TrajectoryPoint &p1, float ratio) const;


        quadrotor_msgs::TrajectoryPoint getCirclePoint(const ros::Duration &duration) const;
        quadrotor_msgs::TrajectoryPoint getSlowdownPoint(const ros::Duration &duration) const;


        /**
         * @brief interpolate to calculate reference state of time duration
         * @param duration
         * @return
         */
        quadrotor_msgs::TrajectoryPoint getTrajectoryPoint(const ros::Duration &duration) const;

        /**
         * @brief calculate [num_timesteps] reference states of cbf_mppi controller
         * @param time_from_start
         * @param ref_states return reference states
         */
        void calRefStates(const ros::Duration &time_from_start, Eigen::Ref<Eigen::Matrix<float, STATE_DIM, Eigen::Dynamic>> ref_states) const;

        /**
         * convert acc of xyz to thrust and rates
         * @param acc acc of xyz
         * @return thrust and rates
         */
        Eigen::Vector4f calThrustRates(const Eigen::Ref<const Eigen::VectorXf> &acc) const;

        /**
         * compensate steady state error with integral controller
         * @return compensation inputs
         */
        Eigen::Vector4f calSteadyCompensation();

        Eigen::Vector2f calculateNearestPoint(const Eigen::Matrix<float, STATE_DIM, 1> current_state, const std::vector<GateState>& gate_states);
        Eigen::Vector2f calculateCrossgatePoint(const Eigen::Matrix<float, STATE_DIM, 1> current_state, const std::vector<GateState>& gate_states);
        float distance(float x1, float y1, float x2, float y2);

        ROSWrapper *wrapper_;

        Controller *controller_;

        Status status_;

        /// [ p_x p_y p_z v_x v_y v_z q_w q_x q_y q_z ]
        Eigen::Matrix<float, STATE_DIM, 1> current_state_;
        
        #ifdef SIMULATION
        std::vector<GateState> current_gate_states_;
        #endif


        Eigen::Matrix<float, CONTROL_DIM, 1> inputs_;

        /// record start time of executing trajectory or circle
        ros::Time start_time_;
        /// total time
        ros::Duration finish_duration_;
        /// trajectory points
        std::vector<quadrotor_msgs::TrajectoryPoint> traj_points_;

        Eigen::Vector3f circle_center_;
        float circle_radius_;
        float circle_velocity_;
        float circle_height_;
        float circle_warmup_time_;
        float circle_start_rad_;
        float circle_start_radius_;
        float circle_start_height_;

        float slow_down_velocity_;
        float slow_down_time_;

        float frequency_;

        bool verbose_;

        bool enable_log_;
        std::string log_filename_;
        std::ofstream fout_;

        /// if enable steady-state error compensation when hover
        bool b_integral_compensation_;
        /// integral saturation constraints
        float max_error_integral_;
        /// param of integral compensation
        float k_integral_compensation_;
        /// integral value
        float error_integral_x_;
        float error_integral_y_;

        float takeoff_height_;

        float takeoff_velocity_;

        float land_velocity_;

        float std_velocity_;
        float std_acceleration_;

        //用于区别无人机行为
        int quad_id_;

        float local_rad;
        float target_rad;

        float raw_radius;
        float raw_vel;

        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        std::string prefix_;

        ros::Timer mainloopTimer_;
    };

}// namespace cnuav

#endif//SRC_AUTOPILOT_H
