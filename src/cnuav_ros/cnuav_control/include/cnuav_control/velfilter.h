//
// Created by lucas on 2022/3/24.
//

#ifndef SRC_VELFILTER_H
#define SRC_VELFILTER_H

#include <Eigen/Core>
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"


namespace cnuav {

    /**
     * velocity filter intermediate variables
     * Fs = 120 Hz Fc = 30Hz
     */
    struct InterVars {
        double interv1{0};
        std::vector<double> interv2{3, 0};
        double interv3{0};
        double interv4{0};
        std::vector<double> interv5{2, 0};
    };

    /**
     * IIR low pass filter
     */
    class VelFilter {

    public:
        VelFilter();

        /**
         * perform filter
         * @param odometry
         */
        Eigen::Vector3f calVel(geometry_msgs::PoseStamped &pose);

    private:
        /**
         * perform filter in each axis
         * @param i order, x:0, y:1, z:2
         * @param x input value
         * @return filtered value
         */
        double updateAxis(int i, double x);

        /**
         * update time
         */
        void shiftInterVars();

        // intermediate variables container
        std::vector<InterVars> intervars_;

        // first time flag
        bool binit_;
        // store last tick motion capture information
        geometry_msgs::PoseStamped last_pose_;
        Eigen::Vector3f last_v_;
    };
}

#endif//SRC_VELFILTER_H
