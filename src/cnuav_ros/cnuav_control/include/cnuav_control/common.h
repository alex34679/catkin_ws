#ifndef SRC_COMMON_H
#define SRC_COMMON_H

namespace cnuav {

    /// [p_x p_y p_z q_w q_x q_y q_z v_x v_y v_z]^T
    constexpr int STATE_DIM = 10;
    /// [T w_x w_y w_z]^T
    constexpr int CONTROL_DIM = 4;

}

#include <fstream>
#define LOG(data) \
    do { \
        std::ofstream outfile; \
        outfile.open("/home/lty/ROS/rosbag/log.txt", std::ios::app); \
        outfile << data << std::endl; \
        outfile.close(); \
    } while(0)

#endif//SRC_COMMON_H
