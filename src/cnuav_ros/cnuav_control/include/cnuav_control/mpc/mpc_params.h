#ifndef SRC_CMPC_PARAMS_H
#define SRC_CMPC_PARAMS_H

#include "cnuav_control/common.h"
#include <Eigen/Core>

namespace cnuav {

    struct MPCParams {

        float max_thrust;
        float min_thrust;

        float max_bodyrate;

        Eigen::Matrix<float, STATE_DIM, 1> Q;

        Eigen::Matrix<float, CONTROL_DIM, 1> R;

    };
}// namespace cnuav

#endif//SRC_CMPC_PARAMS_H
