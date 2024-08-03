#ifndef SRC_CONTROLLER_H
#define SRC_CONTROLLER_H

#include "cnuav_control/common.h"
#include <Eigen/Core>
#include <string>

namespace cnuav {

    /**
     * Base controller class
     */
    class Controller {

    public:
        virtual void reset() = 0;

        /**
         * return time interval
         * @return
         */
        virtual float getDt() const = 0;

        /**
         * return timestep
         * @return
         */
        virtual size_t getTimestep() const = 0;

        /**
         * set up initial state
         * @param init_state
         */
        virtual void setInitState(const Eigen::Ref<const Eigen::Matrix<float, STATE_DIM, 1>> &init_state) = 0;

        /**
         * set up reference states
         * @param ref_states
         */
        virtual void setRefStates(const Eigen::Ref<const Eigen::Matrix<float, STATE_DIM, Eigen::Dynamic>> &ref_states) = 0;

        virtual void preControlProcess() = 0;

        /**
         * main function
         */
        virtual int computeControl() = 0;

        /**
         * return input
         * @return
         */
        virtual Eigen::Matrix<float, CONTROL_DIM, 1> getInputs() const = 0;

        /**
         * post contro process
         */
        virtual void postControlProcess() = 0;

    protected:
        Controller() = default;
    };
}// namespace cnuav

#endif//SRC_CONTROLLER_H