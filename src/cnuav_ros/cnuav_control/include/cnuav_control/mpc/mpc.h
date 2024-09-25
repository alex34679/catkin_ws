#ifndef SRC_CMPC_H
#define SRC_CMPC_H

#include "cnuav_control/controller.h"
#include "cnuav_control/mpc/mpc_params.h"
#include "cnuav_control/roswrapper.h"


namespace cnuav {

    // these head files must be put under your self-defined namespace to avoid " undefined acadoVariables " error

#include "acado_auxiliary_functions.h"
#include "acado_common.h"

    // WARNING !
    // These params are fixed when MPC codes are generated.
    // If you want to change any of them, re-generate codes as well.
    const float dT = 0.05;
    const size_t N = ACADO_N;
    const size_t kState = ACADO_NX;
    const size_t kInput = ACADO_NU;
    const size_t kRef = ACADO_NY;
    const size_t kRefN = ACADO_NYN;

    ACADOvariables acadoVariables;
    ACADOworkspace acadoWorkspace;

    /**
     * conventional model predictive controller with obstacle hard constraint
     */
    class MPCController : public Controller {
    public:
        MPCController() = delete;

        MPCController(const std::string &filename);

        ~MPCController() = default;

        void reset() override;

        bool loadParams(const std::string &filename);

        float getDt() const override;

        size_t getTimestep() const override;

        void setInitState(const Eigen::Ref<const Eigen::Matrix<float, STATE_DIM, 1>> &init_state) override;

        void setRefStates(const Eigen::Ref<const Eigen::Matrix<float, STATE_DIM, Eigen::Dynamic>> &ref_states) override;

        void setInputBound(const Eigen::Ref<Eigen::Matrix<float, kInput, 1>> &lb, const Eigen::Ref<Eigen::Matrix<float, kInput, 1>> &ub);

        void setWeightMatrix(const Eigen::Ref<Eigen::Matrix<float, kRef, 1>> &W, const Eigen::Ref<Eigen::Matrix<float, kRefN, 1>> &WN);

        void preControlProcess() override;
        /**
         * 调用MPC算法的主函数
         */
        int computeControl() override;

        Eigen::Matrix<float, CONTROL_DIM, 1> getInputs() const override;

        void postControlProcess() override;

        std::vector<Eigen::Vector3f> getStatePositions() const override;

    private:
        void init();

        /**
         * prepare for next round
         */
        void prepare();

        /**
         * 向前平移num_input_shift_个控制量，末尾控制量用最后一个控制量填充
         */
        void shiftInputs();

        MPCParams params_;

        bool bPrepared_;

        Eigen::Matrix<float, kInput, 1> uh_;

        // Map acado variables to Eigen Matrix
        //          [ x_0(0)         x_0(1)         ...  x_0(kSamples)    ]
        // state =  [ x_1(0)         x_1(1)         ...  x_1(kSamples)    ]
        //          [  ...           ...            ...  ...              ]
        //          [ x_NX-1(0)      x_NX-1(1)      ...  x_NX-1(kSamples) ]
        Eigen::Map<Eigen::Matrix<float, kState, N + 1, Eigen::ColMajor>> x_{acadoVariables.x};

        Eigen::Map<Eigen::Matrix<float, kInput, N, Eigen::ColMajor>> u_{acadoVariables.u};

        Eigen::Map<Eigen::Matrix<float, kRef, N, Eigen::ColMajor>> y_{acadoVariables.y};

        Eigen::Map<Eigen::Matrix<float, kRefN, 1, Eigen::ColMajor>> yN_{acadoVariables.yN};

        Eigen::Map<Eigen::Matrix<float, kRef, N * kRef, Eigen::ColMajor>> W_{acadoVariables.W};

        Eigen::Map<Eigen::Matrix<float, kRefN, kRefN, Eigen::ColMajor>> WN_{acadoVariables.WN};

        Eigen::Map<Eigen::Matrix<float, kState, 1, Eigen::ColMajor>> x0_{acadoVariables.x0};

        Eigen::Map<Eigen::Matrix<float, kInput, N, Eigen::ColMajor>> lb_{acadoVariables.lbValues};

        Eigen::Map<Eigen::Matrix<float, kInput, N, Eigen::ColMajor>> ub_{acadoVariables.ubValues};


    };
}// namespace cnuav

#endif//SRC_CMPC_H
