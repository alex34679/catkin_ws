
#include "cnuav_control/mpc/mpc.h"
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace cnuav {

    MPCController::MPCController(const std::string &filename) {

        loadParams(filename);
        //9.79
        uh_ << 9.83,
                0,
                0,
                0;

        init();
    }

    void MPCController::reset() {
        init();
    }

    bool MPCController::loadParams(const std::string &filename) {

        bool bSuccess = true;

        YAML::Node node = YAML::LoadFile(filename);

        assert(!node.IsNull());

        if (node["max_thrust"]) {
            params_.max_thrust = node["max_thrust"].as<float>();
            ROS_INFO("max_trust : %f", params_.max_thrust);
        } else {
            bSuccess = false;
        }

        if (node["min_thrust"]) {
            params_.min_thrust = node["min_thrust"].as<float>();
        } else {
            bSuccess = false;
        }

        if (node["max_bodyrate"]) {
            params_.max_bodyrate = node["max_bodyrate"].as<float>();
        } else {
            bSuccess = false;
        }

        if (node["Q"]) {
            std::vector<float> v = node["Q"].as<std::vector<float>>();
            assert(v.size() == STATE_DIM);
            params_.Q = Eigen::Matrix<float, STATE_DIM, 1>(v.data());
        } else {
            bSuccess = false;
        }

        if (node["R"]) {
            std::vector<float> v = node["R"].as<std::vector<float>>();
            assert(v.size() == CONTROL_DIM);
            params_.R = Eigen::Matrix<float, CONTROL_DIM, 1>(v.data());
        } else {
            bSuccess = false;
        }

        return bSuccess;
    }

    float MPCController::getDt() const {

        return dT;
    }

    size_t MPCController::getTimestep() const {

        return N;
    }

    void MPCController::setInitState(const Eigen::Ref<const Eigen::Matrix<float, STATE_DIM, 1>> &init_state) {
        x0_ = init_state;
    }

    void MPCController::setRefStates(const Eigen::Ref<const Eigen::Matrix<float, STATE_DIM, Eigen::Dynamic>> &ref_states) {

        y_.block(0, 0, kState, N) = ref_states;
        y_.block(kState, 0, kInput, N) = uh_.replicate(1, N);

        yN_ = ref_states.rightCols(1);

        acado_initializeNodesByForwardSimulation();
    }

    void MPCController::setInputBound(const Eigen::Ref<Eigen::Matrix<float, kInput, 1>> &lb, const Eigen::Ref<Eigen::Matrix<float, kInput, 1>> &ub) {
        lb_ = lb.replicate(1, N);
        ub_ = ub.replicate(1, N);
    }

    void MPCController::setWeightMatrix(const Eigen::Ref<Eigen::Matrix<float, kRef, 1>> &W, const Eigen::Ref<Eigen::Matrix<float, kRefN, 1>> &WN) {
        Eigen::Matrix<float, kRef, kRef> W_tmp = W.asDiagonal();
        W_ = W_tmp.replicate(1, N);

        WN_ = WN.asDiagonal();
    }

    void MPCController::preControlProcess() {
    }

    int MPCController::computeControl() {

        assert(bPrepared_);

        int code = acado_feedbackStep();

//        std::cout << x0_.transpose() << std::endl
//                  << std::endl;
//        std::cout << y_.transpose() << std::endl
//                  << std::endl;
//        std::cout << u_.transpose() << std::endl
//                  << std::endl;
//        std::cout << x_.transpose() << std::endl
//                  << std::endl;

        acado_shiftStates(2, nullptr, nullptr);
        acado_shiftControls(nullptr);

        bPrepared_ = false;

        if (code == 0) {
            return true;
        } else {
            printf("code: %d\n", code);
            return false;
        }
    }
    
    Eigen::Matrix<float, CONTROL_DIM, 1> MPCController::getInputs() const {
        const float alpha = 0.3;

        static Eigen::Matrix<float, CONTROL_DIM, 1> u_pre = Eigen::Matrix<float, CONTROL_DIM, 1>::Zero();

        Eigen::Matrix<float, CONTROL_DIM, 1> u;
        u = u_.col(0);

        Eigen::Matrix<float, CONTROL_DIM, 1> u_filtered = alpha * u + (1 - alpha) * u_pre;

        u_pre = u;

        return u_filtered;
    }


    // 函数返回状态向量的列表
    std::vector<Eigen::Vector3f> MPCController::getStatePositions() const{


        std::vector<Eigen::Vector3f> positions;

        // 提取每个状态向量的前三个元素 (x, y, z) 并存储到 positions 向量中
        for (int i = 0; i <= N; ++i) {
            Eigen::Vector3f position = x_.col(i).head<3>(); // 提取前三个状态 (x, y, z)
            positions.push_back(position);
        }

        return positions;
    }

    void MPCController::postControlProcess() {

        shiftInputs();

        prepare();
    }

    void MPCController::init() {

        /// clear acado memory
        memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
        memset(&acadoVariables, 0, sizeof(acadoVariables));
        /// init the solver
        acado_initializeSolver();

        /// init mpc variales

        u_ = uh_.replicate(1, N);
        y_ = Eigen::Matrix<float, kRef, N>::Zero();
        yN_ = Eigen::Matrix<float, kRefN, 1>::Zero();

        x0_ = Eigen::Matrix<float, kState, 1>::Zero();
        x0_(3, 0) = 1.0;

        x_ = x0_.replicate(1, N + 1);

        Eigen::Matrix<float, kInput, 1> lb;
        lb << params_.min_thrust,
                -params_.max_bodyrate,
                -params_.max_bodyrate,
                -params_.max_bodyrate;

        Eigen::Matrix<float, kInput, 1> ub;
        ub << params_.max_thrust,
                params_.max_bodyrate,
                params_.max_bodyrate,
                params_.max_bodyrate;

        setInputBound(lb, ub);

        Eigen::Matrix<float, kRef, 1> W;
        W.topRows(kState) = params_.Q;
        W.bottomRows(kInput) = params_.R;

        Eigen::Matrix<float, kState, 1> WN;
        WN = params_.Q;
        setWeightMatrix(W, WN);
        acado_initializeNodesByForwardSimulation();

        acado_preparationStep();

        bPrepared_ = true;
    }

    void MPCController::prepare() {
        if (!bPrepared_) {
            acado_preparationStep();
            bPrepared_ = true;
        }
    }

    void MPCController::shiftInputs() {
        acado_shiftStates(2, nullptr, nullptr);
        acado_shiftControls(nullptr);
    }


}// namespace cnuav