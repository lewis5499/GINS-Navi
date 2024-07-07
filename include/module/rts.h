#ifndef GINS_RTS_H
#define GINS_RTS_H

#include <Eigen/Dense>
#include <vector>

namespace RTS {

class rts {
public:
    static void add(const Eigen::MatrixXd &stateVec, const Eigen::MatrixXd &stateCov, const Eigen::MatrixXd &phi) {
        rts::stateVec_.push_back(stateVec);
        rts::stateCov_.push_back(stateCov);
        rts::phi_.push_back(phi);
    }

    static int smooth() {

        return 1;
    }

private:
    static std::vector<Eigen::MatrixXd> stateVec_;
    static std::vector<Eigen::MatrixXd> stateCov_;
    static std::vector<Eigen::MatrixXd> phi_;
};

} // namespace RTS

#endif // GINS_RTS_H
