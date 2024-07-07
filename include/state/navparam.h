#ifndef GINS_NAVPARAM_H
#define GINS_NAVPARAM_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace nav {

class MeasureNoise {
public:
    Eigen::Vector3d odonhc_measnoise;
    Eigen::Vector3d zupt_vmeasnoise;
    double zupt_wmeasnoise;
    double carrierPhaseNoise; // GNSS carrier phase measurement noise
    double codeNoise;         // GNSS code measurement noise

    MeasureNoise()
        : odonhc_measnoise(Eigen::Vector3d::Zero())
        , zupt_vmeasnoise(Eigen::Vector3d::Zero())
        , zupt_wmeasnoise(0.0)
        , carrierPhaseNoise(0.0)
        , codeNoise(0.0) {
    }

    MeasureNoise(const Eigen::Vector3d &odonhc_measnoise, const Eigen::Vector3d &zupt_vmeasnoise,
                 double zupt_wmeasnoise, double carrierPhaseNoise, double codeNoise)
        : odonhc_measnoise(odonhc_measnoise)
        , zupt_vmeasnoise(zupt_vmeasnoise)
        , zupt_wmeasnoise(zupt_wmeasnoise)
        , carrierPhaseNoise(carrierPhaseNoise)
        , codeNoise(codeNoise) {
    }

    std::string ToString() const;
};

class InstallParam {
public:
    Eigen::Vector3d antlever;
    Eigen::Vector3d odolever;
    Eigen::Vector3d installangle; // cfg.Cbv = Euler2DCM(cfg.installangle);

    InstallParam()
        : antlever(Eigen::Vector3d::Zero())
        , odolever(Eigen::Vector3d::Zero())
        , installangle(Eigen::Vector3d::Zero()) {
    }

    InstallParam(const Eigen::Vector3d &antlever, const Eigen::Vector3d &odolever, const Eigen::Vector3d &installangle)
        : antlever(antlever)
        , odolever(odolever)
        , installangle(installangle) {
    }

    std::string ToString() const;
};
} // namespace navparam

#endif // GINS_NAVPARAM_H
