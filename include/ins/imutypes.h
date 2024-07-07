#ifndef GINS_IMUTYPES_H
#define GINS_IMUTYPES_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <utility>
#include "module/timesys.h"

namespace ins {

class IMU {
public:
    timesys::Time time;
    double dt;
    Eigen::Vector3d dtheta;
    Eigen::Vector3d dvel;
    double odovel; // Odometer velocity
    bool valid;

    int tag; // 0: normal, 1: zaru/zupt, 2: zaru, 3: zupt

    explicit IMU(const double dt = 0.01, bool val = false)
        : time(timesys::Time())
        , dt(dt)
        , odovel(0.0)
        , dtheta(Eigen::Vector3d::Zero())
        , dvel(Eigen::Vector3d::Zero())
        , valid(val)
        , tag(0) {
    }

    IMU(timesys::Time t, double dt, Eigen::Vector3d dtheta, Eigen::Vector3d dvel, double odovel,
        bool val = true)
        : time(std::move(t))
        , dt(dt)
        , dtheta(std::move(dtheta))
        , dvel(std::move(dvel))
        , odovel(odovel)
        , valid(val)
        , tag(0) {
    }

    [[nodiscard]] std::string ToString() const;

    [[nodiscard]] IMU getCopy() const;
};

class Attitude {
public:
    Eigen::Quaterniond qbn, qbe;
    Eigen::Matrix3d cbn, cbe;
    Eigen::Vector3d euler;

    Attitude()
        : qbn(Eigen::Quaterniond::Identity())
        , qbe(Eigen::Quaterniond::Identity())
        , cbn(Eigen::Matrix3d::Identity())
        , cbe(Eigen::Matrix3d::Identity())
        , euler(Eigen::Vector3d::Zero()) {
    }

    Attitude(Eigen::Quaterniond quat_bn, Eigen::Quaterniond quat_be, Eigen::Matrix3d dcm_bn,
             Eigen::Matrix3d dcm_be, Eigen::Vector3d euler_angles)
        : qbn(std::move(quat_bn))
        , qbe(std::move(quat_be))
        , cbn(std::move(dcm_bn))
        , cbe(std::move(dcm_be))
        , euler(std::move(euler_angles)) {
    }

    [[nodiscard]] std::string ToString() const;
};

class ImuError {
public:
    Eigen::Vector3d gyrbias;  // Gyroscope bias
    Eigen::Vector3d accbias;  // Accelerometer bias
    Eigen::Vector3d gyrscale; // Gyroscope scale factor
    Eigen::Vector3d accscale; // Accelerometer scale factor

    ImuError()
        : gyrbias(Eigen::Vector3d::Zero())
        , accbias(Eigen::Vector3d::Zero())
        , gyrscale(Eigen::Vector3d::Zero())
        , accscale(Eigen::Vector3d::Zero()) {
    }

    ImuError(Eigen::Vector3d gyrbias, Eigen::Vector3d accbias, Eigen::Vector3d gyrscale,
             Eigen::Vector3d accscale)
        : gyrbias(std::move(gyrbias))
        , accbias(std::move(accbias))
        , gyrscale(std::move(gyrscale))
        , accscale(std::move(accscale)) {
    }

    [[nodiscard]] std::string ToString() const;
};

class ImuNoise {
public:
    Eigen::Vector3d gyr_arw;      // Gyroscope Angle Random Walk
    Eigen::Vector3d acc_vrw;      // Accelerometer Velocity Random Walk
    Eigen::Vector3d gyrbias_std;  // Gyroscope bias standard deviation
    Eigen::Vector3d accbias_std;  // Accelerometer bias standard deviation
    Eigen::Vector3d gyrscale_std; // Gyroscope scale factor standard deviation
    Eigen::Vector3d accscale_std; // Accelerometer scale factor standard deviation
    double corr_time;             // Correlation time

    ImuNoise()
        : corr_time(0.0)
        , gyr_arw(Eigen::Vector3d::Zero())
        , acc_vrw(Eigen::Vector3d::Zero())
        , gyrbias_std(Eigen::Vector3d::Zero())
        , accbias_std(Eigen::Vector3d::Zero())
        , gyrscale_std(Eigen::Vector3d::Zero())
        , accscale_std(Eigen::Vector3d::Zero()) {
    }

    ImuNoise(Eigen::Vector3d gyr_arw, Eigen::Vector3d acc_vrw, Eigen::Vector3d gyrbias_std,
             Eigen::Vector3d accbias_std, Eigen::Vector3d gyrscale_std,
             Eigen::Vector3d accscale_std, double corr_time)
        : gyr_arw(std::move(gyr_arw))
        , acc_vrw(std::move(acc_vrw))
        , gyrbias_std(std::move(gyrbias_std))
        , accbias_std(std::move(accbias_std))
        , gyrscale_std(std::move(gyrscale_std))
        , accscale_std(std::move(accscale_std))
        , corr_time(corr_time) {
    }

    [[nodiscard]] std::string ToString() const;
};

} // namespace ImuType

#endif // GINS_IMUTYPES_H
