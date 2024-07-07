#ifndef GINS_GNSSTYPES_H
#define GINS_GNSSTYPES_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <utility>
#include "module/timesys.h"

namespace gnss {

class GNSS {
public:
    timesys::Time time;
    Eigen::Vector3d blh;    // Latitude, longitude, and height
    Eigen::Vector3d posstd; // Standard deviation, n-e-d
    Eigen::Vector3d vel;    // vn, ve, vd
    Eigen::Vector3d velstd; // Standard deviation, n-e-d
    bool valid;

    explicit GNSS(bool val = false)
        : time(timesys::Time())
        , blh(Eigen::Vector3d::Zero())
        , posstd(Eigen::Vector3d::Zero())
        , vel(Eigen::Vector3d::Zero())
        , velstd(Eigen::Vector3d::Zero())
        , valid(val) {
    }

    GNSS(timesys::Time t, Eigen::Vector3d blh, Eigen::Vector3d posstd, Eigen::Vector3d vel,
         Eigen::Vector3d velstd, bool val)
        : time(std::move(t))
        , blh(std::move(blh))
        , posstd(std::move(posstd))
        , vel(std::move(vel))
        , velstd(std::move(velstd))
        , valid(val) {
    }

    [[nodiscard]] std::string ToString() const;
};

}

#endif // GINS_GNSSTYPES_H
