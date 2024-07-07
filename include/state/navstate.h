#ifndef GINS_NAVSTATE_H
#define GINS_NAVSTATE_H

#include <iostream>
#include <sstream>
#include <string>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "ins/imutypes.h"

namespace nav {

class NavState {
public:
    double time;
    Eigen::Vector3d pos;   // when in n-Frame: B, L ,H [rad, rad, m]; when in e-Frame: X, Y, Z [m, m, m]
    Eigen::Vector3d vel;   // when in n-Frame: vn, ve, vd [m/s, m/s, m/s]; when in e-Frame: vx, vy, vz [m/s, m/s, m/s]
    ins::Attitude att;     // eg. Euler angles: Roll, Pitch, Yaw, calculated from n-Frame
    ins::ImuError imuerror;

    // Default constructor
    NavState()
        : time(0.0)
        , pos(Eigen::Vector3d::Zero())
        , vel(Eigen::Vector3d::Zero())
        , att()
        , imuerror() {
    }

    // Parameterized constructor
    NavState(double time, const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const ins::Attitude &att, const ins::ImuError &imuerror)
        : time(time)
        , pos(pos)
        , vel(vel)
        , att(att)
        , imuerror(imuerror) {
    }

    // Convert NavState to string
    [[nodiscard]] std::string ToString() const;

    // Get a copy of the current NavState
    [[nodiscard]] NavState getCopy() const;
};

} // namespace nav

#endif // GINS_NAVSTATE_H
