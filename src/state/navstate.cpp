#include "navstate.h"

using namespace nav;
using namespace std;

// Convert NavState to string
std::string NavState::ToString() const {
    std::ostringstream oss;
    oss << "Time: " << time << "\n";
    oss << "Position: " << pos.transpose() << "\n";
    oss << "Velocity: " << vel.transpose() << "\n";
    oss << "Attitude: " << att.ToString() << "\n";
    oss << "IMU Error: " << imuerror.ToString() << "\n";
    return oss.str();
}

// Get a copy of the current NavState
NavState NavState::getCopy() const {
    return NavState(this->time, this->pos, this->vel, this->att, this->imuerror);
}