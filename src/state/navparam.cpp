#include "navparam.h"

using namespace nav;
using namespace std;

string MeasureNoise::ToString() const {
    ostringstream oss;
    oss << "Odometry and Heading Measurement Noise:\n" << odonhc_measnoise.transpose() << "\n";
    oss << "ZUPT Velocity Measurement Noise:\n" << zupt_vmeasnoise.transpose() << "\n";
    oss << "ZUPT Angular Velocity Measurement Noise:\n" << zupt_wmeasnoise << "\n";
    oss << "GNSS Carrier Phase Measurement Noise:\n" << carrierPhaseNoise << "\n";
    oss << "GNSS Code Measurement Noise:\n" << codeNoise << "\n";
    return oss.str();
}

string InstallParam::ToString() const {
    ostringstream oss;
    oss << "Antenna Lever: " << antlever.transpose() << "\n";
    oss << "ODO Lever: " << odolever.transpose() << "\n";
    oss << "Installation Angle: " << installangle.transpose() << "\n";
    return oss.str();
}