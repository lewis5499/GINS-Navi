#include "imutypes.h"

using namespace ins;
using namespace std;

std::string IMU::ToString() const {
    std::ostringstream oss;
    oss << "Time: " << time.ToString() << " (GPST: " << time.ToStringGPST() << "),\n"
        << "DT: " << dt << ",\n"
        << "DTheta: [" << dtheta.transpose() << "],\n"
        << "DVel: [" << dvel.transpose() << "],\n"
        << "Odo Vel: " << odovel;
    return oss.str();
}

IMU IMU::getCopy() const {
    return IMU(this->time,this->dt,this->dtheta,this->dvel,this->odovel,this->valid);
}

string Attitude::ToString() const {
    stringstream ss;
    ss << "qbn: " << qbn.coeffs().transpose() << "\n";
    ss << "qbe: " << qbe.coeffs().transpose() << "\n";
    ss << "cbn: \n" << cbn << "\n";
    ss << "cbe: \n" << cbe << "\n";
    ss << "euler: " << euler.transpose() << "\n";
    return ss.str();
}

string ImuError::ToString() const {
    ostringstream oss;
    oss << "Gyroscope Bias:\n" << gyrbias.transpose() << "\n";
    oss << "Accelerometer Bias:\n" << accbias.transpose() << "\n";
    oss << "Gyroscope Scale Factor:\n" << gyrscale.transpose() << "\n";
    oss << "Accelerometer Scale Factor:\n" << accscale.transpose() << "\n";
    return oss.str();
}

string ImuNoise::ToString() const {
    ostringstream oss;
    oss << "Gyroscope ARW: " << gyr_arw.transpose() << "\n";
    oss << "Accelerometer VRW: " << acc_vrw.transpose() << "\n";
    oss << "Gyroscope Bias Std: " << gyrbias_std.transpose() << "\n";
    oss << "Accelerometer Bias Std: " << accbias_std.transpose() << "\n";
    oss << "Gyroscope Scale Std: " << gyrscale_std.transpose() << "\n";
    oss << "Accelerometer Scale Std: " << accscale_std.transpose() << "\n";
    oss << "Correlation Time: " << corr_time << "\n";
    return oss.str();
}
