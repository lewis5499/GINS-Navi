#include "gnsstypes.h"

using namespace gnss;

std::string GNSS::ToString() const {
    std::ostringstream oss;
    oss << "Time: " << time.ToString() << " (GPST: " << time.ToStringGPST() << "),\n"
        << "BLH: [" << blh.transpose() << "],\n"
        << "Pos Std: [" << posstd.transpose() << "],\n"
        << "Vel: [" << vel.transpose() << "],\n"
        << "Vel Std: [" << velstd.transpose() << "],\n"
        << "Valid: " << std::boolalpha << valid;
    return oss.str();
}