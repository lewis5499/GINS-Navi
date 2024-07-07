#ifndef GINS_RTKPROCESS_H
#define GINS_RTKPROCESS_H

#include "module/rtklib.h"
#include <string>

namespace gnss {

class RTKEntry {
public:
    // WIN32: path should formatted in "\\"
    // Linux: path should formatted in "/"
    static int RunGnssRTK(const std::string &rtkConfigPath, const std::string &rtkResultPath,
                          const std::string &renixObsRoverPath, const std::string &renixObsBasePath,
                          const std::string &renixNavFilePath);

    static int RunGnssRTK(const char *rtkConfigPath, const char *rtkResultPath, const char *renixObsRoverPath,
                          const char *renixObsBasePath, const char *renixNavFilePath);
};
} // namespace rtk

#endif // GINS_RTKPROCESS_H
