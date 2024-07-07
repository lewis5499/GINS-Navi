#include "rtkprocess.h"
#include <iostream>

using namespace gnss;

// WIN32: path should formatted in "\\"
// Linux: path should formatted in "/"
int RTKEntry::RunGnssRTK(const std::string &rtkConfigPath, const std::string &rtkResultPath,
                      const std::string &renixObsRoverPath, const std::string &renixObsBasePath,
                      const std::string &renixNavFilePath) {
    if(renixNavFilePath.empty()||renixObsBasePath.empty()||renixObsRoverPath.empty()) {
        std::cout << "[INFO]:  GNSS-RTK exit: no renix obs/nav data." << std::endl;
        std::flush(std::cout);
        return -1;
    }

    int argc     = 9;
    char *argv[] = {
        (char *) "rtklib-Module",
        (char *) "rnx2rtkp",
        (char *) "-k",
        (char *) (rtkConfigPath.c_str()),
        (char *) "-o",
        (char *) (rtkResultPath.c_str()),
        (char *) (renixObsRoverPath.c_str()),
        (char *) (renixObsBasePath.c_str()),
        (char *) (renixNavFilePath.c_str()),
    };

    return rtkentry(argc, argv);
}

int RTKEntry::RunGnssRTK(const char *rtkConfigPath, const char *rtkResultPath, const char *renixObsRoverPath,
                      const char *renixObsBasePath, const char *renixNavFilePath) {
    if(std::string(renixNavFilePath).empty()||std::string(renixObsBasePath).empty()||std::string(renixObsRoverPath).empty()) {
        std::cout << "[INFO]:  GNSS-RTK exit: no renix obs/nav data." << std::endl;
        std::flush(std::cout);
        return -1;
    }

    int argc     = 9;
    char *argv[] = {
        (char *) "rtklib-Module",
        (char *) "rnx2rtkp",
        (char *) "-k",
        (char *) (rtkConfigPath),
        (char *) "-o",
        (char *) (rtkResultPath),
        (char *) (renixObsRoverPath),
        (char *) (renixObsBasePath),
        (char *) (renixNavFilePath),
    };

    return rtkentry(argc, argv);
}