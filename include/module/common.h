#ifndef GINS_COMMON_H
#define GINS_COMMON_H

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

namespace cmn {

/* HEADER to be printed */
#define PROGRAM_NAME "[POST GNSS/INS NAVIGATION SOFTWARE]"
#define AUTHOR "lewis5499@whu.edu.cn"
#define VERSION "1.0"
#define COPYRIGHT "Copyright(C) 2024 by H.Z.Liu, All rights reserved."
#define _PROJECT_HEADER_                                                                                               \
    "       #########################################################\n"                                               \
    "       ##                                                     ##\n"                                               \
    "       ##         " PROGRAM_NAME "         ##\n"                                                                  \
    "       ##                                                     ##\n"                                               \
    "       ##      Contact: " AUTHOR "                  ##\n"                                                         \
    "       ##                                                     ##\n"                                               \
    "       ##      Version: " VERSION "                                   ##\n"                                       \
    "       ##                                                     ##\n"                                               \
    "       ##  " COPYRIGHT " ##\n"                                                                                    \
    "       ##                                                     ##\n"                                               \
    "       #########################################################\n"

/* configuration file path----------------------------------------------------*/

#ifdef WIN32
#define _PATH_CONFIGURATION_FILE_ "..\\..\\conf\\opts.conf"
#else
#define _PATH_CONFIGURATION_FILE_ "../../conf/opts.conf"
#endif

/* mathmatic functions -------------------------------------------------------*/
#define SQR(x) ((x) * (x))
#define SQRT(x) (((x) <= 0.0 || (x) != (x)) ? 0.0 : std::sqrt(x))
#define SQRT_(x) ((x) >= 0.0 ? std::sqrt(x) : -std::sqrt(-(x)))
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

/* Get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((uint8_t *) (p)))
#define I1(p) (*((int8_t *) (p)))
static uint16_t U2(uint8_t *p) {
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}
static uint32_t U4(uint8_t *p) {
    uint32_t u;
    memcpy(&u, p, 4);
    return u;
}
static int32_t I4(uint8_t *p) {
    int32_t i;
    memcpy(&i, p, 4);
    return i;
}
static float R4(uint8_t *p) {
    float r;
    memcpy(&r, p, 4);
    return r;
}
static double R8(uint8_t *p) {
    double r;
    memcpy(&r, p, 8);
    return r;
}

/* constants -----------------------------------------------------------------*/
static enum ImuCoord { FRD, RFU } imuCoord;        // 0: FRD, 1: RFU
static enum NavSystem { nFrame, eFrame } navFrame; // 0: n-frame, 1: e-frame
static constexpr auto infWeek = 9999;
static constexpr auto infTow  = 604800.0;
static constexpr auto e       = 0.0818191908426;
static constexpr auto g0      = 9.7936174; // 武汉大学信息学部的gravity

static const std::vector<std::string> rtkplotHeaders = {
    "% GPST week", "tow",        "latitude(deg)", "longitude(deg)", "height(m)", "Q",         "ns",
    "sdn(m)",      "sde(m)",     "sdu(m)",        "sdne(m)",        "sdeu(m)",   "sdun(m)",   "age(s)",
    "ratio",       "vn(m/s)",    "ve(m/s)",       "vu(m/s)",        "sdvn(m/s)", "sdve(m/s)", "sdvu(m/s)",
    "roll(deg)",   "pitch(deg)", "yaw(deg)",      "sdr(deg)",       "sdp(deg)",  "sdy(deg)"};

static const std::vector<std::string> imuErrorHeaders = {
    "% GPST week\t\t",     "tow\t\t\t\t\t",     "gb_n[deg/h]\t\t\t",   "gb_e[deg/h]\t\t",   "gb_d[deg/h]\t\t\t",
    "ab_n[mGal]\t\t",      "ab_e[mGal]\t\t\t",  "ab_d[mGal]\t\t\t",    "gs_n[ppm]\t\t\t",   "gs_e[ppm]\t\t\t",
    "gs_d[ppm]\t\t\t\t",   "as_n[ppm]\t\t\t\t", "as_e[ppm]\t\t\t",     "as_d[ppm]\t\t",     "std_gb_n[deg/h]\t",
    "std_gb_e[deg/h]\t",   "std_gb_d[deg/h]",   "std_ab_n[mGal]\t",    "std_ab_e[mGal]\t",  "std_ab_d[mGal]\t",
    "std_gs_n[ppm]\t\t\t", "std_gs_e[ppm]\t\t", "std_gs_d[ppm]\t\t\t", "std_as_n[ppm]\t\t", "std_as_e[ppm]\t\t\t",
    "std_as_d[ppm]"};

/* split line into tokens ----------------------------------------------------*/
template <typename T> std::vector<T> ascimu_line2tokens(const std::string &line) {
    std::vector<T> tokens;

    size_t star_pos        = line.find('*');
    std::string clean_line = (star_pos != std::string::npos) ? line.substr(0, star_pos) : line;

    if (clean_line.substr(0, 9) != "%RAWIMUSA") {
        return tokens;
    }

    std::regex regex("[;,\\*]+"); // "," "*" ";"
    std::sregex_token_iterator iterator(clean_line.begin(), clean_line.end(), regex, -1);
    std::sregex_token_iterator end;

    while (iterator != end) {
        try {
            if (!iterator->str().empty()) { // ignore empty word
                std::stringstream ss(*iterator);
                T value;
                ss >> value;
                if (!ss.fail()) {
                    tokens.push_back(value);
                }
            }
        } catch (const std::invalid_argument &) {
            std::cerr << "Failed to convert 'string' to 'numeric type'! " << *iterator << std::endl;
        }
        ++iterator;
    }
    return tokens;
}

/* Parse GPST Time String "2300 300000.0" into week and tow */
static void parseGPSTimeString(const std::string &timeString, int &week, double &tow) {
    std::stringstream ss(timeString);
    std::vector<std::string> tokens;
    std::string token;

    while (ss >> token) {
        tokens.push_back(token);
    }

    if (tokens.size() != 2) {
        throw std::runtime_error("[ERROR]: Invalid time format");
    }

    if (tokens[0] == "inf" || stod(tokens[0]) >= infWeek) {
        week = infWeek;
    } else {
        week = std::stoi(tokens[0]);
    }

    if (tokens[1] == "inf" || stod(tokens[1]) >= infTow) {
        tow = infTow;
    } else {
        tow = std::stod(tokens[1]);
    }
}

/* convert imu format type 'string' to 'int' */
static enum ImuFormat { IMR, ASC, TXT } imuFormat;
static int ImuFormatType(const std::string &imuformatstring) {
    // 0-> imr file, 1-> asc file, 2->txt file[7 cols].
    return (strcasecmp(imuformatstring.c_str(), "imr") == 0)
               ? IMR
               : ((strcasecmp(imuformatstring.c_str(), "asc") == 0) ? ASC : TXT);
};

/* convert yaw in navigation to yaw in mathematics */
static double yawToMath(double yaw) {
    if (yaw < 0.0 || yaw >= 360.0) {
        throw std::invalid_argument("Yaw must be in the range [0, 360).");
    }

    double attitude;
    if (yaw <= 180.0) {
        attitude = 90.0 - yaw;
    } else {
        attitude = 450.0 - yaw;
    }

    if (attitude > 180.0) {
        attitude -= 360.0;
    }

    return attitude;
}

} // namespace cmn

#endif