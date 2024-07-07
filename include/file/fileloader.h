#ifndef GINS_FILELOADER_H
#define GINS_FILELOADER_H

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "algebra.h"
#include "gnsstypes.h"
#include "imutypes.h"

namespace file {

class GnssFileLoader {
public:
    GnssFileLoader() = default;
    explicit GnssFileLoader(std::string filename, bool bReadAll = false);
    ~GnssFileLoader();

    bool seekBeginningEpoch();
    bool isEOF();
    const gnss::GNSS &getCurrentGNSS();
    const gnss::GNSS &getNextGNSS();
    const std::vector<gnss::GNSS> &getGnssEpochs();

private:
    std::string gnssFileName;
    std::ifstream gnssFileStream;
    std::vector<gnss::GNSS> gnssData;
    gnss::GNSS gnssdata;

    bool parseGnssNextEpoch(const std::string &line);
};

class ImuFileBase {
public:
    ImuFileBase()
        : bParseStatic(false)
        , acc{0.0, 0.0, 0.0}
        , gro{0.0, 0.0, 0.0}
        , att{0.0, 0.0, 0.0}
        , dt(0.0)
        , convScaleAcc(0.0)
        , convScaleGro(0.0)
        , sampRate(0.0)
        , imuCoordType(cmn::FRD)
        , autoWeek (0) {
    }

    virtual ~ImuFileBase() = default; // Virtual destructor for proper cleanup

    virtual const ins::IMU &getCurrentIMU()                                   = 0;
    virtual const ins::IMU &getNextIMU()                                      = 0;
    virtual const std::vector<ins::IMU> &getImuEpochs(bool seekbegin) = 0;

    virtual bool isEOF()              = 0;
    virtual bool seekBeginningEpoch() = 0;

    virtual bool getRPY(double *Att) = 0;
    virtual void roughAlign();

protected:
    bool bParseStatic;
    int imuCoordType;
    uint16_t autoWeek;
    double acc[3], gro[3], att[3], dt;
    double convScaleAcc, convScaleGro, sampRate;
};

class ImuASCLoader : public ImuFileBase {
public:
    ImuASCLoader();
    explicit ImuASCLoader(const std::string &fn, int imucoord, bool parseStatic = true);
    explicit ImuASCLoader(const std::string &fn, int imucoord, double sampRate, double staticMin,
                          bool parseStatic = true);
    explicit ImuASCLoader(const std::string &fn, int imucoord, double sampRate, double staticMin, double scaleAcc,
                          double scaleGyro, bool parseStatic = true);
    ~ImuASCLoader() override;

    bool isEOF() override;
    bool seekBeginningEpoch() override;

    bool getRPY(double *Att) override;
    void roughAlign() override;

    const ins::IMU &getCurrentIMU() override;
    const ins::IMU &getNextIMU() override;
    const std::vector<ins::IMU> &getImuEpochs(bool seekbegin) override;

private:
    double staticMinutes;
    double GPSTime[2];
    std::string filename;
    std::ifstream ascFileStream;

    ins::IMU imudata;
    std::vector<ins::IMU> imuData; // Store all parsed IMU data

    bool parseNextIMUEpoch();
    void parseImuAscStatic();
};

// Structs for IMR file handling
#pragma pack(push, 1)
struct IMRHeader {
    char szHeader[8]{};
    int8_t bIsIntelOrMotorola{};
    double dVersionNumber{};
    int32_t bDeltaTheta{};
    int32_t bDeltaVelocity{};
    double dDataRateHz{};
    double dGyroScaleFactor{};
    double dAccelScaleFactor{};
    int32_t iUtcOrGpsTime{};
    int32_t iRcvTimeOrCorrTime{};
    double dTimeTagBias{};
    char szImuName[32]{};
    uint8_t reserved1[4]{};
    char szProgramName[32]{};
    char tCreate[12]{};
    uint8_t bLeverArmValid{};
    int32_t lXoffset{};
    int32_t lYoffset{};
    int32_t lZoffset{};
    int8_t reserved2[354]{};
};

struct IMRRecord {
    double Time{};
    int32_t gx{}, gy{}, gz{};
    int32_t ax{}, ay{}, az{};
};
#pragma pack(pop)

class ImuIMRLoader : public ImuFileBase {
public:
    ImuIMRLoader();
    explicit ImuIMRLoader(const std::string &fn, int imucoord, uint16_t initWeek, bool parseStatic = false);
    explicit ImuIMRLoader(const std::string &fn, int imucoord, uint16_t initWeek, double staticMinute = 5.0, bool parseStatic = true);
    ~ImuIMRLoader() override;

    const ins::IMU &getCurrentIMU() override;
    const ins::IMU &getNextIMU() override;
    const std::vector<ins::IMU> &getImuEpochs(bool seekbegin) override;

    bool isEOF() override;
    bool seekBeginningEpoch() override;

    bool getRPY(double *Att) override;
    void roughAlign() override;

private:
    std::string imrFileName;
    FILE *imrFileStream;
    double lastEpochTow;
    double staticMinutes{};
    ins::IMU imudata;
    std::vector<ins::IMU> imuData;
    IMRHeader imrHeader;
    IMRRecord imrRecord;

    [[nodiscard]] double getGPSTtow() const;
    void parseIMRHeader();
    int parseNextIMRRecord();
    void parseImuImrStatic();
};

class ImuTxtLoader : public ImuFileBase {
public:
    ImuTxtLoader();
    ~ImuTxtLoader() override;
    explicit ImuTxtLoader(const std::string &fn, int imucoord, uint16_t initWeek, bool parseStatic = false);
    explicit ImuTxtLoader(const std::string &fn, int imucoord, uint16_t initWeek, double sampRate = 200.0, double staticMin = 5.0,
                          bool parseStatic = false);

    const ins::IMU &getCurrentIMU() override;
    const ins::IMU &getNextIMU() override;
    const std::vector<ins::IMU> &getImuEpochs(bool seekbegin) override;
    bool isEOF() override;
    bool seekBeginningEpoch() override;
    bool getRPY(double *Att) override;
    void roughAlign() override;

private:
    ins::IMU imudata;
    std::vector<ins::IMU> imuData;
    std::string fileName;
    std::ifstream txtFileStream;
    double lastEpochTow;
    double staticMinutes;

    int parseNextTxtEpoch();
    void parseImuTxtStatic();
};

class ImuFileLoader {
public:
    ImuFileLoader(int type, const std::string &fname, int imucoord, uint16_t initWeek, bool parseStatic = true);
    ImuFileLoader(int type, const std::string &fname, int imucoord, uint16_t initWeek, double sampRate, double staticMin,
                  bool parseStatic = true);
    ImuFileLoader(int type, const std::string &fname, int imucoord, uint16_t initWeek, double sampRate, double staticMin, double scaleAcc,
                  double scaleGyro, bool parseStatic = true);
    ~ImuFileLoader();

    bool seekBeginningEpoch();
    bool isEOF();
    const ins::IMU &getCurrentIMU();
    const ins::IMU &getNextIMU();
    const std::vector<ins::IMU> &getImuEpochs(bool seekbegin = false);

    bool getRPY(double *Att);
    void roughAlign();

private:
    enum LoaderType { IMR_LOADER, ASC_LOADER, TXT_LOADER };
    std::unique_ptr<ImuFileBase> imuloader;
};

} // namespace file

#endif // GINS_FILELOADER_H
