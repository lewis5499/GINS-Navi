#ifndef GINS_MANAGER_H
#define GINS_MANAGER_H

#include "file/fileloader.h"
#include "file/filesaver.h"
#include "gnss/gnsstypes.h"
#include "ins/imutypes.h"
#include "module/common.h"
#include "module/config.h"
#include "state/navparam.h"
#include "state/navstate.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <string>

namespace Manager {

class BasicParams {
public:
    std::string timeStart, timeEnd;
    std::string rnxObsRov, rnxObsBas, rnxNav;
    std::string rtkSolPath;
    std::string imuPath;
    std::string outputPath;
    std::string odoPath;
    std::string cfgPath;

    BasicParams() {
        timeStart = "0 0.0";
        timeEnd   = std::to_string(cmn::infWeek) + " " + std::to_string(cmn::infTow);
        rnxObsRov = rnxObsBas = rnxNav = rtkSolPath = "";
        imuPath = outputPath = odoPath = "";
        cfgPath                        = _PATH_CONFIGURATION_FILE_;
    }
};

class GINSOptions {
public:
    explicit GINSOptions(const char *filename)
        : gins_conf_manager(filename) {
        navSys            = cmn::nFrame;
        imudataformat     = "imr";
        imuStartGPSTWeek  = 0;
        imuSamplingRate   = 100.0;
        imuInitStaticTime = 5.0;
        odonhc_updaterate = zupt_updaterate = 1.0;
        start_tow                           = 0.0;
        end_tow                             = cmn::infTow;
        start_week                          = 0;
        end_week                            = cmn::infWeek;
        usegnssvel = useZUPT = useodonhc = usesinglenhc = usesingleodo = false;
        onlyinsmech = useRoughAtt = false;
        useInitalPosVelFromRTK    = false;
        estimateImuScale          = plotResults = true;
        imuCoordType              = cmn::FRD;
        parseOptions();
    }

    [[nodiscard]] uint16_t getImuStartWeek() const;
    [[nodiscard]] int getImuCoordType() const;
    [[nodiscard]] int getStartWeek() const;
    [[nodiscard]] int getEndWeek() const;
    [[nodiscard]] double getStartTow() const;
    [[nodiscard]] double getEndTow() const;
    [[nodiscard]] double getImuInitStaticTime() const;
    [[nodiscard]] double getImuSamplingRate() const;
    [[nodiscard]] bool getUsePlotResults() const;
    [[nodiscard]] bool getUseImuScale() const;
    [[nodiscard]] bool getUseodonhc() const;
    [[nodiscard]] bool getUseZUPT() const;
    [[nodiscard]] bool getUsegnssvel() const;
    [[nodiscard]] bool getUseSinglenhc() const;
    [[nodiscard]] bool getUseSingleodo() const;
    [[nodiscard]] const nav::NavState &getInitState() const;
    [[nodiscard]] const nav::NavState &getInitStateStd() const;
    [[nodiscard]] const ins::ImuNoise &getImuNoise() const;
    [[nodiscard]] const nav::InstallParam &getInstallParam() const;
    [[nodiscard]] const nav::MeasureNoise &getMeasNoise() const;
    [[nodiscard]] const std::string &getOutputPath() const;
    [[nodiscard]] const std::string &getImuFilePath() const;
    [[nodiscard]] const std::string &getGnssFilePath() const;
    [[nodiscard]] const std::string &getOdoFilePath() const;
    [[nodiscard]] const std::string &getRnxRovObspath() const;
    [[nodiscard]] const std::string &getRnxBasObspath() const;
    [[nodiscard]] const std::string &getRnxNaviPath() const;
    [[nodiscard]] const std::string &getImuDataFormat() const;
    [[nodiscard]] const std::string &getImuRawCoordinate() const;

    void RenewOptions(const BasicParams &filePaths);
    void RenewOptions(file::ImuFileLoader &imu, file::GnssFileLoader &gnss, const std::vector<double> &initAtt);
    void parseOptions();
    void printOptions() const;

    friend class GINSManager;

private:
    ConfigManager gins_conf_manager;
    nav::NavState initstate;
    nav::NavState initstate_std;
    ins::ImuNoise imunoise;
    nav::InstallParam install_param;
    nav::MeasureNoise meas_noise;
    double odonhc_updaterate, zupt_updaterate;
    double start_tow, end_tow;
    double imuInitStaticTime, imuSamplingRate;
    int start_week, end_week, imuCoordType, navSys;
    uint16_t imuStartGPSTWeek;
    bool usegnssvel, useZUPT, useodonhc, usesinglenhc, usesingleodo, onlyinsmech;
    bool useRoughAtt, useInitalPosVelFromRTK, estimateImuScale, plotResults;
    std::string imrfilepath, gnssfilepath, odofilepath, ascfilepath, o_renixrovpath, o_renixbaspath, n_renixnavpath;
    std::string outputpath, imudataformat, imutxtfilepath, imurawcoordinate;

    std::string readString(const std::string &key);
    Eigen::Vector3d readVector(const std::string &key);
    double readDouble(const std::string &key);
    bool readBool(const std::string &key);

    void unitConvert();
};

// Inherit from KF-GINS
class GINSManager {
public:
    explicit GINSManager(GINSOptions &options);

    ~GINSManager() = default;

    static const int RANK_21      = 21;
    static const int NOISERANK_18 = 18;

    static const int RANK_15      = 15;
    static const int NOISERANK_12 = 12;

    void addImuData(const ins::IMU &imu, bool compensate = false);

    void addGnssData(const gnss::GNSS &gnss);

    void newImuProcess();

    static void imuInterpolate(const ins::IMU &imu1, ins::IMU &imu2, const timesys::Time &timestamp, ins::IMU &midimu);

    [[nodiscard]] timesys::Time getTimestamp() const;

    [[nodiscard]] nav::NavState getNavState() const;

    [[nodiscard]] Eigen::MatrixXd getCovariance() const;

    void setInitUpateTime();

    void writeResult(file::NaviFileSaver &navResFile, file::NaviFileSaver &imuErrFile, const nav::NavState &navstate,
                     const timesys::Time &timeStamp, const Eigen::MatrixXd &cov) const;

    static int detectZUPT(const std::vector<ins::IMU> &imudata, int imuindex, const Eigen::Vector3d &blh,
                          int window_len);

private:
    void initialize(const nav::NavState &initstate, const nav::NavState &initstate_std);

    void imuCompensate(ins::IMU &imu);

    [[nodiscard]] int isToUpdate(const timesys::Time &imutime1, const timesys::Time &imutime2,
                                 const timesys::Time &updatetime) const;

    void insPropagation(ins::IMU &preImu, ins::IMU &curImu);

    void gnssUpdate(gnss::GNSS &gnss);

    void odonhcUpdate(const Eigen::Vector3d &odonhc_vel);

    void zuptUpdate();

    void EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd);

    void EKFUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H, Eigen::MatrixXd &R);

    void errorFeedback();

    void checkCov();

private:
    GINSOptions &options;

    timesys::Time timestamp;
    timesys::Time odonhcUpdatetime;
    timesys::Time zuptUpdatetime;

    ins::IMU imupre;
    ins::IMU imucur;
    gnss::GNSS gnssdata;

    // imu state (position, velocity, attitude and imu error)
    nav::NavState navstatecur;
    nav::NavState navstatepre;

    // ekf variables
    Eigen::MatrixXd P;
    Eigen::MatrixXd q;
    Eigen::MatrixXd dx;

    // updata time align error
    const double TIME_ALIGN_ERR = 0.001; // must match the sampling rate, now is 100/200Hz

    // matrix rank
    int RANK, NOISERANK;

    // state ID and noise ID
    enum StateID { P_ID = 0, V_ID = 3, PHI_ID = 6, BG_ID = 9, BA_ID = 12, SG_ID = 15, SA_ID = 18 };
    enum NoiseID { VRW_ID = 0, ARW_ID = 3, BGSTD_ID = 6, BASTD_ID = 9, SGSTD_ID = 12, SASTD_ID = 15 };
};

} // namespace Manager

#endif // GINS_MANAGER_H
