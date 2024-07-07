#include "fileloader.h"
#include "algebra.h"
#include "common.h"
#include "imutypes.h"
#include "tqdm.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utility>

using namespace file;
using namespace std;
using namespace Algebra;
using namespace gnss;
using namespace cmn;
using namespace ins;
using namespace Eigen;

GnssFileLoader::GnssFileLoader(string filename, bool bReadAll)
    : gnssFileName(std::move(filename)) {
    gnssFileStream.open(gnssFileName);
    if (!this->gnssFileStream.is_open()) {
        cerr << "[ERROR]: Error opening file: " << this->gnssFileName << endl;
    } else if (bReadAll) {
        this->getGnssEpochs();
    } else
        ;
}

GnssFileLoader::~GnssFileLoader() {
    if (gnssFileStream.is_open()) {
        gnssFileStream.close();
    }
}

bool GnssFileLoader::parseGnssNextEpoch(const string &line) {
    istringstream iss(line);
    double tow, lat, lon, height, sdn, sde, sdu, vn, ve, vu, sdvn, sdve, sdvu;
    int q, ns;
    uint16_t week;
    string skip;

    if (iss >> week >> tow >> lat >> lon >> height >> q >> ns >> sdn >> sde >> sdu >> skip >> skip >> skip >> skip >>
        skip >> vn >> ve >> vu >> sdvn >> sdve >> sdvu) {

        this->gnssdata.time.setTime(week, tow);
        this->gnssdata.blh << lat * d2r, lon * d2r, height;
        this->gnssdata.posstd << sdn, sde, sdu;
        this->gnssdata.vel << vn, ve, vu;
        this->gnssdata.velstd << sdvn, sdve, sdvu;
        this->gnssdata.valid = (q == 1 || q == 2);

        return true;
    }

    return false;
}

const vector<GNSS> &GnssFileLoader::getGnssEpochs() {
    this->gnssData.clear();

    // Reset file stream to the beginning of the file
    this->gnssFileStream.clear();            // Clear any error flags
    this->gnssFileStream.seekg(0, ios::beg); // Move to the beginning of the file

    while (!this->gnssFileStream.eof()) {
        auto gnss = this->getNextGNSS();
        if (gnss.valid) {
            this->gnssData.push_back(gnss);
        }
    }

    return this->gnssData;
}

bool GnssFileLoader::seekBeginningEpoch() {
    this->gnssData.clear();

    // Reset file stream to the beginning of the file
    this->gnssFileStream.clear();            // Clear any error flags
    this->gnssFileStream.seekg(0, ios::beg); // Move to the beginning of the file

    // 检查文件流是否成功定位
    if (this->gnssFileStream.fail()) {
        return false;
    } else {
        return true;
    }
}

bool GnssFileLoader::isEOF() {
    return this->gnssFileStream.eof();
}

const GNSS &GnssFileLoader::getCurrentGNSS() {
    return this->gnssdata;
}

const GNSS &GnssFileLoader::getNextGNSS() {
    string line;
    while (getline(this->gnssFileStream, line) && !this->gnssFileStream.eof()) {
        if (line.empty() || line[0] == '%') {
            continue; // Skip header or empty lines
        }
        if (parseGnssNextEpoch(line)) {
            return this->gnssdata;
        } else {
            cerr << "[ERROR]: Error parsing line: " << line << endl;
            continue;
        }
    }
    this->gnssdata.valid = false;
    return this->gnssdata;
}

void ImuFileBase::roughAlign() {
    // Default implementation
    // You can provide a default implementation or leave it empty
    if (!bParseStatic) {
        cerr << "[WARNING]:  No IMU Data For Alignment!" << endl;
        std::fflush(stderr);
        return;
    }

    cout << "[INFO]:  IMU Rough Alignment In Process..." << endl;

    // Dual vector algorithm for pose determination
    Vector3d Acce(this->acc[0], this->acc[1], this->acc[2]);
    Vector3d Gyro(this->gro[0], this->gro[1], this->gro[2]);

    // b-frame: NED
    Vector3d gb(-Acce[0], -Acce[1], -Acce[2]);
    Vector3d wieb = Gyro;

    Vector3d wg  = gb.normalized();
    Vector3d ww  = (gb.cross(wieb)).normalized();
    Vector3d wgw = ((gb.cross(wieb)).cross(gb)).normalized();

    Matrix3d w;
    w << wg, ww, wgw;

    // n-frame: NED
    Vector3d gn(0, 0, cmn::g0);
    Vector3d wien(WGS84_WIE * cos(LAT * d2r), 0, -WGS84_WIE * sin(LAT * d2r));
    Vector3d vg  = gn.normalized();
    Vector3d vw  = (gn.cross(wien)).normalized();
    Vector3d vgw = ((gn.cross(wien)).cross(gn)).normalized();

    Matrix3d v;
    v << vg, vw, vgw;

    // get C:b->n and the EulerAngle Group
    Matrix3d Cbn = v * (w.transpose());

    this->att[0] = atan2(Cbn(2, 1), Cbn(2, 2));
    this->att[1] = atan(-Cbn(2, 0) / sqrt(pow(Cbn(2, 1), 2) + pow(Cbn(2, 2), 2)));
    this->att[2] = atan2(Cbn(1, 0), Cbn(0, 0));

    if(this->att[2]<0) this->att[2] += 2*pi;
}

ImuASCLoader::ImuASCLoader() {
    this->staticMinutes = 5.0;
    this->sampRate      = 100.0;
    this->convScaleAcc  = 1.5258789063E-06;
    this->convScaleGro  = 1.0850694444E-07;
    this->GPSTime[0] = this->GPSTime[1] = 0.0;
    this->bParseStatic                  = false;
    this->ascFileStream.open(this->filename, ios::in);
};

ImuASCLoader::ImuASCLoader(const string &fn, int imucoord, const bool parseStatic) {
    this->staticMinutes = 5.0;
    this->sampRate      = 100.0;
    this->convScaleAcc  = 1.5258789063E-06;
    this->convScaleGro  = 1.0850694444E-07;
    this->GPSTime[0] = this->GPSTime[1] = 0.0;
    this->filename                      = fn;
    this->bParseStatic                  = parseStatic;
    this->imuCoordType=imucoord;
    this->ascFileStream.open(this->filename, ios::in);
    if (parseStatic)
        this->parseImuAscStatic();
};

ImuASCLoader::ImuASCLoader(const string &fn, int imucoord, const double sampRate, const double staticMin, const bool parseStatic) {
    this->staticMinutes = staticMin;
    this->convScaleAcc  = 1.5258789063E-06;
    this->convScaleGro  = 1.0850694444E-07;
    this->sampRate      = sampRate;
    this->GPSTime[0] = this->GPSTime[1] = 0.0;
    this->filename                      = fn;
    this->bParseStatic                  = parseStatic;
    this->imuCoordType= imucoord;
    this->ascFileStream.open(this->filename, ios::in);
    if (parseStatic)
        this->parseImuAscStatic();
}

ImuASCLoader::ImuASCLoader(const string &fn, int imucoord, const double sampRate, const double staticMin, const double scaleAcc,
                           const double scaleGyro, const bool parseStatic) {
    this->staticMinutes = staticMin;
    this->convScaleAcc  = scaleAcc;
    this->convScaleGro  = scaleGyro;
    this->sampRate      = sampRate;
    this->GPSTime[0] = this->GPSTime[1] = 0.0;
    this->filename                      = fn;
    this->bParseStatic                  = parseStatic;
    this->imuCoordType= imucoord;
    this->ascFileStream.open(this->filename, ios::in);
    if (parseStatic)
        this->parseImuAscStatic();
};

ImuASCLoader::~ImuASCLoader() {
    this->ascFileStream.close();
}

bool ImuASCLoader::isEOF() {
    return this->ascFileStream.eof();
}

bool ImuASCLoader::seekBeginningEpoch() {
    this->imuData.clear();
    this->GPSTime[0]=0.0;
    this->GPSTime[1]=0.0;

    // Reset file stream to the beginning of the file
    this->ascFileStream.clear();            // Clear any error flags
    this->ascFileStream.seekg(0, ios::beg); // Move to the beginning of the file

    // 检查文件流是否成功定位
    if (this->ascFileStream.fail()) {
        return false;
    } else {
        return true;
    }
}

bool ImuASCLoader::getRPY(double *Att) {
    try {
        memcpy(Att, this->att, 3 * sizeof(double));
        return true;
    } catch (const std::exception &) {
        cerr << "[ERROR]:  Array:'Att' Fatal Error! " << endl;
        std::fflush(stderr);
        return false;
    }
}

void ImuASCLoader::roughAlign() {
    ImuFileBase::roughAlign();
    cout << "[INFO]:\t Result Format = [R, P, Y], from ASC data" << endl;
    cout << setiosflags(ios::fixed) << setprecision(6);
    cout << "[INFO]:\t IMU Attitude(deg):\t" << this->att[0] * r2d << "\t" << this->att[1] * r2d << "\t"
         << this->att[2] * r2d << endl;
    cout << "[INFO]:\t IMU Attitude(rad):\t" << this->att[0] << "\t" << this->att[1] << "\t" << this->att[2] << endl;
}

const IMU &ImuASCLoader::getCurrentIMU() {
    return this->imudata;
}

const IMU &ImuASCLoader::getNextIMU() {

    while(!this->parseNextIMUEpoch() && !this->isEOF()) {}

    if (!this->isEOF()) {
        this->imudata.time.setTime((uint16_t) this->GPSTime[0], this->GPSTime[1]);
        this->imudata.dt = this->dt;
        this->imudata.valid     = true;
        this->imudata.odovel    = 0.0;
        this->imudata.dtheta(0) = this->gro[0];
        this->imudata.dtheta(1) = this->gro[1];
        this->imudata.dtheta(2) = this->gro[2];
        this->imudata.dvel(0)   = this->acc[0];
        this->imudata.dvel(1)   = this->acc[1];
        this->imudata.dvel(2)   = this->acc[2];
    } else {
        this->imudata.dt = 0.0;
        this->imudata.time.Reset();
        this->imudata.odovel = 0.0;
        this->imudata.valid  = false;
        this->imudata.dtheta.setZero();
        this->imudata.dvel.setZero();
    }

    return this->imudata;
}

const vector<IMU> &ImuASCLoader::getImuEpochs(bool seekbegin) {
    if (seekbegin) {
        if (!this->seekBeginningEpoch()) {
            std::cerr << "[ERROR]: ASC file stream seek failed!" << endl;
        }
    }

    // Save current imu if valid
    if (this->imudata.valid) {
        this->imuData.push_back(this->imudata);
    }

    // Parse records until the end of the file
    while (!this->isEOF()) {
        auto imu = this->getNextIMU();
        if (imu.valid) {
            this->imuData.push_back(imu);
        }
    }

    return this->imuData;
}

bool ImuASCLoader::parseNextIMUEpoch() {
    if (!ascFileStream.is_open()) {
        cerr << "[ERROR]: IMU Asc File is not open!" << endl;
        // Return a default constructed IMU object indicating failure
        return false;
    }

    string line;
    if (!getline(ascFileStream, line)) {
        cout << "[INFO]:  EOF of IMU ASC file." << endl;
        return false;
    }

    auto tokens = cmn::ascimu_line2tokens<double>(line);
    if (tokens.size() != 11) {
        cerr << "[ERROR]: Invalid Asc IMU line format!" << endl;
        return false;
    }

    if (!(fabs(this->GPSTime[0]) < 1e-5 && fabs(this->GPSTime[1]) < 1e-5)) {
        // Parse relevant data from tokens
        this->dt = (int) (tokens[2] - this->GPSTime[0]) * 604800.0 + tokens[3] - this->GPSTime[1];

        if (this->dt < 1.0 / sampRate * 0.5) { // 数据重复
            cout << "[WARNING]: Time discontinuity detected: Time difference is non-positive (" << this->dt
                 << " seconds)." << endl;
            return false;
        } else if (this->dt > 1.0 / sampRate * 1.5) { // 数据中断
            cout << "[WARNING]: Time discontinuity detected: Time difference exceeds threshold (" << this->dt
                 << " seconds)." << endl;
            this->dt = 1.0/this->sampRate;
        } else
            ;
    } else { // initial epoch
        this->dt = 1.0 / this->sampRate;
    }

    // Update imuTime for next iteration
    this->GPSTime[0] = tokens[2];
    this->GPSTime[1] = tokens[3];
    this->acc[0]     = tokens[7] * convScaleAcc;
    this->acc[1]     = -tokens[6] * convScaleAcc;
    this->acc[2]     = tokens[5] * convScaleAcc;
    this->gro[0]     = tokens[10] * convScaleGro;
    this->gro[1]     = -tokens[9] * convScaleGro;
    this->gro[2]     = tokens[8] * convScaleGro;

    if (this->imuCoordType==cmn::RFU) {
        // convert 'right-forward-up' to 'forward-right-down'
        swap(this->acc[0], this->acc[1]);
        swap(this->gro[0], this->gro[1]);
        this->acc[2] = -this->acc[2];
        this->gro[2] = -this->gro[2];
    }

    return true;
}

void ImuASCLoader::parseImuAscStatic() {
    if (seekBeginningEpoch()) {
        cout << "[INFO]:\t IMU Data Parsing..." << endl;
    } else {
        cerr << "[ERROR]: ASC File Error!" << endl;
    }

    // Average over the 'staticMinutes' imu data to suppress gauss white noise
#ifndef WIN32
    tqdm bar;
    bar.set_theme_basic();
    bar.disable_colors();
#endif
    string line;
    double dt, imuTime[2] = {0.0, 0.0};

    // First Epoch
    while (!getline(ascFileStream, line)) {
        auto tokens = cmn::ascimu_line2tokens<double>(line);
        if (tokens.size() == 11) { // match valid line
            imuTime[0] = tokens[2];
            imuTime[1] = tokens[3];
            break;
        }
    }

    // Loop!
    for (int linesRead = 0; linesRead < (int) (this->staticMinutes * 60.0 * this->sampRate);) {
#ifndef WIN32
        bar.progress(linesRead, (int) (this->staticMinutes * 60.0 * this->sampRate));
#endif
        if (!getline(ascFileStream, line)) {
            break;
        }

        auto tokens = cmn::ascimu_line2tokens<double>(line);

        // Calculate average of elements [5] to [10]
        if (tokens.size() == 11) {
            dt = (int) (tokens[2] - imuTime[0]) * 604800.0 + tokens[3] - imuTime[1];
            if (dt < 1.0 / sampRate * 0.5) { // 数据重复
                continue;
            } else if (dt > 1.0 / sampRate * 1.5) { // 数据中断
                dt = 1.0 / sampRate;
            } else
                ;

            this->acc[0] = (tokens[5] / dt + (linesRead) * this->acc[0]) / (linesRead + 1);
            this->acc[1] = (tokens[6] / dt + (linesRead) * this->acc[1]) / (linesRead + 1);
            this->acc[2] = (tokens[7] / dt + (linesRead) * this->acc[2]) / (linesRead + 1);
            this->gro[0] = (tokens[8] / dt + (linesRead) * this->gro[0]) / (linesRead + 1);
            this->gro[1] = (tokens[9] / dt + (linesRead) * this->gro[1]) / (linesRead + 1);
            this->gro[2] = (tokens[10] / dt + (linesRead) * this->gro[2]) / (linesRead + 1);
            linesRead++;

            imuTime[0] = tokens[2];
            imuTime[1] = tokens[3];
        }
    }
#ifndef WIN32
    bar.finish();
#endif

    // parse imu data from *.ASC file
    double Acc[3], Gro[3];                // X, Y, Z
    Acc[0] = this->acc[2] * convScaleAcc; // unit: m/s^2
    Acc[1] = -this->acc[1] * convScaleAcc;
    Acc[2] = this->acc[0] * convScaleAcc;
    Gro[0] = this->gro[2] * convScaleGro; // unit: rad/s
    Gro[1] = -this->gro[1] * convScaleGro;
    Gro[2] = this->gro[0] * convScaleGro;

    if (this->imuCoordType==cmn::RFU) {
        // convert 'right-forward-up' to 'forward-right-down'
        swap(Acc[0], Acc[1]);
        swap(Gro[0], Gro[1]);
        Acc[2] = -Acc[2];
        Gro[2] = -Gro[2];
    }

    // save data
    memcpy(this->acc, Acc, 3 * sizeof(double));
    memcpy(this->gro, Gro, 3 * sizeof(double));
}

ImuIMRLoader::ImuIMRLoader() {
    bParseStatic = false;
    convScaleAcc = convScaleGro = 1.0;
    sampRate                    = 100.0;
    staticMinutes               = 5.0;
    imrFileStream               = nullptr;
    lastEpochTow                = 0.0;
}

ImuIMRLoader::ImuIMRLoader(const string &fn, int imucoord, uint16_t initWeek, bool parseStatic)
    : imrFileName(fn)
    , imrFileStream(nullptr)
    , lastEpochTow(0.0) {
    this->imuCoordType = imucoord;
    this->autoWeek = initWeek;
    staticMinutes = 5.0;
    if (!(imrFileStream = fopen(fn.c_str(), "rb"))) {
        cerr << "[ERROR]: Cannot open imrFileStream: " << fn << endl;
        std::fflush(stderr);
        this->bParseStatic = false;
    } else {
        parseIMRHeader();
        this->bParseStatic = bParseStatic;
        if (bParseStatic)
            parseImuImrStatic();
    }
}

ImuIMRLoader::ImuIMRLoader(const string &fn, int imucoord, uint16_t initWeek, double staticMinute, bool bParseStatic)
    : imrFileName(fn)
    , imrFileStream(nullptr)
    , lastEpochTow(0.0) {
    this->imuCoordType= imucoord;
    this->autoWeek = initWeek;
    if (!(imrFileStream = fopen(fn.c_str(), "rb"))) {
        cerr << "[ERROR]: Cannot open imrFileStream: " << fn << endl;
        std::fflush(stderr);
        this->bParseStatic = false;
    } else {
        parseIMRHeader();
        staticMinutes      = staticMinute;
        this->bParseStatic = bParseStatic;
        if (bParseStatic)
            parseImuImrStatic();
    }
}

ImuIMRLoader::~ImuIMRLoader() {
    if (imrFileStream) {
        fclose(imrFileStream);
    }
}

double ImuIMRLoader::getGPSTtow() const {
    double time = imrRecord.Time;

    // Adjust time with the bias
    if (imrHeader.iRcvTimeOrCorrTime == 1) {
        time += imrHeader.dTimeTagBias;
    }

    // If the time is in UTC, convert it to GPST
    if (imrHeader.iUtcOrGpsTime == 1) {
        // Conversion from UTC to GPST: GPST = UTC + 18 (as of 2023)
        time += 18.0;
    }

    return time;
}

int ImuIMRLoader::parseNextIMRRecord() {
    if (!imrFileStream) {
        cerr << "[ERROR]: File is not open." << endl;
        return 0;
    }

    if (feof(imrFileStream)) {
        cout << "[INFO]: End of IMR file reached." << endl;
        return 0;
    }

READIMRRECORD:

    if (fread(&imrRecord, sizeof(IMRRecord), 1, imrFileStream) != 1) {
        if (feof(imrFileStream)) {
            cout << "[INFO]:  EOF of IMU IMR file." << endl;
        } else {
            cerr << "[ERROR]: Error reading IMR record." << endl;
        }
        return 0;
    }

    // Check for time discontinuities
    if (fabs(lastEpochTow) > 1e-5) {
        this->dt = imrRecord.Time - lastEpochTow;
        if (this->dt <= 0.5 / this->imrHeader.dDataRateHz) { // 处理数据重复
            cout << "[WARNING]: Time discontinuity detected: Time difference is non-positive (" << this->dt
                 << " seconds)." << endl;
            goto READIMRRECORD;
            // lastEpochTow = imrRecord.Time;
            // return -1;

        } else if (this->dt > 1.5 / this->imrHeader.dDataRateHz && abs(this->dt)<604790.0) { // 处理数据中断，中断后恢复的第一个历元弃用
            cout << "[WARNING]: Time discontinuity detected: Time difference exceeds threshold (" << this->dt
                 << " seconds(<604790.0s))." << endl;
            this->dt = 1.0/this->imrHeader.dDataRateHz;
            lastEpochTow = imrRecord.Time;
            return 1;

        } else if(abs(this->dt)>604790.0) { // 简易跨周判断， 不能处理频繁中断的数据
            cout << "\n[WARNING]: Time Across The Next Week detected: Time difference exceeds threshold (" << this->dt
                 << " seconds(abs>604790.0s)).\n" << endl;
            lastEpochTow = imrRecord.Time;
            autoWeek += 1;
            this->dt += 604800.0;
            return 1; // 记作成功解析数据

        } else {
            lastEpochTow = imrRecord.Time;
        }
    } else { // Initial Epoch
        this->dt     = 1.0 / this->imrHeader.dDataRateHz;
        lastEpochTow = imrRecord.Time;
    }

    return 1;
}

void ImuIMRLoader::parseIMRHeader() {
    fread(&imrHeader, sizeof(IMRHeader), 1, imrFileStream);
    if (strncmp(imrHeader.szHeader, "$IMURAW", 7) != 0) {
        cerr << "[ERROR]: Invalid IMR file header." << endl;
        std::fflush(stderr);
        fclose(imrFileStream);
        imrFileStream = nullptr;
    }
    convScaleAcc = imrHeader.dAccelScaleFactor;
    convScaleGro = imrHeader.dGyroScaleFactor;
    sampRate     = imrHeader.dDataRateHz;
}

const IMU &ImuIMRLoader::getCurrentIMU() {
    return this->imudata;
}

const IMU &ImuIMRLoader::getNextIMU() {

    if (parseNextIMRRecord() == 1) {
        this->imudata.time.setTime((uint16_t) autoWeek, this->getGPSTtow());

        if (imrHeader.bDeltaTheta == 0) { // angle rate->delta rad/s->rad
            this->imudata.dtheta(0) = imrRecord.gx * imrHeader.dGyroScaleFactor * dt * d2r,
            this->imudata.dtheta(1) = imrRecord.gy * imrHeader.dGyroScaleFactor * dt * d2r,
            this->imudata.dtheta(2) = imrRecord.gz * imrHeader.dGyroScaleFactor * dt * d2r;
        } else {
            this->imudata.dtheta(0) = imrRecord.gx * imrHeader.dGyroScaleFactor * d2r;
            this->imudata.dtheta(1) = imrRecord.gy * imrHeader.dGyroScaleFactor * d2r;
            this->imudata.dtheta(2) = imrRecord.gz * imrHeader.dGyroScaleFactor * d2r;
        }

        if (imrHeader.bDeltaVelocity == 0) { // m/s^2->m/s
            this->imudata.dvel(0) = imrRecord.ax * imrHeader.dAccelScaleFactor * dt;
            this->imudata.dvel(1) = imrRecord.ay * imrHeader.dAccelScaleFactor * dt;
            this->imudata.dvel(2) = imrRecord.az * imrHeader.dAccelScaleFactor * dt;
        } else {
            this->imudata.dvel(0) = imrRecord.ax * imrHeader.dAccelScaleFactor;
            this->imudata.dvel(1) = imrRecord.ay * imrHeader.dAccelScaleFactor;
            this->imudata.dvel(2) = imrRecord.az * imrHeader.dAccelScaleFactor;
        }

        this->imudata.dt     = this->dt;
        this->imudata.odovel = 0.0;
        this->imudata.valid  = true;

    } else {
        this->imudata.dt = 0.0;
        this->imudata.time.Reset();
        this->imudata.odovel = 0.0;
        this->imudata.valid  = false;
        this->imudata.dtheta.setZero();
        this->imudata.dvel.setZero();
    }

    if (this->imuCoordType==cmn::RFU) {
        // convert 'right-forward-up' to 'forward-right-down'
        std::swap(this->imudata.dtheta(0), this->imudata.dtheta(1));
        std::swap(this->imudata.dvel(0), this->imudata.dvel(1));
        this->imudata.dtheta(2) = -this->imudata.dtheta(2);
        this->imudata.dvel(2)   = -this->imudata.dvel(2);
    }

    return this->imudata;
}

void ImuIMRLoader::parseImuImrStatic() {
    // Seek to the beginning of the file
    if (!this->seekBeginningEpoch()) {
        std::cerr << "[ERROR]: IMR file stream seek failed!" << endl;
        return ;
    }

    double factorAcc, factorGyr;

    factorGyr = ((imrHeader.bDeltaTheta) ? (1.0 * imrHeader.dDataRateHz) : 1.0) * imrHeader.dGyroScaleFactor;
    factorAcc = ((imrHeader.bDeltaVelocity) ? (1.0 * imrHeader.dDataRateHz) : 1.0) * imrHeader.dAccelScaleFactor;

    for (int i = 0; i < (int) (staticMinutes * 60 * sampRate);) {
        int result = parseNextIMRRecord();
        if (result != 1) {
            if (result == 0) {
                cout << "[INFO]: End of IMR file reached(static period)." << endl;
            }
            continue;
        }
        this->acc[0] = (imrRecord.ax * factorAcc + i * this->acc[0]) / (i + 1);
        this->acc[1] = (imrRecord.ay * factorAcc + i * this->acc[1]) / (i + 1);
        this->acc[2] = (imrRecord.az * factorAcc + i * this->acc[2]) / (i + 1);
        this->gro[0] = (imrRecord.gx * factorGyr + i * this->gro[0]) / (i + 1);
        this->gro[1] = (imrRecord.gy * factorGyr + i * this->gro[1]) / (i + 1);
        this->gro[2] = (imrRecord.gz * factorGyr + i * this->gro[2]) / (i + 1);
        i++;
    }

    if (this->imuCoordType==cmn::RFU) {
        // convert 'right-forward-up' to 'forward-right-down'
        swap(this->acc[0], this->acc[1]);
        swap(this->gro[0], this->gro[1]);
        this->acc[2] = -this->acc[2];
        this->gro[2] = -this->gro[2];
    }
}

const vector<IMU> &ImuIMRLoader::getImuEpochs(bool seekbegin) {
    if (seekbegin) {
        if (!this->seekBeginningEpoch()) {
            std::cerr << "[ERROR]: IMR file stream seek failed!" << endl;
        }
    }

    // Save current imu if valid
    if (this->imudata.valid) {
        this->imuData.push_back(this->imudata);
    }

    // Parse records until the end of the file
    while (!feof(this->imrFileStream)) {
        auto imu = this->getNextIMU();
        if (imu.valid) {
            this->imuData.push_back(imu);
        }
    }

    return this->imuData;
}

bool ImuIMRLoader::isEOF() {
    return feof(this->imrFileStream);
}

bool ImuIMRLoader::seekBeginningEpoch() {
    // Clear the vector to ensure it's empty before loading data
    this->imuData.clear();
    this->lastEpochTow = 0.0;

    // Seek to the beginning of the file
    if (fseek(imrFileStream, sizeof(IMRHeader), SEEK_SET) == 0) {
        return true;
    } else {
        perror("[ERROR]: Error seeking in IMR file");
        return false;
    }
}

bool ImuIMRLoader::getRPY(double *Att) {
    try {
        memcpy(Att, this->att, 3 * sizeof(double));
        return true;
    } catch (const std::exception &) {
        cerr << "[ERROR]: Array:'Att' Fatal Error!" << endl;
        std::fflush(stderr);
        return false;
    }
}

void ImuIMRLoader::roughAlign() {
    ImuFileBase::roughAlign();
    cout << "[INFO]:\t Result Format = [R, P, Y], from IMR data" << endl;
    cout << setiosflags(ios::fixed) << setprecision(6);
    cout << "[INFO]:\t IMU Attitude(deg):\t" << this->att[0] * r2d << "\t" << this->att[1] * r2d << "\t"
         << this->att[2] * r2d << endl;
    cout << "[INFO]:\t IMU Attitude(rad):\t" << this->att[0] << "\t" << this->att[1] << "\t" << this->att[2] << endl;
}

ImuFileLoader::ImuFileLoader(int type, const std::string &fname, int coord, uint16_t initweek, bool parsestatic) {
    if (type == IMR_LOADER) {
        imuloader = std::make_unique<ImuIMRLoader>(fname, coord, initweek, parsestatic);
    } else if (type == ASC_LOADER) {
        imuloader = std::make_unique<ImuASCLoader>(fname, coord, parsestatic);
    } else if (type == TXT_LOADER) {
        imuloader = std::make_unique<ImuTxtLoader>(fname, coord, initweek, parsestatic);
    } else
        ;
}

ImuFileLoader::ImuFileLoader(int type, const std::string &fname, int coord, uint16_t initweek, double samprate, double staticmin, bool parsestatic) {
    if (type == IMR_LOADER) {
        imuloader = std::make_unique<ImuIMRLoader>(fname, coord, initweek, staticmin, parsestatic);
    } else if (type == ASC_LOADER) {
        imuloader = std::make_unique<ImuASCLoader>(fname, coord, samprate, staticmin, parsestatic);
    } else if (type == TXT_LOADER) {
        imuloader = std::make_unique<ImuTxtLoader>(fname, coord, initweek, samprate, staticmin, parsestatic);
    } else
        ;
}

ImuFileLoader::ImuFileLoader(int type, const std::string &fname, int coord, uint16_t initweek, double samprate, double staticmin, double scaleAcc,
                             double scaleGyro, bool parsestatic) {
    if (type == IMR_LOADER) {
        imuloader = std::make_unique<ImuIMRLoader>(fname, coord, initweek, staticmin, parsestatic);
    } else if (type == ASC_LOADER)  {
        imuloader = std::make_unique<ImuASCLoader>(fname, coord, samprate, staticmin, scaleAcc, scaleGyro, parsestatic);
    } else if (type == TXT_LOADER) {
        imuloader = std::make_unique<ImuTxtLoader>(fname, coord, initweek, samprate, staticmin, parsestatic);
    } else
        ;
}

ImuFileLoader::~ImuFileLoader() = default;

bool ImuFileLoader::seekBeginningEpoch() {
    return imuloader->seekBeginningEpoch();
}

bool ImuFileLoader::isEOF() {
    return imuloader->isEOF();
}

bool ImuFileLoader::getRPY(double *Att) {
    return imuloader->getRPY(Att);
}

void ImuFileLoader::roughAlign() {
    imuloader->roughAlign();
}

const IMU &ImuFileLoader::getCurrentIMU() {
    return imuloader->getCurrentIMU();
}

const IMU &ImuFileLoader::getNextIMU() {
    return imuloader->getNextIMU();
}

const std::vector<IMU> &ImuFileLoader::getImuEpochs(bool seekbegin) {
    return imuloader->getImuEpochs(seekbegin);
}

ImuTxtLoader::ImuTxtLoader() {
    bParseStatic = false;
    convScaleAcc = convScaleGro = 1.0;
    sampRate                    = 100.0;
    staticMinutes               = 5.0;
    lastEpochTow                = 0.0;
}
ImuTxtLoader::~ImuTxtLoader() {
    if (txtFileStream.is_open()) {
        txtFileStream.close();
    }
}

ImuTxtLoader::ImuTxtLoader(const std::string &fn, int imucoord, uint16_t initWeek, bool parseStatic) {
    this->staticMinutes = 5.0;
    this->sampRate      = 200.0;
    this->lastEpochTow  = 0.0;
    this->fileName      = fn;
    this->bParseStatic  = parseStatic;
    this->imuCoordType=imucoord;
    this->autoWeek = initWeek;

    txtFileStream.open(fn);
    if (!txtFileStream.is_open()) {
        std::cerr << "[ERROR]: Cannot open txtFileStream: " << fn << std::endl;
        std::fflush(stderr);
        this->bParseStatic = false;
    }
    if (parseStatic) {
        this->parseImuTxtStatic();
    }
}

ImuTxtLoader::ImuTxtLoader(const std::string &fn, int imucoord, uint16_t initWeek, double sampRate, double staticMin, bool parseStatic) {
    this->staticMinutes = staticMin;
    this->sampRate      = sampRate;
    this->lastEpochTow  = 0.0;
    this->fileName      = fn;
    this->bParseStatic  = parseStatic;
    this->imuCoordType=imucoord;
    this->autoWeek = initWeek;

    txtFileStream.open(fn);
    if (!txtFileStream.is_open()) {
        std::cerr << "[ERROR]: Cannot open txtFileStream: " << fn << std::endl;
        std::fflush(stderr);
        this->bParseStatic = false;
    }
    if (parseStatic) {
        this->parseImuTxtStatic();
    }
}

const ins::IMU &ImuTxtLoader::getCurrentIMU() {
    return this->imudata;
}
const ins::IMU &ImuTxtLoader::getNextIMU() {
    while (this->parseNextTxtEpoch()!=1 && !this->isEOF()){};
    return this->imudata;
}
const std::vector<ins::IMU> &ImuTxtLoader::getImuEpochs(bool seekbegin) {
    if (seekbegin) {
        if (!this->seekBeginningEpoch()) {
            std::cerr << "[ERROR]: IMR file stream seek failed!" << std::endl;
        }
    }

    // Save current imu if valid
    if (this->imudata.valid) {
        this->imuData.push_back(this->imudata);
    }

    // Parse records until the end of the file
    while (!this->isEOF()) {
        auto imu = this->getNextIMU();
        if (imu.valid) {
            this->imuData.push_back(imu);
        }
    }

    return this->imuData;
}

bool ImuTxtLoader::isEOF() {
    return txtFileStream.eof();
}
bool ImuTxtLoader::seekBeginningEpoch() {
    this->imuData.clear();
    this->lastEpochTow = 0.0;
    if (txtFileStream.is_open()) {
        txtFileStream.clear();
        txtFileStream.seekg(0, std::ios::beg);
        return true;
    } else {
        std::cerr << "[ERROR]: Error seeking in txt IMU file" << std::endl;
        return false;
    }
}

bool ImuTxtLoader::getRPY(double *Att) {
    try {
        memcpy(Att, this->att, 3 * sizeof(double));
        return true;
    } catch (const std::exception &) {
        std::cerr << "[ERROR]: Array:'Att' Fatal Error!" << std::endl;
        std::fflush(stderr);
        return false;
    }
}

void ImuTxtLoader::roughAlign() {
    ImuFileBase::roughAlign();
    std::cout << "[INFO]:\t Result Format = [R, P, Y], from txt IMU data" << std::endl;
    std::cout << setiosflags(std::ios::fixed) << std::setprecision(6);
    std::cout << "[INFO]:\t IMU Attitude(deg):\t" << this->att[0] * Algebra::r2d << "\t" << this->att[1] * Algebra::r2d
              << "\t"
              << this->att[2] * Algebra::r2d << std::endl;
    std::cout << "[INFO]:\t IMU Attitude(rad):\t" << this->att[0] << "\t" << this->att[1] << "\t" << this->att[2] << std::endl;
}

int ImuTxtLoader::parseNextTxtEpoch() {
    std::string line;
    double tow;
    if (std::getline(txtFileStream, line)) {

        std::istringstream iss(line);

        if (!(iss >> tow >> this->gro[0] >> this->gro[1] >> this->gro[2] >> this->acc[0] >> this->acc[1] >>
              this->acc[2])) {
            std::cerr << "[ERROR]: Failed to parse line: " << line << std::endl;
            this->imudata.valid = false;
            return -1;
        }

        // Check for time discontinuities
        if (fabs(lastEpochTow) > 1e-5) {
            this->dt = tow - this->lastEpochTow;

            if (this->dt <= 0.5 / this->sampRate) { // 处理数据重复
                std::cout << "[WARNING]: Time discontinuity detected: Time difference is (" << this->dt << " seconds)."
                          << std::endl;
                return -1;

            } else if (this->dt > 1.5 / this->sampRate &&
                       abs(this->dt) <= 604790.0) { // 处理数据中断，中断后恢复的第一个历元弃用
                std::cout << "[WARNING]: Time discontinuity detected: Time difference exceeds threshold (" << this->dt
                          << " seconds(<604790.0s))." << std::endl;
                this->lastEpochTow = tow;
                return -1;

            } else if (this->dt < -604790.0) { // 简易跨周判断， 不能处理频繁中断的数据
                std::cout << "\n[WARNING]: Time Across The Next Week detected: Time difference exceeds threshold ("
                          << this->dt << " seconds(abs>604790.0s)).\n"
                          << std::endl;
                this->lastEpochTow = tow;
                autoWeek += 1;
                this->dt += 604800.0; // 记作成功解析数据

            } else {
                this->lastEpochTow = tow;
            }
        } else {
            this->dt = 1.0 / this->sampRate;
        }

        this->imudata.time.setTime(autoWeek, tow);
        this->imudata.valid     = true;
        this->imudata.tag       = 0;
        this->imudata.dt        = this->dt;
        this->imudata.odovel    = 0.0;
        this->imudata.dvel(0)   = this->acc[0];
        this->imudata.dvel(1)   = this->acc[1];
        this->imudata.dvel(2)   = this->acc[2];
        this->imudata.dtheta(0) = this->gro[0];
        this->imudata.dtheta(1) = this->gro[1];
        this->imudata.dtheta(2) = this->gro[2];

        if (this->imuCoordType==cmn::RFU) {
            // convert 'right-forward-up' to 'forward-right-down'
            std::swap(this->imudata.dtheta(0), this->imudata.dtheta(1));
            std::swap(this->imudata.dvel(0), this->imudata.dvel(1));
            this->imudata.dtheta(2) = -this->imudata.dtheta(2);
            this->imudata.dvel(2)   = -this->imudata.dvel(2);
        }

        return 1;
    } else {
        this->imudata.valid = false;
        return 0;
    }
}

void ImuTxtLoader::parseImuTxtStatic() {
    // Seek to the beginning of the file
    if (!seekBeginningEpoch()) {
        std::cerr << "[ERROR]: Fail to Seek to the beginning of the imu-txt file" << std::endl;
        return;
    }

    double Acc[3]={0.0,0.0,0.0}, Gro[3]={0.0,0.0,0.0};
    for (int i = 0; i < (int) (staticMinutes * 60 * sampRate);) {
        int result = this->parseNextTxtEpoch();
        if (result != 1) {
            if (result == 0) {
                std::cout << "[INFO]: End of ImuTxt file reached(static period)." << std::endl;
                return ;
            }
            continue;
        }
        Acc[0] = (this->acc[0] + i * Acc[0]) / (i + 1);
        Acc[1] = (this->acc[1] + i * Acc[1]) / (i + 1);
        Acc[2] = (this->acc[2] + i * Acc[2]) / (i + 1);
        Gro[0] = (this->gro[0] + i * Gro[0]) / (i + 1);
        Gro[1] = (this->gro[1] + i * Gro[1]) / (i + 1);
        Gro[2] = (this->gro[2] + i * Gro[2]) / (i + 1);
        i++;
    }

    if (this->imuCoordType==cmn::RFU) {
        // convert 'right-forward-up' to 'forward-right-down'
        std::swap(this->acc[0], this->acc[1]);
        std::swap(this->gro[0], this->gro[1]);
        this->acc[2] = -this->acc[2];
        this->gro[2] = -this->gro[2];
    }
}