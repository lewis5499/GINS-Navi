#include "manager.h"
#include "algebra.h"
#include "fileloader.h"
#include "filesaver.h"
#include "gnsstypes.h"
#include "imutypes.h"
#include "insmech.h"
#include "navstate.h"
#include "timesys.h"


using namespace Manager;
using namespace nav;
using namespace Algebra;
using namespace ins;
using namespace timesys;
using namespace gnss;
using namespace file;
using namespace Eigen;
using namespace std;

GINSManager::GINSManager(GINSOptions &options)
    : options(options) {

    this->options = options;

    if (this->options.estimateImuScale) {
        this->RANK      = Manager::GINSManager::RANK_21;
        this->NOISERANK = Manager::GINSManager::NOISERANK_18;
    } else {
        this->RANK      = Manager::GINSManager::RANK_15;
        this->NOISERANK = Manager::GINSManager::NOISERANK_12;
    }

    // initialize state vector, Q, P
    this->P.resize(this->RANK, this->RANK);
    this->q.resize(this->NOISERANK, this->NOISERANK);
    this->dx.resize(this->RANK, 1);
    this->P.setZero();
    this->q.setZero();
    this->dx.setZero();

    // initialize noise matrix : [equation(50)]
    auto imunoise                                   = this->options.imunoise;
    this->q.block(this->ARW_ID, this->ARW_ID, 3, 3) = imunoise.gyr_arw.cwiseProduct(imunoise.gyr_arw).asDiagonal();
    this->q.block(this->VRW_ID, this->VRW_ID, 3, 3) = imunoise.acc_vrw.cwiseProduct(imunoise.acc_vrw).asDiagonal();
    this->q.block(this->BGSTD_ID, this->BGSTD_ID, 3, 3) =
        2.0 / imunoise.corr_time * imunoise.gyrbias_std.cwiseProduct(imunoise.gyrbias_std).asDiagonal();
    this->q.block(this->BASTD_ID, this->BASTD_ID, 3, 3) =
        2.0 / imunoise.corr_time * imunoise.accbias_std.cwiseProduct(imunoise.accbias_std).asDiagonal();
    if (this->options.estimateImuScale) {
        this->q.block(this->SGSTD_ID, this->SGSTD_ID, 3, 3) =
            2.0 / imunoise.corr_time * imunoise.gyrscale_std.cwiseProduct(imunoise.gyrscale_std).asDiagonal();
        this->q.block(this->SASTD_ID, this->SASTD_ID, 3, 3) =
            2.0 / imunoise.corr_time * imunoise.accscale_std.cwiseProduct(imunoise.accscale_std).asDiagonal();
    }

    // set initial state (position, velocity, attitude and IMU error) and covariance
    this->initialize(this->options.initstate, this->options.initstate_std);
}

void GINSManager::addImuData(const ins::IMU &imu, bool compensate) {

    this->imupre = this->imucur;
    this->imucur = imu;

    if (compensate) {
        this->imuCompensate(this->imucur);
    }
}

void GINSManager::addGnssData(const gnss::GNSS &gnss) {

    this->gnssdata = gnss;
    // do not check the validity of gnssdata, the gnssdata is valid by default
    this->gnssdata.valid = true;
}

void GINSManager::initialize(const NavState &initstate, const NavState &initstate_std) {

    // initialize navstate
    this->navstatecur.pos       = initstate.pos;
    this->navstatecur.vel       = initstate.vel;
    this->navstatecur.att.euler = initstate.att.euler;
    this->navstatecur.att.cbn   = Algebra::euler2dcm(this->navstatecur.att.euler);
    this->navstatecur.att.qbn   = Algebra::euler2quat(this->navstatecur.att.euler);
    this->navstatecur.att.cbe   = Algebra::Cne(initstate.pos) * this->navstatecur.att.cbn;
    this->navstatecur.att.qbe   = Algebra::dcm2quat(this->navstatecur.att.cbe);
    this->navstatecur.imuerror  = initstate.imuerror;

    // set the same value to the previous state
    this->navstatepre = this->navstatecur;

    // initialize covariance matrix: P [21*21]
    this->P.block(this->P_ID, this->P_ID, 3, 3) = initstate_std.pos.cwiseProduct(initstate_std.pos).asDiagonal();
    this->P.block(this->V_ID, this->V_ID, 3, 3) = initstate_std.vel.cwiseProduct(initstate_std.vel).asDiagonal();
    this->P.block(this->PHI_ID, this->PHI_ID, 3, 3) =
        initstate_std.att.euler.cwiseProduct(initstate_std.att.euler).asDiagonal();
    this->P.block(this->BG_ID, this->BG_ID, 3, 3) =
        initstate_std.imuerror.gyrbias.cwiseProduct(initstate_std.imuerror.gyrbias).asDiagonal();
    this->P.block(this->BA_ID, this->BA_ID, 3, 3) =
        initstate_std.imuerror.accbias.cwiseProduct(initstate_std.imuerror.accbias).asDiagonal();
    if (this->options.estimateImuScale) { // P [15*15]
        this->P.block(this->SG_ID, this->SG_ID, 3, 3) =
            initstate_std.imuerror.gyrscale.cwiseProduct(initstate_std.imuerror.gyrscale).asDiagonal();
        this->P.block(this->SA_ID, this->SA_ID, 3, 3) =
            initstate_std.imuerror.accscale.cwiseProduct(initstate_std.imuerror.accscale).asDiagonal();
    }
}

void GINSManager::newImuProcess() {

    // set current IMU time as the current state time
    this->timestamp = this->imucur.time;

    // determine whether use insmech only
    if (this->options.onlyinsmech) {
        // only propagate navigation state
        this->insPropagation(this->imupre, this->imucur);
        // update system state and imudata at the previous epoch
        this->navstatepre = this->navstatecur;
        this->imupre      = this->imucur;
        return;
    }

    // set update time as the gnss time if gnssdata is valid
    Time updatetime = this->gnssdata.valid ? this->gnssdata.time : Time();

    // determine if we should do GNSS update
    int res = this->isToUpdate(this->imupre.time, this->imucur.time, updatetime);

    if (res == 0) {
        // only propagate navigation state
        this->insPropagation(this->imupre, this->imucur);
    } else if (res == 1) {
        // gnssdata is near to the previous imudata, we should firstly do gnss update
        this->gnssUpdate(this->gnssdata);
        this->errorFeedback();

        this->navstatepre = this->navstatecur;
        this->insPropagation(this->imupre, this->imucur);
    } else if (res == 2) {
        // gnssdata is near current imudata, we should firstly propagate navigation state
        this->insPropagation(this->imupre, this->imucur);
        this->gnssUpdate(this->gnssdata);
        this->errorFeedback();
    } else {
        // gnssdata is between the two imudata, we interpolate current imudata to gnss time
        IMU midimu;
        Manager::GINSManager::imuInterpolate(this->imupre, this->imucur, updatetime, midimu);

        // propagate navigation state for the first half imudata
        this->insPropagation(this->imupre, midimu);

        // do GNSS position update at the sampling time and feedback system states
        this->gnssUpdate(this->gnssdata);
        this->errorFeedback();

        // propagate navigation state for the second half imudata
        this->navstatepre = this->navstatecur;
        this->insPropagation(midimu, this->imucur);
    }

    // odo/nhc Update
    if (this->imucur.time > this->odonhcUpdatetime && this->options.useodonhc) {
        this->odonhcUpdate(Vector3d(this->imucur.odovel, 0.0, 0.0));
        this->errorFeedback();
        this->odonhcUpdatetime.setTime(this->imucur.time.getTime().Week,
                                       this->imucur.time.getTime().SecOfWeek + 1.0 / this->options.odonhc_updaterate);
    }

    // zupt Update
    if (this->imucur.time > this->zuptUpdatetime && this->options.useZUPT && this->imucur.tag) {
        this->zuptUpdate();
        this->errorFeedback();
        this->zuptUpdatetime.setTime(this->imucur.time.getTime().Week,
                                     this->imucur.time.getTime().SecOfWeek + 1.0 / this->options.zupt_updaterate);
    }

    // check diagonal elements of current covariance matrix
    this->checkCov();

    // update system state and imudata at the previous epoch
    this->navstatepre = this->navstatecur;
    this->imupre      = this->imucur;
}

void GINSManager::imuCompensate(IMU &imu) {

    // compensate the imu bias
    imu.dtheta -= this->navstatecur.imuerror.gyrbias * imu.dt;
    imu.dvel -= this->navstatecur.imuerror.accbias * imu.dt;

    if (this->options.estimateImuScale) {
        // compensate the imu scale
        Eigen::Vector3d gyrscale, accscale;
        gyrscale   = Eigen::Vector3d::Ones() + this->navstatecur.imuerror.gyrscale;
        accscale   = Eigen::Vector3d::Ones() + this->navstatecur.imuerror.accscale;
        imu.dtheta = imu.dtheta.cwiseProduct(gyrscale.cwiseInverse());
        imu.dvel   = imu.dvel.cwiseProduct(accscale.cwiseInverse());
    }
}

void GINSManager::insPropagation(IMU &preImu, IMU &curImu) {

    // compensate imu error to 'imucur', 'imupre' has been compensated
    this->imuCompensate(curImu);

    // update imustate(mechanization)
    switch (this->options.navSys) {
        case cmn::nFrame: {
            nFrameIns::insMech(this->navstatepre, this->navstatecur, preImu, curImu);
            break;
        }
        case cmn::eFrame: {
            eFrameIns::insMech(this->navstatepre, this->navstatecur, preImu, curImu);
            break;
        }
        default: {
            nFrameIns::insMech(this->navstatepre, this->navstatecur, preImu, curImu);
            break;
        }
    }

    // system noise propagate, phi-angle error model for attitude error
    Eigen::MatrixXd Phi, F, Qk, G;
    Eigen::Vector2d rmrn;
    Eigen::Vector3d win_n, accel, omega;
    Eigen::Matrix3d block3d;
    double gravity, rmh, rnh;

    // initialize Phi (state transition), F matrix, Qk(propagation noise) and G(noise driven) matrix
    Phi.resizeLike(this->P);
    F.resizeLike(this->P);
    Qk.resizeLike(this->P);
    G.resize(this->RANK, this->NOISERANK);
    Phi.setIdentity();
    F.setZero();
    Qk.setZero();
    G.setZero();

    // temp variables using the previous state
    rmrn    = Algebra::RmRn(this->navstatepre.pos[0]);
    gravity = Algebra::NormalGravity(this->navstatepre.pos);
    win_n   = Algebra::inwn(this->navstatepre.pos, this->navstatepre.vel);
    rmh     = rmrn[0] + this->navstatepre.pos[2];
    rnh     = rmrn[1] + this->navstatepre.pos[2];
    accel   = curImu.dvel / curImu.dt;
    omega   = curImu.dtheta / curImu.dt;

    /* position error */
    // Frr [equation(46)]
    block3d.setZero();
    block3d(0, 0) = -this->navstatepre.vel[2] / rmh;
    block3d(0, 2) = this->navstatepre.vel[0] / rmh;
    block3d(1, 0) = this->navstatepre.vel[1] * tan(this->navstatepre.pos[0]) / rnh;
    block3d(1, 1) = -(this->navstatepre.vel[2] + this->navstatepre.vel[0] * tan(this->navstatepre.pos[0])) / rnh;
    block3d(1, 2) = this->navstatepre.vel[1] / rnh;
    F.block(this->P_ID, this->P_ID, 3, 3) = block3d;
    // Frv [equation(45)]
    F.block(this->P_ID, this->V_ID, 3, 3) = Eigen::Matrix3d::Identity();

    /* velocity error */
    // Fvr [equation(47)]
    block3d.setZero();
    block3d(0, 0) = -2 * this->navstatepre.vel[1] * WGS84_WIE * cos(this->navstatepre.pos[0]) / rmh -
                    pow(this->navstatepre.vel[1], 2) / rmh / rnh / pow(cos(this->navstatepre.pos[0]), 2);
    block3d(0, 2) = this->navstatepre.vel[0] * this->navstatepre.vel[2] / rmh / rmh -
                    pow(this->navstatepre.vel[1], 2) * tan(this->navstatepre.pos[0]) / rnh / rnh;
    block3d(1, 0) =
        2 * WGS84_WIE *
            (this->navstatepre.vel[0] * cos(this->navstatepre.pos[0]) -
             this->navstatepre.vel[2] * sin(this->navstatepre.pos[0])) /
            rmh +
        this->navstatepre.vel[0] * this->navstatepre.vel[1] / rmh / rnh / pow(cos(this->navstatepre.pos[0]), 2);
    block3d(1, 2) = (this->navstatepre.vel[1] * this->navstatepre.vel[2] +
                     this->navstatepre.vel[0] * this->navstatepre.vel[1] * tan(this->navstatepre.pos[0])) /
                    rnh / rnh;
    block3d(2, 0) = 2 * WGS84_WIE * this->navstatepre.vel[1] * sin(this->navstatepre.pos[0]) / rmh;
    block3d(2, 2) = -pow(this->navstatepre.vel[1], 2) / rnh / rnh - pow(this->navstatepre.vel[0], 2) / rmh / rmh +
                    2 * gravity / (sqrt(rmrn[0] * rmrn[1]) + this->navstatepre.pos[2]);
    F.block(this->V_ID, this->P_ID, 3, 3) = block3d;
    // Fvv [equation(48)]
    block3d.setZero();
    block3d(0, 0) = this->navstatepre.vel[2] / rmh;
    block3d(0, 1) = -2 * (WGS84_WIE * sin(this->navstatepre.pos[0]) +
                          this->navstatepre.vel[1] * tan(this->navstatepre.pos[0]) / rnh);
    block3d(0, 2) = this->navstatepre.vel[0] / rmh;
    block3d(1, 0) =
        2 * WGS84_WIE * sin(this->navstatepre.pos[0]) + this->navstatepre.vel[1] * tan(this->navstatepre.pos[0]) / rnh;
    block3d(1, 1) = (this->navstatepre.vel[2] + this->navstatepre.vel[0] * tan(this->navstatepre.pos[0])) / rnh;
    block3d(1, 2) = 2 * WGS84_WIE * cos(this->navstatepre.pos[0]) + this->navstatepre.vel[1] / rnh;
    block3d(2, 0) = -2 * this->navstatepre.vel[0] / rmh;
    block3d(2, 1) = -2 * (WGS84_WIE * cos(this->navstatepre.pos(0)) + this->navstatepre.vel[1] / rnh);
    F.block(this->V_ID, this->V_ID, 3, 3) = block3d;
    // Fv_phi [equation(45)]
    F.block(this->V_ID, this->PHI_ID, 3, 3) = Algebra::vector2skewsym(this->navstatepre.att.cbn * accel);
    // Fv_accBias [equation(45)]
    F.block(this->V_ID, this->BA_ID, 3, 3) = this->navstatepre.att.cbn;
    if (this->options.estimateImuScale) {
        // Fv_accScale [equation(45)]
        F.block(this->V_ID, this->SA_ID, 3, 3) = this->navstatepre.att.cbn * (accel.asDiagonal());
    }

    /* attitude error */
    // Fphi_r [equation(49)]
    block3d.setZero();
    block3d(0, 0) = -WGS84_WIE * sin(this->navstatepre.pos[0]) / rmh;
    block3d(0, 2) = this->navstatepre.vel[1] / rnh / rnh;
    block3d(1, 2) = -this->navstatepre.vel[0] / rmh / rmh;
    block3d(2, 0) = -WGS84_WIE * cos(this->navstatepre.pos[0]) / rmh -
                    this->navstatepre.vel[1] / rmh / rnh / pow(cos(this->navstatepre.pos[0]), 2);
    block3d(2, 2)                           = -this->navstatepre.vel[1] * tan(this->navstatepre.pos[0]) / rnh / rnh;
    F.block(this->PHI_ID, this->P_ID, 3, 3) = block3d;
    // Fphi_v [equation(49)]
    block3d.setZero();
    block3d(0, 1)                           = 1 / rnh;
    block3d(1, 0)                           = -1 / rmh;
    block3d(2, 1)                           = -tan(this->navstatepre.pos[0]) / rnh;
    F.block(this->PHI_ID, this->V_ID, 3, 3) = block3d;
    // Fphi_phi [equation(45)]
    F.block(this->PHI_ID, this->PHI_ID, 3, 3) = -Algebra::vector2skewsym(win_n);
    // Fphi_groBias [equation(45)]
    F.block(this->PHI_ID, this->BG_ID, 3, 3) = -this->navstatepre.att.cbn;
    if (this->options.estimateImuScale) {
        // Fphi_groScale [equation(45)]
        F.block(this->PHI_ID, this->SG_ID, 3, 3) = -this->navstatepre.att.cbn * (omega.asDiagonal());
    }

    /* imu bias error and scale error, modeled as the first-order Gauss-Markov process */
    // [equation(45)]
    F.block(this->BG_ID, this->BG_ID, 3, 3) = -1 / this->options.imunoise.corr_time * Eigen::Matrix3d::Identity();
    F.block(this->BA_ID, this->BA_ID, 3, 3) = -1 / this->options.imunoise.corr_time * Eigen::Matrix3d::Identity();
    if (this->options.estimateImuScale) {
        F.block(this->SG_ID, this->SG_ID, 3, 3) = -1 / this->options.imunoise.corr_time * Eigen::Matrix3d::Identity();
        F.block(this->SA_ID, this->SA_ID, 3, 3) = -1 / this->options.imunoise.corr_time * Eigen::Matrix3d::Identity();
    }

    /* system noise driven matrix */
    // [equation(51)]
    G.block(this->V_ID, this->VRW_ID, 3, 3)    = this->navstatepre.att.cbn;
    G.block(this->PHI_ID, this->ARW_ID, 3, 3)  = this->navstatepre.att.cbn;
    G.block(this->BG_ID, this->BGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(this->BA_ID, this->BASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    if (this->options.estimateImuScale) {
        G.block(this->SG_ID, this->SGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
        G.block(this->SA_ID, this->SASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    }

    /* compute the state transition matrix */
    Phi.setIdentity();
    Phi = Phi + F * curImu.dt;

    /* compute system propagation noise */
    Qk = G * this->q * G.transpose() * curImu.dt;
    Qk = (Phi * Qk * Phi.transpose() + Qk) / 2;

    /* do EKF predict to propagate covariance and error state */
    this->EKFPredict(Phi, Qk);
}

void GINSManager::gnssUpdate(GNSS &gnss) {

    // convert IMU position to GNSS antenna phase center position
    Eigen::Vector3d antenna_pos;
    Eigen::Matrix3d Dr, Dr_inv;
    Dr_inv      = Algebra::DRinv(this->navstatecur.pos);
    Dr          = Algebra::DR(this->navstatecur.pos);
    antenna_pos = this->navstatecur.pos + Dr_inv * this->navstatecur.att.cbn * this->options.install_param.antlever;

    // compute GNSS position innovation
    // [equation(31)]
    Eigen::MatrixXd dz;
    dz = Dr * (antenna_pos - gnss.blh);

    // construct GNSS position measurement matrix
    // [equation(33)]
    Eigen::MatrixXd H_gnsspos;
    H_gnsspos.resize(3, this->P.rows());
    H_gnsspos.setZero();
    H_gnsspos.block(0, this->P_ID, 3, 3) = Eigen::Matrix3d::Identity();
    H_gnsspos.block(0, this->PHI_ID, 3, 3) =
        Algebra::vector2skewsym(this->navstatecur.att.cbn * this->options.install_param.antlever);

    // construct measurement noise matrix
    Eigen::MatrixXd R_gnsspos;
    R_gnsspos = gnss.posstd.cwiseProduct(gnss.posstd).asDiagonal();

    // Judge GNSS measurement validity
    if (gnss.posstd[0] > 5 || gnss.posstd[1] > 5 || gnss.posstd[2] > 5) {
        std::cout << "[WARNING]: Abandon gnss position measurement at: " << gnss.time.ToStringGPST() << std::endl;
    } else {
        // If ODO/NHC or ZUPT is enabled, judge each direction measurement validity
        if (this->options.useodonhc || this->options.useZUPT) {
            if (gnss.posstd[0] > 0.9) {
                std::cout << "[WARNING]: Abandon gnss NORTH measurement at: " << gnss.time.ToStringGPST() << std::endl;
                H_gnsspos(0, Eigen::all) = Eigen::VectorXd::Zero(this->P.rows());
                dz(0, Eigen::all)        = Eigen::VectorXd::Zero(1);
            }
            if (gnss.posstd[1] > 0.9) {
                std::cout << "[WARNING]: Abandon gnss EAST measurement at: " << gnss.time.ToStringGPST() << std::endl;
                H_gnsspos(1, Eigen::all) = Eigen::VectorXd::Zero(this->P.rows());
                dz(1, Eigen::all)        = Eigen::VectorXd::Zero(1);
            }
            if (gnss.posstd[2] > 1.8) {
                std::cout << "[WARNING]: Abandon gnss DOWN measurement at: " << gnss.time.ToStringGPST() << std::endl;
                H_gnsspos(2, Eigen::all) = Eigen::VectorXd::Zero(this->P.rows());
                dz(2, Eigen::all)        = Eigen::VectorXd::Zero(1);
            }
        }

        // do EKF update to update covariance and error state
        this->EKFUpdate(dz, H_gnsspos, R_gnsspos);
    }

    // If GNSS velocity update is enabled
    if (this->options.usegnssvel) {
        // Judge GNSS velocity measurement validity
        if (gnss.velstd[0] > 0.5 || gnss.velstd[1] > 0.5 || gnss.velstd[2] > 0.5) {
            std::cout << "[WARNING]: Abandon gnss velocity measurement at: " << gnss.time.ToStringGPST() << std::endl;
        } else {
            // Compute GNSS velocity innovation
            // [equation(34)]
            Eigen::Vector3d win_n = Algebra::inwn(this->navstatecur.pos, this->navstatecur.vel);
            Eigen::Vector3d wib_b = this->imucur.dtheta / this->imucur.dt;
            dz                    = this->navstatecur.vel - gnss.vel -
                 Algebra::vector2skewsym(win_n) * this->navstatecur.att.cbn * this->options.install_param.antlever -
                 this->navstatecur.att.cbn * Algebra::vector2skewsym(this->options.install_param.antlever) * wib_b;

            // Construct GNSS velocity measurement matrix
            Eigen::MatrixXd Hv3, Hv6, H_gnssvel;
            Hv3 = -Algebra::vector2skewsym(win_n) *
                      Algebra::vector2skewsym(this->navstatecur.att.cbn * this->options.install_param.antlever) -
                  Algebra::vector2skewsym(this->navstatecur.att.cbn *
                                          Algebra::vector2skewsym(this->options.install_param.antlever) * wib_b);
            if (this->options.estimateImuScale) {
                Hv6 = -this->navstatecur.att.cbn * Algebra::vector2skewsym(this->options.install_param.antlever) *
                      wib_b.asDiagonal();
            }
            H_gnssvel.resize(3, this->P.rows());
            H_gnssvel.setZero();
            H_gnssvel.block(0, this->V_ID, 3, 3)   = Eigen::Matrix3d::Identity();
            H_gnssvel.block(0, this->PHI_ID, 3, 3) = Hv3;
            H_gnssvel.block(0, this->BG_ID, 3, 3) =
                -this->navstatecur.att.cbn * Algebra::vector2skewsym(this->options.install_param.antlever);
            if (this->options.estimateImuScale) {
                H_gnssvel.block(0, this->SG_ID, 3, 3) = Hv6;
            }

            // Construct GNSS velocity measurement noise matrix
            Eigen::MatrixXd R_gnssvel;
            R_gnssvel = gnss.velstd.cwiseProduct(gnss.velstd).asDiagonal();

            // If ODO/NHC or ZUPT is enabled, judge each direction measurement validity
            if (this->options.useodonhc || this->options.useZUPT) {
                if (gnss.velstd[0] > 0.1) {
                    std::cout << "[WARNING]: Abandon gnss vn measurement at: " << gnss.time.ToStringGPST() << std::endl;
                    H_gnssvel(0, Eigen::all) = Eigen::VectorXd::Zero(this->P.rows());
                    dz(0, Eigen::all)        = Eigen::VectorXd::Zero(1);
                }
                if (gnss.velstd[1] > 0.1) {
                    std::cout << "[WARNING]: Abandon gnss ve measurement at: " << gnss.time.ToStringGPST() << std::endl;
                    H_gnssvel(1, Eigen::all) = Eigen::VectorXd::Zero(this->P.rows());
                    dz(1, Eigen::all)        = Eigen::VectorXd::Zero(1);
                }
                if (gnss.velstd[2] > 0.3) {
                    std::cout << "[WARNING]: Abandon gnss vd measurement at: " << gnss.time.ToStringGPST() << std::endl;
                    H_gnssvel(2, Eigen::all) = Eigen::VectorXd::Zero(this->P.rows());
                    dz(2, Eigen::all)        = Eigen::VectorXd::Zero(1);
                }
            }

            // do EKF update to update covariance and error state
            this->EKFUpdate(dz, H_gnssvel, R_gnssvel);
        }
    }

    // Set GNSS invalid after update
    gnss.valid = false;
}

void GINSManager::odonhcUpdate(const Eigen::Vector3d &odonhc_vel) {
    Eigen::Vector3d wib_b = this->imucur.dtheta / this->imucur.dt;
    Eigen::Vector3d win_n = Algebra::inwn(this->navstatecur.pos, this->navstatecur.vel);
    Eigen::Vector3d win_b = this->navstatecur.att.cbn.transpose() * win_n;
    Eigen::Vector3d wnb_b = wib_b - win_b;

    Matrix3d Cbv = euler2dcm(this->options.install_param.installangle);

    // compute ODO/NHC vel innovation
    Eigen::MatrixXd dz;
    dz = Cbv * this->navstatecur.att.cbn.transpose() * this->navstatecur.vel +
         Cbv * Algebra::vector2skewsym(wnb_b) * this->options.install_param.odolever - odonhc_vel;

    // construct ODO/NHC measurement matrix
    Eigen::MatrixXd H_odonhc;
    H_odonhc.resize(3, this->P.rows());
    H_odonhc.setZero();
    H_odonhc.block(0, this->V_ID, 3, 3) = Cbv * this->navstatecur.att.cbn.transpose();
    H_odonhc.block(0, this->PHI_ID, 3, 3) =
        -Cbv * this->navstatecur.att.cbn.transpose() * Algebra::vector2skewsym(this->navstatecur.vel);
    H_odonhc.block(0, this->BG_ID, 3, 3) = -Cbv * Algebra::vector2skewsym(this->options.install_param.odolever);
    if (this->options.estimateImuScale) {
        H_odonhc.block(0, this->SG_ID, 3, 3) =
            -Cbv * Algebra::vector2skewsym(this->options.install_param.odolever) * wib_b.asDiagonal();
    }

    // construct ODO/NHC measurement noise matrix
    Eigen::MatrixXd R_odonhc;
    R_odonhc =
        this->options.meas_noise.odonhc_measnoise.cwiseProduct(this->options.meas_noise.odonhc_measnoise).asDiagonal();

    // judge whether use single ODO/NHC
    if (this->options.usesinglenhc && !this->options.usesingleodo) {
        dz       = dz.block(1, 0, 2, 1);
        R_odonhc = R_odonhc.block(1, 1, 2, 2);
        H_odonhc = H_odonhc.block(1, 0, 2, P.rows());
    } else if (!this->options.usesinglenhc && this->options.usesingleodo) {
        dz       = dz.block(0, 0, 1, 1);
        R_odonhc = R_odonhc.block(0, 0, 1, 1);
        H_odonhc = H_odonhc.block(0, 0, 1, P.rows());
    }

    // do EKF update to update covariance and error state
    EKFUpdate(dz, H_odonhc, R_odonhc);
}

int GINSManager::isToUpdate(const Time &imutime1, const Time &imutime2, const Time &updatetime) const {

    if (updatetime.getTime().Week == 0 && abs(updatetime.getTime().SecOfWeek) < 1e-5) {
        return 0;
    }

    if (abs(imutime1 - updatetime) < this->TIME_ALIGN_ERR) {
        // updatetime is near to imutime1
        return 1;
    } else if (abs(imutime2 - updatetime) <= this->TIME_ALIGN_ERR) {
        // updatetime is near to imutime2
        return 2;
    } else if (imutime1 < updatetime && updatetime < imutime2) {
        // updatetime is between imutime1 and imutime2, but not near to either
        return 3;
    } else {
        // updatetime is not bewteen imutime1 and imutime2, and not near to either.
        return 0;
    }
}

void GINSManager::EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd) {

    assert(Phi.rows() == this->P.rows());
    assert(Qd.rows() == this->P.rows());

    // propagate system covariance and error state
    this->P  = Phi * this->P * Phi.transpose() + Qd;
    this->dx = Phi * this->dx;
}

void GINSManager::EKFUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H, Eigen::MatrixXd &R) {

    assert(H.cols() == this->P.rows());
    assert(dz.rows() == H.rows());
    assert(dz.rows() == R.rows());
    assert(dz.cols() == 1);

    // Compute Kalman Gain
    auto temp         = H * this->P * H.transpose() + R;
    Eigen::MatrixXd K = this->P * H.transpose() * temp.inverse();

    // update system error state and covariance
    Eigen::MatrixXd I;
    I.resizeLike(this->P);
    I.setIdentity();
    I = I - K * H;

    // if state feedback is performed after every update, dx_ is always zero before the update
    // the following formula can be simplified as : dx_ = K * dz;
    this->dx = this->dx + K * (dz - H * this->dx);
    this->P  = I * this->P * I.transpose() + K * R * K.transpose();
}

void GINSManager::errorFeedback() {

    Eigen::Vector3d vectemp;

    // posisiton error feedback
    Eigen::Vector3d delta_r = this->dx.block(this->P_ID, 0, 3, 1);
    Eigen::Matrix3d Dr_inv  = Algebra::DRinv(this->navstatecur.pos);
    this->navstatecur.pos -= Dr_inv * delta_r;

    // velocity error feedback
    vectemp = this->dx.block(this->V_ID, 0, 3, 1);
    this->navstatecur.vel -= vectemp;

    // attitude error feedback
    vectemp                     = this->dx.block(this->PHI_ID, 0, 3, 1);
    Eigen::Quaterniond qpn      = Algebra::vector2quat(vectemp);
    this->navstatecur.att.qbn   = qpn * this->navstatecur.att.qbn;
    this->navstatecur.att.cbn   = Algebra::quat2dcm(this->navstatecur.att.qbn);
    this->navstatecur.att.euler = Algebra::dcm2euler(this->navstatecur.att.cbn);

    // IMU bias error feedback
    vectemp = this->dx.block(this->BG_ID, 0, 3, 1);
    this->navstatecur.imuerror.gyrbias += vectemp;
    vectemp = this->dx.block(this->BA_ID, 0, 3, 1);
    this->navstatecur.imuerror.accbias += vectemp;

    if (this->options.estimateImuScale) {
        // IMU scale error feedback
        vectemp = this->dx.block(this->SG_ID, 0, 3, 1);
        this->navstatecur.imuerror.gyrscale += vectemp;
        vectemp = this->dx.block(this->SA_ID, 0, 3, 1);
        this->navstatecur.imuerror.accscale += vectemp;
    }

    // set 'dx' to zero after feedback error state to system state
    this->dx.setZero();
}

void GINSManager::checkCov() {

    for (int i = 0; i < this->RANK; i++) {
        if (this->P(i, i) < 0) {
            std::cout << "Covariance is negative at " << std::setprecision(10) << this->timestamp.getTime().Week << " "
                      << this->timestamp.getTime().SecOfWeek << " !" << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }
}

void GINSManager::imuInterpolate(const ins::IMU &imu1, ins::IMU &imu2, const timesys::Time &timestamp,
                                 ins::IMU &midimu) {

    if (imu1.time > timestamp || imu2.time < timestamp) {
        return;
    }

    double lamda = (timestamp - imu1.time) / (imu2.time - imu1.time);

    midimu.time   = timestamp;
    midimu.dtheta = imu2.dtheta * lamda;
    midimu.dvel   = imu2.dvel * lamda;
    midimu.dt     = timestamp - imu1.time;

    imu2.dtheta = imu2.dtheta - midimu.dtheta;
    imu2.dvel   = imu2.dvel - midimu.dvel;
    imu2.dt     = imu2.dt - midimu.dt;
}

timesys::Time GINSManager::getTimestamp() const {
    return this->timestamp;
}

NavState GINSManager::getNavState() const {

    NavState state;

    state.pos       = this->navstatecur.pos;
    state.vel       = this->navstatecur.vel;
    state.att.euler = this->navstatecur.att.euler;
    state.imuerror  = this->navstatecur.imuerror;

    return state;
}

Eigen::MatrixXd GINSManager::getCovariance() const {
    return this->P;
}

void GINSManager::zuptUpdate() {

    if (this->imucur.tag == 1 || this->imucur.tag == 3) {
        // ZUPT - Zero velocity update
        Eigen::Vector3d dz_zupt_vec = this->navstatecur.vel - Eigen::Vector3d::Zero();
        Eigen::MatrixXd dz_zupt     = dz_zupt_vec;
        Eigen::MatrixXd R_zupt =
            this->options.meas_noise.zupt_vmeasnoise.cwiseProduct(this->options.meas_noise.zupt_vmeasnoise)
                .asDiagonal();
        Eigen::MatrixXd H_zupt            = Eigen::MatrixXd::Zero(3, this->P.rows());
        H_zupt.block(0, this->V_ID, 3, 3) = Eigen::Matrix3d::Identity();

        // EKF update for zero velocity
        EKFUpdate(dz_zupt, H_zupt, R_zupt);
    }

    if (this->imucur.tag == 1 || this->imucur.tag == 2) {
        // ZARU - Zero angular rate update
        Eigen::Vector3d wib_b   = this->imucur.dtheta / this->imucur.dt;
        Eigen::Vector3d win_n   = Algebra::inwn(this->navstatecur.pos, this->navstatecur.vel);
        Eigen::Vector3d win_b   = this->navstatecur.att.cbn.transpose() * win_n;
        Eigen::MatrixXd dz_zaru = Eigen::MatrixXd::Zero(1, 1);
        dz_zaru(0, 0)           = wib_b[2] - win_b[2] - 0;

        Eigen::MatrixXd R_zaru     = Eigen::MatrixXd::Identity(1, 1) * pow(this->options.meas_noise.zupt_wmeasnoise, 2);
        Eigen::MatrixXd H_zaru     = Eigen::MatrixXd::Zero(1, this->P.rows());
        H_zaru(0, this->BG_ID + 2) = 1;
        if (this->options.estimateImuScale) {
            H_zaru(0, this->SG_ID + 2) = wib_b[2];
        }

        // EKF update for zero angular rate
        EKFUpdate(dz_zaru, H_zaru, R_zaru);
    }
}

int GINSManager::detectZUPT(const vector<IMU> &imudata, int imuindex, const Vector3d &blh, int window_len) {
    int state   = 0;
    int halfnum = (int) (window_len / 2.0);

    bool zuptacc_norm   = false;
    bool zuptacc_sigma2 = false;
    bool zuptgyr_length = false;
    bool zuptgyr_std    = false;

    if (imuindex < imudata.size()) {
        double g = Algebra::NormalGravity(blh);

        const IMU &thisimu = imudata[imuindex];
        Vector3d acc       = thisimu.dvel / thisimu.dt;
        Vector3d gyr       = thisimu.dtheta / thisimu.dt;

        // Acceleration amplitude detection
        if (abs(acc.norm() - g) < 0.01) {
            zuptacc_norm = true;
        }

        // Acceleration sliding window variance detection
        if (imuindex > halfnum && imuindex < imudata.size() - halfnum) {
            bool flag[3] = {false, false, false};
            for (int k = 0; k < 3; ++k) {
                double sum_a    = 0;
                double sigma2_a = 0;
                for (int i = imuindex - halfnum; i <= imuindex + halfnum; ++i) {
                    sum_a += imudata[i].dvel(k);
                }
                double ave = sum_a / (halfnum * 2 + 1);
                for (int i = imuindex - halfnum; i <= imuindex + halfnum; ++i) {
                    sigma2_a += pow(imudata[i].dvel(k) - ave, 2);
                }
                sigma2_a /= (halfnum * 2 + 1);
                if (sigma2_a < 0.01) {
                    flag[k] = true;
                }
            }
            if (flag[0] && flag[1] && flag[2]) {
                zuptacc_sigma2 = true;
            }
        }

        // Angular velocity amplitude detection
        if (gyr.norm() < 0.005) { // rad
            zuptgyr_length = true;
        }

        // Angular velocity sliding window detection
        if (imuindex > halfnum && imuindex < imudata.size() - halfnum) {
            bool flag[3] = {false, false, false};
            for (int k = 0; k < 3; ++k) {
                double sum_w    = 0;
                double sigma2_w = 0;
                for (int i = imuindex - halfnum; i <= imuindex + halfnum; ++i) {
                    sum_w += imudata[i].dtheta(k);
                }
                double ave = sum_w / (halfnum * 2 + 1);
                for (int i = imuindex - halfnum; i <= imuindex + halfnum; ++i) {
                    sigma2_w += pow(imudata[i].dtheta(k) - ave, 2);
                }
                sigma2_w /= (halfnum * 2 + 1);
                if (sigma2_w < 0.005) {
                    flag[k] = true;
                }
            }
            if (flag[0] && flag[1] && flag[2]) {
                zuptgyr_std = true;
            }
        }
    }

    if (zuptacc_norm && zuptacc_sigma2 && zuptgyr_length && zuptgyr_std) {
        state = 1;
    } else if (zuptgyr_length && zuptgyr_std) {
        state = 2;
    } else if (zuptacc_norm && zuptacc_sigma2) {
        state = 3;
    } else {
        state = 0;
    }

    return state;
}

void GINSManager::writeResult(NaviFileSaver &navResFile, NaviFileSaver &imuErrFile, const NavState &navstate,
                              const Time &timeStamp, const MatrixXd &cov) const {
    if (this->options.estimateImuScale) {
        navResFile.writeData(timeStamp.getTime().Week, timeStamp.getTime().SecOfWeek, navstate.pos(0) * r2d,
                             navstate.pos(1) * r2d, navstate.pos(2), "2", "20", SQRT_(cov(0, 0)), SQRT_(cov(1, 1)),
                             SQRT_(cov(2, 2)), SQRT_(cov(0, 1)), SQRT_(cov(1, 2)), SQRT_(cov(2, 0)), 0.0, 1.0,
                             navstate.vel(0), navstate.vel(1), navstate.vel(2), SQRT_(cov(3, 3)), SQRT_(cov(4, 4)),
                             SQRT_(cov(5, 5)), navstate.att.euler(0) * r2d, navstate.att.euler(1) * r2d,
                             navstate.att.euler(2) * r2d, SQRT_(cov(6, 6)) * r2d, SQRT_(cov(7, 7)) * r2d,
                             SQRT_(cov(8, 8)) * r2d);

        imuErrFile.writeData(
            timeStamp.getTime().Week, timeStamp.getTime().SecOfWeek, navstate.imuerror.gyrbias(0) * r2d * 3600,
            navstate.imuerror.gyrbias(1) * r2d * 3600, navstate.imuerror.gyrbias(2) * r2d * 3600,
            navstate.imuerror.accbias(0) * 1e5, navstate.imuerror.accbias(1) * 1e5, navstate.imuerror.accbias(2) * 1e5,
            navstate.imuerror.gyrscale(0) * 1e6, navstate.imuerror.gyrscale(1) * 1e6,
            navstate.imuerror.gyrscale(2) * 1e6, navstate.imuerror.accscale(0) * 1e6,
            navstate.imuerror.accscale(1) * 1e6, navstate.imuerror.accscale(2) * 1e6, SQRT_(cov(9, 9)) * r2d * 3600,
            SQRT_(cov(10, 10)) * r2d * 3600, SQRT_(cov(11, 11)) * r2d * 3600, SQRT_(cov(12, 12)) * 1e5,
            SQRT_(cov(13, 13)) * 1e5, SQRT_(cov(14, 14)) * 1e5, SQRT_(cov(15, 15)) * 1e6, SQRT_(cov(16, 16)) * 1e6,
            SQRT_(cov(17, 17)) * 1e6, SQRT_(cov(18, 18)) * 1e6, SQRT_(cov(19, 19)) * 1e6, SQRT_(cov(20, 20)) * 1e6);
    } else {
        navResFile.writeData(timeStamp.getTime().Week, timeStamp.getTime().SecOfWeek, navstate.pos(0) * r2d,
                             navstate.pos(1) * r2d, navstate.pos(2), "2", "20", SQRT_(cov(0, 0)), SQRT_(cov(1, 1)),
                             SQRT_(cov(2, 2)), SQRT_(cov(0, 1)), SQRT_(cov(1, 2)), SQRT_(cov(2, 0)), 0.0, 1.0,
                             navstate.vel(0), navstate.vel(1), navstate.vel(2), SQRT_(cov(3, 3)), SQRT_(cov(4, 4)),
                             SQRT_(cov(5, 5)), navstate.att.euler(0) * r2d, navstate.att.euler(1) * r2d,
                             navstate.att.euler(2) * r2d, SQRT_(cov(6, 6)) * r2d, SQRT_(cov(7, 7)) * r2d,
                             SQRT_(cov(8, 8)) * r2d);

        imuErrFile.writeData(timeStamp.getTime().Week, timeStamp.getTime().SecOfWeek,
                             navstate.imuerror.gyrbias(0) * r2d * 3600, navstate.imuerror.gyrbias(1) * r2d * 3600,
                             navstate.imuerror.gyrbias(2) * r2d * 3600, navstate.imuerror.accbias(0) * 1e5,
                             navstate.imuerror.accbias(1) * 1e5, navstate.imuerror.accbias(2) * 1e5, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, SQRT_(cov(9, 9)) * r2d * 3600, SQRT_(cov(10, 10)) * r2d * 3600,
                             SQRT_(cov(11, 11)) * r2d * 3600, SQRT_(cov(12, 12)) * 1e5, SQRT_(cov(13, 13)) * 1e5,
                             SQRT_(cov(14, 14)) * 1e5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
}

void GINSManager::setInitUpateTime() {
    this->odonhcUpdatetime.setTime(this->gnssdata.time.getTime().Week, this->gnssdata.time.getTime().SecOfWeek + 0.5);
    this->zuptUpdatetime.setTime(this->gnssdata.time.getTime().Week, this->gnssdata.time.getTime().SecOfWeek + 0.75);
}

void GINSOptions::RenewOptions(const BasicParams &filePaths) {
    try {
        cmn::parseGPSTimeString(filePaths.timeStart, this->start_week, this->start_tow);
        cmn::parseGPSTimeString(filePaths.timeEnd, this->end_week, this->end_tow);
        this->o_renixrovpath = filePaths.rnxObsRov;
        this->o_renixbaspath = filePaths.rnxObsBas;
        this->n_renixnavpath = filePaths.rnxNav;
        this->odofilepath    = filePaths.odoPath;

        switch (cmn::ImuFormatType(this->imudataformat)) {
            case cmn::IMR: {
                this->imrfilepath = filePaths.imuPath;
                break;
            }
            case cmn::ASC: {
                this->ascfilepath = filePaths.imuPath;
                break;
            }
            case cmn::TXT: {
                this->imutxtfilepath = filePaths.imuPath;
                break;
            }
            default:
                break;
        }

        this->outputpath   = filePaths.outputPath;
        this->gnssfilepath = filePaths.rtkSolPath;
    } catch (std::exception &e) {
        std::cerr << e.what() << endl;
        std::fflush(stderr);
    }
}

void GINSOptions::RenewOptions(ImuFileLoader &imu, GnssFileLoader &gnss, const vector<double> &initAtt) {
    Time cfgStartTime = Time((uint16_t) this->start_week, this->start_tow);

    imu.seekBeginningEpoch();
    gnss.seekBeginningEpoch();

    // find first valid epoch
    GNSS gnssStart = gnss.getCurrentGNSS();
    IMU imuStart   = imu.getCurrentIMU();

    // load first gnss/imu epoch
    while (!gnssStart.valid) {
        gnssStart = gnss.getNextGNSS();
    }
    while (!imuStart.valid) {
        imuStart = imu.getNextIMU();
    }
    if (imu.isEOF() || gnss.isEOF()) {
        std::cerr << "[ERROR] : gnss/imu data error! cannot load valid data." << std::endl;
    }

    // Determine the intersection of the time periods
    // [default 'week' param: false]: set gnsstime->week = imutime->week (GPST)
    // In Func'diffTime', we donn't compare the week, or you can modify it to 'true'
    Time startTime = (imuStart.time.diffTime(gnssStart.time) > 0) ? imuStart.time : gnssStart.time;
    startTime      = (startTime.diffTime(cfgStartTime) > 0) ? startTime : cfgStartTime;

    // Update the GINSOptions with the intersection time period
    this->start_week = startTime.getTime().Week;
    this->start_tow  = startTime.getTime().SecOfWeek;

    // Update the GINSOptions with the initial attitude if rough attitude is used
    if (this->useRoughAtt) {
        this->initstate.att.euler(0) = initAtt[0];
        this->initstate.att.euler(1) = initAtt[1];
        this->initstate.att.euler(2) = initAtt[2];
        this->initstate.att.cbn      = Algebra::euler2dcm(this->initstate.att.euler);
        this->initstate.att.qbn      = Algebra::euler2quat(this->initstate.att.euler);
    }

    // Update the GINSOptions with the initial blh and vel from GNSS
    if (this->useInitalPosVelFromRTK) {
        this->initstate.pos = gnssStart.blh;
        this->initstate.vel = gnssStart.vel;
    }

    // Find the GNSS/IMU data element closest to and greater than or equal to the start time
    while (gnssStart.time.diffTime(startTime) < 0) {
        gnssStart = gnss.getNextGNSS();
    }
    while (imuStart.time.diffTime(startTime) < 0) {
        imuStart = imu.getNextIMU();
    }
}

uint16_t GINSOptions::getImuStartWeek() const {
    return imuStartGPSTWeek;
}
bool GINSOptions::getUsePlotResults() const {
    return plotResults;
}
bool GINSOptions::getUseImuScale() const {
    return estimateImuScale;
}
int GINSOptions::getImuCoordType() const {
    return imuCoordType;
}
int GINSOptions::getStartWeek() const {
    return start_week;
}
int GINSOptions::getEndWeek() const {
    return end_week;
}
double GINSOptions::getStartTow() const {
    return start_tow;
}
double GINSOptions::getEndTow() const {
    return end_tow;
}
double GINSOptions::getImuInitStaticTime() const {
    return imuInitStaticTime;
}
double GINSOptions::getImuSamplingRate() const {
    return imuSamplingRate;
}
const NavState &GINSOptions::getInitState() const {
    return initstate;
}
const NavState &GINSOptions::getInitStateStd() const {
    return initstate_std;
}
const ImuNoise &GINSOptions::getImuNoise() const {
    return imunoise;
}
const InstallParam &GINSOptions::getInstallParam() const {
    return install_param;
}
const MeasureNoise &GINSOptions::getMeasNoise() const {
    return meas_noise;
}
bool GINSOptions::getUseodonhc() const {
    return useodonhc;
}
bool GINSOptions::getUseZUPT() const {
    return useZUPT;
}
bool GINSOptions::getUsegnssvel() const {
    return usegnssvel;
}
bool GINSOptions::getUseSinglenhc() const {
    return usesinglenhc;
}
bool GINSOptions::getUseSingleodo() const {
    return usesingleodo;
}
const string &GINSOptions::getOutputPath() const {
    return outputpath;
}
const string &GINSOptions::getImuFilePath() const {
    if (cmn::ImuFormatType(this->getImuDataFormat()) == 1) {
        return ascfilepath;
    } else if (cmn::ImuFormatType(this->getImuDataFormat()) == 2) {
        return imutxtfilepath;
    } else if (cmn::ImuFormatType(this->getImuDataFormat()) == 0) {
        return imrfilepath;
    } else {
        return imrfilepath; // default: imr format
    };
}
const string &GINSOptions::getGnssFilePath() const {
    return gnssfilepath;
}
const string &GINSOptions::getOdoFilePath() const {
    return odofilepath;
}
const string &GINSOptions::getRnxRovObspath() const {
    return o_renixrovpath;
}
const string &GINSOptions::getRnxBasObspath() const {
    return o_renixbaspath;
}
const string &GINSOptions::getRnxNaviPath() const {
    return n_renixnavpath;
}
const string &GINSOptions::getImuDataFormat() const {
    return imudataformat;
}
const string &GINSOptions::getImuRawCoordinate() const {
    return imurawcoordinate;
}
void GINSOptions::parseOptions() {
    initstate.pos               = readVector("initpos");
    initstate.vel               = readVector("initvel");
    initstate.att.euler         = readVector("initatt");
    initstate.imuerror.gyrbias  = readVector("initgyrbias");
    initstate.imuerror.accbias  = readVector("initaccbias");
    initstate.imuerror.gyrscale = readVector("initgyrscale");
    initstate.imuerror.accscale = readVector("initaccscale");

    initstate_std.pos               = readVector("initposstd");
    initstate_std.vel               = readVector("initvelstd");
    initstate_std.att.euler         = readVector("initattstd");
    initstate_std.imuerror.gyrbias  = readVector("initgyrbiasstd");
    initstate_std.imuerror.accbias  = readVector("initaccbiasstd");
    initstate_std.imuerror.gyrscale = readVector("initgyrscalestd");
    initstate_std.imuerror.accscale = readVector("initaccscalestd");

    imunoise.gyr_arw      = readVector("gyrarw");
    imunoise.acc_vrw      = readVector("accvrw");
    imunoise.gyrbias_std  = readVector("gyrbiasstd");
    imunoise.accbias_std  = readVector("accbiasstd");
    imunoise.gyrscale_std = readVector("gyrscalestd");
    imunoise.accscale_std = readVector("accscalestd");
    imunoise.corr_time    = std::stod(gins_conf_manager.getConfig("corrtime"));

    install_param.antlever     = readVector("antlever");
    install_param.odolever     = readVector("odolever");
    install_param.installangle = readVector("installangle");

    meas_noise.codeNoise         = readDouble("CodeNoise");
    meas_noise.carrierPhaseNoise = readDouble("CPNoise");
    meas_noise.zupt_wmeasnoise   = readDouble("zupt_wmeasnoise");
    meas_noise.zupt_vmeasnoise   = readVector("zupt_vmeasnoise");
    meas_noise.odonhc_measnoise  = readVector("odonhc_measnoise");

    odonhc_updaterate = readDouble("odonhcupdaterate");
    zupt_updaterate   = readDouble("zuptupdaterate");

    if (readString("starttow") != "auto") {
        start_tow = readDouble("starttow");
    }
    if (readString("endtow") != "auto") {
        end_tow = readDouble("endtow");
    }
    if (readString("startweek") != "auto") {
        start_week = (int) readDouble("startweek");
    }
    if (readString("endweek") != "auto") {
        end_week = (int) readDouble("endweek");
    }
    imuStartGPSTWeek = (uint16_t) readDouble("imuInitialGPSTWeek");

    useodonhc              = readBool("useodonhc");
    usegnssvel             = readBool("usegnssvel");
    useZUPT                = readBool("useZUPT");
    usesinglenhc           = readBool("usesinglenhc");
    usesingleodo           = readBool("usesingleodo");
    useRoughAtt            = readBool("useAttFromRoughAlign");
    useInitalPosVelFromRTK = readBool("useInitalPosVelFromRTK");
    estimateImuScale       = readBool("estimateImuScale");
    onlyinsmech            = readBool("onlyinsmech");
    plotResults            = readBool("plotResults");
    imudataformat          = readString("imudataformat");
    imurawcoordinate       = readString("imurawcoordinate");
    imuInitStaticTime      = readDouble("imuInitStaticTime");
    imuSamplingRate        = readDouble("imuSamplingRate");

    imrfilepath    = gins_conf_manager.getConfig("imrfilepath");
    gnssfilepath   = gins_conf_manager.getConfig("gnssfilepath");
    odofilepath    = gins_conf_manager.getConfig("odofilepath");
    ascfilepath    = gins_conf_manager.getConfig("ascfilepath");
    imutxtfilepath = gins_conf_manager.getConfig("imutxtfilepath");
    o_renixrovpath = gins_conf_manager.getConfig("file-roverobs-renix");
    o_renixbaspath = gins_conf_manager.getConfig("file-baseobs-renix");
    n_renixnavpath = gins_conf_manager.getConfig("file-navi-renix");
    outputpath     = gins_conf_manager.getConfig("outputpath");

    imuCoordType = (strcasecmp(imurawcoordinate.c_str(), "FRD") == 0) ? cmn::FRD : cmn::RFU; // 1: FRD, 2: RFU
    navSys       = strcasecmp(readString("navSys").c_str(), "nFrame") == 0 ? cmn::nFrame : cmn::eFrame;

    this->unitConvert();
}

void GINSOptions::printOptions() const {
    std::cout << "------------------------------GINS Options------------------------------" << std::endl << std::endl;
    std::cout << " - Initial State: " << std::endl;
    std::cout << '\t' << "- initial position: " << std::setprecision(12) << initstate.pos[0] * r2d << "  "
              << std::setprecision(12) << initstate.pos[1] * r2d << "  " << std::setprecision(6) << initstate.pos[2]
              << " [deg, deg, m] " << std::endl;
    std::cout << '\t' << "- initial velocity: " << initstate.vel.transpose() << " [m/s] " << std::endl;
    std::cout << '\t' << "- initial attitude: " << initstate.att.euler.transpose() * r2d << " [deg] " << std::endl;
    std::cout << setiosflags(ios::fixed) << setprecision(3);
    std::cout << '\t' << "- initial gyrbias : " << initstate.imuerror.gyrbias.transpose() * (r2d * 3600.0)
              << " [deg/h] " << std::endl;
    std::cout << '\t' << "- initial accbias : " << initstate.imuerror.accbias.transpose() * 1e5 << " [mGal] "
              << std::endl;
    std::cout << '\t' << "- initial gyrscale: " << initstate.imuerror.gyrscale.transpose() * 1e6 << " [ppm] "
              << std::endl;
    std::cout << '\t' << "- initial accscale: " << initstate.imuerror.accscale.transpose() * 1e6 << " [ppm] "
              << std::endl
              << std::endl;

    std::cout << " - Initial State Std: " << std::endl;
    std::cout << '\t' << "- initial position std: " << initstate_std.pos.transpose() << " [m, m, m] " << std::endl;
    std::cout << '\t' << "- initial velocity std: " << initstate_std.vel.transpose() << " [m/s] " << std::endl;
    std::cout << '\t' << "- initial attitude std: " << initstate_std.att.euler.transpose() * r2d << " [deg] "
              << std::endl;
    std::cout << '\t' << "- initial gyrbias  std: " << initstate_std.imuerror.gyrbias.transpose() * (r2d * 3600.0)
              << " [deg/h] " << std::endl;
    std::cout << '\t' << "- initial accbias  std: " << initstate_std.imuerror.accbias.transpose() * 1e5 << " [mGal] "
              << std::endl;
    std::cout << '\t' << "- initial gyrscale std: " << initstate_std.imuerror.gyrscale.transpose() * 1e6 << " [ppm] "
              << std::endl;
    std::cout << '\t' << "- initial accscale std: " << initstate_std.imuerror.accscale.transpose() * 1e6 << " [ppm] "
              << std::endl
              << std::endl;

    std::cout << " - IMU Noise: " << std::endl;
    std::cout << '\t' << "- gyrarw: " << imunoise.gyr_arw.transpose() * (r2d * 60.0) << " [deg/sqrt(h)] " << std::endl;
    std::cout << '\t' << "- accvrw: " << imunoise.acc_vrw.transpose() * 60.0 << " [m/s/sqrt(h)] " << std::endl;
    std::cout << '\t' << "- gyrbias  std: " << imunoise.gyrbias_std.transpose() * (r2d * 3600.0) << " [deg/h] "
              << std::endl;
    std::cout << '\t' << "- accbias  std: " << imunoise.accbias_std.transpose() * 1e5 << " [mGal] " << std::endl;
    std::cout << '\t' << "- gyrscale std: " << imunoise.gyrscale_std.transpose() * 1e6 << " [ppm] " << std::endl;
    std::cout << '\t' << "- accscale std: " << imunoise.accscale_std.transpose() * 1e6 << " [ppm] " << std::endl;
    std::cout << '\t' << "- correlation time: " << imunoise.corr_time / 3600.0 << " [h] " << std::endl << std::endl;

    std::cout << " - Installation Parameters: " << std::endl;
    std::cout << '\t' << "- antenna  leverarm: " << install_param.antlever.transpose() << " [m] " << std::endl;
    std::cout << '\t' << "- odolever leverarm: " << install_param.odolever.transpose() << " [m] " << std::endl;
    std::cout << '\t' << "- installangle: " << install_param.installangle.transpose() * r2d << " [deg] " << std::endl
              << std::endl;

    std::cout << " - Measurement Noise: " << std::endl;
    std::cout << '\t' << "- Code Noise: " << meas_noise.codeNoise << " [m] " << std::endl;
    std::cout << '\t' << "- Carrier Phase Noise: " << meas_noise.carrierPhaseNoise << " [m] " << std::endl;
    std::cout << '\t' << "- ZUPT W Meas Noise: " << meas_noise.zupt_wmeasnoise * (r2d * 3600.0) << " [deg/h] "
              << std::endl;
    std::cout << '\t' << "- ZUPT V Meas Noise: " << meas_noise.zupt_vmeasnoise.transpose() << " [m/s] " << std::endl;
    std::cout << '\t' << "- ODONHC Meas Noise: " << meas_noise.odonhc_measnoise.transpose() << " [m/s] " << std::endl
              << std::endl;

    std::cout << " - Updating Frequency(Hz): " << std::endl;
    std::cout << '\t' << "- ODONHC Update Rate: " << odonhc_updaterate << " [Hz]" << std::endl;
    std::cout << '\t' << "- ZUPT Update Rate: " << zupt_updaterate << " [Hz]" << std::endl << std::endl;

    std::cout << " - Processing Time Span(GPST): " << std::endl;
    std::cout << '\t' << "- Start time: " << start_week << " " << start_tow << " [sec]" << std::endl;
    std::cout << '\t' << "- End time: " << end_week << " " << end_tow << " [sec]" << std::endl << std::endl;

    std::cout << " - Processing configure: " << std::endl;
    std::cout << '\t' << "- Navigation GEO Frame  : " << (navSys == cmn::nFrame ? "nFrame" : "eFrame") << std::endl;
    std::cout << '\t' << "- Use GNSS velocity     : " << (usegnssvel ? "true" : "false") << std::endl;
    std::cout << '\t' << "- Use ZUPT & ZARU       : " << (useZUPT ? "true" : "false") << std::endl;
    std::cout << '\t' << "- Use odo & nhc         : " << (useodonhc ? "true" : "false") << std::endl;
    std::cout << '\t' << "- Use single odo        : " << (usesingleodo ? "true" : "false") << std::endl;
    std::cout << '\t' << "- Use insmech only      : " << (onlyinsmech ? "true" : "false") << std::endl;
    std::cout << '\t' << "- Use Init P-V from RTK : " << (useInitalPosVelFromRTK ? "true" : "false") << std::endl;
    std::cout << '\t' << "- Use Init Att from RA  : " << (useRoughAtt ? "true" : "false") << std::endl;
    std::cout << '\t' << "- Add Imu Scale Factor  : " << (estimateImuScale ? "true" : "false") << std::endl;
    std::cout << '\t' << "- IMU Data Format       : " << imudataformat << std::endl;
    std::cout << '\t' << "- IMU Data Coordinate   : " << imurawcoordinate << std::endl;
    std::cout << '\t' << "- IMU Alignment Period  : " << imuInitStaticTime << " [min]" << std::endl;
    std::cout << '\t' << "- IMU Sampling Rate     : " << imuSamplingRate << " [Hz]" << std::endl;
    std::cout << '\t' << "- IMU Inital GPST Week  : " << imuStartGPSTWeek << " [GPST Week]" << std::endl;
    std::cout << '\t' << "- Plot Nav Results/STD  : " << (plotResults ? "true" : "false") << std::endl << std::endl;

    std::cout << " - Processing filepaths: " << std::endl;
    std::cout << '\t' << "- rov renix*.o   : " << o_renixrovpath << std::endl;
    std::cout << '\t' << "- bas renix*.o   : " << o_renixbaspath << std::endl;
    std::cout << '\t' << "- nav renix*.n   : " << n_renixnavpath << std::endl;
    std::cout << '\t' << "- gnssfilepath   : " << gnssfilepath << std::endl;
    std::cout << '\t' << "- imuimrfilepath : " << imrfilepath << std::endl;
    std::cout << '\t' << "- imuascfilepath : " << ascfilepath << std::endl;
    std::cout << '\t' << "- imutxtfilepath : " << imutxtfilepath << std::endl;
    std::cout << '\t' << "- odofilepath    : " << odofilepath << std::endl;
    std::cout << '\t' << "- output path    : " << outputpath << std::endl << std::endl;

    std::cout << "-----------------------------------EOF----------------------------------" << std::endl << std::endl;
}

Vector3d GINSOptions::readVector(const string &key) {
    string valStr = gins_conf_manager.getConfig(key);

    // Regular expression pattern to match [num; num; num] format with support for negative numbers and floating points
    regex pattern(R"(\[(-?\d+(\.\d+)?), (-?\d+(\.\d+)?), (-?\d+(\.\d+)?)\])");

    smatch matches;
    Vector3d vec;

    // If the pattern matches, extract the numbers and store in vec
    if (regex_search(valStr, matches, pattern)) {
        if (matches.size() == 7) { // Total matches, including the full match and each capturing group
            vec(0) = stod(matches[1]);
            vec(1) = stod(matches[3]);
            vec(2) = stod(matches[5]);
        }
    }

    return vec;
}

double GINSOptions::readDouble(const string &key) {
    string valStr = gins_conf_manager.getConfig(key);
    double value  = 0.0;

    try {
        value = stod(valStr);
    } catch (const std::invalid_argument &e) {
        cerr << "[ERROR]: Invalid argument: " << e.what() << endl;
        std::fflush(stderr);
    } catch (const std::out_of_range &e) {
        cerr << "[ERROR]: Out of range: " << e.what() << endl;
        std::fflush(stderr);
    }

    return value;
}

bool GINSOptions::readBool(const string &key) {
    string valStr = gins_conf_manager.getConfig(key);

    if (valStr == "true" || valStr == "1") {
        return true;
    } else if (valStr == "false" || valStr == "0") {
        return false;
    } else {
        cerr << "[ERROR]: Invalid Iuput of key '" << key << "'" << endl;
        std::fflush(stderr);
        return false;
    }
}

string GINSOptions::readString(const string &key) {
    return gins_conf_manager.getConfig(key);
}

void GINSOptions::unitConvert() {
    initstate.pos[0] *= d2r;
    initstate.pos[1] *= d2r;
    initstate.att.euler *= d2r;
    initstate.imuerror.gyrbias /= (r2d * 3600.0);
    initstate.imuerror.accbias /= 1e5;
    initstate.imuerror.gyrscale /= 1e6;
    initstate.imuerror.accscale /= 1e6;

    initstate_std.att.euler *= d2r;
    initstate_std.imuerror.gyrbias /= (r2d * 3600.0);
    initstate_std.imuerror.accbias /= 1e5;
    initstate_std.imuerror.gyrscale /= 1e6;
    initstate_std.imuerror.accscale /= 1e6;

    imunoise.gyr_arw /= (r2d * 60.0);
    imunoise.acc_vrw /= 60.0;
    imunoise.gyrbias_std /= (r2d * 3600.0);
    imunoise.accbias_std /= 1e5;
    imunoise.gyrscale_std /= 1e6;
    imunoise.accscale_std /= 1e6;
    imunoise.corr_time *= 3600.0;

    install_param.installangle *= d2r;

    meas_noise.zupt_wmeasnoise /= (r2d * 3600.0);
}