#ifndef GINS_ALGEBRA_H
#define GINS_ALGEBRA_H

#include "common.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Algebra {

static constexpr auto pi        = 3.14159265358979323;
static constexpr auto d2r       = pi / 180.0;
static constexpr auto r2d       = 180.0 / pi;
static constexpr auto LAT       = 30.52780368;  /* latitude in Wuhan University, HuBei Province, China */
static constexpr auto LON       = 114.35579096; /* longitude in Wuhan University, HuBei Province, China */
static constexpr auto WGS84_WIE = 7.2921151467E-5;
static constexpr auto WGS84_F   = 0.0033528106647474805; /* oblateness */
static constexpr auto WGS84_RA  = 6378137.0000000000;
static constexpr auto WGS84_RB  = 6356752.3142451793;
static constexpr auto WGS84_GM  = 398600441800000.00;
static constexpr auto WGS84_E1  = 0.0066943799901413156; /* note: SQUARE */
static constexpr auto WGS84_E2  = 0.0067394967422764341; /* note: SQUARE */

/* Vector2/double: calculating the radius of the meridian circle and the radius of the prime vertical circle */
static Eigen::Vector2d RmRn(double lat) {
    double tmp, sqrttmp;
    tmp     = 1 - WGS84_E1 * SQR(sin(lat));
    sqrttmp = SQRT(tmp);
    return {WGS84_RA * (1 - WGS84_E1) / (sqrttmp * tmp), WGS84_RA / sqrttmp};
}
static double Rn(double lat) {
    return WGS84_RA / sqrt(1.0 - WGS84_E1 * SQR(sin(lat)));
}

/* Vector3: wie->e frame (projection) */
static Eigen::Vector3d iewe() {
    return {0, 0, WGS84_WIE};
}

/* double: Normal gravity calculation */
static double NormalGravity(const Eigen::Vector3d &blh) {
    double sin2 = SQR(sin(blh[0]));
    return 9.7803267715 * (1 + 0.0052790414 * sin2 + 0.0000232718 * SQR(sin2)) +
           blh[2] * (0.0000000043977311 * sin2 - 0.0000030876910891) + 0.0000000000007211 * SQR(blh[2]);
}

static Eigen::Vector3d gn(const Eigen::Vector3d &blh) {
    return {0, 0, NormalGravity(blh)};
}

static Eigen::Vector3d ge(const Eigen::Vector3d &blh) {
    double g = NormalGravity(blh);
    return {-cos(blh(0)) * cos(blh(1)) * g, -cos(blh(0)) * sin(blh(1)) * g, -sin(blh(0)) * g};
}

/* Matrix3: n(Nav Frame)->e(ECEF) transformation */
static Eigen::Matrix3d Cne(const Eigen::Vector3d &blh) {
    Eigen::Matrix3d dcm;
    double cosl, sinl, cosb, sinb;
    sinb      = sin(blh[0]);
    sinl      = sin(blh[1]);
    cosb      = cos(blh[0]);
    cosl      = cos(blh[1]);
    dcm(0, 0) = -sinb * cosl;
    dcm(0, 1) = -sinl;
    dcm(0, 2) = -cosb * cosl;
    dcm(1, 0) = -sinb * sinl;
    dcm(1, 1) = cosl;
    dcm(1, 2) = -cosb * sinl;
    dcm(2, 0) = cosb;
    dcm(2, 1) = 0;
    dcm(2, 2) = -sinb;
    return dcm;
}

/* Quaternion: n(Nav Frame)->e(ECEF) transformation */
static Eigen::Quaterniond Qne(const Eigen::Vector3d &blh) {
    Eigen::Quaterniond quat;
    double cosl, sinl, cosb, sinb;
    cosl     = cos(blh[1] * 0.5);
    sinl     = sin(blh[1] * 0.5);
    cosb     = cos(-pi * 0.25 - blh[0] * 0.5);
    sinb     = sin(-pi * 0.25 - blh[0] * 0.5);
    quat.w() = cosb * cosl;
    quat.x() = -sinb * sinl;
    quat.y() = sinb * cosl;
    quat.z() = cosb * sinl;
    return quat;
}

/* Get Cbn from 'blh'(->Cne) and 'Cbe' */
static Eigen::Matrix3d Cbn(const Eigen::Vector3d &blh, const Eigen::Matrix3d &Cbe) {
    return Cne(blh).transpose() * Cbe;
}

/* Vector3: get lat, lon from Quaternion(n->e Frame) */
static Eigen::Vector3d blh(const Eigen::Quaterniond &qne, double height) {
    return {-2 * atan(qne.y() / qne.w()) - pi * 0.5, 2 * atan2(qne.z(), qne.w()), height};
}

/* Vector3: 'geodetic' coordinates -> 'ECEF' coordinates */
static Eigen::Vector3d blh2ecef(const Eigen::Vector3d &blh) {
    double cosb, sinb, cosl, sinl, rnh, rn;
    cosb = cos(blh[0]);
    sinb = sin(blh[0]);
    cosl = cos(blh[1]);
    sinl = sin(blh[1]);
    rn   = Rn(blh[0]);
    rnh  = rn + blh[2];
    return {rnh * cosb * cosl, rnh * cosb * sinl, (rnh - rn * WGS84_E1) * sinb};
}

/* Vector3: 'ECEF' coordinates -> 'geodetic' coordinates */
static Eigen::Vector3d ecef2blh(const Eigen::Vector3d &ecef) {
    double p, rn, lat, lon, h = 0, h0;
    p   = sqrt(ecef[0] * ecef[0] + ecef[1] * ecef[1]);
    lat = atan(ecef[2] / (p * (1.0 - WGS84_E1)));
    lon = 2.0 * atan2(ecef[1], ecef[0] + p);
    do {
        h0  = h;
        rn  = Rn(lat);
        h   = p / cos(lat) - rn;
        lat = atan(ecef[2] / (p * (1.0 - WGS84_E1 * rn / (rn + h))));
    } while (fabs(h - h0) > 1.0e-4);
    return {lat, lon, h};
}

/* Matrix3: geodetic coordinate vector3(vel) -> n-frame vector3(vel) at 'blh' */
static Eigen::Matrix3d DR(const Eigen::Vector3d &blh) {
    Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
    Eigen::Vector2d rmn = RmRn(blh[0]);
    mat(0, 0)           = rmn[0] + blh[2];
    mat(1, 1)           = (rmn[1] + blh[2]) * cos(blh[0]);
    mat(2, 2)           = -1;
    return mat;
}

/* Matrix3: n-frame vector3(vel) -> geodetic coordinate vector3(vel) at 'blh' */
static Eigen::Matrix3d DRinv(const Eigen::Vector3d &blh) {
    Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
    Eigen::Vector2d rmn = RmRn(blh[0]);
    mat(0, 0)           = 1.0 / (rmn[0] + blh[2]);
    mat(1, 1)           = 1.0 / ((rmn[1] + blh[2]) * cos(blh[0]));
    mat(2, 2)           = -1;
    return mat;
}

/* Vector3: local coordinates (expanded around the origin) -> geodetic coordinates */
static Eigen::Vector3d ned2blh(const Eigen::Vector3d &origin, const Eigen::Vector3d &local) {
    Eigen::Vector3d ecef0 = blh2ecef(origin);
    Eigen::Matrix3d cn0e  = Cne(origin);
    Eigen::Vector3d ecef1 = ecef0 + cn0e * local;
    Eigen::Vector3d blh1  = ecef2blh(ecef1);
    return blh1;
}

/* Vector3: geodetic coordinates -> local coordinates (expanded around the origin) */
static Eigen::Vector3d blh2ned(const Eigen::Vector3d &origin, const Eigen::Vector3d &global) {
    Eigen::Vector3d ecef0 = blh2ecef(origin);
    Eigen::Matrix3d cn0e  = Cne(origin);
    Eigen::Vector3d ecef1 = blh2ecef(global);
    return cn0e.transpose() * (ecef1 - ecef0);
}

/* Vector3: wie->n frame (projection) */
static Eigen::Vector3d iewn(double lat) {
    return {WGS84_WIE * cos(lat), 0, -WGS84_WIE * sin(lat)};
}

static Eigen::Vector3d iewn(const Eigen::Vector3d &origin, const Eigen::Vector3d &local) {
    Eigen::Vector3d global = ned2blh(origin, local);
    return iewn(global[0]);
}

/* Vector3: wen->n frame (projection) */
static Eigen::Vector3d enwn(const Eigen::Vector2d &rmrn, const Eigen::Vector3d &blh, const Eigen::Vector3d &vel) {
    return {vel[1] / (rmrn[1] + blh[2]), -vel[0] / (rmrn[0] + blh[2]), -vel[1] * tan(blh[0]) / (rmrn[1] + blh[2])};
}

static Eigen::Vector3d enwn(const Eigen::Vector3d &origin, const Eigen::Vector3d &local, const Eigen::Vector3d &vel) {
    Eigen::Vector3d global = ned2blh(origin, local);
    Eigen::Vector2d rmrn   = RmRn(global[0]);
    return enwn(rmrn, global, vel);
}

/* Vector3: win->n frame (projection) */
static Eigen::Vector3d inwn(const Eigen::Vector3d &origin, const Eigen::Vector3d &local, const Eigen::Vector3d &vel) {
    auto wie_n = iewn(origin, local);
    auto wen_n = enwn(origin, local, vel);
    return wie_n + wen_n;
}
static Eigen::Vector3d inwn(const Eigen::Vector3d &blh, const Eigen::Vector3d &vel) {
    auto wie_n = iewn(blh(0));
    auto rmrn  = RmRn(blh(0));
    auto wen_n = enwn(rmrn, blh, vel);
    return wie_n + wen_n;
}

/* Euler Angle <-> DCM */
static Eigen::Matrix3d euler2dcm(const Eigen::Vector3d &euler) { // Roll-Pitch-Yaw --> C_b^n, ZYX rotation sequence
    return Eigen::Matrix3d(Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
}

static Eigen::Vector3d dcm2euler(const Eigen::Matrix3d &dcm) { // ZYX rotation sequence, IMU located at the
    Eigen::Vector3d euler;                                     // front-right-down position, outputs Roll-Pitch-Yaw
    euler[1] = atan(-dcm(2, 0) / sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));
    if (dcm(2, 0) <= -0.999) {
        euler[0] = 0;
        euler[2] = atan2((dcm(1, 2) - dcm(0, 1)), (dcm(0, 2) + dcm(1, 1)));
        printf("[WARNING] Rotation::dcm2euler: Singular Euler Angle! Set the roll angle to 0!\n");
    } else if (dcm(2, 0) >= 0.999) {
        euler[0] = 0;
        euler[2] = pi + atan2((dcm(1, 2) + dcm(0, 1)), (dcm(0, 2) - dcm(1, 1)));
        printf("[WARNING] Rotation::dcm2euler: Singular Euler Angle! Set the roll angle to 0!\n");
    } else {
        euler[0] = atan2(dcm(2, 1), dcm(2, 2));
        euler[2] = atan2(dcm(1, 0), dcm(0, 0));
    }
    // yaw: 0~2PI
    if (euler[2] < 0) {
        euler[2] = pi * 2 + euler[2];
    }
    return euler;
}

/* DCM <-> Quaternion */
static Eigen::Quaterniond dcm2quat(const Eigen::Matrix3d &dcm) {
    return Eigen::Quaterniond(dcm);
}

static Eigen::Matrix3d quat2dcm(const Eigen::Quaterniond &quaternion) {
    return quaternion.toRotationMatrix();
}

/* Quaternion <-> Euler Angle */
static Eigen::Vector3d quat2euler(const Eigen::Quaterniond &quaternion) {
    return dcm2euler(quaternion.toRotationMatrix());
}

static Eigen::Quaterniond euler2quat(const Eigen::Vector3d &euler) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
}

/* Vector <-> Quaternion */
static Eigen::Quaterniond vector2quat(const Eigen::Vector3d &rotvec) {
    double angle        = rotvec.norm();
    Eigen::Vector3d vec = rotvec.normalized();
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, vec));
}

static Eigen::Vector3d quat2vector(const Eigen::Quaterniond &quaternion) {
    Eigen::AngleAxisd axisd(quaternion);
    return axisd.angle() * axisd.axis();
}

/* Vector <-> skew symmetric */
static Eigen::Matrix3d vector2skewsym(const Eigen::Vector3d &vector) {
    Eigen::Matrix3d mat;
    mat << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1), vector(0), 0;
    return mat;
}

static Eigen::Vector3d skewsym2vector(const Eigen::Matrix3d &mat) {
    Eigen::Vector3d vec;
    vec << mat(2, 1), mat(0, 2), mat(1, 0);
    return vec;
}

} // namespace Algebra

#endif