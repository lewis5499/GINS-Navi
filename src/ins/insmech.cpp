#include "navstate.h"
#include "imutypes.h"
#include "algebra.h"
#include "insmech.h"

using namespace ins;
using namespace nav;
using namespace Eigen;
using namespace Algebra;

nFrameIns::nFrameIns() = default;

void nFrameIns::insMech(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU) {
    static nFrameIns instance;
    instance.insUpdate(preState, curState, preIMU, curIMU);
}

void nFrameIns::insUpdate(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU) {
    // NOTE: irreversible order!
    attUpdate(preState, curState, preIMU, curIMU);
    velUpdate(preState, curState, preIMU, curIMU);
    posUpdate(preState, curState, preIMU, curIMU);
}

void nFrameIns::attUpdate(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU) {
    Vector2d rmrn;
    Vector3d wie_n, wen_n;
    Quaterniond qnn, qbb;

    /* compute rmrn, wie_n, wen_n: k-1 */
    rmrn  = RmRn(preState.pos[0]);
    wie_n = iewn(preState.pos[0]);
    wen_n = enwn(rmrn, preState.pos, preState.vel);

    /* n-frame rotation vector (n(k-1)->n(k)) */
    qnn = vector2quat(-(wie_n + wen_n) * curIMU.dt);
    /* b-frame rotation vector (b(k)->b(k-1)): compensate the second-order coning correction term */
    qbb = vector2quat(curIMU.dtheta + preIMU.dtheta.cross(curIMU.dtheta) / 12.0);

    /* attitude update finish! */
    curState.att.qbn   = (qnn * preState.att.qbn * qbb).normalized();
    curState.att.cbn   = quat2dcm(curState.att.qbn);
    curState.att.euler = dcm2euler(curState.att.cbn);
}

void nFrameIns::velUpdate(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU) {
    Vector2d rmrn;
    Vector3d d_vfb, d_vfn, d_vgn, gl, midvel, midpos, wie_n, wen_n, zetaVec3;
    Matrix3d Cnn, I33 = Matrix3d::Identity();

    /* estimate k-1 -> k-1/2: vel, pos */
    // calculate vec3d: rmrn, wie(projected->n-frame), wen(projected->n-frame), gl(normal gravity)
    rmrn  = RmRn(preState.pos[0]);
    wie_n = iewn(preState.pos[0]);
    wen_n = enwn(rmrn, preState.pos, preState.vel);

    // d_vgn: gravity and Coriolis force
    gl << 0, 0, NormalGravity(preState.pos);
    d_vgn = (gl - (2 * wie_n + wen_n).cross(preState.vel)) * curIMU.dt;

    // d_vfn: the specific force (compensate rotational and sculling motion)
    d_vfb = curIMU.dvel + curIMU.dtheta.cross(curIMU.dvel) / 2.0 +
            (preIMU.dtheta.cross(curIMU.dvel) + preIMU.dvel.cross(curIMU.dtheta)) / 12.0;
    zetaVec3 = (wie_n + wen_n) * curIMU.dt / 2;
    Cnn      = I33 - vector2skewsym(zetaVec3); // project d_vfb->n-frame
    d_vfn    = Cnn * preState.att.cbn * d_vfb;

    // velocity at k-1/2: estimated!
    midvel = preState.vel + (d_vfn + d_vgn) / 2.0;

    // position at k-1/2, extrapolation!
    midpos(2) = preState.pos[2] - midvel[2] * curIMU.dt / 2.0;
    midpos(0) = preState.pos[0] + midvel[0] / (rmrn[0] + midpos[2]) * curIMU.dt / 2.0;
    midpos(1) = preState.pos[1] + midvel[1] / ((rmrn[1] + midpos[2]) * cos(midpos[0])) * curIMU.dt / 2.0;

    /* recompute vel Update -> k */
    // vel, pos at k-1/2 exist! -> rmrn, wie_n, wen_n at k-1/2
    rmrn  = RmRn(midpos[0]);
    wie_n = iewn(midpos[0]);
    wen_n = enwn(rmrn, midpos, midvel);

    // d_vfn: the specific force
    Cnn   = I33 - vector2skewsym((wie_n + wen_n) * curIMU.dt / 2.0);
    d_vfn = Cnn * preState.att.cbn * d_vfb;

    // d_vgn: gravity and Coriolis force
    gl << 0, 0, NormalGravity(midpos);
    d_vgn = (gl - (2 * wie_n + wen_n).cross(midvel)) * curIMU.dt;

    // velocity update finish!
    curState.vel = preState.vel + d_vfn + d_vgn;
}

void nFrameIns::posUpdate(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU) {
    Vector3d midvel, midpos;

    /* Vel has been updated: vel at k exist! */
    // compute velocity and position at k-1/2
    midvel = (curState.vel + preState.vel) / 2.0;
    midpos = preState.pos + DRinv(preState.pos) * midvel * curIMU.dt / 2.0;

    // position update finish!
    curState.pos = preState.pos + DRinv(midpos) * midvel * curIMU.dt;
}

eFrameIns::eFrameIns() = default;

void eFrameIns::insMech(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU) {
    static eFrameIns instance;

    auto preState_ = preState.getCopy();
    auto curState_ = curState.getCopy();

    preState_.pos=Algebra::blh2ecef(preState.pos);
    preState_.vel=Algebra::Cne(preState.pos)*preState.vel;
    preState_.att.cbe=Algebra::Cne(preState.pos)*preState.att.cbn;
    preState_.att.qbe=Algebra::dcm2quat(preState_.att.cbe);

    curState_.pos=Algebra::blh2ecef(curState.pos);
    curState_.vel=Algebra::Cne(curState.pos)*curState.vel;
    curState_.att.cbe=Algebra::Cne(curState.pos)*curState.att.cbn;
    curState_.att.qbe=Algebra::dcm2quat(curState_.att.cbe);

    instance.insUpdate(preState_, curState_, preIMU, curIMU);

    curState.pos=Algebra::ecef2blh(curState_.pos);
    curState.vel=(Algebra::Cne(curState.pos).transpose())*curState_.vel;
    curState.att.cbn=Algebra::euler2dcm(curState.att.euler);
    curState.att.qbn=Algebra::euler2quat(curState.att.euler);
}

void eFrameIns::insUpdate(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU) {
    // NOTE: irreversible order!
    velUpdate(preState, curState, preIMU, curIMU);
    posUpdate(preState, curState, preIMU, curIMU);
    attUpdate(preState, curState, preIMU, curIMU);
}

void eFrameIns::attUpdate(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU) {
    Quaterniond qee, qbb;

    /* b-frame rotation vector (b(k)->b(k-1)): compensate the second-order coning correction term */
    qbb = vector2quat(curIMU.dtheta + preIMU.dtheta.cross(curIMU.dtheta) / 12.0);
    /* e-frame rotation vector (e(k-1)->e(k)) */
    qee = vector2quat(-1.0 * iewe() * curIMU.dt);

    /* attitude update finish! */
    curState.att.qbe   = (qee * preState.att.qbe * qbb).normalized();
    curState.att.cbe   = quat2dcm(curState.att.qbe);
    curState.att.euler = dcm2euler(Cbn(ecef2blh(curState.pos), curState.att.cbe));
}

void eFrameIns::velUpdate(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU) {
    Vector3d d_vge, d_vrot, d_vscul, d_vfe;
    Matrix3d Cee;

    /* d_vge: gravity and Coriolis force */
    d_vge = (ge(ecef2blh(preState.pos)) - (iewe() * 2.0).cross(preState.vel)) * curIMU.dt;

    /* d_vfe: the specific force */
    Cee     = Matrix3d::Identity() - vector2skewsym(iewe() * 0.5 * curIMU.dt);
    d_vrot  = curIMU.dtheta.cross(curIMU.dvel) * 0.5;
    d_vscul = (preIMU.dtheta.cross(curIMU.dvel) + preIMU.dvel.cross(curIMU.dtheta)) / 12.0;
    d_vfe   = Cee * preState.att.cbe * (curIMU.dvel + d_vrot + d_vscul);

    /* velocity update finish! */
    curState.vel = preState.vel + d_vge + d_vfe;
}

void eFrameIns::posUpdate(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU) {
    /* position update finish! */
    curState.pos = preState.pos + (curState.vel + preState.vel) / 2.0 * curIMU.dt;
}