#ifndef GINS_INSMECH_H
#define GINS_INSMECH_H

#include "ins/imutypes.h"
#include "state/navstate.h"

namespace ins {

/* ----------------------------- INS Algorithms ----------------------------- */
class InsMech {
public:
    virtual void insUpdate(const nav::NavState &preState, nav::NavState &curState, const ins::IMU &preIMU,
                           const ins::IMU &curIMU) = 0;

private:
    virtual void attUpdate(const nav::NavState &preState, nav::NavState &curState, const ins::IMU &preIMU,
                           const ins::IMU &curIMU) = 0;
    virtual void velUpdate(const nav::NavState &preState, nav::NavState &curState, const ins::IMU &preIMU,
                           const ins::IMU &curIMU) = 0;
    virtual void posUpdate(const nav::NavState &preState, nav::NavState &curState, const ins::IMU &preIMU,
                           const ins::IMU &curIMU) = 0;
};

class nFrameIns : public InsMech {
public:
    static void insMech(const nav::NavState &preState, nav::NavState &curState, const ins::IMU &preIMU,
                        const ins::IMU &curIMU);

private:
    nFrameIns();
    nFrameIns(const nFrameIns &)            = delete;
    nFrameIns &operator=(const nFrameIns &) = delete;

    void insUpdate(const nav::NavState &preState, nav::NavState &curState, const ins::IMU &preIMU,
                   const ins::IMU &curIMU) override;
    void attUpdate(const nav::NavState &preState, nav::NavState &curState, const ins::IMU &preIMU,
                   const ins::IMU &curIMU) override;
    void velUpdate(const nav::NavState &preState, nav::NavState &curState, const ins::IMU &preIMU,
                   const ins::IMU &curIMU) override;
    void posUpdate(const nav::NavState &preState, nav::NavState &curState, const ins::IMU &preIMU,
                   const ins::IMU &curIMU) override;
};

class eFrameIns : public InsMech {
public:
    static void insMech(const nav::NavState &preState, nav::NavState &curState, const ins::IMU &preIMU,
                        const ins::IMU &curIMU);

private:
    eFrameIns();
    eFrameIns(const eFrameIns &)            = delete;
    eFrameIns &operator=(const eFrameIns &) = delete;

    void insUpdate(const nav::NavState &preState, nav::NavState &curState, const ins::IMU &preIMU,
                   const ins::IMU &curIMU) override;
    void attUpdate(const nav::NavState &preState, nav::NavState &curState, const ins::IMU &preIMU,
                   const ins::IMU &curIMU) override;
    void velUpdate(const nav::NavState &preState, nav::NavState &curState, const ins::IMU &preIMU,
                   const ins::IMU &curIMU) override;
    void posUpdate(const nav::NavState &preState, nav::NavState &curState, const ins::IMU &preIMU,
                   const ins::IMU &curIMU) override;
};

} // namespace insAlgorithm

#endif // GINS_INSMECH_H
