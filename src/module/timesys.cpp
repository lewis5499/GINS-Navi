#include "timesys.h"

using namespace timesys;

bool Time::CT2MJD(const utime_t &CT, mjdtime_t &MJDT) {
    int y, m = 0;
    if (CT.Month == 0 || CT.Day == 0) {
        return false;
    } else if (CT.Month <= 2) {
        y = CT.Year - 1;
        m = CT.Month + 12;
    } else {
        y = CT.Year;
        m = CT.Month;
    }

    MJDT.Days    = (int) (365.25 * y) + (int) (30.6001 * (m + 1)) + int(CT.Day) - 679019;
    MJDT.FracDay = (CT.Hour + CT.Minute / 60.0 + CT.Second / 3600.0) / 24.0;

    return true;
}

bool Time::MJD2CT(const mjdtime_t &MJDT, utime_t &CT) {
    if (MJDT.Days == 0 && std::abs(MJDT.FracDay) < EPSILON) {
        return false;
    }
    int a = (int) (MJDT.Days + MJDT.FracDay + 2400001);
    int b = a + 1537;
    int c = (int) ((b - 122.1) / 365.25);
    int d = (int) (365.25 * c);
    int e = (int) ((b - d) / 30.6001);

    CT.Day   = b - d - (int) (30.6001 * e);
    CT.Month = e - 1 - 12 * (int) (e / 14);
    CT.Year  = c - 4715 - (int) ((7 + CT.Month) / 10);

    CT.Hour   = (int) (MJDT.FracDay * 24);
    CT.Minute = (int) ((MJDT.FracDay * 24 - CT.Hour) * 60);
    CT.Second = ((MJDT.FracDay * 24 - CT.Hour) * 60 - CT.Minute) * 60.0;
    return true;
}

bool Time::GPST2MJD(const gtime_t &GT, mjdtime_t &MJDT) {
    if (GT.Week == 0 && std::abs(GT.SecOfWeek) < EPSILON) {
        return false;
    }
    MJDT.Days    = 44244 + GT.Week * 7 + (int) (GT.SecOfWeek / 86400.0);
    MJDT.FracDay = GT.SecOfWeek / 86400.0 - (int) (GT.SecOfWeek / 86400.0);
    return true;
}

bool Time::MJD2GPST(const mjdtime_t &MJDT, gtime_t &GT) {
    if (MJDT.Days == 0 && std::abs(MJDT.FracDay) < EPSILON) {
        return false;
    }
    GT.Week      = (int) ((MJDT.Days - 44244) / 7);
    GT.SecOfWeek = (MJDT.Days - 44244 - GT.Week * 7 + MJDT.FracDay) * 86400;
    return true;
}

bool Time::CT2GPST(const utime_t &CT, gtime_t &GT) {
    if (CT.Month == 0 || CT.Day == 0) {
        return false;
    }
    mjdtime_t MJD;
    if (!CT2MJD(CT, MJD)) {
        return false;
    }
    return MJD2GPST(MJD, GT);
}

bool Time::GPST2CT(const gtime_t &GT, utime_t &CT) {
    if (GT.Week == 0 && std::abs(GT.SecOfWeek) < EPSILON) {
        return false;
    }
    mjdtime_t MJD;
    if (!GPST2MJD(GT, MJD)) {
        return false;
    }
    return MJD2CT(MJD, CT);
}
