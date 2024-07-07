#ifndef GINS_TIMESYS_H
#define GINS_TIMESYS_H

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace timesys {

struct mjdtime_t {
    int Days;
    double FracDay;

    mjdtime_t()
        : Days(0)
        , FracDay(0.0) {
    }
    mjdtime_t(int d, double f)
        : Days(d)
        , FracDay(f) {
    }
};

struct utime_t {
    short Year;
    uint16_t Month;
    uint16_t Day;
    uint16_t Hour;
    uint16_t Minute;
    double Second;

    utime_t()
        : Year(0)
        , Month(0)
        , Day(0)
        , Hour(0)
        , Minute(0)
        , Second(0.0) {
    }
    utime_t(short y, uint16_t mon, uint16_t d, uint16_t h, uint16_t min, double s)
        : Year(y)
        , Month(mon)
        , Day(d)
        , Hour(h)
        , Minute(min)
        , Second(s) {
    }
};

struct gtime_t {
    uint16_t Week;
    double SecOfWeek;

    gtime_t()
        : Week(0)
        , SecOfWeek(0.0) {
    }
    gtime_t(uint16_t w, double s)
        : Week(w)
        , SecOfWeek(s) {
    }
    gtime_t(const gtime_t &t)
        : Week(t.Week)
        , SecOfWeek(t.SecOfWeek) {
    }
};

class Time {
private:
    gtime_t gtime;
    bool valid;
    static constexpr auto EPSILON = 1e-9;

public:

    static bool CT2MJD(const utime_t &CT, mjdtime_t &MJDT);
    static bool MJD2CT(const mjdtime_t &MJDT, utime_t &CT);
    static bool GPST2MJD(const gtime_t &GT, mjdtime_t &MJDT);
    static bool MJD2GPST(const mjdtime_t &MJDT, gtime_t &GT);
    static bool CT2GPST(const utime_t &CT, gtime_t &GT);
    static bool GPST2CT(const gtime_t &GT, utime_t &CT);

    Time()
        : gtime(gtime_t())
        , valid(true) {
    }

    Time(const gtime_t &gt)
        : gtime(gt)
        , valid(true) {
    }

    Time(const utime_t &ct) {
        valid = CT2GPST(ct, gtime);
    }

    Time(const mjdtime_t &mjd) {
        valid = MJD2GPST(mjd, gtime);
    }

    Time(uint16_t gpsWeek, double gpsSecOfWeek)
        : gtime(gpsWeek, gpsSecOfWeek)
        , valid(true) {
    }

    Time(short year, uint16_t month, uint16_t day, uint16_t hour, uint16_t minute, double second) {
        utime_t ct(year, month, day, hour, minute, second);
        valid = CT2GPST(ct, gtime);
    }

    Time(int mjdDays, double mjdFracDay) {
        mjdtime_t mjd(mjdDays, mjdFracDay);
        valid = MJD2GPST(mjd, gtime);
    }

    // Getters
    gtime_t getTime() const {
        return gtime;
    }

    bool getTimeValid() const {
        return valid;
    }

    // Setters
    void setTime(const gtime_t &gt) {
        gtime = gt;
        valid = true;
    }

    void setTime(uint16_t gpsWeek, double gpsSecOfWeek) {
        gtime = gtime_t(gpsWeek, gpsSecOfWeek);
        valid = true;
    }

    void setTime(short year, uint16_t month, uint16_t day, uint16_t hour, uint16_t minute, double second) {
        utime_t ct(year, month, day, hour, minute, second);
        valid = CT2GPST(ct, gtime);
    }

    void setTime(int mjdDays, double mjdFracDay) {
        mjdtime_t mjd(mjdDays, mjdFracDay);
        valid = MJD2GPST(mjd, gtime);
    }

    void setInvalid() {
        this->valid = false;
    }

    // this time - other time
    double diffTime(const Time &other, bool useWeek = false) const {
        if (!valid || !other.valid) {
            std::cerr << "[ERROR]: Invalid time data" << std::endl;
            return 0.0;
        }
        if (!useWeek) {
            return (gtime.SecOfWeek - other.gtime.SecOfWeek);
        } else {
            return (gtime.Week - other.gtime.Week) * 7 * 86400.0 + (gtime.SecOfWeek - other.gtime.SecOfWeek);
        }
    }

    // 重载运算符: 注意!!!使用了Week!!!
    double operator-(const Time &other) const { // 未使用Week!!  + 604800.0*(gtime.Week-other.gtime.Week)
        if (!valid || !other.valid) {
            std::cerr << "[ERROR]: Invalid time data" << std::endl;
            return 0.0;
        }
        return (gtime.SecOfWeek - other.gtime.SecOfWeek + 604800.0*(gtime.Week-other.gtime.Week));
    }

    bool operator<(const Time &other) const {
        if (!valid || !other.valid) {
            std::cerr << "[ERROR]: Invalid time data" << std::endl;
            return false;
        }
        return ((gtime.SecOfWeek - other.gtime.SecOfWeek + 604800.0*(gtime.Week-other.gtime.Week))<0);
    }

    bool operator>(const Time &other) const {
        if (!valid || !other.valid) {
            std::cerr << "[ERROR]: Invalid time data" << std::endl;
            return false;
        }
        return ((gtime.SecOfWeek - other.gtime.SecOfWeek + 604800.0*(gtime.Week-other.gtime.Week))>0);
    }

    void Reset() {
        this->gtime.Week      = 0;
        this->gtime.SecOfWeek = 0.0;
        this->valid           = true;
    }

    // ToString method
    std::string ToString() const {
        if (!valid) {
            return "Class 'Time': Invalid Time";
        }

        // Convert GPST to CT (Calendar Time) for a more human-readable format
        utime_t ct;
        if (!GPST2CT(gtime, ct)) {
            return "Class 'Time': Time Conversion Error in Func 'GPST2CT";
        }

        std::ostringstream oss;
        oss << std::setw(4) << std::setfill('0') << ct.Year << '-' << std::setw(2) << std::setfill('0') << ct.Month
            << '-' << std::setw(2) << std::setfill('0') << ct.Day << ' ' << std::setw(2) << std::setfill('0') << ct.Hour
            << ':' << std::setw(2) << std::setfill('0') << ct.Minute << ':' << std::fixed << std::setprecision(9)
            << ct.Second;
        return oss.str();
    }

    // ToStringGPST method
    std::string ToStringGPST() const {
        if (!valid) {
            return "Class 'Time': Invalid Time";
        }

        std::ostringstream oss;
        oss << "Week: " << gtime.Week << ", "
            << "Seconds of Week: " << std::fixed << std::setprecision(9) << gtime.SecOfWeek;
        return oss.str();
    }
};


}

#endif // GINS_TIMESYS_H
