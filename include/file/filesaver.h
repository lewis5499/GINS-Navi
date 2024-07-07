#ifndef GINS_FILESAVER_H
#define GINS_FILESAVER_H

#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace file {

enum class FileMode { BINARY, TEXT };

class NaviFileSaver {
public:
    NaviFileSaver(std::string filePath, FileMode mode);
    ~NaviFileSaver();

    // Set the headers for the columns
    void writeHeaders(const std::vector<std::string> &subheaders);
    // Save data to the file
    template <typename... Args> void writeData(const Args &...args);
    // Save raw imu data
    static void saveRawImuToTxt(const std::vector<ins::IMU> &imuall, const std::string &path) {
        std::FILE *ff = std::fopen((path + "imu.txt").c_str(), "w");
        for (auto item : imuall) {
            fprintf(ff, "%.4f %.10f %.10f %.10f %.10f %.10f %.10f\n\r", item.time.getTime().SecOfWeek, item.dtheta(0),
                    item.dtheta(1), item.dtheta(2), item.dvel(0), item.dvel(1), item.dvel(2));
        }
        if (ff) {
            fclose(ff);
        }
    }

private:
    std::string filePath;
    FileMode mode;
    std::ofstream file;
    std::vector<std::string> headers;

    void openFile();
    template <typename T> void saveTextData(const T &arg);
    template <typename T, typename... Args> void saveTextData(const T &first, const Args &...rest);
    template <typename T> void saveBinaryData(const T &arg);
    template <typename T, typename... Args> void saveBinaryData(const T &first, const Args &...rest);
};

inline NaviFileSaver::NaviFileSaver(std::string filePath, FileMode mode)
    : filePath(std::move(filePath))
    , mode(mode) {
    openFile();
}

inline NaviFileSaver::~NaviFileSaver() {
    if (file.is_open()) {
        file.close();
    }
}

inline void NaviFileSaver::writeHeaders(const std::vector<std::string> &subheaders) {
    this->headers.insert(this->headers.end(), subheaders.begin(), subheaders.end());
    // Write headers
    if (!headers.empty()) {
        for (const auto &header : headers) {
            file << header << "\t";
        }
        file << "\n";
    }
}

template <typename... Args> inline void NaviFileSaver::writeData(const Args &...args) {
    if (mode == FileMode::TEXT) {
        saveTextData(args...);
    } else if (mode == FileMode::BINARY) {
        saveBinaryData(args...);
    } else {
        throw std::runtime_error("Unsupported file mode.");
    }
}

inline void NaviFileSaver::openFile() {
    if (mode == FileMode::TEXT) {
        file.open(filePath, std::ios::out | std::ios::trunc);
        if (!file.is_open()) {
            throw std::runtime_error("Error opening text file.");
        }
    } else if (mode == FileMode::BINARY) {
        file.open(filePath, std::ios::out | std::ios::trunc | std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("Error opening binary file.");
        }
    } else {
        throw std::runtime_error("Unsupported file mode.");
    }
}

template <typename T> inline void NaviFileSaver::saveTextData(const T &arg) {
    file << std::setiosflags(std::ios::fixed) << std::setprecision(12) << arg << "\n";
}

template <typename T, typename... Args> inline void NaviFileSaver::saveTextData(const T &first, const Args &...rest) {
    file << std::setiosflags(std::ios::fixed) << std::setprecision(12) << first << "   ";
    saveTextData(rest...);
}

template <typename T> inline void NaviFileSaver::saveBinaryData(const T &arg) {
    file.write(reinterpret_cast<const char *>(&arg), sizeof(T));
}

template <typename T, typename... Args> inline void NaviFileSaver::saveBinaryData(const T &first, const Args &...rest) {
    file.write(reinterpret_cast<const char *>(&first), sizeof(T));
    saveBinaryData(rest...);
}

} // namespace file

#endif // GINS_FILESAVER_H
