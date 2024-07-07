#include <utility>
#include <vector>
#include <array>
#include <matplotlibcpp.h>
#include <thread_pool.hpp>
#include <thread_pool_utils.hpp>
#include <future>

namespace plt = matplotlibcpp;

class Plotter {
public:
    Plotter(const std::vector<double>& t,
            const std::vector<double>& lat,
            const std::vector<double>& lon,
            const std::vector<double>& height,
            const std::vector<double>& vn,
            const std::vector<double>& ve,
            const std::vector<double>& vd,
            const std::vector<double>& roll,
            const std::vector<double>& pitch,
            const std::vector<double>& yaw,
            const std::vector<std::array<double, 3>>& gyroBias,
            const std::vector<std::array<double, 3>>& acceBias,
            const std::vector<std::array<double, 3>>& gyroScale,
            const std::vector<std::array<double, 3>>& acceScale,
            const std::vector<std::array<double, 3>>& std_gyroBias,
            const std::vector<std::array<double, 3>>& std_acceBias,
            const std::vector<std::array<double, 3>>& std_gyroScale,
            const std::vector<std::array<double, 3>>& std_acceScale,
            std::string  outputPath,
            ThreadPool::synced_stream& syncOut,
            ThreadPool::thread_pool& threadPool)
        : t(t), lat(lat), lon(lon), height(height),
        vn(vn), ve(ve), vd(vd), roll(roll), pitch(pitch), yaw(yaw),
        gyroBias(gyroBias), acceBias(acceBias),
        gyroScale(gyroScale), acceScale(acceScale),
        std_gyroBias(std_gyroBias), std_acceBias(std_acceBias),
        std_gyroScale(std_gyroScale), std_acceScale(std_acceScale),
        outputPath(std::move(outputPath)), syncOut(syncOut), threadPool(threadPool) {}

    void plotAll() {
        // Launch data extraction tasks
        threadPool.detach_task([this]() {
            extractValues(std_gyroBias, std_gyroBiasX, std_gyroBiasY, std_gyroBiasZ);
        });
        threadPool.detach_task([this]() {
            extractValues(std_acceBias, std_acceBiasX, std_acceBiasY, std_acceBiasZ);
        });
        threadPool.detach_task([this]() {
            extractValues(std_gyroScale, std_gyroScaleX, std_gyroScaleY, std_gyroScaleZ);
        });
        threadPool.detach_task([this]() {
            extractValues(std_acceScale, std_acceScaleX, std_acceScaleY, std_acceScaleZ);
        });
        threadPool.detach_task([this]() {
            extractValues(gyroBias, gyroBiasX, gyroBiasY, gyroBiasZ);
        });
        threadPool.detach_task([this]() {
            extractValues(acceBias, acceBiasX, acceBiasY, acceBiasZ);
        });
        threadPool.detach_task([this]() {
            extractValues(gyroScale, gyroScaleX, gyroScaleY, gyroScaleZ);
        });
        threadPool.detach_task([this]() {
            extractValues(acceScale, acceScaleX, acceScaleY, acceScaleZ);
        });

        // wait all tasks to complete
        std::cout << "[Thread Pool]: Waiting for the " << threadPool.get_tasks_running() << " running tasks to complete."
                  << std::endl;
        threadPool.wait();
        std::cout << "[Thread Pool]: All running tasks completed. " << threadPool.get_tasks_queued()
                  << " tasks still queued." << std::endl;

        // Plot in the main thread
        plotGroundTrack();
        plotHeight();
        plotGyroBiasSTD();
        plotAcceBiasSTD();
        plotGyroScaleSTD();
        plotAcceScaleSTD();
        plot3DTrack();
        plotGyroBias();
        plotAcceBias();
        plotGyroScale();
        plotAcceScale();
        plotVel();
        plotAtt();
    }

    void showAll() {
        // Show all plots
        try {
            plt::show();
        } catch (const std::exception& e) {
            syncOut.println("[ERROR]: Exception in plt::show(): ", e.what());
        } catch (...) {
            syncOut.println("[ERROR]: Unknown exception in plt::show().");
        }
    }

private:
    void plotGroundTrack() {
        try {
            plt::figure(1);
            plt::plot(lon, lat);
            plt::named_plot("2d-track", lon, lat);
            plt::title("Ground Track");
            plt::legend();
            plt::xlabel("lon[deg]");
            plt::ylabel("lat[deg]");
            plt::grid(true);
            plt::save(outputPath + "GroundTrack.png",300);
        } catch (const std::exception& e) {
            syncOut.println("[ERROR]: plotGroundTrack exception: ", e.what());
        } catch (...) {
            syncOut.println("[ERROR]: plotGroundTrack unknown exception.");
        }
    }

    void plotHeight() {
        try {
            plt::figure(2);
            plt::plot(t, height);
            plt::named_plot("height", t, height);
            plt::title("Height");
            plt::legend();
            plt::xlabel("Epochs");
            plt::ylabel("Height[m]");
            plt::grid(true);
            plt::save(outputPath + "Height.png",300);
        } catch (const std::exception& e) {
            syncOut.println("[ERROR]: plotHeight exception: ", e.what());
        } catch (...) {
            syncOut.println("[ERROR]: plotHeight unknown exception.");
        }
    }

    void plotGyroBiasSTD() {
        try {
            plt::figure(3);
            plt::plot(t, std_gyroBiasX);
            plt::plot(t, std_gyroBiasY);
            plt::plot(t, std_gyroBiasZ);
            plt::named_plot("stdx", t, std_gyroBiasX);
            plt::named_plot("stdy", t, std_gyroBiasY);
            plt::named_plot("stdz", t, std_gyroBiasZ);
            plt::title("GyroBias-STD");
            plt::legend();
            plt::xlabel("Epochs");
            plt::ylabel("GyroBias: std[deg/h]");
            plt::grid(true);
            plt::save(outputPath + "GyroBias-STD.png",300);
        } catch (const std::exception& e) {
            syncOut.println("[ERROR]: plotGyroBiasSTD exception: ", e.what());
        } catch (...) {
            syncOut.println("[ERROR]: plotGyroBiasSTD unknown exception.");
        }
    }

    void plotAcceBiasSTD() {
        try {
            plt::figure(4);
            plt::plot(t, std_acceBiasX);
            plt::plot(t, std_acceBiasY);
            plt::plot(t, std_acceBiasZ);
            plt::named_plot("stdx", t, std_acceBiasX);
            plt::named_plot("stdy", t, std_acceBiasY);
            plt::named_plot("stdz", t, std_acceBiasZ);
            plt::title("AcceBias-STD");
            plt::legend();
            plt::xlabel("Epochs");
            plt::ylabel("AcceBias: std[mGal]");
            plt::grid(true);
            plt::save(outputPath + "AcceBias-STD.png",300);
        } catch (const std::exception& e) {
            syncOut.println("[ERROR]: plotAcceBiasSTD exception: ", e.what());
        } catch (...) {
            syncOut.println("[ERROR]: plotAcceBiasSTD unknown exception.");
        }
    }

    void plotGyroScaleSTD() {
        try {
            plt::figure(5);
            plt::plot(t, std_gyroScaleX);
            plt::plot(t, std_gyroScaleY);
            plt::plot(t, std_gyroScaleZ);
            plt::named_plot("stdx", t, std_gyroScaleX);
            plt::named_plot("stdy", t, std_gyroScaleY);
            plt::named_plot("stdz", t, std_gyroScaleZ);
            plt::title("GyroScale-STD");
            plt::legend();
            plt::xlabel("Epochs");
            plt::ylabel("GyroScale: std[ppm]");
            plt::grid(true);
            plt::save(outputPath + "GyroScale-STD.png",300);
        } catch (const std::exception& e) {
            syncOut.println("[ERROR]: plotGyroScaleSTD exception: ", e.what());
        } catch (...) {
            syncOut.println("[ERROR]: plotGyroScaleSTD unknown exception.");
        }
    }

    void plotAcceScaleSTD() {
        try {
            plt::figure(6);
            plt::plot(t, std_acceScaleX);
            plt::plot(t, std_acceScaleY);
            plt::plot(t, std_acceScaleZ);
            plt::named_plot("stdx", t, std_acceScaleX);
            plt::named_plot("stdy", t, std_acceScaleY);
            plt::named_plot("stdz", t, std_acceScaleZ);
            plt::title("AcceScale-STD");
            plt::legend();
            plt::xlabel("Epochs");
            plt::ylabel("AcceScale: std[ppm]");
            plt::grid(true);
            plt::save(outputPath + "AcceScale-STD.png",300);
        } catch (const std::exception& e) {
            syncOut.println("[ERROR]: plotAcceScaleSTD exception: ", e.what());
        } catch (...) {
            syncOut.println("[ERROR]: plotAcceScaleSTD unknown exception.");
        }
    }

    void plot3DTrack() {
        try {
            plt::figure(6); // not 7, because a small bug in matplotlib when using plot3
            plt::plot3(lon,lat,height);
            plt::named_plot("3d-track", lon, lat);
            plt::title("3D Track");
            plt::legend();
            plt::grid(true);
            plt::save(outputPath + "3DTrack.png",300);
            plt::xlabel("lon[deg]");
            plt::ylabel("lat[deg]");
            plt::set_zlabel("height[m]");
        } catch (const std::exception& e) {
            syncOut.println("[ERROR]: plot3DTrack exception: ", e.what());
        } catch (...) {
            syncOut.println("[ERROR]: plot3DTrack unknown exception.");
        }
    }

    void plotGyroBias() {
        try {
            plt::figure(8);
            plt::plot(t, gyroBiasX);
            plt::plot(t, gyroBiasY);
            plt::plot(t, gyroBiasZ);
            plt::named_plot("x", t, gyroBiasX);
            plt::named_plot("y", t, gyroBiasY);
            plt::named_plot("z", t, gyroBiasZ);
            plt::title("GyroBias");
            plt::legend();
            plt::xlabel("Epochs");
            plt::ylabel("GyroBias[deg/h]");
            plt::grid(true);
            plt::save(outputPath + "GyroBias.png",300);
        } catch (const std::exception& e) {
            syncOut.println("[ERROR]: plotGyroBias exception: ", e.what());
        } catch (...) {
            syncOut.println("[ERROR]: plotGyroBias unknown exception.");
        }
    }

    void plotGyroScale() {
        try {
            plt::figure(9);
            plt::plot(t, gyroScaleX);
            plt::plot(t, gyroScaleY);
            plt::plot(t, gyroScaleZ);
            plt::named_plot("x", t, gyroScaleX);
            plt::named_plot("y", t, gyroScaleY);
            plt::named_plot("z", t, gyroScaleZ);
            plt::title("GyroScale");
            plt::legend();
            plt::xlabel("Epochs");
            plt::ylabel("GyroScale[ppm]");
            plt::grid(true);
            plt::save(outputPath + "GyroScale.png",300);
        } catch (const std::exception& e) {
            syncOut.println("[ERROR]: plotGyroScale exception: ", e.what());
        } catch (...) {
            syncOut.println("[ERROR]: plotGyroScale unknown exception.");
        }
    }

    void plotAcceBias() {
        try {
            plt::figure(10);
            plt::plot(t, acceBiasX);
            plt::plot(t, acceBiasY);
            plt::plot(t, acceBiasZ);
            plt::named_plot("x", t, acceBiasX);
            plt::named_plot("y", t, acceBiasY);
            plt::named_plot("z", t, acceBiasZ);
            plt::title("AcceBias");
            plt::legend();
            plt::xlabel("Epochs");
            plt::ylabel("AcceBias[mGal]");
            plt::grid(true);
            plt::save(outputPath + "AcceBias.png",300);
        } catch (const std::exception& e) {
            syncOut.println("[ERROR]: plotAcceBias exception: ", e.what());
        } catch (...) {
            syncOut.println("[ERROR]: plotAcceBias unknown exception.");
        }
    }

    void plotAcceScale() {
        try {
            plt::figure(11);
            plt::plot(t, acceScaleX);
            plt::plot(t, acceScaleY);
            plt::plot(t, acceScaleZ);
            plt::named_plot("x", t, acceScaleX);
            plt::named_plot("y", t, acceScaleY);
            plt::named_plot("z", t, acceScaleZ);
            plt::title("AcceScale");
            plt::legend();
            plt::xlabel("Epochs");
            plt::ylabel("AcceScale[ppm]");
            plt::grid(true);
            plt::save(outputPath + "AcceScale.png",300);
        } catch (const std::exception& e) {
            syncOut.println("[ERROR]: plotAcceScale exception: ", e.what());
        } catch (...) {
            syncOut.println("[ERROR]: plotAcceScale unknown exception.");
        }
    }

    void plotVel() {
        try {
            plt::figure(12);
            plt::plot(t, vn);
            plt::plot(t, ve);
            plt::plot(t, vd);
            plt::named_plot("vn", t, vn);
            plt::named_plot("ve", t, ve);
            plt::named_plot("vd", t, vd);
            plt::title("Velocity");
            plt::legend();
            plt::xlabel("Epochs");
            plt::ylabel("Velocity[m/s]");
            plt::grid(true);
            plt::save(outputPath + "Velocity.png",300);
        } catch (const std::exception& e) {
            syncOut.println("[ERROR]: plotVelocity exception: ", e.what());
        } catch (...) {
            syncOut.println("[ERROR]: plotVelocity unknown exception.");
        }
    }

    void plotAtt() {
        try {
            plt::figure(13);
            plt::plot(t, roll);
            plt::plot(t, pitch);
            plt::plot(t, yaw);
            plt::named_plot("roll", t, roll);
            plt::named_plot("pitch", t, pitch);
            plt::named_plot("yaw", t, yaw);
            plt::title("Attitude");
            plt::legend();
            plt::xlabel("Epochs");
            plt::ylabel("Euler Angle[deg]");
            plt::grid(true);
            plt::save(outputPath + "Attitude.png",300);
        } catch (const std::exception& e) {
            syncOut.println("[ERROR]: plotAttitude exception: ", e.what());
        } catch (...) {
            syncOut.println("[ERROR]: plotAttitude unknown exception.");
        }
    }

    static void extractValues(const std::vector<std::array<double, 3>>& source,
                              std::vector<double>& x_values,
                              std::vector<double>& y_values,
                              std::vector<double>& z_values) {
        x_values.clear();
        y_values.clear();
        z_values.clear();
        x_values.reserve(source.size());
        y_values.reserve(source.size());
        z_values.reserve(source.size());
        for (const auto& values : source) {
            x_values.push_back(values[0]);
            y_values.push_back(values[1]);
            z_values.push_back(values[2]);
        }
    }

    std::vector<double> t, lat, lon, height, vn, ve, vd, roll, pitch, yaw;
    std::vector<std::array<double, 3>> std_gyroBias, std_acceBias, std_gyroScale, std_acceScale, gyroBias, acceBias, gyroScale, acceScale;
    std::string outputPath;
    ThreadPool::synced_stream& syncOut;
    ThreadPool::thread_pool& threadPool;

    // Vectors to hold extracted data
    std::vector<double> std_gyroBiasX, std_gyroBiasY, std_gyroBiasZ;
    std::vector<double> std_acceBiasX, std_acceBiasY, std_acceBiasZ;
    std::vector<double> std_gyroScaleX, std_gyroScaleY, std_gyroScaleZ;
    std::vector<double> std_acceScaleX, std_acceScaleY, std_acceScaleZ;
    std::vector<double> gyroBiasX, gyroBiasY, gyroBiasZ;
    std::vector<double> acceBiasX, acceBiasY, acceBiasZ;
    std::vector<double> gyroScaleX, gyroScaleY, gyroScaleZ;
    std::vector<double> acceScaleX, acceScaleY, acceScaleZ;
};
