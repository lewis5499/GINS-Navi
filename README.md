# GNSS/INS Integrated Navigation GUI Software

[English Version](./README.md) | [中文版](./README-zh_CN.md)

GINS-Navi is a GNSS/INS loosely coupled integrated navigation software based on the ESKF algorithm, compatible with both Windows (GUI) and Linux (CUI) environments.

![GUI Image](./res/GUI.png)

[Please read the "Configuration" section carefully before using the software.]

Contact: Hengzhen Liu, Wuhan University, lewis5499@whu.edu.cn

## 1 Configuration

The project's configuration files are located in the ./conf directory. Detailed parameters for the algorithm can be set through these configuration files.

| **Section** | **Parameter**               | **Value**                      | **Description**                                                              |
|-------------|-----------------------------|--------------------------------|------------------------------------------------------------------------------|
| **Configure** | `navSys`                    | `nFrame`                       | Navigation frame, especially the INS mechanism                               |
|             | `usegnssvel`                | `true`                         | Whether use GNSS velocity update or not                                         |
|             | `useZUPT`                   | `false`                        | Whether use ZUPT detection and update or not                                     |
|             | `useodonhc`                 | `true`                         | Whether use ODO/NHC update or not (ODO not completed yet)                        |
|             | `usesinglenhc`              | `true`                         | Whether to use single NHC update or not                                             |
|             | `usesingleodo`              | `false`                        | Whether use single ODO update or not (ODO not completed yet)                     |
|             | `onlyinsmech`               | `false`                        | Whether use only the INS mechanism or not                                        |
| **IMU Settings** | `imudataformat`             | `asc`                          | IMU data format (0: imr, 1: asc, 2: txt[7 columns])                           |
|             | `imurawcoordinate`          | `RFU`                          | Raw IMU data in RFU or FRD                                                   |
|             | `imuInitStaticTime`         | `5.0`                          | IMU initial alignment time [min]                                             |
|             | `imuSamplingRate`           | `100.0`                        | IMU sampling rate [Hz]                                                       |
|             | `imuInitialGPSTWeek`        | `2315`                         | IMU initial GPS week                                                         |
|             | `useAttFromRoughAlign`      | `false`                        | Use initial attitude from rough alignment                                    |
| **GNSS Settings** | `useInitalPosVelFromRTK`    | `false`                        | Use initial position and velocity from RTK solution                          |
| **State Settings** | `estimateImuScale`          | `true`                         | Estimate IMU scale (21-dimension states if true, 15 if false)                 |
| **Plot Settings** | `plotResults`              | `true`                         | Use `matplotlib` to automatically plot navigation results                    |
| **File Paths** | `file-roverobs-renix`       | `F:\GINS-Navi\data\WHU20240522-XWGI7660\rover.24O` | Rover observation file path                 |
|             | `file-baseobs-renix`        | `F:\GINS-Navi\data\WHU20240522-XWGI7660\base.24O`  | Base observation file path                    |
|             | `file-navi-renix`           | `F:\GINS-Navi\data\WHU20240522-XWGI7660\base.24N`  | Navigation file path                          |
|             | `gnssfilepath`              | `F:\GINS-Navi\data\WHU20240522-XWGI7660\GNSS-RTK.txt` | GNSS file path                             |
|             | `imrfilepath`               | `F:\GINS-Navi\data\WHU20240522-XWGI7660\rover.imr` | IMR file path                              |
|             | `ascfilepath`               | `F:\GINS-Navi\data\WHU20240522-XWGI7660\rover.ASC` | ASC file path                              |
|             | `imutxtfilepath`            | ` `                            | IMU TXT file path                          |
|             | `odofilepath`               | ` `                            | ODO file path                             |
|             | `outputpath`                | `F:\GINS-Navi\data\WHU20240522-XWGI7660\` | Output path                                |
| **Initial Information** | `startweek`               | `2315`                         | Start week (or you can use 'auto')                       |
|             | `endweek`                 | `2315`                         | End week (or you can use 'auto')                         |
|             | `starttow`                 | `291724.0`                     | Start time of week (or you can use 'auto')              |
|             | `endtow`                   | `295006.0`                     | End time of week (or you can use 'auto')                |
|             | `initpos`                  | `[30.528078962, 114.355762445, 40.9652]` | Initial position [deg, deg, m] (derived from RTK results)     |
|             | `initvel`                  | `[-0.002, -0.838, -0.028]`     | Initial velocity [m/s] (derived from RTK results)         |
|             | `initatt`                  | `[0.183, 0.183, 272.55]`       | Initial attitude [deg] (not used if `useAttFromRoughAlign` is true)    |
|             | `initposstd`               | `[1.0, 1.0, 2.0]`              | Initial position standard deviation [m] (N-E-D)            |
|             | `initvelstd`               | `[0.5, 0.5, 0.5]`              | Initial velocity standard deviation [m/s]                   |
|             | `initattstd`               | `[0.5, 0.5, 1.0]`              | Initial attitude standard deviation [deg]                    |
|             | `initgyrbias`              | `[0.5, -1.0, 6.0]`             | Initial gyroscope bias [deg/h]                              |
|             | `initaccbias`              | `[-200, 700, 0]`               | Initial accelerometer bias [mGal]                           |
|             | `initgyrscale`             | `[0, 0, 0]`                    | Initial gyroscope scale [ppm]                               |
|             | `initaccscale`             | `[0, 0, 0]`                    | Initial accelerometer scale [ppm]                           |
|             | `initgyrbiasstd`           | `[50, 50, 50]`                 | Initial gyroscope bias standard deviation [deg/h]           |
|             | `initaccbiasstd`           | `[250, 250, 250]`              | Initial accelerometer bias standard deviation [mGal]        |
|             | `initgyrscalestd`          | `[1000.0, 1000.0, 1000.0]`     | Initial gyroscope scale standard deviation [ppm]            |
|             | `initaccscalestd`          | `[1000.0, 1000.0, 1000.0]`     | Initial accelerometer scale standard deviation [ppm]        |
|             | `gyrarw`                   | `[0.24, 0.24, 0.24]`           | Gyroscope angle random walk [deg/s/sqrt(h)]                 |
|             | `accvrw`                   | `[0.24, 0.24, 0.24]`           | Accelerometer velocity random walk [m/s/sqrt(h)]            |
|             | `gyrbiasstd`               | `[50.0, 50.0, 50.0]`           | Gyroscope bias standard deviation [deg/h]                   |
|             | `accbiasstd`               | `[250.0, 250.0, 250.0]`        | Accelerometer bias standard deviation [mGal]                |
|             | `gyrscalestd`              | `[1000.0, 1000.0, 1000.0]`     | Gyroscope scale standard deviation [ppm]                    |
|             | `accscalestd`              | `[1000.0, 1000.0, 1000.0]`     | Accelerometer scale standard deviation [ppm]                |
|             | `corrtime`                 | `1.0`                          | Correlation time [h]                                        |
| **Installation Parameters** | `antlever`                | `[0.164, -0.035, -0.890]`     | Antenna lever arm [m] (NED)                                 |
|             | `odolever`                | `[0.0, 0.0, 0.0]`              | Odometer lever arm [m]                                      |
|             | `installangle`            | `[0.0, 0.0, 0.0]`              | Installation angle [deg]                                     |
| **Measurement Noise** | `odonhc_measnoise`          | `[0.10, 0.07, 0.07]`           | ODO/NHC measurement noise [m/s]                             |
|             | `zupt_vmeasnoise`         | `[0.10, 0.10, 0.10]`           | ZUPT velocity measurement noise [m/s]                       |
|             | `zupt_wmeasnoise`         | `50`                           | ZUPT angular velocity measurement noise [deg/h]             |
|             | `CodeNoise`               | `4.0`                          | Code noise [m]                                              |
|             | `CPNoise`                 | `0.05`                         | Carrier phase noise [m]                                      |
| **Update Frequency** | `odonhcupdaterate`         | `1.0`                          | ODO/NHC update rate [Hz]                                    |
|             | `zuptupdaterate`          | `1.0`                          | ZUPT update rate [Hz]                                       |
| **RTK Post Options** | `pos1-posmode`             | `kinematic`                   | Positioning mode                                            |
|             | `pos1-frequency`          | `l1+2+3`                       | Frequency                                                   |
|             | `pos1-soltype`            | `combined`                     | Solution type                                               |
|             | `pos1-elmask`             | `10`                           | Elevation mask [deg]                                        |
|             | `pos1-snrmask_r`          | `off`                          | SNR mask for rover                                          |
|             | `pos1-snrmask_b`          | `off`                          | SNR mask for base                                           |
|             | `pos1-snrmask_L1`         | `0,0,0,0,0,0,0,0,0`            | SNR mask for L1                                             |
|             | `pos1-snrmask_L2`         | `0,0,0,0,0,0,0,0,0`            | SNR mask for L2                                             |
|             | `pos1-snrmask_L5`         | `0,0,0,0,0,0,0,0,0`            | SNR mask for L5                                             |
|             | `pos1-dynamics`           | `on`                           | Dynamics mode                                               |
|             | `pos1-tidecorr`           | `off`                          | Tidal correction                                            |
|             | `pos1-ionoopt`            | `brdc`                         | Ionospheric option                                          |
|             | `pos1-tropopt`            | `saas`                         | Tropospheric option                                         |
|             | `pos1-sateph`             | `brdc`                         | Satellite ephemeris option                                  |
|             | `pos1-posopt1`            | `off`                          | Positioning option 1                                        |
|             | `pos1-posopt2`            | `off`                          | Positioning option 2                                        |
|             | `pos1-posopt3`            | `off`                          | Positioning option 3                                        |
|             | `pos1-posopt4`            | `off`                          | Positioning option 4                                        |
|             | `pos1-posopt5`            | `off`                          | Positioning option 5                                        |
|             | `pos1-posopt6`            | `off`                          | Positioning option 6                                        |
|             | `pos1-exclsats`           | ` `                            | Excluded satellites                                         |
|             | `pos1-navsys`             | `59`                           | Navigation systems                                          |
|             | `pos2-armode`             | `off`                          | Ambiguity resolution mode                                   |
|             | `pos2-gloarmode`          | `on`                           | GLONASS AR mode                                             |
|             | `pos2-bdsarmode`          | `on`                           | BDS AR mode                                                 |
|             | `pos2-arthres`            | `3`                            | Ambiguity resolution threshold                              |
|             | `pos2-arthres1`           | `0.9999`                       | Ambiguity resolution threshold 1                            |
|             | `pos2-arthres2`           | `0.25`                         | Ambiguity resolution threshold 2                            |
|             | `pos2-arthres3`           | `0.1`                          | Ambiguity resolution threshold 3                            |
|             | `pos2-arthres4`           | `0.05`                         | Ambiguity resolution threshold 4                            |
|             | `pos2-arlockcnt`          | `0`                            | AR lock count                                               |
|             | `pos2-arelmask`           | `0`                            | Elevation mask for AR [deg]                                 |
|             | `pos2-arminfix`           | `10`                           | Minimum number of satellites for AR                         |
|             | `pos2-armaxiter`          | `1`                            | Maximum number of iterations for AR                         |
|             | `pos2-elmaskhold`         | `0`                            | Elevation mask for hold [deg]                               |
|             | `pos2-aroutcnt`           | `5`                            | Output count for AR                                         |
|             | `pos2-maxage`             | `60`                           | Maximum age of differential [s]                             |
|             | `pos2-syncsol`            | `off`                          | Synchronized solution                                       |
|             | `pos2-slipthres`          | `0.05`                         | Slip threshold [m]                                          |
|             | `pos2-rejionno`           | `30`                           | Ionospheric delay rejection threshold [m]                   |
|             | `pos2-rejgdop`            | `30`                           | GDOP rejection threshold                                    |
|             | `pos2-niter`              | `1`                            | Number of iterations                                        |
|             | `pos2-baselen`            | `0`                            | Base station length [m]                                     |
|             | `pos2-basesig`            | `0`                            | Base station sigma [m]                                      |
| **Output**  | `out-solformat`           | `llh`                          | Solution format                                             |
|             | `out-outhead`             | `on`                           | Output header                                               |
|             | `out-outopt`              | `on`                           | Output option                                               |
|             | `out-outvel`              | `on`                           | Output velocity                                             |
|             | `out-timesys`             | `gpst`                         | Time system                                                 |
|             | `out-timeform`            | `tow`                          | Time format                                                 |
|             | `out-timendec`            | `3`                            | Time decimal places                                         |
|             | `out-degform`             | `deg`                          | Degree format                                               |
|             | `out-fieldsep`            | ` `                            | Field separator                                             |
|             | `out-outsingle`           | `off`                          | Output single                                               |
|             | `out-maxsolstd`           | `0`                            | Maximum solution standard deviation [m]                     |
|             | `out-height`              | `geodetic`                     | Height reference                                            |
|             | `out-geoid`               | `internal`                     | Geoid model                                                 |
|             | `out-solstatic`           | `all`                          | Static solution                                             |
|             | `out-nmeaintv1`           | `0`                            | NMEA interval 1 [s]                                         |
|             | `out-nmeaintv2`           | `0`                            | NMEA interval 2 [s]                                         |
|             | `out-outstat`             | `off`                          | Output status                                               |
| **Statistics** | `stats-eratio1`            | `100`                          | Error ratio 1                                               |
|             | `stats-eratio2`            | `100`                          | Error ratio 2                                               |
|             | `stats-errphase`           | `0.003`                        | Phase error [m]                                             |
|             | `stats-errphaseel`         | `0.003`                        | Phase error elevation [m]                                   |
|             | `stats-errphasebl`         | `0`                            | Phase error baseline [m/10km]                               |
|             | `stats-errdoppler`         | `1`                            | Doppler error [Hz]                                          |
|             | `stats-stdbias`            | `30`                           | Standard bias [m]                                           |
|             | `stats-stdiono`            | `0.03`                         | Standard ionospheric error [m]                              |
|             | `stats-stdtrop`            | `0.3`                          | Standard tropospheric error [m]                             |
|             | `stats-prnaccelh`          | `1.0`                          | PRN acceleration horizontal [m/s²]                          |
|             | `stats-prnaccelv`          | `0.1`                          | PRN acceleration vertical [m/s²]                            |
|             | `stats-prnbias`            | `1e-05`                        | PRN bias [m]                                                |
|             | `stats-prniono`            | `0.001`                        | PRN ionospheric error [m]                                   |
|             | `stats-prntrop`            | `0.0001`                       | PRN tropospheric error [m]                                  |
|             | `stats-prnpos`             | `0`                            | PRN position error [m]                                      |
|             | `stats-clkstab`            | `5e-12`                        | Clock stability [s/s]                                       |
| **Antenna 1** | `ant1-postype`             | `llh`                          | Position type                                               |
|             | `ant1-pos1`                | `90`                           | Position 1 [deg|m]                                          |
|             | `ant1-pos2`                | `0`                            | Position 2 [deg|m]                                          |
|             | `ant1-pos3`                | `-6335367.6285`                | Position 3 [m|m]                                            |
|             | `ant1-anttype`             | ` `                            | Antenna type                                                |
|             | `ant1-antdele`             | `0`                            | Antenna delta E [m]                                         |
|             | `ant1-antdeln`             | `0`                            | Antenna delta N [m]                                         |
|             | `ant1-antdelu`             | `0`                            | Antenna delta U [m]                                         |
| **Antenna 2** | `ant2-postype`             | `llh`                          | Position type                                               |
|             | `ant2-pos1`                | `30.528231`                    | Position 1 [deg|m]                                          |
|             | `ant2-pos2`                | `114.356985`                   | Position 2 [deg|m]                                          |
|             | `ant2-pos3`                | `42.6548`                      | Position 3 [m|m]                                            |
|             | `ant2-anttype`             | ` `                            | Antenna type                                                |
|             | `ant2-antdele`             | `0`                            | Antenna delta E [m]                                         |
|             | `ant2-antdeln`             | `0`                            | Antenna delta N [m]                                         |
|             | `ant2-antdelu`             | `0`                            | Antenna delta U [m]                                         |
|             | `ant2-maxaveep`            | `0`                            | Maximum averaging epochs                                    |
|             | `ant2-initrst`             | `off`                          | Initial reset                                               |
| **Miscellaneous** | `misc-timeinterp`          | `off`                          | Time interpolation                                          |
|             | `misc-sbasatsel`           | `0`                            | SBAS satellite selection                                    |
|             | `misc-rnxopt1`             | ` `                            | RINEX option 1                                              |
|             | `misc-rnxopt2`             | ` `                            | RINEX option 2                                              |
|             | `misc-pppopt`              | ` `                            | PPP option                                                  |
|             | `file-satantfile`          | ` `                            | Satellite antenna file                                      |
|             | `file-rcvantfile`          | ` `                            | Receiver antenna file                                       |
|             | `file-staposfile`          | ` `                            | Station position file                                       |
|             | `file-geoidfile`           | ` `                            | Geoid file                                                  |
|             | `file-ionofile`            | ` `                            | Ionosphere file                                             |
|             | `file-dcbfile`             | ` `                            | DCB file                                                    |
|             | `file-eopfile`             | ` `                            | EOP file                                                    |
|             | `file-blqfile`             | ` `                            | BLQ file                                                    |
|             | `file-tempdir`             | ` `                            | Temporary directory                                         |
|             | `file-geexefile`           | ` `                            | GEE executable file                                         |
|             | `file-solstatfile`         | ` `                            | Solution status file                                        |
|             | `file-tracefile`           | ` `                            | Trace file                                                  |

## 2 Compilation and Execution

### 2.1 Source Code and Compilation

The project is managed using CMake. All source code is located in the ./include and ./src directories. It is recommended to use Mingw for compilation.

### 2.2 Dependencies

On Windows, the software relies on the WIN32 API for window management. Third-party libraries such as Eigen, MatPlotlib-cpp, tqdm-cpp, and thread-pool are included in the ./ThirdParty directory.

The CMakeLists file is fully configured, allowing users to compile the program in one simple step. The recommended IDE is CLion.

### 2.3 Output Results

The "Output Path" in the configuration file specifies the output paths for navigation results (including STD), IMU error information (including STD), and visualization results (2D and 3D trajectories, elevation, Euler angles [RPY], IMU bias/scale factor errors, and STD).

### 2.4 Visualization

This C++ GUI program uses the Matplotlib library to automatically generate result plots, including 2D and 3D trajectories, velocity/attitude, IMU errors, and STD. If this affects performance, it can be disabled as detailed in the configuration file.

The 'nav_result.pos' file in the output directory conforms to the RTKlib standard and can be directly imported into rtkplot.exe for RTK result visualization.

## 3 Datasets

### 3.1 Test Data

The ./data/ directory contains six sets of IMU data with varying accuracies, including:

- LeadorA15 (navigation grade, high accuracy)
- XWGI7680 (tactical grade, medium-high accuracy)
- CHC CGI-430 (MEMS, low accuracy)
- InvenSense ICM-20602 (MEMS, low accuracy)

The truth.nav file contains reference truth data obtained through backward smoothing.

Configuration files for these datasets are located in the ./conf/ directory.

### 3.2 Compatible Data Formats

The software supports NovAtel's IMR and ASC format data, as well as custom seven-column txt data (see the [i2nav open-source dataset](https://github.com/i2Nav-WHU/awesome-gins-datasets) for details).

## 4 Inheritable Modules

The software provides several inheritable modules, including a progress bar class (tqdm), a configuration manager class (configManager), file read/write classes (fileloader/filesaver), algebra calculation class (algebra), and a plotting class (matplotlib-cpp).

## 5 Features

- Compatible with both Windows (GUI) and Linux (CUI)

- Detailed parameter settings for the algorithm

- Lightweight and efficient GUI design using native Windows API, similar to RTKlib style

- GUI features include mailto help, GUI progress bar, and robust configuration read/write error handling

- Integrated RTK post-processing module from RTKlib, providing high-precision algorithms and detailed computation strategies

- Object-oriented core code for loosely coupled integration (factory pattern, abstraction, function templates, etc.), with extensive error handling and clean code

- Efficient algorithm implementation using thread pool for parallel task submission

- Single-header inheritable classes for configuration, file read/write, algebra calculation, and tqdm progress bar

- GNSS information quality control (observation information), such as setting absolute and relative thresholds for GNSS position and velocity observation updates, constraining data in the x, y, and z directions to exclude poor-quality data

- Compatible with various inertial navigation data formats (asc, imr, txt)

- Week-crossing data handling (under testing)

- Navigation framework options for n-frame and e-frame (e-frame has a minor bug yet to be fixed)

- Estimation/non-estimation of IMU scale factors (ESKF state vector is 21-dimensional/15-dimensional)

- Robust algorithm tested with multiple datasets: navigation grade, tactical grade, MEMS

---

The algorithm is not perfect, lacking backward smoothing and unsmoothed results. 

Further research in the field of integrated navigation is planned during graduate studies. 

The current content, which has been developed during my undergraduate studies, remains at a reproduction level.

---