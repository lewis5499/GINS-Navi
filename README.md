# GNSS/INS 组合导航GUI软件

GINS-Navi软件是一款实现了GNSS/INS松组合算法的组合导航软件，兼容windows(GUI)与Linux(CUI)。

联系作者：刘恒祯，武汉大学，lewis5499@whu.edu.cn

## 1 配置

项目的配置文件为./conf目录下的*.conf，可通过配置文件提供详细的解算参数选择：

## 2 程序编译与运行

### 2.1 源码及编译

项目采用CMake管理, ./include ./src目录下提供了所有源码。建议使用Mingw编译。

### 2.2 依赖库

windows下依赖WIN32 API以实现窗体, 第三方的Eigen库、MatPlotlib-cpp库、tqdm-cpp库、thread-pool库等内容均在./ThirdParty目录下。

CMakeLists已经配置完善，用户简单一步编译程序即可，推荐CLion。

### 2.3 运行结果

配置中的“Output Path”设置了导航结果信息(含STD)、IMU误差信息(含STD)的输出路径，

### 2.4 可视化

本C++窗体程序调用了Matplotlib库进行自动的结果图绘制：二维平面轨迹、三维轨迹、vel/att、imu误差以及STD等。如果您觉得影响了效率，则可禁用之，请详见配置文件。

输出目录下的'nav_result.pos'文件对接rtklib标准，亦可以直接使用rtkplot.exe导入该文件进行RTK结果可视化。

## 3 数据集

### 3.1 测试数据

./data/ 目录下放置了六组不同精度IMU的数据，主要有：

LeadorA15（导航级，高精度），XWGI7680（战术级，中高精度），CHC CGI-430 （MEMS，低精度），InvenSense ICM-20602（MEMS，低精度）。

truth.nav为反向平滑的参考真值数据。

./conf/目录下存放了对应数据的配置文件

### 3.2 兼容的数据格式

兼容NovAtel公司的IMR、ASC格式数据，以及自定义的七列txt数据（请详见i2nav开源数据集的说明）。

## 4 模块继承

提供了独立可继承的模块：可继承的进度条类(tqdm)、增删改查配置类(configManager)、文件读写类(fileloader/filesaver)、代数计算类(algebra)等等。

## 5 特色

仿 rtklib 风格精美窗体，Windows原生 API， 轻量高效

mailto 帮助，GUI progress  bar，窗体容错处理

同时兼容 win(GUI)、linux(CUI)，Cmake 管理项目

面向对象思想，容错处理，项目代码简洁; 工厂模式，抽象的思想。函数模版

线程池并行提交任务，算法高效

可继承的配置类、文件读写类、代数计算类

可继承的二次开发的 tqdm 进度条类，均为单头文件

详细解算参数设置，

集成 rtklib，算法高精

兼容asc imr txt 多种惯导数据选择，

跨周的容错处理

提供 n系 e系编排的导航框架选择

估计/不估计 IMU比例因子

多数据集测试，算法鲁棒：导航级、战术级、MEMS
