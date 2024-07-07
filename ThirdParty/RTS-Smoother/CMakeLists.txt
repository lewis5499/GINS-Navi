cmake_minimum_required (VERSION 3.10)
project(BPRTS VERSION 1.0.0
        DESCRIPTION "A Rauch Tung Striebel Smoother on boxplus manifolds"
        LANGUAGES CXX)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")

find_package(ADEKF CONFIG REQUIRED)
find_package(ADEKF_VIZ CONFIG REQUIRED)
find_package(Boost 1.41 COMPONENTS system  REQUIRED)


# Configure project for usage of Qt5
find_package(Qt5 COMPONENTS Core Gui Widgets PrintSupport Svg Xml  REQUIRED)

# include JKQTPlotter
find_package(JKQTCommonSharedLib REQUIRED)
find_package(JKQTMathTextSharedLib REQUIRED)
find_package(JKQTPlotterSharedLib REQUIRED)
#Include Threads to start ekf loop in other thread
find_package(Threads REQUIRED)
#find VTK
find_package(VTK REQUIRED)

# include all components

add_executable(RadarFlightExample MACOSX_BUNDLE examples/radar_flight.cpp)
target_include_directories(RadarFlightExample PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} include/)
target_link_libraries(RadarFlightExample PUBLIC  ${Boost_LIBRARIES}  ADEKF_VIZ ${CMAKE_THREAD_LIBS_INIT})
target_compile_features(RadarFlightExample PUBLIC  cxx_std_17 )


#add_executable(BugTest MACOSX_BUNDLE minimal/no_access_without_this.cpp)
#target_compile_features(BugTest PUBLIC  cxx_std_17 )
