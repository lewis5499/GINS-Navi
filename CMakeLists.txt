cmake_minimum_required(VERSION 3.0.0)

# 设置策略 CMP0079 为 NEW，避免警告
if (POLICY CMP0079)
    cmake_policy(SET CMP0079 NEW)
endif ()

project(GINS VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# 根据构建类型设置编译选项和输出目录
if (CMAKE_BUILD_TYPE MATCHES "Release")
    add_compile_options(-O3)
    set(CMAKE_BUILD_POSTFIX "_release")
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib/Release)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/Release)
elseif (CMAKE_BUILD_TYPE MATCHES "Debug")
    add_compile_options(-g3 -Wall -Wextra)
    set(CMAKE_BUILD_POSTFIX "_debug")
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib/Debug)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/Debug)
endif ()

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/file)
include_directories(${PROJECT_SOURCE_DIR}/include/gnss)
include_directories(${PROJECT_SOURCE_DIR}/include/ins)
include_directories(${PROJECT_SOURCE_DIR}/include/state)
include_directories(${PROJECT_SOURCE_DIR}/include/manager)
include_directories(${PROJECT_SOURCE_DIR}/include/module)

include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/eigen3)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/thread-pool/include)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/matplotlib)

if (WIN32)
    set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")
    set(CMAKE_EXE_LINKER_FLAGS "-static")
    link_libraries(ws2_32)
    link_libraries(comctl32)
else()
    link_libraries(pthread)
endif ()

# 添加 ThirdParty/matplotlib 子目录
add_subdirectory(${PROJECT_SOURCE_DIR}/ThirdParty/matplotlib)

# 链接 matplotlib_cpp 库
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)

# 确保正确链接 matplotlib_cpp
target_link_libraries(matplotlib_cpp INTERFACE Python3::Python Python3::Module)
if (Python3_NumPy_FOUND)
    target_link_libraries(matplotlib_cpp INTERFACE Python3::NumPy)
else ()
    target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif ()



# 添加源文件
file(GLOB SRC_GINS src/*.c src/*.cpp
        src/module/rtklib/*.cpp src/module/rtklib/*.c src/module/rtklib/rcv/*.cpp src/module/rtklib/rcv/*.c
        src/file/*.c src/file/*.cpp
        src/gnss/*.c src/gnss/*.cpp
        src/ins/*.c src/ins/*.cpp
        src/manager/*.c src/manager/*.cpp
        src/module/*.c src/module/*.cpp
        src/state/*.c src/state/*.cpp)

# 创建可执行文件
add_executable(GINS ${SRC_GINS} res/gins.rc)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# 链接 matplotlib_cpp
target_link_libraries(GINS PRIVATE matplotlib_cpp)

# 设置编译定义
target_compile_definitions(GINS PRIVATE ENAGLO ENAGAL ENACMP ENAQZS ENAIRN ENALEO TRACE)
