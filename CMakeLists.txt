# 声明要求的 cmake 最低版本
cmake_minimum_required(VERSION 3.10)
# 声明一个 cmake 工程
set(PROJECT_NAME BinocularVisionAlg)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 11)

#设置调试模式，debug会输出调试信息
SET(CMAKE_BUILD_TYPE DEBUG)


###添加OPENCV库
#find_package(PkgConfig REQUIRED)
###opencv&&ffmpeg
#set(Opencv_path "/opt/opencv454")
###set PKG_CONFIG_LIBDIR
#set(ENV{PKG_CONFIG_LIBDIR} "${Opencv_path}/lib/pkgconfig")
###opencv
#pkg_search_module(OpenCV REQUIRED IMPORTED_TARGET opencv4)
##list(APPEND Samples_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
##list(APPEND Samples_LINK_LIBS ${OpenCV_LDFLAGS})
##set install path
#set(CMAKE_BUILD_RPATH "${Opencv_path}/lib")
#include_directories(${OpenCV_INCLUDE_DIRS})
#
#add_link_options("LINKER:-rpath-link,/opt/opencv454/lib")


#1.自动去找OpenCVConfig.cmake，OpenCV-config.cmake 两个文件，进而确定你要引入的opencv头文件和库函数在哪里
find_package(OpenCV REQUIRED)

#2.手动
set(OpenCV_INCLUDE_DIRS "/cytech_ai/env/dependency/opt/include/opencv4")
set(OpenCV_LDFLAGS "/opt/BinocularVisionAlg/lib")
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LDFLAGS})

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "    libraries: ${OpenCV_LDFLAGS}")

#添加日志库
set(GLOG_INCLUDE "/opt/glog/include/")
set(GLOG_LIB "/opt/BinocularVisionAlg/lib")

include_directories(${GLOG_INCLUDE})
link_directories(${GLOG_LIB})

message(STATUS "GLOG library status:")
message(STATUS "    include path: ${GLOG_INCLUDE}")
message(STATUS "    libraries: ${GLOG_LIB}")

#添加pylon
set(PYLON_INCLUDE "/opt/pylon7/include/")
set(PYLON_LIB "/opt/pylon7/lib")

include_directories(${PYLON_INCLUDE})
link_directories(${PYLON_LIB})

message(STATUS "Basler library status:")
message(STATUS "    include path: ${PYLON_INCLUDE}")
message(STATUS "    libraries: ${PYLON_LIB}")

#添加cuda
set(CUDA_INCLUDE "/usr/local/cuda/include")
set("CUDA_LIB" "/usr/local/cuda/lib64")

include_directories(${CUDA_INCLUDE})
link_directories(${CUDA_LIB})

message(STATUS "CUDA library status:")
message(STATUS "    include path: ${CUDA_INCLUDE}")
message(STATUS "    libraries: ${CUDA_LIB}")


#添加自己写的其他函数文件，注意这个名字不能和工程名冲突
aux_source_directory(include DIR_HEADER)
include_directories(${DIR_HEADER})	#添加头文件

aux_source_directory(src DIR_CPP)

# 如果想生成动态库，SHARE .so
#add_library(${PROJECT_NAME} SHARED  ${DIR_CPP})
#VERSION 指代动态库版本，SOVERSION 指代 API 版本
#set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 1.0.0.1 OUTPUT_NAME ${PROJECT_NAME})
add_library(${PROJECT_NAME} SHARED  ${DIR_CPP})
set(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})


#生成静态库 STATIC .a
#add_library(${PROJECT_NAME}_static STATIC ${DIR_CPP})
#set_target_properties(${PROJECT_NAME}_static PROPERTIES VERSION 1.0.0.1 OUTPUT_NAME ${PROJECT_NAME})


#编译可执行程序
#add_executable(${PROJECT_NAME} testdemo.cpp ${DIR_CPP})

#指定链接给定目标和/或其依赖项时要使用的库或标志。将传播链接库目标的使用要求。目标依赖项的使用要求会影响其自身源的编译。
#target_link_libraries不仅可以链接动态库，静态库也可以用它来链接。
#参数解释：
#target：指定要加载动态库的文件的名字
#该文件可能是一个源文件
#该文件可能是一个动态库文件
#该文件可能是一个可执行文件

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LDFLAGS}
        -lopencv_core
        -lopencv_imgproc
        -lopencv_imgcodecs
        -lopencv_highgui
        -lopencv_core
        -lopencv_calib3d

        ${GLOG_LIB}
        -lglog
        ${PYLON_LIB}
        -lXmlParser_gcc_v3_1_Basler_pylon
        -lLog_gcc_v3_1_Basler_pylon
        -lGCBase_gcc_v3_1_Basler_pylon
        -lgxapi
        -luxapi
        -lpylon_TL_gige
        -lpylonc
        -lpylonbase
        -lpylonutility
        -lGenApi_gcc_v3_1_Basler_pylon
        -lPylonDataProcessingCore

        ${CUDA_LIB}


        )
