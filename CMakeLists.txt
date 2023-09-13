# ����Ҫ��� cmake ��Ͱ汾
cmake_minimum_required(VERSION 3.10)
# ����һ�� cmake ����
set(PROJECT_NAME BinocularVisionAlg)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 11)

#���õ���ģʽ��debug�����������Ϣ
SET(CMAKE_BUILD_TYPE DEBUG)


###����OPENCV��
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


#1.�Զ�ȥ��OpenCVConfig.cmake��OpenCV-config.cmake �����ļ�������ȷ����Ҫ�����opencvͷ�ļ��Ϳ⺯��������
find_package(OpenCV REQUIRED)

#2.�ֶ�
set(OpenCV_INCLUDE_DIRS "/cytech_ai/env/dependency/opt/include/opencv4")
set(OpenCV_LDFLAGS "/opt/BinocularVisionAlg/lib")
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LDFLAGS})

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "    libraries: ${OpenCV_LDFLAGS}")

#������־��
set(GLOG_INCLUDE "/opt/glog/include/")
set(GLOG_LIB "/opt/BinocularVisionAlg/lib")

include_directories(${GLOG_INCLUDE})
link_directories(${GLOG_LIB})

message(STATUS "GLOG library status:")
message(STATUS "    include path: ${GLOG_INCLUDE}")
message(STATUS "    libraries: ${GLOG_LIB}")

#����pylon
set(PYLON_INCLUDE "/opt/pylon7/include/")
set(PYLON_LIB "/opt/pylon7/lib")

include_directories(${PYLON_INCLUDE})
link_directories(${PYLON_LIB})

message(STATUS "Basler library status:")
message(STATUS "    include path: ${PYLON_INCLUDE}")
message(STATUS "    libraries: ${PYLON_LIB}")

#����cuda
set(CUDA_INCLUDE "/usr/local/cuda/include")
set("CUDA_LIB" "/usr/local/cuda/lib64")

include_directories(${CUDA_INCLUDE})
link_directories(${CUDA_LIB})

message(STATUS "CUDA library status:")
message(STATUS "    include path: ${CUDA_INCLUDE}")
message(STATUS "    libraries: ${CUDA_LIB}")


#�����Լ�д�����������ļ���ע��������ֲ��ܺ͹�������ͻ
aux_source_directory(include DIR_HEADER)
include_directories(${DIR_HEADER})	#����ͷ�ļ�

aux_source_directory(src DIR_CPP)

# ��������ɶ�̬�⣬SHARE .so
#add_library(${PROJECT_NAME} SHARED  ${DIR_CPP})
#VERSION ָ����̬��汾��SOVERSION ָ�� API �汾
#set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 1.0.0.1 OUTPUT_NAME ${PROJECT_NAME})
add_library(${PROJECT_NAME} SHARED  ${DIR_CPP})
set(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})


#���ɾ�̬�� STATIC .a
#add_library(${PROJECT_NAME}_static STATIC ${DIR_CPP})
#set_target_properties(${PROJECT_NAME}_static PROPERTIES VERSION 1.0.0.1 OUTPUT_NAME ${PROJECT_NAME})


#�����ִ�г���
#add_executable(${PROJECT_NAME} testdemo.cpp ${DIR_CPP})

#ָ�����Ӹ���Ŀ���/����������ʱҪʹ�õĿ���־�����������ӿ�Ŀ���ʹ��Ҫ��Ŀ���������ʹ��Ҫ���Ӱ��������Դ�ı��롣
#target_link_libraries�����������Ӷ�̬�⣬��̬��Ҳ�������������ӡ�
#�������ͣ�
#target��ָ��Ҫ���ض�̬����ļ�������
#���ļ�������һ��Դ�ļ�
#���ļ�������һ����̬���ļ�
#���ļ�������һ����ִ���ļ�

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