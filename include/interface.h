#pragma once

#define GOOGLE_GLOG_DLL_DECL
#define GLOG_NO_ABBREVIATED_SEVERITIES

#include <memory>
#include<iomanip>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <string>
#include <chrono>
#include <fstream>
#include <string>
#include <string>
#include <dirent.h>
#include <cstdio>
#include<unistd.h>  //linux下创建文件夹
#include<sys/stat.h>   // 注意：这个头文件在sys下面

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include<opencv2/calib3d/calib3d.hpp> //双目标定的头文件

#include <mutex>
#include <vector>
#include <glog/logging.h>

#include "../include/tinystr.h"
#include "../include/tinyxml.h"

//加载balser 工业相机的PYLON API.
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/_BaslerUniversalCameraParams.h>
#include <pylon/BaslerUniversalInstantCameraArray.h>
#include <pylon/Info.h>
#include <pylon/gige/GigETransportLayer.h>
#include <pylon/gige/ActionTriggerConfiguration.h>
#include <pylon/gige/BaslerGigEDeviceInfo.h>
//命名空间.
using namespace Pylon;
using namespace GenApi;
using namespace Basler_UniversalCameraParams;
using namespace std;
using namespace cv;

