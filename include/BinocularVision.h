#pragma once
#include "../include/interface.h"
#include "../include/BinocularVisionInterface.h"
class BinocularVisionDetector {
public:
	BinocularVisionDetector();
	~BinocularVisionDetector();
	int ReadXmlDocument(const std::string& config_file);
	int InitAlg(const char* config_path, CameraInit cameraconfig);
	int OnlineCameraGet(INPUT_DATA* onlineimage,int skipframe);	//获取相机数据
	
	int InitBaslerImages(Mat &imageL,Mat &imageR);//初始化工业相机，获取相机数据
	int BinocularVisionInit();
	void binocular_vision(Mat rgbImageL, Mat rgbImageR, Mat& rectifyImageL1, Mat& rectifyImageR1);
	void stereo_check(Mat grayImageL, Mat grayImageR, Mat& rectifyImageL, Mat& rectifyImageR);
	void BinocularVisionComputeDis(INPUT_BOX *input_box,float *dis);

	float compute_distance(Mat xyz, float up_x, float up_y, float down_x, float down_y);
	void Release();
	bool loginitflag = true;
	//------------------config配置文件参数-------------------------
	//远轨参数--双目相机
	float left_fx, left_fy, left_cx, left_cy;
	int left_jibian_num;
	float left_p1, left_p2, left_k1, left_k2, left_k3;

	float right_fx, right_fy, right_cx, right_cy;
	int right_jibian_num;
	float right_p1, right_p2, right_k1, right_k2, right_k3;

	//旋转矩阵
	float double_xz_x1, double_xz_x2, double_xz_x3;
	float double_xz_x4, double_xz_x5, double_xz_x6;
	float double_xz_x7, double_xz_x8, double_xz_x9;

	//平移矩阵
	float double_py_x1, double_py_x2, double_py_x3;

	float cameradis;		//两个相机之间的间距
	float camera_f;			//焦距f
	float shicha_offset;	//视差偏移量
	int save_flag;
	int camera_online_flag;	//在线接相机

	//string save_path = R"(E:\project\双目视觉技术\BinocularVisionAlg\x64\Release\alg_log\\)";
	string save_path = R"(/cytech_ai/env/BinocularVision/lib/alg_log/)";

	//双目视觉参数
	Mat mapLx, mapLy, mapRx, mapRy;
	Size imageSize = Size(2448, 2048);
	Mat Q;//投影矩阵-矩阵Q可以把像素坐标系下的点转换到世界坐标系下
	Rect validROIL, validROIR;

	//工业相机配置
	enum BaslerCamera_Type {
		Type_Basler_Freerun,				//设置相机的内触发
		Type_Basler_Line1,					//设置相机的外触发
		Type_Basler_ExposureTimeAbs,		//设置相机的曝光时间
		Type_Basler_GainRaw,				//设置相机的增益
		Type_Basler_AcquisitionFrameRateAbs,//设置相机的频率
		Type_Basler_Width,					//图片的宽度
		Type_Basler_Height,					//图片的高度
		Type_Basler_LineSource,				//灯的触发信号
		
	};


	void initCamera();                   //初始化相机
	void GetCameraImage(int skipframe);

	void CloseCamera();                   //关闭相机
	void deleteAll();                    //删除相机
	int  getExposureTime(CInstantCamera& m_basler, BaslerCamera_Type balserparams);   //获得曝光时间
	void setExposureTime(CInstantCamera& m_basler, BaslerCamera_Type balserparams, int time);       //设置曝光时间
	int getGain(CInstantCamera& m_basler, BaslerCamera_Type balserparams);                        //获得增益
	void setGain(CInstantCamera& m_basler, BaslerCamera_Type balserparams,int value);              //设置增益
	void setFrameRate(CInstantCamera& m_basler, BaslerCamera_Type balserparams, int value); //设置帧率
	int getFrameRate(CInstantCamera& m_basler, BaslerCamera_Type balserparams);                   //获得帧率
	
	void OneKeyAutoExTime(CInstantCamera& m_basler);              //一键自动曝光
	
	void SetCamera(CInstantCamera& m_basler, BaslerCamera_Type balserparams, int tmpValue = 0); // 设置各种参数
	int GetCamera(CInstantCamera& m_basler, BaslerCamera_Type balserparams); // 获取各种参数
	
	//void AutoExposureOnce(CInstantCamera& camera);      //自动曝光

	void StartAcquire(CInstantCamera& m_basler);  //开始采集
	void StopAcquire(CInstantCamera& m_basler);   //结束采集
	void StartSnap(CInstantCamera& m_basler);     //抓图

	//创建一个相机实例数组
    CBaslerUniversalInstantCameraArray  cameras = { 2 };
	//CInstantCamera cameral,camerar;
//	CGrabResultPtr ptrGrabResult;   //智能抓取指针
    CBaslerUniversalGrabResultPtr ptrGrabResultl,ptrGrabResultr;
    IGigETransportLayer* pTL;
	CPylonImage pylonImage;// 创建一个Pylonlmage后续将用来创建OpenCV images
	CImageFormatConverter formatConverter;// 新建pylon ImageFormatConverter对象.
	
	string left_camera_ip;
	string right_camera_ip;
	int expore_time_l, gain_l, fps_l;
	int expore_time_r, gain_r, fps_r;
	Mat images[2];
	Mat preimage;
	int count_grab = 0;
	uint32_t DeviceKey;
	uint32_t GroupKey;
	String_t subnet;

	vector<Mat> imagesL;
    vector<Mat> imagesR;

};		
