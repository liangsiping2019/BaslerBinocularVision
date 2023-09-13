#pragma once
#pragma pack(1)

//输入
struct INPUT_DATA {
	unsigned char *left_img_data;
	int left_img_width;
	int left_img_height;
	int left_img_channels;

	unsigned char *right_img_data;
	int right_img_width;
	int right_img_height;
	int right_img_channels;

};


//双目视觉-输出校正后的图像
struct OUTPUT_DATA {
	INPUT_DATA output;
};

//目标检测框
struct BOX{
	int left_x;
	int left_y;
	int width;
	int height;
};

//校正后的图像做目标检测：在两幅图像中都有时，才传入
struct INPUT_BOX {
	BOX left_box;		//左相机检测到的人的box位置
	BOX right_box;		//右相机检测到的人的box位置
};

//初始化相机参数配置
struct CameraInit {
	const char* CameraIP_L;	//左相机IP
	const char* CameraIP_R;	//右相机IP
	int ExporeTime_L;			//左相机曝光
	int ExporeTime_R;			//右相机曝光
	int Gain_L;					//左相机增益
	int Gain_R;					//右相机增益

};

#pragma pack()
extern "C" {

	//版本信息
	void GetAlgVersion(const char version_code[2048], const char update_version_info[2048]);

	/***********************************************************************************************************
		函数名称：Init(初始化)
		参数说明：
		config_path：初始化文件路径
		返回值：
		* 算法句柄，初始化成功返回有效指针，初始化失败返回空指针
	********************************************************************************************************/
	void*  InitMethod(const char* config_path, CameraInit cameraconfig);
	
	/***********************************************************************************************************
		函数名称：CameraOnline(初始化)
		参数说明：
		onlineimage：实时采集的图像
	    skipframe：跳帧
		返回值：
		* 算法句柄，初始化成功返回有效指针，初始化失败返回空指针
	********************************************************************************************************/
	 int  CameraOnline(void* handle,INPUT_DATA* onlineimage,int skipframe);

	/***********************************************************************************************************
		函数名称：BinocularRecitify(双目视觉畸变校正)
		参数说明：
		handle:算法句柄
		input：输入的相机数据 左右2个相机数据
		out：输出校正后的图像
	********************************************************************************************************/
	 void  BinocularRecitify(void* handle, INPUT_DATA *input,OUTPUT_DATA *out);
	
	/***********************************************************************************************************
		函数名称：BinocularComputeDis(双目视觉测距)
		参数说明：
		handle:算法句柄
		input_box：校正后的左右图像中的目标box；
		dis：输出当前目标的距离信息
		

	********************************************************************************************************/
	 void  BinocularComputeDis(void* handle, INPUT_BOX *input_box,float *dis);

	/***********************************************************************************************************
		函数名称：Release(释放)
		参数说明：
		handle：算法句柄
		********************************************************************************************************/
	 void  Release(void* handle);
}

