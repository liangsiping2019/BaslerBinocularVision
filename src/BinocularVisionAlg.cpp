#include "../include/BinocularVision.h"
BinocularVisionDetector::BinocularVisionDetector() {

}
BinocularVisionDetector::~BinocularVisionDetector() {

}

//读取 xml 文件
int BinocularVisionDetector::ReadXmlDocument(const std::string& config_file)
{
	TiXmlDocument* pNode = new TiXmlDocument();
	if (NULL == pNode)
	{
		LOG(INFO) << "NULL == pNode";
		return -1;
	}
	//使用给定的文件名解析加载文件。 如果成功返回true。
	if (!pNode->LoadFile(config_file.c_str(), TIXML_DEFAULT_ENCODING))
	{
		LOG(INFO) << "无法加载xml配置文件！";
		return -1;
	}

	//返回根节点 即binocularvision
	TiXmlElement* RootElement = pNode->RootElement();

	LOG(INFO) << RootElement->Value();	//输出根元素名称 binocularvision

	if (NULL == RootElement)
	{
		LOG(INFO) << "NULL == pNode";
		return -1;
	}

	//获得LeftCameraIntrinsic节点。
	TiXmlElement* LeftCameraIntrinsic = RootElement->FirstChildElement();

	TiXmlElement* leftfxElement = LeftCameraIntrinsic->FirstChildElement();
	TiXmlElement* leftfyElement = leftfxElement->NextSiblingElement();
	TiXmlElement* leftcxElement = leftfyElement->NextSiblingElement();
	TiXmlElement* leftcyElement = leftcxElement->NextSiblingElement();


	left_fx = std::stof(leftfxElement->FirstChild()->Value());
	left_fy = std::stof(leftfyElement->FirstChild()->Value());
	left_cx = std::stof(leftcxElement->FirstChild()->Value());
	left_cy = std::stof(leftcyElement->FirstChild()->Value());

	//获得LeftDistCoeffs节点。
	TiXmlElement* LeftDistCoeffs = LeftCameraIntrinsic->NextSiblingElement();

	TiXmlElement* tleft_k1 = LeftDistCoeffs->FirstChildElement();
	TiXmlElement* tleft_k2 = tleft_k1->NextSiblingElement();
	TiXmlElement* tleft_p1 = tleft_k2->NextSiblingElement();
	TiXmlElement* tleft_p2 = tleft_p1->NextSiblingElement();
	TiXmlElement* tleft_k3 = tleft_p2->NextSiblingElement();

	left_k1 = std::stof(tleft_k1->FirstChild()->Value());
	left_k2 = std::stof(tleft_k2->FirstChild()->Value());
	left_k3 = std::stof(tleft_p1->FirstChild()->Value());
	left_p1 = std::stof(tleft_p2->FirstChild()->Value());
	left_p2 = std::stof(tleft_k3->FirstChild()->Value());

	//获得LeftCameraIntrinsic节点。
	TiXmlElement* RightCameraIntrinsic = LeftDistCoeffs->NextSiblingElement();

	TiXmlElement* rightfxElement = RightCameraIntrinsic->FirstChildElement();
	TiXmlElement* rightfyElement = rightfxElement->NextSiblingElement();
	TiXmlElement* rightcxElement = rightfyElement->NextSiblingElement();
	TiXmlElement* rightcyElement = rightcxElement->NextSiblingElement();


	right_fx = std::stof(rightfxElement->FirstChild()->Value());
	right_fy = std::stof(rightfyElement->FirstChild()->Value());
	right_cx = std::stof(rightcxElement->FirstChild()->Value());
	right_cy = std::stof(rightcyElement->FirstChild()->Value());

	//获得LeftDistCoeffs节点。
	TiXmlElement* rightDistCoeffs = RightCameraIntrinsic->NextSiblingElement();

	TiXmlElement* tright_k1 = rightDistCoeffs->FirstChildElement();
	TiXmlElement* tright_k2 = tright_k1->NextSiblingElement();
	TiXmlElement* tright_p1 = tright_k2->NextSiblingElement();
	TiXmlElement* tright_p2 = tright_p1->NextSiblingElement();
	TiXmlElement* tright_k3 = tright_p2->NextSiblingElement();

	right_k1 = std::stof(tright_k1->FirstChild()->Value());
	right_k2 = std::stof(tright_k2->FirstChild()->Value());
	right_k3 = std::stof(tright_p1->FirstChild()->Value());
	right_p1 = std::stof(tright_p2->FirstChild()->Value());
	right_p2 = std::stof(tright_k3->FirstChild()->Value());

	//旋转矩阵
	TiXmlElement* TranslationVector = rightDistCoeffs->NextSiblingElement();

	TiXmlElement* TMatrix1 = TranslationVector->FirstChildElement();
	TiXmlElement* TMatrix2 = TMatrix1->NextSiblingElement();
	TiXmlElement* TMatrix3 = TMatrix2->NextSiblingElement();
	TiXmlElement* TMatrix4 = TMatrix3->NextSiblingElement();
	TiXmlElement* TMatrix5 = TMatrix4->NextSiblingElement();
	TiXmlElement* TMatrix6 = TMatrix5->NextSiblingElement();
	TiXmlElement* TMatrix7 = TMatrix6->NextSiblingElement();
	TiXmlElement* TMatrix8 = TMatrix7->NextSiblingElement();
	TiXmlElement* TMatrix9 = TMatrix8->NextSiblingElement();


	double_xz_x1 = std::stod(TMatrix1->FirstChild()->Value());
	double_xz_x2 = std::stod(TMatrix2->FirstChild()->Value());
	double_xz_x3 = std::stod(TMatrix3->FirstChild()->Value());
	double_xz_x4 = std::stod(TMatrix4->FirstChild()->Value());
	double_xz_x5 = std::stod(TMatrix5->FirstChild()->Value());
	double_xz_x6 = std::stod(TMatrix6->FirstChild()->Value());
	double_xz_x7 = std::stod(TMatrix7->FirstChild()->Value());
	double_xz_x8 = std::stod(TMatrix8->FirstChild()->Value());
	double_xz_x9 = std::stod(TMatrix9->FirstChild()->Value());


	//平移矩阵
	TiXmlElement* RotateMartix = TranslationVector->NextSiblingElement();

	TiXmlElement* RMatrix1 = RotateMartix->FirstChildElement();
	TiXmlElement* RMatrix2 = RMatrix1->NextSiblingElement();
	TiXmlElement* RMatrix3 = RMatrix2->NextSiblingElement();

	double_py_x1 = std::stod(RMatrix1->FirstChild()->Value());
	double_py_x2 = std::stod(RMatrix2->FirstChild()->Value());
	double_py_x3 = std::stod(RMatrix3->FirstChild()->Value());


	//相机属性
	TiXmlElement* CameraProperities = RotateMartix->NextSiblingElement();
	TiXmlElement* camerabase = CameraProperities->FirstChildElement();
	TiXmlElement* camerajiaoju = camerabase->NextSiblingElement();
	TiXmlElement* camerashicha = camerajiaoju->NextSiblingElement();
	TiXmlElement* save_f = camerashicha->NextSiblingElement();
	TiXmlElement* cameraonline = save_f->NextSiblingElement();

	cameradis = std::stod(camerabase->FirstChild()->Value());
	camera_f = std::stod(camerajiaoju->FirstChild()->Value());
	shicha_offset = std::stod(camerashicha->FirstChild()->Value());
	save_flag = std::stoi(save_f->FirstChild()->Value());
	camera_online_flag = std::stoi(cameraonline->FirstChild()->Value());


	//相机固定属性
	TiXmlElement* leftcameraproperties = CameraProperities->NextSiblingElement();
	TiXmlElement* leftserialnumber = leftcameraproperties->FirstChildElement();
	TiXmlElement* exporetimel = leftserialnumber->NextSiblingElement();
	TiXmlElement* gainl = exporetimel->NextSiblingElement();
	TiXmlElement* frameratel = gainl->NextSiblingElement();

	left_camera_ip = string(leftserialnumber->FirstChild()->Value());
	expore_time_l = std::stoi(exporetimel->FirstChild()->Value());
	gain_l = std::stoi(gainl->FirstChild()->Value());
	fps_l = std::stoi(frameratel->FirstChild()->Value());


	TiXmlElement* rightcameraproperties = leftcameraproperties->NextSiblingElement();
	TiXmlElement* rightserialnumber = rightcameraproperties->FirstChildElement();
	TiXmlElement* exporetimer = rightserialnumber->NextSiblingElement();
	TiXmlElement* gainr = exporetimer->NextSiblingElement();
	TiXmlElement* framerater = gainr->NextSiblingElement();

	right_camera_ip = string(rightserialnumber->FirstChild()->Value());
	expore_time_r = std::stoi(exporetimer->FirstChild()->Value());
	gain_r = std::stoi(gainr->FirstChild()->Value());
	fps_r = std::stoi(framerater->FirstChild()->Value());

	delete pNode;

	// 3. 打印文件内容
	//pNode->Print();

}

//双目视觉初始化
int BinocularVisionDetector::BinocularVisionInit() {

	Mat Rl, Rr, Pl, Pr;              //校正旋转矩阵R，投影矩阵P 重投影矩阵Q

	//左边相机的内参矩阵
	Mat cameraMatrixL = (Mat_<double>(3, 3) <<
		left_fx, 0, left_cx,
		0, left_fy, left_cy,
		0, 0, 1
		);

	//左边相机的畸变矩阵
	Mat distCoeffL = (Mat_<double>(5, 1) << left_k1, left_k2, left_p1, left_p2, left_k3);

	//右边相机的内参矩阵
	Mat cameraMatrixR = (Mat_<double>(3, 3) <<
		right_fx, 0, right_cx,
		0, right_fy, right_cy,
		0, 0, 1
		);
	//右边相机的畸变矩阵
	Mat distCoeffR = (Mat_<double>(5, 1) << right_k1, right_k2, right_p1, right_p2, right_k3);

	//两个相机标定后的平移矩阵
	Mat T = (Mat_<double>(3, 1) << double_py_x1, double_py_x2, double_py_x3);//T平移向量

	//两个相机标定后的旋转矩阵
	Mat rec = (Mat_<double>(3, 3) <<
		double_xz_x1, double_xz_x2, double_xz_x3,
		double_xz_x4, double_xz_x5, double_xz_x6,
		double_xz_x7, double_xz_x8, double_xz_x9);



	Mat R;//R 旋转矩阵
	//R = rec.clone();
	cv::Rodrigues(rec, R); //Rodrigues变换，获得3*3的旋转矩阵转换为旋转向量

	//立体校正
	cv::stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
		0, imageSize, &validROIL, &validROIR);



	//计算无畸变和修正转换关系
	cv::initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);//mapL——输出的X坐标重映射参数
	cv::initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);//mapR——输出的Y坐标重映射参数
	if (save_flag) {
		LOG(INFO) << "\n" << "cameraMatrixL:" << "\n" << cameraMatrixL;
		LOG(INFO) << "\n" << "distCoeffL:" << "\n" << distCoeffL;
		LOG(INFO) << "\n" << "cameraMatrixR:" << "\n" << cameraMatrixR;
		LOG(INFO) << "\n" << "distCoeffR:" << "\n" << distCoeffR;

		LOG(INFO) << "\n" << "TranslationVector:" << "\n" << T;
		LOG(INFO) << "\n" << "RotateMartix:" << "\n" << rec;
	}
	//LOG(INFO) << "mapLx：" << mapLx.cols;

	return 0;
}

//1.初始化接口
int BinocularVisionDetector::InitAlg(const char* config_path, CameraInit cameraconfig) {
	try {
        if (0) {
            cout<<"初始化，不执行"<<endl;
            char *buffer;
            buffer = getcwd(NULL, 0);

            std::string strPath(buffer);

            loginitflag = false;
            std::string log_folder = "/alg_log/";
            if (0 != access((strPath + log_folder).c_str(), 0)) {
                mkdir((strPath + log_folder).c_str(), 0771);
            }

            // 日志名称
            google::InitGoogleLogging("双目视觉算法 / BinocularVisionDetector (handler) SDK 初始化中...");
            // 设置级别高于 google::INFO 的日志同时输出到屏幕
            google::SetStderrLogging(google::GLOG_INFO);

            //设置 google::INFO 级别的日志存储路径和文件名称前缀
            string info_name = (string) log_folder.c_str() + "//INFO_";
            google::SetLogDestination(google::GLOG_INFO, info_name.c_str());

            //设置 google::WARNING 级别的日志存储路径和文件名称前缀
            string warning_name = (string) log_folder.c_str() + "//WARNING_";
            google::SetLogDestination(google::GLOG_WARNING, warning_name.c_str());

            //设置 google::ERROR 级别的日志存储路径和文件名称前缀
            string error_name = (string) log_folder.c_str() + "//ERROR_";
            google::SetLogDestination(google::GLOG_ERROR, error_name.c_str());

            // 最大日志大小为 1000MB
            FLAGS_max_log_size = 30;
            FLAGS_colorlogtostderr = true;    //设置输出到屏幕的日志显示对应颜色

            // 当磁盘被写满时，停止日志输出
            FLAGS_stop_logging_if_full_disk = true;
            // 即时写入
            FLAGS_logbufsecs = 0;
        }
    }
	catch(...){
        LOG(INFO) <<"Glog init twice ,pass..."<<endl;
	}

	string config_file = string(config_path) + "//config.xml";

	ReadXmlDocument(config_file);
	left_camera_ip = (cameraconfig.CameraIP_L);
	expore_time_l = cameraconfig.ExporeTime_L;
	gain_l = cameraconfig.Gain_L;

	right_camera_ip = (cameraconfig.CameraIP_R);
	expore_time_r = cameraconfig.ExporeTime_R;
	gain_r = cameraconfig.Gain_R;

	LOG(INFO)<<left_camera_ip<<endl;
	LOG(INFO)<<right_camera_ip<<endl;

	int stasus = -1;
	try {
		//初始化双目视觉算法的参数
		stasus = BinocularVisionInit();
	}
	catch (...) {
		LOG(ERROR) << "BinocularVisionInit init failed!";
		return -1;
	}

	if (stasus == 0) {
		LOG(INFO) << " config.xml init success,next link cameras...";
	}
	else {
		LOG(ERROR) << "handle init failed!";
		return -1;
	}

	try {
	    LOG(INFO)<<"online flag："<<camera_online_flag<<endl;
		if (camera_online_flag) {
            initCamera();
		}
	}
	catch (...) {
        CloseCamera();
		LOG(INFO) << "Online Camera Init Failed... ";
		return -1;
	}


	return 0;

}


//获取在线图像数据
int BinocularVisionDetector::OnlineCameraGet(INPUT_DATA* onlineimage,int skipframe) {
	try {
        GetCameraImage(skipframe);
        //LOG(INFO)<<"Balser grab image success" <<endl;

        //memcpy(onlineimage->left_img_data, images[0].data, images[0].cols * images[0].rows * images[0].channels());
        onlineimage->left_img_data = images[0].data;
        onlineimage->left_img_height = images[0].rows;
        onlineimage->left_img_width = images[0].cols;
        onlineimage->left_img_channels = images[0].channels();

        //memcpy(onlineimage->right_img_data, images[1].data, images[1].cols * images[1].rows * images[1].channels());
        onlineimage->right_img_data = images[1].data;
        onlineimage->right_img_height = images[1].rows;
        onlineimage->right_img_width = images[1].cols;
        onlineimage->right_img_channels = images[1].channels();
//        if(onlineimage->right_img_channels==3) {
//            Mat ll = Mat(onlineimage->right_img_height, onlineimage->right_img_width, CV_8UC3,
//                         onlineimage->left_img_data, 0);
//            Mat rr = Mat(onlineimage->right_img_height, onlineimage->right_img_width, CV_8UC3,
//                         onlineimage->right_img_data, 0);
//            imwrite("/cytech_ai/env/BinocularVision/lib/alg_log/savel.jpg",ll);
//            imwrite("/cytech_ai/env/BinocularVision/lib/alg_log/saver.jpg",rr);
//        }
//        else{
//            Mat ll = Mat(onlineimage->right_img_height, onlineimage->right_img_width, CV_8UC1,
//                         onlineimage->left_img_data, 0);
//            Mat rr = Mat(onlineimage->right_img_height, onlineimage->right_img_width, CV_8UC1,
//                         onlineimage->right_img_data, 0);
//            imwrite("/cytech_ai/env/BinocularVision/lib/alg_log/savel.jpg",ll);
//            imwrite("/cytech_ai/env/BinocularVision/lib/alg_log/saver.jpg",rr);
//        }

    }
    catch (...)
    {
        LOG(INFO)<<"ERROR Grab image..."<<endl;
        return 0;
    }
	return 1;
}


//初始化相机 只执行一次
void BinocularVisionDetector::initCamera()
{
    try
    {
        PylonInitialize();
        CTlFactory& tlFactory = CTlFactory::GetInstance();
        pTL = dynamic_cast<IGigETransportLayer*>(tlFactory.CreateTl( BaslerGigEDeviceClass ));
        if (pTL == NULL)
        {
            throw RUNTIME_EXCEPTION( "No GigE cameras available." );
        }

        DeviceInfoList_t allDeviceInfos;
        if (pTL->EnumerateDevices( allDeviceInfos ) == 0)
        {
            throw RUNTIME_EXCEPTION( "No GigE cameras available." );
        }

        DeviceInfoList_t usableDeviceInfos;

        if (string(allDeviceInfos[0].GetIpAddress()) == left_camera_ip) {
            usableDeviceInfos.push_back(allDeviceInfos[0]);
            subnet = allDeviceInfos[0].GetSubnetAddress();//主相机
            usableDeviceInfos.push_back(allDeviceInfos[1]);
            LOG(INFO)<<"主相机："<<allDeviceInfos[0].GetIpAddress()<<endl;
            LOG(INFO)<<"副相机："<<allDeviceInfos[1].GetIpAddress()<<endl;
        }
        else if(string(allDeviceInfos[1].GetIpAddress()) == left_camera_ip) {
            usableDeviceInfos.push_back(allDeviceInfos[1]);
            subnet = allDeviceInfos[1].GetSubnetAddress();//主相机
            usableDeviceInfos.push_back(allDeviceInfos[0]);
            LOG(INFO)<<"主相机IP："<<allDeviceInfos[1].GetIpAddress()<<endl;
            LOG(INFO)<<"SubnetAddress："<<allDeviceInfos[1].GetSubnetAddress()<<endl;
            LOG(INFO)<<"DefaultGateway："<<allDeviceInfos[1].GetDefaultGateway()<<endl;
            LOG(INFO)<<"SubnetMask："<<allDeviceInfos[1].GetSubnetMask()<<endl;

            LOG(INFO)<<"副相机IP："<<allDeviceInfos[0].GetIpAddress()<<endl;
            LOG(INFO)<<"SubnetAddress："<<allDeviceInfos[0].GetSubnetAddress()<<endl;
            LOG(INFO)<<"DefaultGateway:" <<allDeviceInfos[0].GetDefaultGateway()<<endl;
            LOG(INFO)<<"SubnetMask："<<allDeviceInfos[0].GetSubnetMask()<<endl;


        }
        else{
            LOG(INFO) << "Camera IP is error ,please set IP" << endl;
        }
        for (size_t i = 0; i < 2; ++i)
        {
            cameras[i].Attach(tlFactory.CreateDevice(usableDeviceInfos[i]));
            const CBaslerGigEDeviceInfo& di = cameras[i].GetDeviceInfo();
            LOG(INFO) << "Camera serial: " << di.GetSerialNumber() << endl;
        }

//        srand( (unsigned) time( NULL ) );
//        DeviceKey = rand();
//        GroupKey = 0x112233;

        for (size_t i = 0; i < cameras.GetSize(); ++i)
        {
            cameras[i].Attach( tlFactory.CreateDevice( usableDeviceInfos[i] ) );
            //cameras[i].RegisterConfiguration( new CActionTriggerConfiguration( DeviceKey, GroupKey, AllGroupMask ), RegistrationMode_Append, Cleanup_Delete );
            //cameras[i].SetCameraContext( i );
            const CBaslerGigEDeviceInfo& di = cameras[i].GetDeviceInfo();
            cout << "Using camera " << i << ": " << di.GetSerialNumber() << " (" << di.GetIpAddress() << ")" << endl;
        }

        cameras.Open();

        //相机基本设置
        SetCamera(cameras[0], Type_Basler_ExposureTimeAbs, expore_time_l);			//曝光时间
        SetCamera(cameras[0], Type_Basler_GainRaw, gain_l);						//增益
        SetCamera(cameras[0], Type_Basler_AcquisitionFrameRateAbs, fps_l);			//频率
        SetCamera(cameras[0], Type_Basler_Width, 2448);
        SetCamera(cameras[0], Type_Basler_Height, 2048);

        SetCamera(cameras[1], Type_Basler_ExposureTimeAbs, expore_time_r);			//曝光时间
        SetCamera(cameras[1], Type_Basler_GainRaw, gain_r);						//增益
        SetCamera(cameras[1], Type_Basler_AcquisitionFrameRateAbs, fps_r);			//频率
        SetCamera(cameras[1], Type_Basler_Width, 2448);
        SetCamera(cameras[1], Type_Basler_Height, 2048);

        //设置相机触发模式 	TriggerSelector
        //TriggerSoftware
        //主相机设置为软件触发，输出设置为exposure active
        //SetCamera(cameras[0], Type_Basler_Freerun, 0);

        //从相机设置：触发模式为外触发，IO设置为1
        //SetCamera(cameras[1], Type_Basler_Line1, 0);

    }
    catch (const GenericException& e)
    {
        if(cameras.IsGrabbing())
            cameras.StopGrabbing();
        // Error handling
        LOG(INFO) << "init,An exception occurred." << endl
                  << e.GetDescription() << endl;
    }


}


//抓取在线数据，通过设置不同帧率选择不同的数据图像
/*
 * skipframe=1,间隔1帧，50ms取一次
 * skipframe=2，间隔2帧，100ms取一次
 * skipframe=3，间隔3帧，150ms取一次
 * skipframe=4，间隔4帧，200ms取一次
 * skipframe=5，间隔5帧，250ms取一次
 * skipframe=6，间隔6帧，300ms取一次
 * skipframe=7，间隔7帧，350ms取一次
 *
 *
 * */
void BinocularVisionDetector::GetCameraImage(int skipframe) {
	try {
	    if(skipframe>20 || skipframe<0){
            LOG(INFO)<<"warning：skipframe is error,0-20: "<<skipframe<<endl;
            skipframe = 1;
	    }

		static int ccc = 0;
		int count_grab_once = 0;
        //pTL->IssueActionCommand(DeviceKey, GroupKey, AllGroupMask, subnet );
		//1秒内抓取了多少张图，全部存下来
        int skiptime = 1000;
        //LOG(INFO)<<"采集图像的最长时间："<<skiptime<<" ms"<<endl;
        cameras.StartGrabbing(GrabStrategy_OneByOne,GrabLoop_ProvidedByUser);

        if(cameras.IsGrabbing() && skipframe>0 && skipframe<=fps_l) {
            std::chrono::high_resolution_clock::time_point tStartTime(std::chrono::high_resolution_clock::now());
            int lTimeAloInterval = 0;
            count_grab_once++;
            cameras[0].RetrieveResult(skiptime, ptrGrabResultl, TimeoutHandling_ThrowException);
            cameras[1].RetrieveResult(skiptime, ptrGrabResultr, TimeoutHandling_ThrowException);

            if (ptrGrabResultl->GrabSucceeded() && ptrGrabResultr->GrabSucceeded() ) {
                intptr_t cameraContextValuel = ptrGrabResultl->GetCameraContext();
                intptr_t cameraContextValuer = ptrGrabResultr->GetCameraContext();
//                LOG(INFO) << "Camera " << cameraContextValuel << ": " << cameras[cameraContextValuel].GetDeviceInfo().GetAddress() << endl;
//                LOG(INFO) << "Camera " << cameraContextValuer << ": " << cameras[cameraContextValuer].GetDeviceInfo().GetAddress() << endl;

                const uint8_t *pImageBufferl = (uint8_t *) ptrGrabResultl->GetBuffer();
                const uint8_t *pImageBufferr = (uint8_t *) ptrGrabResultr->GetBuffer();
                // 将 pylon image转成OpenCV image.
                Mat SaveImagel = cv::Mat(ptrGrabResultl->GetHeight(), ptrGrabResultl->GetWidth(), CV_8UC1,
                                        (uint8_t *) pImageBufferl);
                Mat SaveImager = cv::Mat(ptrGrabResultr->GetHeight(), ptrGrabResultr->GetWidth(), CV_8UC1,
                                         (uint8_t *) pImageBufferr);

                if(!SaveImagel.empty() && !SaveImager.empty()) {
                    if (cameraContextValuel == 0) {
                        images[0] = SaveImagel.clone();
                        //imwrite(R"(/cytech_ai/env/BinocularVision/lib/alg_log/l/)" + to_string(ccc) + "_l.jpg",images[0]);
                    }
                    if (cameraContextValuer == 1) {
                        images[1] = SaveImager.clone();
                        //imwrite(R"(/cytech_ai/env/BinocularVision/lib/alg_log/r/)" + to_string(ccc) + "_r.jpg",images[1]);
                    }
                }
            }

            else {
                LOG(INFO) << "Error l: " << std::hex << ptrGrabResultl->GetErrorCode() << std::dec << " "
                          << ptrGrabResultl->GetErrorDescription() << endl;
                LOG(INFO) << "Error r: " << std::hex << ptrGrabResultr->GetErrorCode() << std::dec << " "
                          << ptrGrabResultr->GetErrorDescription() << endl;
            }
            ccc++;

            lTimeAloInterval =std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000> >>(std::chrono::high_resolution_clock::now() - tStartTime).count();
            LOG(INFO) << "------------ single Grab image cost time:----------------" << lTimeAloInterval << " ms" << endl;
        }

        if(images[0].empty()){
		    LOG(INFO)<<"左相机丢帧，原因：图像帧率设置过小，抓取超时，跳帧次数不在0-20："<<images[0].empty()<<endl;
            LOG(INFO)<<"当前帧率设置："<<fps_l<<endl;
            LOG(INFO)<<"skipframe: "<<skipframe<<endl;
		}
        if(images[1].empty()){
            LOG(INFO)<<"右相机丢帧，原因：图像帧率设置过小，抓取超时，跳帧次数不在0-20："<<images[1].empty()<<endl;
            LOG(INFO)<<"当前帧率设置："<<fps_r<<endl;
            LOG(INFO)<<"skipframe: "<<skipframe<<endl;
        }
        cameras.StopGrabbing();

	}
	catch (const GenericException& e)
	{
        if(cameras.IsGrabbing())
            cameras.StopGrabbing();
		// Error handling
		LOG(INFO) << "An exception occurred." << endl
			<< e.GetDescription() << endl;
	}
	
}

void BinocularVisionDetector::CloseCamera()
{
	//最后终止Pylon相机，即调用PylonTerminate。//关闭摄像头
	try
	{
		if (cameras.IsOpen()) {
            cameras.StopGrabbing();
			cameras.DetachDevice();
			cameras.Close();
			cameras.DestroyDevice();
			//关闭库
			LOG(INFO) << "SBaslerCameraControl deleteAll: PylonTerminate";
			PylonTerminate();
		}
	}
	catch (const Pylon::GenericException& e)
	{
		LOG(INFO) << "close camera failed..." << e.what();
	}
}


//手动设置各种参数,单独调用，switch只有一个
void BinocularVisionDetector::SetCamera(CInstantCamera& m_basler, BaslerCamera_Type balserparams, int tmpValue)
{
	INodeMap& cameraNodeMap = m_basler.GetNodeMap();
	switch (balserparams) {
	case Type_Basler_Freerun: { //内触发
		CEnumerationPtr  ptrTriggerSel = cameraNodeMap.GetNode("TriggerSelector");
		ptrTriggerSel->FromString("FrameStart");
		CEnumerationPtr  ptrTrigger = cameraNodeMap.GetNode("TriggerMode");
		ptrTrigger->SetIntValue(1);
		CEnumerationPtr  ptrTriggerSource = cameraNodeMap.GetNode("TriggerSource");
		ptrTriggerSource->FromString("Software");


	} break;
	case Type_Basler_Line1: {   //外触发
		CEnumerationPtr  ptrTriggerSel = cameraNodeMap.GetNode("TriggerSelector");
		ptrTriggerSel->FromString("FrameStart");
		CEnumerationPtr  ptrTrigger = cameraNodeMap.GetNode("TriggerMode");
		ptrTrigger->SetIntValue(1);

		/*CEnumerationPtr  ptrTriggerActivation = cameraNodeMap.GetNode("TriggerActivation");
		ptrTriggerActivation->FromString("RisingEdge");*/

		CEnumerationPtr  ptrTriggerSource = cameraNodeMap.GetNode("TriggerSource");
		ptrTriggerSource->FromString("Line1");

		//TriggerActivation = RisingEdge
	} break;
	case Type_Basler_ExposureTimeAbs: { //曝光时间
		const CFloatPtr exposureTime = cameraNodeMap.GetNode("ExposureTimeAbs");
		exposureTime->SetValue(tmpValue);
	} break;
	case Type_Basler_GainRaw: { //增益
		const CIntegerPtr cameraGen = cameraNodeMap.GetNode("GainRaw");
		cameraGen->SetValue(tmpValue);
	} break;
	case Type_Basler_AcquisitionFrameRateAbs: { //设置相机的频率
		const CBooleanPtr frameRate = cameraNodeMap.GetNode("AcquisitionFrameRateEnable");
		//frameRate->SetValue(True);
		const CFloatPtr frameRateABS = cameraNodeMap.GetNode("AcquisitionFrameRateAbs");
		frameRateABS->SetValue(tmpValue);
	} break;
	case Type_Basler_Width: {
		const CIntegerPtr widthPic = cameraNodeMap.GetNode("Width");
		widthPic->SetValue(tmpValue);
	} break;
	case Type_Basler_Height: {
		const CIntegerPtr heightPic = cameraNodeMap.GetNode("Height");
		heightPic->SetValue(tmpValue);
	} break;
	case Type_Basler_LineSource: {
		CEnumerationPtr  ptrLineSource = cameraNodeMap.GetNode("LineSource");
		ptrLineSource->SetIntValue(2);
	} break;
	default:
		break;
	}
}

//自动获取各种参数，直接传所有类型switch
int BinocularVisionDetector::GetCamera(CInstantCamera& m_basler, BaslerCamera_Type balserparams)
{
	INodeMap& cameraNodeMap = m_basler.GetNodeMap();
	switch (balserparams) {
	case Type_Basler_ExposureTimeAbs: {//曝光时间
		const CFloatPtr exposureTime = cameraNodeMap.GetNode("ExposureTimeAbs");
		return exposureTime->GetValue();
	} break;
	case Type_Basler_GainRaw: {//增益
		const CIntegerPtr cameraGen = cameraNodeMap.GetNode("GainRaw");
		return cameraGen->GetValue();
	} break;
	case Type_Basler_AcquisitionFrameRateAbs: {//相机频率
		const CBooleanPtr frameRate = cameraNodeMap.GetNode("AcquisitionFrameRateEnable");
		//frameRate->SetValue(True);
		const CFloatPtr frameRateABS = cameraNodeMap.GetNode("AcquisitionFrameRateAbs");
		return frameRateABS->GetValue();
	} break;
	case Type_Basler_Width: {	//图像宽度
		const CIntegerPtr widthPic = cameraNodeMap.GetNode("Width");
		return widthPic->GetValue();
	} break;
	case Type_Basler_Height: { //图像高度
		const CIntegerPtr heightPic = cameraNodeMap.GetNode("Height");
		return heightPic->GetValue();
	} break;
	default:
		return -1;
		break;
	}
}


//自动获取相机曝光时间switch只有一个
int BinocularVisionDetector::getExposureTime(CInstantCamera& m_basler, BaslerCamera_Type balserparams)
{
	return GetCamera(m_basler, balserparams);

}

//手动设置曝光时间 switch只有一个
void BinocularVisionDetector::setExposureTime(CInstantCamera& m_basler, BaslerCamera_Type balserparams, int time)
{
	SetCamera(m_basler, balserparams, time);
}

//设置相机增益switch只有一个
void BinocularVisionDetector::setGain(CInstantCamera& m_basler, BaslerCamera_Type balserparams, int value)
{
	SetCamera(m_basler, balserparams, value);
}

//获得相机增益switch只有一个
int BinocularVisionDetector::getGain(CInstantCamera& m_basler, BaslerCamera_Type balserparams)
{
	return GetCamera(m_basler, balserparams);

}

//获得相机频率
int BinocularVisionDetector::getFrameRate(CInstantCamera& m_basler, BaslerCamera_Type Type_Basler_AcquisitionFrameRateAbs)
{
	return GetCamera(m_basler, Type_Basler_AcquisitionFrameRateAbs);

}

//设置相机频率
void BinocularVisionDetector::setFrameRate(CInstantCamera& m_basler, BaslerCamera_Type balserparams, int value)
{
	SetCamera(m_basler, balserparams, value);
}

//双目的立体标定和立体校正
void BinocularVisionDetector::stereo_check(Mat grayImageL, Mat grayImageR, Mat& rectifyImageL, Mat& rectifyImageR) {
	//一幅图像中某位置的像素放置到另一个图片指定位置，经过remap之后，左右相机的图像已经共面并且行对准了

	remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
	remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

	//LOG(INFO) << "image recitify finished";

	if (save_flag) {
		//把校正结果显示出来
		Mat rgbRectifyImageL;
		Mat rgbRectifyImageR;
		cv::cvtColor(rectifyImageL, rgbRectifyImageL, cv::COLOR_GRAY2BGR);  //伪彩色图
		cv::cvtColor(rectifyImageR, rgbRectifyImageR, cv::COLOR_GRAY2BGR);
		imwrite(save_path + "Rectifyleft.jpg", rgbRectifyImageL);
		imwrite(save_path + "Rectifyright.jpg", rgbRectifyImageR);
		//LOG(INFO) << "single image saved success";
		Mat canvas;
		double sf;
		int w, h;
		sf = 2448. / MAX(rgbRectifyImageL.cols, rgbRectifyImageL.rows);
		w = cvRound(rgbRectifyImageL.cols * sf);
		h = cvRound(rgbRectifyImageL.rows * sf);
		canvas.create(h, w * 2, CV_8UC3);   //注意通道

		//左图像画到画布上
		Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //得到画布的一部分  
		resize(rgbRectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);     //把图像缩放到跟canvasPart一样大小 

		//右图像画到画布上
		canvasPart = canvas(Rect(w, 0, w, h));                                      //获得画布的另一部分  
		resize(rgbRectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);

		//画上对应的线条
		for (int i = 0; i < canvas.rows; i += 48)
			line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 0, 255), 2, 8);

		imwrite(save_path + "RectifyimagesUnion.jpg", canvas);
		LOG(INFO) << "image union saved success"<<save_path + "RectifyimagesUnion.jpg";
	}

}

//2.双目视觉算法-输出两张校正后的图像
void BinocularVisionDetector::binocular_vision(Mat rgbImageL, Mat rgbImageR, Mat& rectifyImageL1, Mat& rectifyImageR1) {

    Mat grayImageL;
    Mat grayImageR;
    const int imageWidth = rgbImageL.cols;
    const int imageHeight = rgbImageL.rows;

    //1.图像获取
    cvtColor(rgbImageL, grayImageL, COLOR_RGB2GRAY);
    cvtColor(rgbImageR, grayImageR, COLOR_RGB2GRAY);

    Mat rectifyImageL = grayImageL.clone();
    Mat rectifyImageR = grayImageR.clone();

    //2.立体校正--图像对齐了-在同一极线上检测左右摄像机图像的相同特征
    stereo_check(grayImageL, grayImageR, rectifyImageL, rectifyImageR);
//    rectifyImageL1 = rectifyImageL.clone();
//    rectifyImageR1 = rectifyImageR.clone();
    cv::cvtColor(rectifyImageL, rectifyImageL1, cv::COLOR_GRAY2BGR);
    cv::cvtColor(rectifyImageR, rectifyImageR1, cv::COLOR_GRAY2BGR);

}


void BinocularVisionDetector::BinocularVisionComputeDis(INPUT_BOX* input_box, float* dis) {
	//双目视觉测距--根据输入的目标
	//根据person对的个数，计算距离

	int left_box_center_x = input_box->left_box.left_x + (input_box->left_box.width / 2);
	int left_box_center_y = input_box->left_box.left_y + (input_box->left_box.height / 2);
	int right_box_center_x = input_box->right_box.left_x + (input_box->right_box.width / 2);
	int right_box_center_y = input_box->right_box.left_y + (input_box->right_box.height / 2);
	int shichavalue = left_box_center_x - right_box_center_x;//视差
	if (shichavalue + shicha_offset == 0)
		*dis = 50;

	float now_dis = abs((cameradis * left_fx) / ((shichavalue + shicha_offset) * 1000.0));
	*dis = now_dis;
	if (save_flag) {

//		LOG(INFO) << "left_box_center_x:" << cameradis;
//        LOG(INFO) << "left_box_center_x:" << cameradis;
//		LOG(INFO) << "相机基线距离:" << cameradis;
//		LOG(INFO) << "相机焦距:" << left_fx;
		LOG(INFO) << "左右视差值:" << shichavalue;
//		LOG(INFO) << "视差偏移量:" << shicha_offset;
//		LOG(INFO) << "now dis:" << now_dis <<" 米" << endl;
	}
	if (now_dis > 100) {
	    LOG(INFO)<<"异常的视差值，无效值..."<<endl;
        *dis = 50;
    }
}

void BinocularVisionDetector::Release() {
	CloseCamera();

}