#include "../include/BinocularVisionInterface.h"
#include "../include/interface.h"
#include "../include/BinocularVision.h"

void  GetAlgVersion(const char version_code[2048], const char update_version_info[2048]) {
	string version_code1 = "1.0.0.2";
	string update_version_info1 = "1.20230821 更新算法，双目视觉测距，第一次实验标定，基线距离24/n 20230824更新，外部调用./n 20230828 接balser相机/n";

	version_code = version_code1.c_str();
	update_version_info = update_version_info1.c_str();
}

int  CameraOnline(void* handle, INPUT_DATA* onlineimage,int skipframe) {
	if (handle == nullptr) {
		LOG(INFO) << "init failed or handle is nullptr";
		return 0;
	}
	auto camera_handler = (BinocularVisionDetector*)handle;

    int statuson = camera_handler->OnlineCameraGet(onlineimage,skipframe);
    if(statuson==0){
        LOG(INFO) << "grab failed...";
        return 0;
    }
    return 1;
}


void* InitMethod(const char* config_path, CameraInit cameraconfig) {
	BinocularVisionDetector* handler = new BinocularVisionDetector();
	int status = handler->InitAlg(config_path,cameraconfig);
	if (status == -1) {
		LOG(INFO) << "Init Failed ...";
		return nullptr;
	}
	auto handle = (void*)handler;
	return handle;

}



//双目校正
void BinocularRecitify(void* handle, INPUT_DATA *input,OUTPUT_DATA *out)
{
	if (handle == nullptr) {
		LOG(INFO) << "init failed or handle is nullptr";
	}
	auto alg_handler = (BinocularVisionDetector*)handle ;

	Mat rgbImageL;
	Mat rgbImageR;
	Mat rectifyImageL1;
	Mat rectifyImageR1;

    if(1) {

        try {
            if (input->left_img_data != NULL && input->right_img_data != NULL) {
                if (input->left_img_channels == 3 && input->right_img_channels == 3) {
                    rgbImageL = Mat(input->left_img_height, input->left_img_width, CV_8UC3, input->left_img_data,
                                    0);
                    rgbImageR = Mat(input->right_img_height, input->right_img_width, CV_8UC3, input->right_img_data,
                                    0);
                    //LOG(INFO) << "当前传入的图像通道数： 3";
                }
                else if (input->left_img_channels == 1 && input->right_img_channels == 1) {
                    rgbImageL = Mat(input->left_img_height, input->left_img_width, CV_8UC1, input->left_img_data,
                                    0);
                    cv::cvtColor(rgbImageL, rgbImageL, cv::COLOR_GRAY2BGR);
                    rgbImageR = Mat(input->right_img_height, input->right_img_width, CV_8UC1, input->right_img_data,
                                    0);
                    cv::cvtColor(rgbImageR, rgbImageR, cv::COLOR_GRAY2BGR);
                    //LOG(INFO) << "当前传入的图像通道数： 1";
                }
                else {
                    LOG(INFO) << "左右图像通道数不一致,请检查...";
                    LOG(INFO) << "left channel: " << input->left_img_channels << endl;
                    LOG(INFO) << "right channel: " << input->right_img_channels << endl;
                }
            }
            else {
                LOG(INFO) << "input->data is null ,check input data";
                return;
            }
        }
        catch (...) {
            LOG(INFO) << "input image is error,check image channels..." << endl;
        }
    }
    else{
        string aa = "/cytech_ai/env/BinocularVision/testimage/l/30_26_l.jpg";
        string bb = "/cytech_ai/env/BinocularVision/testimage/r/30_26_r.jpg";
        rgbImageL = imread(aa);
        rgbImageR = imread(bb);
    }

	if(rgbImageL.empty() || rgbImageR.empty()){
	    LOG(INFO)<<"传入图像数据存在问题，请对应检查宽度，高度，通道数是否一一对应..."<<endl;
	    LOG(INFO)<<"left is empty : "<< rgbImageL.empty()<<endl;
        LOG(INFO)<<"right is empty : "<<rgbImageR.empty()<<endl;
        return;
	}

	alg_handler->binocular_vision(rgbImageL, rgbImageR, rectifyImageL1, rectifyImageR1);

	//out->output.left_img_data = new unsigned char[rectifyImageL1.cols * rectifyImageL1.rows * rectifyImageL1.channels()];//new的单位为字节

//    memcpy(out->output.left_img_data, rectifyImageL1.data,
//           rectifyImageL1.cols * rectifyImageL1.rows * rectifyImageL1.channels());
    out->output.left_img_data = rectifyImageL1.data;
    out->output.left_img_width = rectifyImageL1.cols;
    out->output.left_img_height = rectifyImageL1.rows;
    out->output.left_img_channels = rectifyImageL1.channels();

    //out->output.right_img_data = new unsigned char[rectifyImageL1.cols * rectifyImageL1.rows * rectifyImageL1.channels()];//new的单位为字节

//    memcpy(out->output.right_img_data, rectifyImageR1.data,
//           rectifyImageR1.cols * rectifyImageR1.rows * rectifyImageR1.channels());

    out->output.right_img_data = rectifyImageR1.data;
    out->output.right_img_width = rectifyImageR1.cols;
    out->output.right_img_height = rectifyImageR1.rows;
    out->output.right_img_channels = rectifyImageR1.channels();

    LOG(INFO)<<"Recitify finished"<<endl;

}
	
//双目测距
void BinocularComputeDis(void* handle, INPUT_BOX *input_box,float *dis)
{
	auto compute_handler = (BinocularVisionDetector*)handle;
	
	compute_handler->BinocularVisionComputeDis(input_box,dis);
	if (compute_handler->save_flag) {
		//LOG(INFO) << "value,now obj dis: "<< dis<<" m";
		LOG(INFO) << "BinocularComputeDis Finished,now obj dis: " << *dis << " m";
	}

}


void  Release(void* handle) {
	auto delete_handler = (BinocularVisionDetector*)handle;
	delete_handler->Release();
	if (handle == nullptr) {
		return;
	}
	auto mrr_core = (BinocularVisionDetector*)handle;
	delete mrr_core;
	handle = nullptr;

}

