#include "include/interface.h"
#include "include/BinocularVisionInterface.h"

int main() {
    int c = 21;
    string config_file = R"(../../config)";
    void* handler = nullptr;
    int online_flag = 1;		//是否在线调用相机
    INPUT_DATA *input = new INPUT_DATA();
    //分配空间
    input->left_img_data = new unsigned char[2048 * 2448 * 3];
    input->right_img_data = new unsigned char[2048 * 2448 * 3];

    string left_img_path  = R"(/cytech_ai/env/BinocularVision/testimage/l/)";
    string right_img_path = R"(/cytech_ai/env/BinocularVision/testimage/r/)";
    vector<string> left_imgs, right_imgs;
    CameraInit cameraconfig;
    cameraconfig.CameraIP_L = "172.16.102.21";
    cameraconfig.CameraIP_R = "172.16.102.22";
    cameraconfig.ExporeTime_L = 7000;
    cameraconfig.ExporeTime_R = 7000;
    cameraconfig.Gain_L = 100;
    cameraconfig.Gain_R = 100;
    while (1) {
        //初始化
        handler = InitMethod(config_file.c_str(), cameraconfig);
        if (handler == nullptr) {
            LOG(INFO) << "InitMethod failed..." << endl;
        }
        OUTPUT_DATA *out = new OUTPUT_DATA();
        //分配空间
        out->output.left_img_data = new unsigned char[2048 * 2448 * 3];
        out->output.right_img_data = new unsigned char[2048 * 2448 * 3];

        INPUT_BOX *input_box = new INPUT_BOX[10];
        float dis = 0;
        int forloop = 1000;

        if (online_flag == 0) {
            forloop = 1000;
        }

        for (int i = 0; i < forloop; i++) {
            std::chrono::high_resolution_clock::time_point tStartTime(std::chrono::high_resolution_clock::now());

            if (i == 999) {
                i = 0;
                c++;
            }
            if (online_flag == 0) {
                cv::Mat left_img;
                cv::Mat right_img;
                std::cout << "id:" << i << ":" << left_img_path+"/30_26_l.jpg" << endl;
                std::cout << "id:" << i << ":" << right_img_path+"/30_26_r.jpg" << endl;
                string aa = "/cytech_ai/env/BinocularVision/testimage/l/30_26_l.jpg";
                string bb = "/cytech_ai/env/BinocularVision/testimage/r/30_26_r.jpg";
                left_img = imread(aa,0);
                right_img = imread(bb,0);
                memcpy(input->left_img_data, left_img.data,left_img.cols * left_img.rows * left_img.channels());

                input->left_img_width = left_img.cols;
                input->left_img_height = left_img.rows;
                input->left_img_channels = left_img.channels();

                memcpy(input->right_img_data, right_img.data,right_img.cols * right_img.rows * right_img.channels());
                input->right_img_width = right_img.cols;
                input->right_img_height = right_img.rows;
                input->right_img_channels = right_img.channels();
            }
            else {
                LOG(INFO) << "i:" << i;
                int status = CameraOnline(handler, input);
                if(status==0){
                    LOG(INFO)<<"camera is offline"<<endl;
                    continue;
                }
                if (input->left_img_width == 0 || input->right_img_width == 0) {
                    continue;
                }
            }



            int lTimeAloInterval =std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000> >>(std::chrono::high_resolution_clock::now() - tStartTime).count();

            LOG(INFO) << "------------Cmaera get image cost time:----------------" << lTimeAloInterval << " ms" << endl;

            //双目校正后的图像
            BinocularRecitify(handler, input, out);


            //LOG(INFO)<<"recity success"<<endl;
            input_box->left_box.left_x = 1970;
            input_box->left_box.left_y = 200;
            input_box->left_box.width = 435;
            input_box->left_box.height = 1429;

            input_box->right_box.left_x = 1760;
            input_box->right_box.left_y = 200;
            input_box->right_box.width = 444;
            input_box->right_box.height = 1436;

            Mat LeftRecitifyImage, RightRecitifyImage;
            //还原图像
            if (out->output.left_img_data != NULL && out->output.right_img_data != NULL) {
                if (out->output.left_img_channels == 3) {
                    LeftRecitifyImage = Mat(out->output.left_img_height, out->output.left_img_width, CV_8UC3,
                                            out->output.left_img_data, 0);    // unsigned char*  => Mat
                    RightRecitifyImage = Mat(out->output.right_img_height, out->output.right_img_width, CV_8UC3,
                                             out->output.right_img_data, 0);
                } else {
                    //LOG(INFO) << "当前传入的图像通道数： 1";
                    LeftRecitifyImage = Mat(out->output.left_img_height, out->output.left_img_width, CV_8UC1,
                                            out->output.left_img_data, 0);
                    RightRecitifyImage = Mat(out->output.right_img_height, out->output.right_img_width, CV_8UC1,
                                             out->output.right_img_data, 0);
                }
                /*imwrite(R"(E:\project\双目视觉技术\BinocularVisionAlg\x64\Release\alg_log\left\)" + to_string(i) +"_"+ to_string(c) + "_l.jpg", LeftRecitifyImage);
                imwrite(R"(E:\project\双目视觉技术\BinocularVisionAlg\x64\Release\alg_log\right\)" + to_string(i) +"_"+to_string(c) + "_r.jpg", RightRecitifyImage);
                waitKey(0);*/

            }

            BinocularComputeDis(handler, input_box, &dis);
            LOG(INFO) << "当前目标距离：" << dis << " 米";
            cvtColor(LeftRecitifyImage, LeftRecitifyImage, COLOR_GRAY2RGB);
            rectangle(LeftRecitifyImage, Point(input_box->left_box.left_x, input_box->right_box.left_y),
                      Point(input_box->left_box.left_x + input_box->left_box.width,
                            input_box->right_box.left_y + input_box->right_box.height), Scalar(0, 0, 255), 8, 2, 0);
            putText(LeftRecitifyImage, +"Dis: " + to_string(dis) + ",",
                    Point(input_box->left_box.left_x, input_box->left_box.left_y - 10), cv::FONT_HERSHEY_COMPLEX, 2,
                    Scalar(0, 0, 255), 2, 8, 0);
            imwrite(R"(/cytech_ai/env/BinocularVision/build/alg_log/save.jpg)",LeftRecitifyImage);
        }
    }
    Release(handler);
    int dd;
    cin >> dd;

    return 0;
}
