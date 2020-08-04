/**
* @file SaveFilePostProcess.cpp
*
* Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include <ctime>
#include <hiaiengine/log.h>
#include <string>
#include <vector>
#include <string.h>
#include <unistd.h>
#include <thread>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <stdlib.h>
#include <sys/stat.h>
#include <sstream>
#include <fcntl.h>
#include "SaveFilePostProcess.h"

// 数据处理
// 安装Eigen OpenCV
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "opencv2/core/hal/interface.h"
#include "opencv2/core/types.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>


#include <cmath>

int IMG_NUM;
using cv::Mat;
std::shared_ptr<EngineTransNewT> motion_data_old = std::make_shared<EngineTransNewT>();
std::shared_ptr<EngineTransNewT> motion_data_new = std::make_shared<EngineTransNewT>();

//void SaveFilePostProcess::

HIAI_StatusT SaveFilePostProcess::Init(const hiai::AIConfig& config, const  std::vector<hiai::AIModelDescription>& model_desc) {
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[SaveFilePostProcess] Start init!");
    IMG_NUM = 0;

    uint32_t graphId = Engine::GetGraphId();
    std::shared_ptr<Graph> graph = Graph::GetInstance(graphId);
    if (graph == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SaveFilePostProcess] Fail to get the graph id.");
        return HIAI_ERROR;
    }
    std::ostringstream deviceId;
    deviceId << graph->GetDeviceID();
    string deviceDir = RESULT_FOLDER + "/" + deviceId.str();
    storePath = deviceDir + "/" + ENGINE_NAME;
    if (HIAI_OK != CreateFolder(RESULT_FOLDER, PERMISSION)) {
        return HIAI_ERROR;
    }
    if (HIAI_OK != CreateFolder(deviceDir, PERMISSION)) {
        return HIAI_ERROR;
    }
    if (HIAI_OK != CreateFolder(storePath, PERMISSION)) {
        return HIAI_ERROR;
    }
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[SaveFilePostProcess] End init!");
//    std::cout << "Save engine inited!!" << std::endl;
    return HIAI_OK;
}

HIAI_StatusT SaveFilePostProcess::SendBatch(std::shared_ptr<EngineTransNewT> imageInfoBatch){
    HIAI_StatusT hiaiRet = HIAI_OK;

    do {
        hiaiRet = SendData(DEFAULT_DATA_PORT, "EngineTransNewT", std::static_pointer_cast<void>(imageInfoBatch));
        if (HIAI_QUEUE_FULL == hiaiRet) {
            HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[DataInput] The queue is full, sleep 200ms.");
            usleep(SEND_DATA_INTERVAL_MS);
        }
    } while (hiaiRet == HIAI_QUEUE_FULL);

    return hiaiRet;
}

HIAI_IMPL_ENGINE_PROCESS("SaveFilePostProcess", SaveFilePostProcess, INPUT_SIZE)
{
clock_t start_time = clock();
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[SaveFilePostProcess] Start process!");
    if (arg0 == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_WARNING, "[SaveFilePostProcess] The arg0 is null.");
        return HIAI_ERROR;
    }
    std::shared_ptr<EngineTransT> tran = std::static_pointer_cast<EngineTransT>(arg0);
    //add sentinel image for showing this data in dataset are all sent, this is last step.
    BatchImageParaWithScaleT imageHandle = {tran->b_info, tran->v_img};

    if (isSentinelImage(std::make_shared<BatchImageParaWithScaleT>(imageHandle))) {
        HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[SaveFilePostProcess] Send sentinel image, process over.");
        std::shared_ptr<std::string> result(new std::string);
        HIAI_StatusT hiaiRet = HIAI_OK;
        do {
          // 收到了sentinel image 回调最开始的引擎，结束！！！！！
            hiaiRet = SendData(0, "string", std::static_pointer_cast<void>(result));
            if (HIAI_QUEUE_FULL == hiaiRet) {
                HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[SaveFilePostProcess] The queue is full, sleep 200ms");
                usleep(SEND_DATA_INTERVAL_MS);
            }
        } while (hiaiRet == HIAI_QUEUE_FULL);

        if (HIAI_OK != hiaiRet) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SaveFilePostProcess] Failed to send data, error code: %d", hiaiRet);
        }
        return hiaiRet;
    }
    if (!tran->status) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, tran->msg.c_str());
        return HIAI_ERROR;
    }
    std::vector<OutputT> outputDataVec = tran->output_data_vec;
    
    // tran->b_info.batch_size ========= 1  因为每次读取一张照片就放进引擎里，所以是1
    for (unsigned int ind = 0; ind < tran->b_info.batch_size; ind++) {
        int frameId = (int)tran->b_info.frame_ID[ind];
        if (frameId == -1) {
            HIAI_ENGINE_LOG(HIAI_IDE_WARNING, "[SaveFilePostProcess] There is no image result for index %d.", ind);
            continue;
        }

        // outputDataVec.size() ======== 2    因为是两个txt文件
        for (unsigned int i = 0 ; i < outputDataVec.size(); ++i) {

            OutputT out = outputDataVec[i];
            if (out.size / sizeof(float) == 9728){
                // 只需要获取18个关键点的位置，不需要互相之间的连接关系
                continue;
            }
            if (out.size <= 0) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SaveFilePostProcess] The OutPutT size(%d) less than 0.", out.size);
                return HIAI_ERROR;
            }

            /*jiashi changed*/
            // 获取raw data 存进newresult里面
            int newsize = out.size / sizeof(float);
            float* newresult = nullptr;
            try{
                newresult = new float[newsize];
            }catch (const std::bad_alloc& e) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SaveFilePostProcess_1] alloc output data error!");
                return HIAI_ERROR;
            }

            int ret  = memcpy_s(newresult, sizeof(float)*newsize, out.data.get(), sizeof(float)*newsize);

            // 遍历18张16x16的图，每一张图得出一个关键点的位置和可信度

//            cout << "=========================IMAGE:" << IMG_NUM << "'s KEY POINTS ==============================\n";
            IMG_NUM++;
            IMG_NUM = IMG_NUM % 100;
            clock_t start_time = clock();

            Mat temp_mat;
            Eigen::Matrix <float, 128, 128> resized_matrix;
            Eigen::MatrixXd::Index maxRow, maxCol;
            Eigen::MatrixXd::Index maxRow_new, maxCol_new;
            Eigen::MatrixXd::Index temp_maxRow, temp_maxCol;
            float temp_aa;

            for (int pic_num = 0; pic_num < 14; pic_num++){
                float *v = newresult+pic_num*256;
                // 按照列映射到Matrix
                Eigen::Map<Eigen::MatrixXf> matrQQ(v, 16, 16);

                Eigen::Matrix <float, 16, 16> m=matrQQ;
                temp_aa = m.maxCoeff(&maxRow, &maxCol);
//                cout << "maxRow" << maxRow << "   maxCol" << maxCol << endl;
//                cout << "m(0,0)" << m(0,0) << endl;
//                cout << "m(0,1)" << m(0,1) << endl;
//                cout << "m(0,2)" << m(0,2) << endl;
//                cout << "m(0,3)" << m(0,3) << endl;
//                cout << "m(0,4)" << m(0,4) << endl;
//                cout << "m(0,5)" << m(0,5) << endl;
//                cout << "m(0,6)" << m(0,6) << endl;
//                cout << "m(0,7)" << m(0,7) << endl;
//                cout << "m(0,8)" << m(0,8) << endl;
//                cout << "m(0,9)" << m(0,9) << endl;
//                cout << "m(0,10)" << m(0,10) << endl;
//                cout << "m(0,11)" << m(0,11) << endl;
//                cout << "m(0,12)" << m(0,12) << endl;
//                cout << "m(0,13)" << m(0,13) << endl;
//                cout << "m(0,14)" << m(0,14) << endl;
//                cout << "m(0,15)" << m(0,15) << endl;
//                cout << "m(1,1)" << m(1,1) << endl;
//                cout << "m(3,3)" << m(3,3) << endl;
//                cout << "m(5,8)" << m(5,8) << endl;

                cv::eigen2cv(m, temp_mat);
                cv::resize(temp_mat, temp_mat, cv::Size(128, 128), cv::INTER_CUBIC);
                cv::GaussianBlur(temp_mat, temp_mat, cv::Size(3,3), 3);


                cv::cv2eigen(temp_mat, resized_matrix);
//                for (int jjj=0; jjj < 128; jjj++){
//                    cout << "resized(0, " << jjj << ")"<< resized_matrix(0,jjj) << endl;
//                }
                temp_maxRow = maxRow*9 - 10;
                temp_maxCol = maxCol*9 - 10;
                if(temp_maxRow < 0){
                    temp_maxRow = 0;
                }
                if(temp_maxCol < 0){
                    temp_maxCol = 0;
                }
                if(temp_maxRow > 107){
                    temp_maxRow = 107;
                }
                if(temp_maxCol > 107){
                    temp_maxCol = 107;
                }
                cout << "===========" << temp_maxRow << "    " << temp_maxCol << endl;
                Eigen::MatrixXf small_matrix = resized_matrix.block<20, 20>(temp_maxRow, temp_maxCol);
                small_matrix.maxCoeff(&maxRow_new, &maxCol_new);
                cout << "new" << maxRow_new + temp_maxRow << "   " << temp_maxCol<< "  " << temp_aa << endl;
                temp_aa = resized_matrix.maxCoeff(&maxRow, &maxCol);
                std::cout << "Process 8 time::" << double(clock() - start_time) / CLOCKS_PER_SEC << std::endl;

                if (temp_aa > 0.1){
                    motion_data_new->data[0][99][pic_num] = maxRow / 128.f;
                    motion_data_new->data[1][99][pic_num] = maxCol / 128.f;
                    cout << "normal" << maxRow << "   " << maxCol << "  " << temp_aa << endl;
                }
                else{
                    motion_data_new->data[0][99][pic_num] = 0.0;
                    motion_data_new->data[1][99][pic_num] = 0.0;
                    cout << "max < 0.1" << maxRow << "   " << maxCol << "  " << temp_aa << endl;
                }
            }
            // x
            memcpy(motion_data_new->data[0][0], motion_data_old->data[0][1], sizeof(float)*14*99);
            // y
            memcpy(motion_data_new->data[1][0], motion_data_old->data[1][1], sizeof(float)*14*99);
            memcpy(motion_data_old->data, motion_data_new->data, sizeof(float)*2*100*14);





            //求中心点坐标,并中心化
            float skeleton_center[2][100]={0.0};
            for ( int c = 0; c < 2; c++ )
            {
                for ( int t = 0; t < 100; t++ )
                {
        //		    cout << (skeleton[c][t][1]+skeleton[c][t][8]+skeleton[c][t][11]) << endl;
                    skeleton_center[c][t] = float((motion_data_new->data[c][t][1]+motion_data_new->data[c][t][8]+motion_data_new->data[c][t][11])/float(3.0));
                    for ( int v = 0; v < 14; v++ )
                    {
                        motion_data_new->data[c][t][v] = motion_data_new->data[c][t][v]-skeleton_center[c][t];
        //				cout << skeleton_center[c][t] << endl;
                    }
                }
            }
//            cout << "=====================X new======================" << endl;
//            cout << motion_data_new->data[0][97][0] << endl;
//            cout << motion_data_new->data[0][97][1] << endl;
//            cout << motion_data_new->data[0][98][0] << endl;
//            cout << motion_data_new->data[0][98][1] << endl;
//            cout << motion_data_new->data[0][99][0] << endl;
//            cout << motion_data_new->data[0][99][1] << endl;
//            cout << "=====================Y new======================" << endl;
//            cout << motion_data_new->data[1][97][0] << endl;
//            cout << motion_data_new->data[1][97][1] << endl;
//            cout << motion_data_new->data[1][98][0] << endl;
//            cout << motion_data_new->data[1][98][1] << endl;
//            cout << motion_data_new->data[1][99][0] << endl;
//            cout << motion_data_new->data[1][99][1] << endl;
        }
    }

    // send result

//    HIAI_StatusT hiaiRet = HIAI_OK;
//    do {
//        hiaiRet = SendData(0, "EngineTransT", std::static_pointer_cast<void>(tranData_));
//        if (HIAI_QUEUE_FULL == hiaiRet) {
//            HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngineGesture] The queue is full, sleep 200ms");
//            usleep(SEND_DATA_INTERVAL_MS);
//        }
//    } while (hiaiRet == HIAI_QUEUE_FULL);
//
//    if (HIAI_OK != hiaiRet) {
//        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngineGesture] Failed to send data , error code: %d", hiaiRet);
//    }





    HIAI_StatusT ret = HIAI_OK;
//    ret = SendBatch(motion_data_new);




//    motion_data_new = motion
//    memcpy(motion_data_new->data, motion_data_old->data[1][1], sizeof(float)*14*99);



    if (HIAI_OK != ret) {
    cout << "ERRORERRORERRORERRORERROR" << endl;
        return ret;
    }
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[SaveFilePostProcess] End process!");
        std::cout << "engine4 SaveFileEngine time::" << double(clock() - start_time) / CLOCKS_PER_SEC << std::endl;

    return HIAI_OK;
}
