/**
* @file SaveFilePostProcess.cpp
*
* Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include <hiaiengine/log.h>
#include <string>
#include <vector>
#include <string.h>
#include <cmath>
#include <unistd.h>
#include <thread>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <stdlib.h>
#include <sys/stat.h>
#include <sstream>
#include <fcntl.h>
#include "Common.h"
#include "SaveFilePostProcess.h"

// 数据处理
// 安装Eigen OpenCV
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "hiaiengine/status.h"
#include "opencv2/core/hal/interface.h"
#include "opencv2/core/types.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>


#include <cmath>

int IMG_NUM;
int file_num = 0;
float temp_key_points[2][14] = {0};
Eigen::MatrixXf left_matrix(16, 16);
Eigen::MatrixXf right_matrix(16, 16);
Eigen::MatrixXf top_matrix(16, 16);
Eigen::MatrixXf bottom_matrix(16, 16);
Eigen::MatrixXf thre(16, 16);
//Eigen::MatrixXf thre_2(10, 1);
Eigen::MatrixXf thre_result(16, 16);
//int limbSeq[19][2] = {{2,3}, {2,6}, {3,4}, {4, 5}, {6, 7}, {7, 8}, {2, 9}, {9, 10}, {10, 11}, {2, 12},
//                        {12, 13}, {13, 14}, {2, 1}, {1, 15}, {15, 17}, {1, 16}, {16, 18}, {3, 17}, {6,18}};
int limbSeq[13][2] = {{2,3}, {2,6}, {3,4}, {4, 5}, {6, 7}, {7, 8}, {2, 9}, {9, 10}, {10, 11}, {2, 12},
                        {12, 13}, {13, 14}, {2, 1}};

//int mapIdx[19][2] = {{31,32}, {39,40}, {33,34}, {35,36}, {41,42}, {43,44}, {19,20}, {21,22}, {23,24},
//                        {25,26}, {27,28}, {29,30}, {47,48}, {49,50}, {53,54}, {51,52}, {55,56}, {37,38}, {45,46}};
int mapIdx[19][2] = {{31,32}, {39,40}, {33,34}, {35,36}, {41,42}, {43,44}, {19,20}, {21,22}, {23,24},
                        {25,26}, {27,28}, {29,30}, {47,48}};
int BAD_NUM;
float TOTAL_RIGHT, TOTAL_LEFT, TOTAL_TOP, TOTAL_BOTTOM;
using cv::Mat;
float TEMP_LEFT, TEMP_RIGHT, TEMP_BOTTOM, TEMP_TOP;
std::shared_ptr<EngineTransNewT> motion_data_old = std::make_shared<EngineTransNewT>();
std::shared_ptr<EngineTransNewT> motion_data_new = std::make_shared<EngineTransNewT>();



//void SaveFilePostProcess::

HIAI_StatusT SaveFilePostProcess::Init(const hiai::AIConfig& config, const  std::vector<hiai::AIModelDescription>& model_desc) {
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[SaveFilePostProcess] Start init!");

    //
    thre = Eigen::MatrixXf::Ones(16, 16) * 0.1;
//    thre_2 = Eigen::MatrixXf::Constant(0.05);
//    thre = thre*0.1;
//cout << thre(33, 33) << "  " << thre(44, 1) << endl;
    IMG_NUM = 0;
    BAD_NUM = 0;
    TEMP_RIGHT = TEMP_BOTTOM = 0.0;
    TEMP_LEFT = TEMP_TOP = 128.0;
    TOTAL_RIGHT = TOTAL_LEFT = TOTAL_BOTTOM = TOTAL_TOP = -1.0;
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
            cout << "wwwwwwwwwwwwwwwwwwwwww" << endl;
            usleep(SEND_DATA_INTERVAL_MS);
        }
    } while (hiaiRet == HIAI_QUEUE_FULL);

    return hiaiRet;
}

HIAI_StatusT SaveFilePostProcess::ProcessData(){
    float temp_left = 128;
    float temp_right = 0;
    float temp_top = 128;
    float temp_bottom = 0;
    float total_left = 0;
    float total_right = 0;
    float total_top = 0;
    float total_bottom = 0;
//    cout << "2222222222222222222" << endl;
    // 50 frames
    for (int pic_num = 0; pic_num < 5; pic_num++){
//        motion_data_new->data[1][pic_num][1] = motion_data_new->data[1][pic_num][1] * 128; // y
//        motion_data_new->data[1][pic_num][8] = motion_data_new->data[1][pic_num][8] * 128; // y
//        motion_data_new->data[1][pic_num][11] = motion_data_new->data[1][pic_num][11] * 128; // y
        motion_data_new->data[1][pic_num][1] = motion_data_new->data[1][pic_num][1] ; // y
        motion_data_new->data[1][pic_num][8] = motion_data_new->data[1][pic_num][8] ; // y
        motion_data_new->data[1][pic_num][11] = motion_data_new->data[1][pic_num][11]; // y
        // 累加
        total_bottom +=  float(motion_data_new->data[1][pic_num][8] + motion_data_new->data[1][pic_num][11]) / 2 - float(motion_data_new->data[1][pic_num][1]);
//        cout << "motion_data_new->data[1][pic_num][1]    " << motion_data_new->data[1][pic_num][1] << endl;
//        cout << "motion_data_new->data[1][pic_num][8]    " << motion_data_new->data[1][pic_num][8] << endl;
//        cout << "motion_data_new->data[1][pic_num][11]    " << motion_data_new->data[1][pic_num][11] << endl;
    }

//    cout << "----" << total_bottom << endl;
//    cout << "aaaaaaaaaaaaaaaaa" << endl;
    total_bottom /= 5.0;
//    cout << "----22222" << total_bottom << endl;
    for (int pic_num = 0; pic_num < 50; pic_num++){
        for(int key_num = 0; key_num < 14; key_num++){
            motion_data_new->data[0][pic_num][key_num] /= total_bottom;
            motion_data_new->data[1][pic_num][key_num] /= total_bottom;
        }
    }
//    clock_t temp_start_time = clock();
    //求中心点坐标,并中心化
//    float skeleton_center[2][50]={0.0};
//    for ( int c = 0; c < 2; c++ )
//    {
//        for ( int t = 0; t < 50; t++ )
////        for ( int t = 0; t < 30; t++ )
//        {
//            skeleton_center[c][t] = float((motion_data_new->data[c][t][1]+motion_data_new->data[c][t][8]+motion_data_new->data[c][t][11])/float(3.0));
//            for ( int v = 0; v < 14; v++ )
//            {
//                motion_data_new->data[c][t][v] = motion_data_new->data[c][t][v]-skeleton_center[c][t];
//            }
//        }
//    }
    return HIAI_OK;
}

HIAI_StatusT SaveFilePostProcess::SaveData(){
    string file_name = "./data/" + to_string(file_num) + "_raw_data.txt";
    ofstream file;
    file.open(file_name.c_str(), ios::trunc);
//    cout << "IMG_NUM" << IMG_NUM << endl;
//    if (IMG_NUM >= 55){
//        cout << "=================NEW TXT===============" << file_num << endl;
//            cout << "60606060606" << endl;
    for (int jj=0; jj < 2; jj++){
        for (int aa = 0; aa < 50; aa++){
            for (int qq = 0; qq < 14; qq++){
                file << motion_data_new->data[jj][aa][qq] << "\n";
            }
        }
    }
    IMG_NUM = 0;
    file_num++;
    file.close();
//    file_name = "./data/" + to_string(file_num) + "_raw_data.txt";
//    file.open(file_name.c_str(), ios::trunc);
//    }
//    if_final = true;
    return HIAI_OK;
}
//vector<int>::it

int find_index(vector<int>::iterator begin, vector<int>::iterator end, int element){
    auto temp = begin;
    while(temp != end){
        if(*temp == element){
            return element;
        }
        temp += 1;
    }
    return -1;
}

bool cmp2(connectionT a,connectionT b)
{
return a.score>b.score;//按照学号降序排列
//return a.id<b.id;//按照学号升序排列
}

HIAI_IMPL_ENGINE_PROCESS("SaveFilePostProcess", SaveFilePostProcess, INPUT_SIZE)
{
    clock_t start_time = clock();
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[SaveFilePostProcess] Start process!");
    if (arg0 == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_WARNING, "[SaveFilePostProcess] The arg0 is null.");
        return HIAI_ERROR;
    }
    // 获取最新的数据
    std::shared_ptr<EngineTransT> tran = std::static_pointer_cast<EngineTransT>(arg0);
    //add sentinel image for showing this data in dataset are all sent, this is last step.
    BatchImageParaWithScaleT imageHandle = {tran->b_info, tran->v_img};


    if (!tran->status) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, tran->msg.c_str());
        return HIAI_ERROR;
    }
    std::vector<OutputT> outputDataVec = tran->output_data_vec;

    int frameId = (int)tran->b_info.frame_ID[0];
    if (frameId == -1) {
        HIAI_ENGINE_LOG(HIAI_IDE_WARNING, "[SaveFilePostProcess] There is no image result for index %d.", 0);
        return HIAI_OK;
    }




//// outputDataVec.size() ======== 2    因为是两个txt文件
//    for (unsigned int i = 0 ; i < outputDataVec.size(); ++i) {
//        std::string prefix = storePath  + "/" + to_string(i);
////        IMG_NUM++;
//        OutputT outttt = outputDataVec[i];
////            if (out.size / sizeof(float) == 9728){
////                // 只需要获取18个关键点的位置，不需要互相之间的连接关系
////                continue;
////            }
//        if (outttt.size <= 0) {
//            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SaveFilePostProcess] The OutPutT size(%d) less than 0.", outttt.size);
//            return HIAI_ERROR;
//        }
//
////            Eigen::Matrix2Xf
//
//
//        // 记录数据在一个txt文档内
//        // 这里替换成先获取18个关键点的位置和可信度，然后提交给下一个Engine
//        std::string name(outttt.name);
//        GetOutputName(name);
//        std::string outFileName = prefix + "_" + name + ".txt";
//        int fd = open(outFileName.c_str(), O_CREAT| O_WRONLY, FIlE_PERMISSION);
//        if (fd == -1) {
//            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SaveFilePostProcess] Failed to open file %s.", outFileName.c_str());
//            return HIAI_ERROR;
//        }
//        int oneResultSize = outttt.size / tran->b_info.max_batch_size;
//
//
//
//        /*jiashi changed*/
//        // 获取raw data 存进newresult里面
//        int newsize = outttt.size / sizeof(float);
//        float* newresult = nullptr;
//        try{
//            newresult = new float[newsize];
//        }catch (const std::bad_alloc& e) {
//            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SaveFilePostProcess_1] alloc output data error!");
//            return HIAI_ERROR;
//        }
//////            origin version
////            int ret = write(fd, out.data.get() + oneResultSize * ind, oneResultSize);
////            std::cout << out.data.get();
//        int ret  = memcpy_s(newresult, sizeof(float)*newsize, outttt.data.get(), sizeof(float)*newsize);
//
//
////         std::cout << "iiiiii" << std::endl;
//
//
//        // 把推理结果写入txt文件
////        std::cout << "newsize" << newsize << std::endl;
////             new size ======== 9728 和 4846
//        for (int k = 0; k < newsize; k++){
//            std::string value = std::to_string(newresult[k]);
//            if(k > 0){
//                value = "\n" + value;
//            }
//            ret = write(fd, value.c_str(), value.length());
//            if(ret == -1){
//                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SaveFilePostProcess_1] write file error!");
//                ret = close(fd);
//                if(ret == -1){
//                    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SaveFilePostProcess_1] close file error!");
//                }
//                delete[] newresult;
//                newresult = NULL;
//                return HIAI_ERROR;
//            }
//        }
//
//
//        if(ret == -1){
//            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SaveFilePostProcess] Failed to write data to file.");
//            ret = close(fd);
//            if(ret == -1){
//                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SaveFilePostProcess] Failed to close file.");
//            }
//            return HIAI_ERROR;
//        }
//        ret = close(fd);
//        if(ret == -1){
//            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SaveFilePostProcess] Failed to close file.");
//            return HIAI_ERROR;
//        }
//    }






//   std::cout << "temp11111 calculate time::" << double(clock() - start_time) / CLOCKS_PER_SEC << std::endl;
//    start_time = clock();
    // 获取第二个输出数据
    OutputT out = outputDataVec[1];
    // 判断是否有效
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
    // 拷贝数据至newresult
    int ret  = memcpy_s(newresult, sizeof(float)*newsize, out.data.get(), sizeof(float)*newsize);

    // 遍历18张16x16的图，每一张图得出一个关键点的位置和可信度

//            cout << "=========================IMAGE:" << IMG_NUM << "'s KEY POINTS ==============================\n";
    // 这里的IMG_NUM只是用于计数

//   std::cout << "temp22222 calculate time::" << double(clock() - start_time) / CLOCKS_PER_SEC << std::endl;

    // 获取前14个关节特征点
//    start_time = clock();
    Mat temp_mat;
    Mat temp_mat_0;
    Mat temp_mat_1;
    Eigen::Matrix <float, 128, 128> resized_matrix;
    Eigen::Matrix <float, 128, 128> score_mid_0;
    Eigen::Matrix <float, 128, 128> score_mid_1;
    Eigen::MatrixXd::Index maxRow, maxCol;
    Eigen::MatrixXd::Index maxRow_F, maxCol_F;
    Eigen::MatrixXd::Index maxRow_new, maxCol_new;
    Eigen::MatrixXd::Index temp_maxRow, temp_maxCol;
    vector <key_pointsT> one_pic_key_points;
    vector <vector<key_pointsT>> all_key_points;
    vector <float> one_pic_peaks;
    float temp_key_points[2][14];
    int all_peak_index = 0;
    float temp_aa;
    bool if_valid = true;
//    cv::Point x2, x3, x4, y5, y6, y7;
    // 生成一张纯白图
    Mat out1(cv::Size(128,128), CV_8UC3, cv::Scalar(255, 255, 255));
//     cout << "+++++++++++++++++++++++++++" << endl;
//     cout << (0.2)&(-0.2) << endl;
    for (int pic_num = 0; pic_num < 14; pic_num++){
        float *v = newresult+pic_num*256;
        // 按照列映射到Matrix
        Eigen::Map<Eigen::MatrixXf> matrQQ(v, 16, 16);

        Eigen::Matrix <float, 16, 16> m=matrQQ;
//        cout << "==========================" << pic_num << "==========================\n";
//        cout << "---------------------------ORIGIN------------------------------------\n";
//        for (int aa = 0; aa < 16; aa++){
//            for(int bbc = 0; bbc < 16; bbc++){
//                cout << m(aa, bbc) << "   ";
//            }
//            cout << endl;
//        }
        temp_aa = m.maxCoeff(&maxRow_F, &maxCol_F);
        // 有一个点找不到就break，认为无效
        if (temp_aa < 0.1){
            if_valid = false;
//            motion_data_new->data[0][FRAME_LENGTH-1][pic_num] = 0.0;
//            motion_data_new->data[1][FRAME_LENGTH-1][pic_num] = 0.0;
            break;
        }


        left_matrix.leftCols(15) = m.rightCols(15);
        left_matrix.col(15) = Eigen::MatrixXf::Zero(16, 1);

//        cout << "-------------------------------LEFT-------------------------------------\n";
//        for (int aa = 0; aa < 16; aa++){
//            for(int bbc = 0; bbc < 16; bbc++){
//                cout << left_matrix(aa, bbc) << "   ";
//            }
//            cout << endl;
//        }

//        cout << "--------------------------" << endl;
        right_matrix.rightCols(15) = m.leftCols(15);
        right_matrix.col(0) = Eigen::MatrixXf::Zero(16, 1);
        top_matrix.topRows(15) = m.bottomRows(15);
        top_matrix.row(15) = Eigen::MatrixXf::Zero(1, 16);
        bottom_matrix.bottomRows(15) = m.topRows(15);
        bottom_matrix.row(0) = Eigen::MatrixXf::Zero(1, 16);
        left_matrix = m - left_matrix;
        right_matrix = m - right_matrix;
        top_matrix = m - top_matrix;
        bottom_matrix = m - bottom_matrix;
//        thre_result = m - thre;
//        cout << "----------------------------------FINAL RESULT-------------------------------\n";
//        for (int aa = 0; aa < 16; aa++){
//            for(int bbc = 0; bbc < 16; bbc++){
//                cout << left_matrix(aa, bbc) << "   ";
//            }
//            cout << endl;
//        }

        for (int aa = 0; aa < 16; aa++){
            for(int bb = 0; bb < 16; bb++){
                if(left_matrix(aa, bb) > 0 && right_matrix(aa, bb) > 0 && bottom_matrix(aa, bb) > 0 && top_matrix(aa, bb) > 0 && m(aa, bb) > 0.1){
//                    cout << "aa  " << aa << "  bb  " << bb << endl;
                    one_pic_peaks.push_back(aa);
                    one_pic_peaks.push_back(bb);
                }
            }
        }
//        cout << "one_pic_peaks.size()" << one_pic_peaks.size() << endl;;

        cv::eigen2cv(m, temp_mat);
        cv::resize(temp_mat, temp_mat, cv::Size(128, 128), cv::INTER_CUBIC);
        cv::GaussianBlur(temp_mat, temp_mat, cv::Size(3,3), 5);

        cv::cv2eigen(temp_mat, resized_matrix);

        // 寻找每一个大于0.1的波峰
//        cout << "note" << endl;

        // 寻找每张图的局部最大值
        for (int aa = 0; aa < one_pic_peaks.size(); aa+=2){
            temp_maxRow = one_pic_peaks[aa] * 8 - 6;
            temp_maxCol = one_pic_peaks[aa+1] * 8 - 6;
            if(temp_maxRow < 0){
                temp_maxRow = 0;
            }
            if(temp_maxCol < 0){
                temp_maxCol = 0;
            }
            if(temp_maxRow > 121){
                temp_maxRow = 121;
            }
            if(temp_maxCol > 121){
                temp_maxCol = 121;
            }
            // 128中的局部最大值下标
            // TODO
            Eigen::MatrixXf small_matrix = resized_matrix.block<12, 12>(temp_maxRow, temp_maxCol);
            temp_aa = small_matrix.maxCoeff(&maxRow_new, &maxCol_new);
//            cout << "temp_maxRow  " << temp_maxRow + maxRow_new << "  temp_maxCol  " << temp_maxCol + maxCol_new << endl;
            key_pointsT temp = {float(temp_maxRow + maxRow_new), float(temp_maxCol + maxCol_new), all_peak_index};
            all_peak_index++;
            one_pic_key_points.push_back(temp);
        }
        // 如果有一个部位一个点都没找到
        if(one_pic_key_points.size() == 0){
//            cout << "one_pic_key_points.size() == 0" << endl;
            return HIAI_OK;
        }
        // 每张图计算出的keypoints存到一个vector，然后vector再存入总的keypoints
        all_key_points.push_back(one_pic_key_points);
        one_pic_peaks.clear();
        one_pic_key_points.clear();
    }
//    cout << "========all key points========size=====" << all_key_points.size() << "   " << endl;
//    for (int zz = 0; zz < all_key_points.size(); zz++){
//        cout << "each size=== " << all_key_points[zz].size() << " ";
//    }
//    cout << endl;

    // 只要有一个点找不到
    if(!if_valid){
        cout << "invalid image!!" << endl;
        return HIAI_OK;
    }

    // 寻找关键点之间的关系
    // 获取第一个输出数据
    vector <connectionT> connection_candidate;
    vector <vector<connectionT>> connection_all;
    OutputT out_0 = outputDataVec[0];
    // 判断是否有效
    if (out_0.size <= 0) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SaveFilePostProcess] The OutPutT size(%d) less than 0.", out_0.size);
        return HIAI_ERROR;
    }

    // 获取raw data 存进newresult里面
    newsize = out_0.size / sizeof(float);
//    cout << "out_o size              =============================" << newsize << endl;
    float* newresult_0 = nullptr;
    try{
        newresult_0 = new float[newsize];
    }catch (const std::bad_alloc& e) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SaveFilePostProcess_1] alloc output data error!");
        return HIAI_ERROR;
    }
    // 拷贝数据至newresult
    ret  = memcpy_s(newresult_0, sizeof(float)*newsize, out_0.data.get(), sizeof(float)*newsize);

    // 遍历mapIdx
    for (int kk = 0; kk < 13; kk++){
//        cout << "..................................." << endl;
//        cout << "kkkkkkkkkkkkkkkkkkkk   " << kk << endl;
//        cout << "mapIdx[kk][0] - 19" << mapIdx[kk][0] - 19 << endl;
//        cout << "mapIdx[kk][1] - 19" << mapIdx[kk][1] - 19 << endl;
        float *v = newresult_0 + (mapIdx[kk][0] - 19)*256;
//        cout << "vvvvvvvvvvvvvvvvvvv  " << v[0] << " " << v[1] << "  " << v[2] << endl;
        // 按照列映射到Matrix
        Eigen::Map<Eigen::MatrixXf> matrQQ_0(v, 16, 16);

        Eigen::Map<Eigen::MatrixXf> matrQQ_1(v + 256, 16, 16);

        Eigen::Matrix <float, 16, 16> m_0=matrQQ_0; // score_mid
        Eigen::Matrix <float, 16, 16> m_1=matrQQ_1; // score_mid

//        cout << "00000000000000000000000000000000" << endl;
//        for (int aa = 0; aa < 16; aa++){
//            for(int bbc = 0; bbc < 16; bbc++){
//                cout << m_0(aa, bbc) << "   ";
//            }
//            cout << endl;
//        }
//        cout << "00000000000000000000000000000000" << endl;
//        for (int aa = 0; aa < 16; aa++){
//            for(int bbc = 0; bbc < 16; bbc++){
//                cout << m_1(aa, bbc) << "   ";
//            }
//            cout << endl;
//        }
//    cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;

        cv::eigen2cv(m_0, temp_mat_0);
        cv::eigen2cv(m_1, temp_mat_1);
        cv::resize(temp_mat_0, temp_mat_0, cv::Size(128, 128), cv::INTER_CUBIC);
        cv::resize(temp_mat_1, temp_mat_1, cv::Size(128, 128), cv::INTER_CUBIC);
        cv::GaussianBlur(temp_mat_0, temp_mat_0, cv::Size(3,3), 3);
        cv::GaussianBlur(temp_mat_1, temp_mat_1, cv::Size(3,3), 3);
//        cout << "##################################" << endl;

        cv::cv2eigen(temp_mat_0, score_mid_0); // score_mid
        cv::cv2eigen(temp_mat_1, score_mid_1); // score_mid
//        cout << "score_mid_0 shape  " << score_mid_0.size() << endl; // 16384 = 128*128
//        cout << "score_mid_1 shape  " << score_mid_1.size() << endl; // 16384 = 128*128
//        cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
//        cout << "limbSeq[kk][0] -1  " << limbSeq[kk][0] -1 << endl;
//        cout << "limbSeq[kk][1] -1  " << limbSeq[kk][1] -1 << endl;
        vector <key_pointsT> temp_A = all_key_points[limbSeq[kk][0] -1];
//        cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;

        vector <key_pointsT> temp_B = all_key_points[limbSeq[kk][1] -1];

        int LA = temp_A.size();
        int LB = temp_B.size();
//        cout << "  LA   " << LA << "   LB   " << LB << endl;
//        cout << "limbSeq[kk][0] - 1  " << limbSeq[kk][0] - 1 << "  limbSeq[kk][1] - 1  " << limbSeq[kk][1] - 1 << endl;
//        cout << "  LA   " << LA << "   LB   " << LB << endl;
        if(LA != 0 && LB != 0){
            for (int aa = 0; aa < LA; aa++){
//                cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaa" << aa << endl;
                for(int bb = 0; bb < LB; bb++){
//                cout << "bbbbbbbbbbbbbbbbbbbbbbbbbbb" << bb << endl;
//                    cout << "tempA[aa]  X  " << temp_A[aa].point_x << "  Y  " << temp_A[aa].point_y << endl;
//                    cout << "tempB[bb]  X  " << temp_B[bb].point_x << "  Y  " << temp_B[bb].point_y << endl;
                    float vec[2] = {temp_B[bb].point_x - temp_A[aa].point_x, temp_B[bb].point_y - temp_A[aa].point_y};
//                    cout << "veccccc  " << vec[0] << "   " << vec[1] << endl;
                    float norm = sqrt(vec[0]*vec[0] + vec[1]*vec[1]);
//                    cout << "norm  " << norm << endl;
                    // TODO 查看是不是float类型
                    vec[0] /= norm;
                    vec[1] /= norm;
//                    cout << "new vector  " << vec[0] << "  " << vec[1] << endl;
                    Eigen::Matrix <float ,10 ,2> startend;
                    startend.col(0) = Eigen::ArrayXf::LinSpaced(10, temp_A[aa].point_x, temp_B[bb].point_x);
                    startend.col(1) = Eigen::ArrayXf::LinSpaced(10, temp_A[aa].point_y, temp_B[bb].point_y);
//                    cout << "tempA X" << temp_A[aa].point_x << "tempB X" << temp_B[bb].point_x << endl;
//                    cout << "================startend=========================00000" << endl;
//                    for (int qqqq = 0; qqqq < 10; qqqq++){
//                        cout << startend(qqqq, 0)<< "   ";
//                    }
//                    cout << "================startend=========================11111" << endl;
//                    for (int qqqq = 0; qqqq < 10; qqqq++){
//                        cout << startend(qqqq, 1)<< "   ";
//                    }
//                    cout << endl;
                    // TODO vector
                    Eigen::Matrix <float, 10, 1> vec_x;
                    Eigen::Matrix <float, 10, 1> vec_y;
//                    cout << "int(round(startend(0 ,0))" << int(round(startend(0 ,0))) << endl;
//                    cout << "int(round(startend(0 ,1))" << int(round(startend(0 ,1))) << endl;
//                    cout << "round(startend(0 ,1))" << round(startend(0 ,1)) << endl;
//                    cout << typeid(round(startend(0 ,1))).name() << endl;
//                    cout << "startend(0 ,1)" << startend(0 ,1) << endl;
//                    cout << "score_mid_0(int(round(startend(0 ,1))), int(round(startend(0 ,0))))" << typeid(score_mid_0(int(round(startend(0 ,1))), int(round(startend(0 ,0))))).name() << endl;
                    // TODO transformed!!!!
                    vec_x << score_mid_0(int(round(startend(0 ,0))), int(round(startend(0 ,1)))), score_mid_0(int(round(startend(1 ,0))), int(round(startend(1 ,1)))), score_mid_0(int(round(startend(2 ,0))), int(round(startend(2 ,1))))
                           , score_mid_0(int(round(startend(3 ,0))), int(round(startend(3 ,1)))), score_mid_0(int(round(startend(4 ,0))), int(round(startend(4 ,1)))), score_mid_0(int(round(startend(5 ,0))), int(round(startend(5 ,1))))
                           , score_mid_0(int(round(startend(6 ,0))), int(round(startend(6 ,1)))), score_mid_0(int(round(startend(7 ,0))), int(round(startend(7 ,1)))), score_mid_0(int(round(startend(8 ,0))), int(round(startend(8 ,1))))
                           , score_mid_0(int(round(startend(9 ,0))), int(round(startend(9 ,1))));

                    vec_y << score_mid_1(int(round(startend(0 ,0))), int(round(startend(0 ,1)))), score_mid_1(int(round(startend(1 ,0))), int(round(startend(1 ,1)))), score_mid_1(int(round(startend(2 ,0))), int(round(startend(2 ,1))))
                           , score_mid_1(int(round(startend(3 ,0))), int(round(startend(3 ,1)))), score_mid_1(int(round(startend(4 ,0))), int(round(startend(4 ,1)))), score_mid_1(int(round(startend(5 ,0))), int(round(startend(5 ,1))))
                           , score_mid_1(int(round(startend(6 ,0))), int(round(startend(6 ,1)))), score_mid_1(int(round(startend(7 ,0))), int(round(startend(7 ,1)))), score_mid_1(int(round(startend(8 ,0))), int(round(startend(8 ,1))))
                           , score_mid_1(int(round(startend(9 ,0))), int(round(startend(9 ,1))));
//                    cout << "vec_x.size()  " << vec_x.size() << endl;
//                    cout << "vec_y.size()  " << vec_y.size() << endl;
//                    if(temp_A[aa].num == 13 && temp_B[bb].num == 18){
//
//                        cout << "score_mid_0" << endl;
//                        for(int tem = 0; tem < 10; tem++){
//                            cout << vec_x(tem, 0) << "   ";
//                        }
//                        cout << endl;
//                        cout << "score_mid_1" << endl;
//                        for(int tem = 0; tem < 10; tem++){
//                            cout << vec_y(tem, 0) << "   ";
//                        }
//                        cout << endl;
//                        cout << "vec[0]  " << vec[0] << "  vec[1]  " << vec[1] << endl;
//                    }
                    Eigen::Matrix <float, 10, 1>score_midpts = vec_x * vec[0] + vec_y * vec[1];
//                    cout << score_midpts.data() << endl;
//                    cout << "score_midpts" << endl;
//                    for(int tem = 0; tem < 10; tem++){
//                        cout << score_midpts[tem] << "   ";
//                    }
//                    cout << endl;
//                    cout << "score_midpts.sum()  " << score_midpts.sum() << endl;
//                    cout << "score_midpts.size()  " << score_midpts.size() << endl;
                    float score_with_dist_prior = score_midpts.sum() / (10.001) + min(64/(norm-1+1e-3), 0.0);

//                    cout << "score_with_dist_prior" << score_with_dist_prior << endl;
//                    if(temp_A[aa].num == 0 && temp_B[bb].num == 0){
//                        cout << "---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n";
//                        cout << "---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n";
//                        cout << "---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n";
//                        cout << "---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n";
//                        cout << "---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n";
//                        cout << "---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n";
//                        cout << "---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n";
//                        cout <<
//                    }
                    if (score_with_dist_prior > 0){
                        int bad_num = 0;
                        for (int fff = 0; fff < 10; fff++){
                            if(score_midpts(fff) < 0.05){
                                bad_num++;
                            }
                        }
//                        cout << "bad_num" << bad_num << endl;
                        if(bad_num < 2){
//                            cout << "PUSHPUSHPUSH\n";
//                            cout << "aaaaaaaa  " << aa << "  bbbbbbbb  " << bb << endl;
//                            cout << " temp_A[aa].num " << temp_A[aa].num << "  temp_B[bb].num  " << temp_B[bb].num << endl;
                            connectionT temp_connection{aa, bb, score_with_dist_prior};
                            connection_candidate.push_back(temp_connection);
                        }
                    }
                }
            }

//            cout << "1111111111111111111111111111" << endl;
            if(connection_candidate.size() == 0){
//                cout << " connection_candidata.size() == 0 " << endl;
                return HIAI_OK;
            }
            sort(connection_candidate.begin(), connection_candidate.end(), cmp2);
//            cout << "connection_candidate size  " << connection_candidate.size() << endl;
            vector<int> temp_1;
            vector<int> temp_2;
            temp_1.push_back(33);
            temp_2.push_back(33);
//            vector<int>::iterator p_i;
//            vector<int>::iterator p_j;
            int p_i;
            int p_j;
            vector<connectionT> one_connection;
            for (int tt = 0; tt < connection_candidate.size(); tt++){
//            cout << "tttttttttttttttttttttttttttt" << tt << endl;
                int i = connection_candidate[tt].point_1;
                int j = connection_candidate[tt].point_2;
                float s = connection_candidate[tt].score;

                p_i = find_index(temp_1.begin(), temp_1.end(), i);
//                cout << "++++++++++++++++++++++++++++++++++  " << p_i << "    " << i << endl;
//                cout << typeid(temp_1.begin()).name() << "  ---++++----" << endl;
//                cout << "=================temp11111111111111=========================\n";
//                for(int rrrr = 0; rrrr < temp_1.size(); rrrr++){
//                    cout << temp_1[rrrr] << "    " ;
//                }
//                cout << endl;
                p_j = find_index(temp_2.begin(), temp_2.end(), j);
//                if(p_i == temp_1.end() && p_j == temp_2.end()){
                if(p_i != i && p_j != j){
//                    cout << "pppppppppppiiiiiiiiii  YES  " << p_i << endl;
//                    cout << "pppppppppppjjjjjjjjjj  YES  " << p_j << endl;
//                    cout << "iiiiiiiiiiiii  YES " << i << endl;
//                    cout << "jjjjjjjjjjjjj  YES " << j << endl;
//                    cout << "tempAAAAAAAAA  YES " << temp_A[i].num << endl;
//                    cout << "tempBBBBBBBBB  YES " << temp_B[j].num << endl;
                    temp_1.push_back(i);
                    temp_2.push_back(j);
                    connectionT temp{temp_A[i].num, temp_B[j].num};
                    one_connection.push_back(temp);
                    if (one_connection.size() >= min(LA, LB)){
                        break;
                    }
                }
//                else{
//                    cout << "pppppppppppiiiiiiiiii  NO  " << p_i << endl;
//                    cout << "pppppppppppjjjjjjjjjj  NO  " << p_j << endl;
//                    cout << "iiiiiiiiiiiii  NO " << i << endl;
//                    cout << "jjjjjjjjjjjjj  NO " << j << endl;
//                    cout << "tempAAAAAAAAA  NO " << temp_A[i].num << endl;
//                    cout << "tempBBBBBBBBB  NO " << temp_B[j].num << endl;
//                }
            }
            connection_candidate.clear();
//            cout << "thisssssssssssssssss  oneconnectionnnnnnnnnn sizeeeeeeee  " << one_connection.size() << endl;

            connection_all.push_back(one_connection);
            one_connection.clear();
        }
    }

//    cout << "--------------------------------\n";
    int mid_index = -1;
    int min_dis = 200;
    int temp_index[14] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    // 寻找正中间人的关键点
    int key_0_size = all_key_points[0].size();
    for (int aa = 0; aa < connection_all[0].size(); aa++){
        int this_point_x = all_key_points[1][connection_all[0][aa].point_1 % key_0_size].point_x;
//        cout << "aaaaaa  " << aa << " point11111   " << connection_all[0][aa].point_1 << "  dis  " << this_point_x - 64 << endl;
        if (abs(this_point_x - 64) < min_dis){
            min_dis = abs(this_point_x - 64);
            mid_index = aa;
        }
    }
    temp_index[1] = connection_all[0][mid_index].point_1;
    temp_index[2] = connection_all[0][mid_index].point_2;
//    int start_index = ;
    for (int aa = 1; aa < 13; aa ++){
        int index_A = limbSeq[aa][0] - 1;
        int index_B = limbSeq[aa][1] - 1;
        if(temp_index[index_A] == -1){
            return HIAI_OK;
        }
        // connection_all 里面找index_A对应的序号的连接关系
        for (int bb = 0; bb < connection_all[aa].size(); bb++){
            if(connection_all[aa][bb].point_1 == temp_index[index_A]){
                // 找到index B对应的关键点序号，存进来
                temp_index[index_B] = connection_all[aa][bb].point_2;
            }
        }
        // 有一个连接点没找到，就认为这张照片无效，直接返回
        if(temp_index[index_B] == -1){
//            cout << "temp_index[index_B] == -1" << endl;
            return HIAI_OK;
        }
//        connection_all[aa]
    }
//    cout << "alllllllllll indexxxxxxxxxxxxx\n";
//    for (int aa = 0; aa < 14; aa++){
//        cout << temp_index[aa] << "   ";
//    }
//    cout << endl;

    // 保存颈部关键点
//    temp_key_points[0][1] = all_key_points[1][connection_all[00][mid_index].point_1 % key_0_size].point_x;
//    temp_key_points[1][1] = all_key_points[1][connection_all[00][mid_index].point_1 % key_0_size].point_y;
//    temp_key_points[0][2] = all_key_points[2][]
    for (int aa = 0; aa < 14; aa++){
        temp_key_points[0][aa] = all_key_points[aa][temp_index[aa]].point_x;
        temp_key_points[1][aa] = all_key_points[aa][temp_index[aa]].point_y;
        cv::Point p(temp_key_points[0][aa], temp_key_points[1][aa]);//初始化点坐标为(20,20)
        cv::circle(out1, p, 1, cv::Scalar(0, 0, 0), -1);  // 画半径为1的圆(画点）
        for(int bb = aa + 1; bb < 14; bb++){
            temp_index[bb] -= all_key_points[aa].size();
        }
//        cout << "temp_index[aa]  " << temp_index[aa] << endl;
    }
    cv::Point x2(temp_key_points[0][2], temp_key_points[0][2]);
    cv::Point x3(temp_key_points[0][3], temp_key_points[0][3]);
    cv::Point x4(temp_key_points[0][4], temp_key_points[0][4]);
    cv::Point y5(temp_key_points[0][5], temp_key_points[0][5]);
    cv::Point y6(temp_key_points[0][6], temp_key_points[0][6]);
    cv::Point y7(temp_key_points[0][7], temp_key_points[0][7]);
    cv::line(out1, x2, x3, cv::Scalar(255, 0, 0), 1);
    cv::line(out1, x3, x4, cv::Scalar(255, 0, 0), 1);
    cv::line(out1, y5, y6, cv::Scalar(255, 0, 0), 1);
    cv::line(out1, y6, y7, cv::Scalar(255, 0, 0), 1);
    string out_path = storePath + "/out1.jpg";
    cv::imwrite(out_path, out1);

//    cout << "alllllllllll indexxxxxxxxxxxxx\n";
//    for (int aa = 0; aa < 14; aa++){
//        cout << temp_key_points[0][aa] << "   " << temp_key_points[1][aa] << "  ";
//    }
//    cout << endl;
//    for (int aa = 0; aa < connection_all.size(); aa++){
//    cout << "connection all size  " << connection_all.size() << endl;
//        for (int bb = 0; bb < connection_all[0].size(); bb++){
//            cout << "connection all part size  " << connection_all[aa].size() << endl;
//            cout << "aa  " << aa << "  bb  " << bb << "  connection_1  " << connection_all[aa][bb].point_1 << "  connection_2  " << connection_all[aa][bb].point_2 << endl;
//        }
//    }





//   std::cout << "temp333333 calculate time::" << double(clock() - start_time) / CLOCKS_PER_SEC << std::endl;
    memcpy(motion_data_new->data[0][FRAME_LENGTH-1], temp_key_points[0], sizeof(float)*14);
    memcpy(motion_data_new->data[1][FRAME_LENGTH-1], temp_key_points[1], sizeof(float)*14);
    // x
    memcpy(motion_data_new->data[0][0], motion_data_old->data[0][1], sizeof(float)*14*(FRAME_LENGTH - 1));
    // y
    memcpy(motion_data_new->data[1][0], motion_data_old->data[1][1], sizeof(float)*14*(FRAME_LENGTH - 1));
    memcpy(motion_data_old->data, motion_data_new->data, sizeof(float)*2*FRAME_LENGTH*14);


////    clock_t temp_start_time = clock();
    //求中心点坐标,并中心化
    float skeleton_center[2][FRAME_LENGTH]={0.0};
    for ( int c = 0; c < 2; c++ )
    {
        for ( int t = 0; t < FRAME_LENGTH; t++ )
//        for ( int t = 0; t < 30; t++ )
        {
            skeleton_center[c][t] = float((motion_data_new->data[c][t][1]+motion_data_new->data[c][t][8]+motion_data_new->data[c][t][11])/float(3.0));
            for ( int v = 0; v < 14; v++ )
            {
                motion_data_new->data[c][t][v] = motion_data_new->data[c][t][v]-skeleton_center[c][t];
            }
        }
    }
    HIAI_StatusT status_ret = HIAI_OK;


    // 发送结果
//    if(IMG_NUM%25 == 0){
    if(IMG_NUM%5 == 0){
        ProcessData();
//        SaveData();
        status_ret = SendBatch(motion_data_new);
    }
    IMG_NUM++;

    IMG_NUM = IMG_NUM % 100;


    if (HIAI_OK != status_ret) {
    cout << "ERRORERRORERRORERRORERROR" << endl;
        return status_ret;
    }
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[SaveFilePostProcess] End process!");
//   std::cout << "[SaveFileProcess] calculate time4444444444444::" << double(clock() - start_time) / CLOCKS_PER_SEC << std::endl;

    return HIAI_OK;
}
