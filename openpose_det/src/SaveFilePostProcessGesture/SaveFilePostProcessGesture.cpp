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
#include <vector>
#include <unistd.h>
#include <thread>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <stdlib.h>
#include <sys/stat.h>
#include <sstream>
#include <fcntl.h>
#include "SaveFilePostProcessGesture.h"

// Eigen
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

HIAI_StatusT SaveFilePostProcessGesture::Init(const hiai::AIConfig& config, const  std::vector<hiai::AIModelDescription>& model_desc) {
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[SaveFilePostProcessGesture] Start init!");

//    frameIdToName[0] = "2008_000014";
//    frameIdToName[1] = "2008_000159";
//    frameIdToName[2] = "2010_004236";

    uint32_t graphId = Engine::GetGraphId();
    std::shared_ptr<Graph> graph = Graph::GetInstance(graphId);
    if (graph == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SaveFilePostProcessGesture] Fail to get the graph id.");
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
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[SaveFilePostProcessGesture] End init!");
    return HIAI_OK;
}

HIAI_IMPL_ENGINE_PROCESS("SaveFilePostProcessGesture", SaveFilePostProcessGesture, INPUT_SIZE)
{
    clock_t start_time = clock();

    std::shared_ptr<std::string> result(new std::string);
//    result->push_back("a");
    HIAI_StatusT hiaiRet = HIAI_OK;

    hiaiRet = SendData(0, "string", std::static_pointer_cast<void>(result));


    std::cout << "engine1 dataInput time::" << double(clock() - start_time) / CLOCKS_PER_SEC << std::endl;

    return HIAI_OK;
}
