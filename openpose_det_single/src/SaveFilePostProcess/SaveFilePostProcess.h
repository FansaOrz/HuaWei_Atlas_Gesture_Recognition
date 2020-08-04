/**
* @file SaveFilePostProcess.h
*
* Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#ifndef SAVE_FILE_POST_PROCESS_ENGINE_H_
#define SAVE_FILE_POST_PROCESS_ENGINE_H_

#include <map>
#include "hiaiengine/api.h"
#include "hiaiengine/data_type.h"
#include "hiaiengine/engine.h"
#include "hiaiengine/data_type_reg.h"
#include "Common.h"

#define INPUT_SIZE 1
#define OUTPUT_SIZE 1
#define DEFAULT_DATA_PORT 0

using hiai::Engine;
using namespace hiai;

class SaveFilePostProcess : public Engine {
private:

public:
    SaveFilePostProcess() :storePath(""){}
    ~SaveFilePostProcess(){}
    HIAI_StatusT Init(const hiai::AIConfig& config, const  std::vector<hiai::AIModelDescription>& model_desc);
    HIAI_StatusT SendBatch(std::shared_ptr<EngineTransNewT> imageInfoBatch);

    /**
    * @ingroup hiaiengine
    * @brief HIAI_DEFINE_PROCESS : Overloading Engine Process processing logic
    * @[in]: Define an input port, an output port*/
    HIAI_DEFINE_PROCESS(INPUT_SIZE, OUTPUT_SIZE);

private:
    const std::string RESULT_FOLDER = "result_files";
    const std::string ENGINE_NAME = "SaveFilePostProcess";
    std::string storePath;
    std::unordered_map<int, std::string> frameIdToName;
};

#endif // SAVE_FILE_POST_PROCESS_ENGINE_H_
