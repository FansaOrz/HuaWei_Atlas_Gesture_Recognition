/**
* @file MindInferenceEngine.cpp
*
* Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include <hiaiengine/log.h>
#include <hiaiengine/ai_types.h>
#include <vector>
#include <unistd.h>
#include <thread>
#include <fstream>
#include <ctime>
#include <algorithm>
#include <iostream>
#include <cmath>
#include "MindInferenceEngine.h"
#include "hiaiengine/ai_memory.h"
#include "hiaiengine/status.h"

const static int IMAGE_INFO_DATA_NUM = 3;
const static uint32_t INPUT_INDEX_0 = 0;

/**
* @brief: clear buffer in vector
*/
void MindInferenceEngine::ClearOutData() {
    inputDataVec_.clear();
    // release outData_ pre allocate memory
    for (auto buffer : outData_) {
        if (buffer != nullptr) {
            hiai::HIAIMemory::HIAI_DFree(buffer);
            buffer = nullptr;
        }
    }
    outData_.clear();
}

/**
* @brief: init, inherited from hiaiengine lib
*/
HIAI_StatusT MindInferenceEngine::Init(const hiai::AIConfig& config,
   const std::vector<hiai::AIModelDescription>& model_desc) {
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine] Start init!");
    // 初始化一个模型管家
    aiModelManager_ = std::make_shared<hiai::AIModelManager>();
    if (aiModelManager_ == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine] Failed to call make_shared for AIModelManager.");
        return HIAI_ERROR;
    }

    std::vector<hiai::AIModelDescription> modelDescVec;
    hiai::AIModelDescription modelDesc;
    for (int index = 0; index < config.items_size(); ++index) {
        const ::hiai::AIConfigItem& item = config.items(index);
        // 加载模型文件
        if (item.name() == "model_path") {
            std::string modelPath = item.value();
            if (modelPath.empty()) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine] The model_path does not exist in graph.config.");
                return HIAI_ERROR;
            }
            // 模型描述文件设置路径
            modelDesc.set_path(modelPath);
            // 加载模型名字
            std::size_t modelNameStartPos = modelPath.find_last_of("/\\");
            std::size_t modelNameEndPos = modelPath.find_last_of(".");
            if (std::string::npos != modelNameStartPos && std::string::npos != modelNameEndPos
                && modelNameEndPos > modelNameStartPos) {
                modelName_ = modelPath.substr(modelNameStartPos + 1, modelNameEndPos - modelNameStartPos - 1);
            }
        } else if (item.name() == "passcode") {
            std::string passcode = item.value();
            modelDesc.set_key(passcode);
        }
    }
    // 模型描述文件设置名字，（声明了vector向量，可以加载多个模型)
    modelDesc.set_name(modelName_);
    modelDescVec.push_back(modelDesc);
    // 模型初始化（传入config和刚才的模型描述文件）
    hiai::AIStatus ret = aiModelManager_->Init(config, modelDescVec);
    if (hiai::SUCCESS != ret) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine] Failed to call Init for AIModelManager.");
        return HIAI_ERROR;
    }
    // TODO 检查是否可以修改outputtensorvec
    ret = aiModelManager_->GetModelIOTensorDim(modelName_, inputTensorVec_, outputTensorVec_);
    if (hiai::SUCCESS != ret) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine] Failed to call GetModelIOTensorDim for AIModelManager.");
        return HIAI_ERROR;
    }

    batchSize_ = inputTensorVec_[INPUT_INDEX_0].n;
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine] End init!");
//    std::cout << "inference engine inited end!!!!" << std::endl;
    return HIAI_OK;
}

/**
* @brief: handle the exceptions when the dataset batch failed
* @in: error_msg: the error message
* @return: HIAI_StatusT
*/
HIAI_StatusT MindInferenceEngine::HandleExceptions(std::string errorMsg) {
    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, errorMsg.c_str());
    tranData_->status = false;
    tranData_->msg = errorMsg;
    return SendResult();
};

/**
* @brief: call ai model manager to do the prediction
* @return: HIAI_StatusT
*/
HIAI_StatusT MindInferenceEngine::Predict() {
    //pre malloc OutData
    HIAI_StatusT hiaiRet = HIAI_OK;
    for (uint32_t index = 0; index < outputTensorVec_.size(); index++) {
        hiai::AITensorDescription outputTensorDesc = hiai::AINeuralNetworkBuffer::GetDescription();
        uint8_t* buf = nullptr;
        hiaiRet = hiai::HIAIMemory::HIAI_DMalloc(outputTensorVec_[index].size, (void *&)buf, 1000);
        if (hiaiRet != HIAI_OK || buf == nullptr) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine] Failed to call HIAI_DMalloc.");
            ClearOutData();
            return HIAI_ERROR;
        }
        outData_.push_back(buf);
        std::shared_ptr<hiai::IAITensor> outputTensor = hiai::AITensorFactory::GetInstance()->CreateTensor(outputTensorDesc, buf, outputTensorVec_[index].size);
        shared_ptr<hiai::AINeuralNetworkBuffer> nnTensor = static_pointer_cast<hiai::AINeuralNetworkBuffer>(outputTensor);
        nnTensor->SetName(outputTensorVec_[index].name);
        outputDataVec_.push_back(outputTensor);
    }

    // put buffer to FrameWork directly, InputSize has only one
    hiai::AITensorDescription inputTensorDesc = hiai::AINeuralNetworkBuffer::GetDescription();
    for (unsigned int i = 0; i < predictInputData_.size(); i++) {
        std::map<uint8_t *, int> tmp = predictInputData_[i];
        for (std::map<uint8_t *, int>::iterator it = tmp.begin();it != tmp.end(); ++it) {
            shared_ptr<hiai::IAITensor> inputTensor =
                hiai::AITensorFactory::GetInstance()->CreateTensor(inputTensorDesc, (void *)(it->first), it->second);
            inputDataVec_.push_back(inputTensor); // AIModelManager push input data
        }
    }

    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine] Start to call Process for AIModelManager.");

    hiai::AIContext aiContext;
    hiaiRet = aiModelManager_->Process(aiContext, inputDataVec_, outputDataVec_, 0);
    if (hiai::SUCCESS != hiaiRet) {
        ClearOutData();
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine] Failed to call Process for AIModelManager, error code: %u", hiaiRet);
        return HIAI_ERROR;
    }
    return HIAI_OK;
}

/**
* @brief: prepare the data buffer for image data
* @in: buffer: buffer pointer
* @in: imageCount: total number of received images
* @in: start: the index of the first image of each batch
* @in: imageSize: size of each image
* @return: HIAI_StatusT
*/
HIAI_StatusT MindInferenceEngine::PrepareForInputBuffer0(uint8_t* buffer, const int imageCount, const int start, const int imageSize) {
    //1.prepare input buffer for each batch
    //the loop for each image
    for (int j = 0; j < batchSize_; j++) {
        if (start + j < imageCount) {
            if (memcpy_s(buffer + j * imageSize, imageSize, imageHandle_->v_img[start + j].img.data.get(), imageSize)) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine] Failed to call memcpy_s for inputBuffer0 to copy image buffer.");
                return HIAI_ERROR;
            }
        } else {
            if (memset_s(buffer + j * imageSize, imageSize, static_cast<char>(0), imageSize)) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine] Failed to call memcpy_s for inputBuffer0 to fill zero.");
                return HIAI_ERROR;
            }
        }
    }
    return HIAI_OK;
}

/**
* @brief: prepare the data buffer for image information
* @in: buffer: buffer pointer
* @in: imageCount: total number of received images
* @in: start: the index of the first image of each batch
* @in: multiInput1: the second input received from the previous engine
* @return: HIAI_StatusT
*/
HIAI_StatusT MindInferenceEngine::PrepareForInputBuffer1(uint8_t* buffer, const int imageCount, const int start, std::shared_ptr<hiai::BatchRawDataBuffer> multiInput1) {
    //the loop for each info
    for (int j = 0; j < batchSize_; j++) {
        if (start + j < imageCount) {
            hiai::RawDataBuffer rawDataBuffer = multiInput1->v_info[start + j];
            int size = rawDataBuffer.len_of_byte;
            if (memcpy_s(buffer + j * size, size, rawDataBuffer.data.get(), size)) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine] Failed to call memcpy_s for inputBuffer1 to copy image info.");
                return HIAI_ERROR;
            }
        } else {
            float info[3] = { 0.0, 0.0, 0.0 };
            int size = sizeof(info);
            if (memcpy_s(buffer + j * size, size, info, size)) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine] Failed to call memcpy_s for inputBuffer1 to fill zero.");
                return HIAI_ERROR;
            }
        }
    }
    return HIAI_OK;
}

/**
* @brief: set the frame ID as -1 to indicate this batch failed
* @in: start: index of the begin of this batch
* @in: imageCount: the image count
*/
void MindInferenceEngine::HandleBatchFailure(const int start, const int imageCount) {
    for (int i = 0; i < batchSize_; i++) {
        if(start + i < imageCount){
            tranData_->b_info.frame_ID[i + start] = -1;
        }
    }
}

/**
* @brief: set the tranData_ with the result of this batch
* @in: start: index of the begin of this batch
* @return: HIAI_StatusT
*/
HIAI_StatusT MindInferenceEngine::SetOutputStruct(const int start) {
    for (unsigned int i = 0; i < outputDataVec_.size(); ++i) {
        std::shared_ptr<hiai::AINeuralNetworkBuffer> resultTensor = std::static_pointer_cast<hiai::AINeuralNetworkBuffer>(outputDataVec_[i]);
        auto tensorSize = resultTensor->GetSize();
        if (memcpy_s(tranData_->output_data_vec[i].data.get() + start / batchSize_ * tensorSize, tensorSize, resultTensor->GetBuffer(), tensorSize)) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine] Failed to call memcpy_s for tranData_->output_data_vec[%d].data.", i);
            return HIAI_ERROR;
        }
    }
    // jiashi Changed
//    SendResult();
    return HIAI_OK;
}

/**
* @brief: send the predicted result for one batch
* @return: HIAI_StatusT
*/
HIAI_StatusT MindInferenceEngine::SendResult() {
    HIAI_StatusT hiaiRet = HIAI_OK;
    do {
        hiaiRet = SendData(0, "EngineTransT", std::static_pointer_cast<void>(tranData_));
        if (HIAI_QUEUE_FULL == hiaiRet) {
            HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine] The queue is full, sleep 200ms");
            std::cout << "{{{{{{{{{{{{{{{{{{{{{{{{{{{{{" << std::endl;
            usleep(SEND_DATA_INTERVAL_MS);
        }
    } while (hiaiRet == HIAI_QUEUE_FULL);

    if (HIAI_OK != hiaiRet) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine] Failed to send data , error code: %d", hiaiRet);
    }
    return hiaiRet;
}

/**
* @ingroup hiaiengine
* @brief HIAI_DEFINE_PROCESS : Realize the port input/output processing
* @[in]: Define an input port, an output port,
*        And the Engine is registered, its called "HIAIMultiEngineExample"
*/
HIAI_IMPL_ENGINE_PROCESS("MindInferenceEngine", MindInferenceEngine, INPUT_SIZE) {
//    std::cout << "===================================get!!===================================================" << std::endl;
    clock_t start_time = clock();
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine] Start process!");
    HIAI_StatusT hiaiRet = HIAI_OK;
    std::lock_guard<std::mutex> lk(memoryRecursiveMutex_);
    if (tranData_ == nullptr) {
        tranData_ = std::make_shared<EngineTransT>();
    }
    // 1.PreProcess:Framework input data
    // 1. 预处理输入数据
    imageHandle_ = nullptr;
    // arg0代表第0个端口输入的数据
    inputQueue_.PushData(0, arg0);
#if (INPUT_SIZE == 1)
    if (!inputQueue_.PopAllData(imageHandle_)) {
        return HIAI_OK;
        return HandleExceptions("[MindInferenceEngine] Failed to call PopAllData.");
    }
#endif

#if (INPUT_SIZE == 2)
    DEFINE_MULTI_INPUT_ARGS_POP(2);
#endif
    if (imageHandle_ == nullptr) {
        return HandleExceptions("[MindInferenceEngine] The imageHandle_ is null.");
    }
    //add sentinel image for showing this data in dataset are all sent, this is last step.

    int imageCount = imageHandle_->v_img.size();
//    std::cout << "imageCount::" << imageCount << std::endl;

#if (INPUT_SIZE == 2)
    if (multiInput1 == nullptr) {
        return HandleExceptions("[MindInferenceEngine] The multiInput1 is null.");
    }

    if (multiInput1->v_info.size() != imageCount) {
        return HandleExceptions("[MindInferenceEngine] The number of image data and information data doesn't match.");
    }
    int inputBuffer1Size = sizeof(float) * IMAGE_INFO_DATA_NUM * batchSize_;
    uint8_t * inputBuffer1 = nullptr;
    hiaiRet = hiai::HIAIMemory::HIAI_DMalloc(inputBuffer1Size, (void *&)inputBuffer1, 1000);
    if (hiaiRet != HIAI_OK || inputBuffer1 == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine] Failed to call HIAI_DMalloc for inputBuffer1.");
        return HIAI_ERROR;
    }
#endif
    int imageSize = imageHandle_->v_img[0].img.size * sizeof(uint8_t);
//    std::cout << "imageSize::" << imageSize << std::endl;

    int inputBuffer0Size  = imageSize * batchSize_;
//    std::cout << "inputBuffer0Size::" << inputBuffer0Size << std::endl;

if (inputBuffer0Size <= 0) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine] inputBuffer0Size <= 0");
        return HIAI_ERROR;
    }
    // 声明一个输入数据的buffer并分配内存
    uint8_t *inputBuffer0 = nullptr;
    hiaiRet = hiai::HIAIMemory::HIAI_DMalloc(inputBuffer0Size, (void *&)inputBuffer0, 1000);
    if (hiaiRet != HIAI_OK || inputBuffer0 == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine] Failed to call HIAI_DMalloc for inputBuffer0.");
#if (INPUT_SIZE == 2)
        hiai::HIAIMemory::HIAI_DFree(inputBuffer1);
        inputBuffer1 = nullptr;
#endif
        return HIAI_ERROR;
    }

    int batchCount = imageHandle_->b_info.batch_size / batchSize_;
    if (imageHandle_->b_info.batch_size % batchSize_ != 0) {
        batchCount++;
    }
    tranData_->b_info = imageHandle_->b_info;
    tranData_->v_img = imageHandle_->v_img;
    tranData_->status = true;
    tranData_->b_info.max_batch_size = batchCount * batchSize_;

    for (int i = 0; i < imageCount; i+= batchSize_) {
        predictInputData_.clear();
        if (HIAI_OK != PrepareForInputBuffer0(inputBuffer0, imageCount, i, imageSize)) {
            HandleBatchFailure(i, imageCount);
            continue;
        }
        std::map<uint8_t *, int> input0;
        input0.insert(std::make_pair(inputBuffer0, inputBuffer0Size));
        predictInputData_.push_back(input0);
#if (INPUT_SIZE == 1)
        DEFINE_MULTI_INPUT_ARGS(1);
#endif
#if (INPUT_SIZE == 2)
        if (HIAI_OK != PrepareForInputBuffer1(inputBuffer1, imageCount, i, multiInput1)) {
            HandleBatchFailure(i, imageCount);
            continue;
        }
        std::map<uint8_t *, int> input1;
        input1.insert(std::make_pair(inputBuffer1, inputBuffer1Size));
        predictInputData_.push_back(input1);
        DEFINE_MULTI_INPUT_ARGS(2);
#endif

        // 2.Call Process, Predict
        // 调用引擎开始识别
        inputDataVec_.clear();
        if (HIAI_OK != Predict()) {
            outputDataVec_.clear();
            HandleBatchFailure(i, imageCount);
            continue;
        }
        //init the output buffer for one dataset batch(might be multiple model batches)
        if (tranData_->output_data_vec.empty()) {
            tranData_->size = outputDataVec_.size();
            HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine] Alloc memory for dataset batch, number of outputs of the network: %d", outputDataVec_.size());
            for (unsigned int i = 0; i < outputDataVec_.size(); i++) {
                OutputT out;
                std::shared_ptr<hiai::AINeuralNetworkBuffer> resultTensor = std::static_pointer_cast<hiai::AINeuralNetworkBuffer>(outputDataVec_[i]);
                int bufferSize = resultTensor->GetSize();
                out.name = resultTensor->GetName();
                out.size = bufferSize * batchCount;
                if(out.size <= 0){
                    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine] out.size <= 0");
                    hiai::HIAIMemory::HIAI_DFree(inputBuffer0);
                    inputBuffer0 = nullptr;
#if (INPUT_SIZE == 2)
                    hiai::HIAIMemory::HIAI_DFree(inputBuffer1);
                    inputBuffer1 = nullptr;
#endif
                    ClearOutData();
                    return HIAI_ERROR;
                }
                u_int8_t *ptr = nullptr;
                try {
                    ptr = new u_int8_t[out.size];
                } catch (const std::bad_alloc& e) {
                    hiai::HIAIMemory::HIAI_DFree(inputBuffer0);
                    inputBuffer0 = nullptr;
#if (INPUT_SIZE == 2)
                    hiai::HIAIMemory::HIAI_DFree(inputBuffer1);
                    inputBuffer1 = nullptr;
#endif
                    ClearOutData();
                    return HIAI_ERROR;
                }
                out.data.reset(ptr);
                tranData_->output_data_vec.push_back(out);
                HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine] The image count is %d.", imageCount);
            }
        }

        //3.set the tranData_ with the result of this batch
        if (HIAI_OK != SetOutputStruct(i)) {
            ClearOutData();
            outputDataVec_.clear();
            HandleBatchFailure(i, imageCount);
            continue;
        }
        outputDataVec_.clear();
    }
    hiaiRet = SendResult();
    //6. release sources
    hiai::HIAIMemory::HIAI_DFree(inputBuffer0);
    inputBuffer0 = nullptr;
#if (INPUT_SIZE == 2)
    hiai::HIAIMemory::HIAI_DFree(inputBuffer1);
    inputBuffer1 = nullptr;
#endif
    ClearOutData();
    tranData_ = nullptr;
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine] End process!");
//    std::cout << "engine3 OpenPoseEngine time::" << double(clock() - start_time) / CLOCKS_PER_SEC << std::endl;

    return hiaiRet;
}