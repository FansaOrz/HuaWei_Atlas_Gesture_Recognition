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
#include <algorithm>
#include <iostream>
#include <cmath>
#include <cstdlib>                // For atoi()
#include "MindInferenceEngineGesture.h"
#include "hiaiengine/ai_memory.h"
#include "hiaiengine/status.h"
// Socket通信
#include "PracticalSocket.h"

#define MAX_TIME 1
#define GESTURE_NUM 5

const static int IMAGE_INFO_DATA_NUM = 3;
const static uint32_t INPUT_INDEX_0 = 0;

bool if_sent = false;
int IMAGE_NUM = 0;
int LAST_GES = -1;
int GES_TIMES = 0;
int OUTPUT_GES = -1;
/**
* @brief: clear buffer in vector
*/
void MindInferenceEngineGesture::ClearOutData() {
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
HIAI_StatusT MindInferenceEngineGesture::Init(const hiai::AIConfig& config,
   const std::vector<hiai::AIModelDescription>& model_desc) {
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngineGesture] Start init!");

    aiModelManager_ = std::make_shared<hiai::AIModelManager>();
    if (aiModelManager_ == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngineGesture] Failed to call make_shared for AIModelManager.");
        return HIAI_ERROR;
    }

    std::vector<hiai::AIModelDescription> modelDescVec;
    hiai::AIModelDescription modelDesc;
    for (int index = 0; index < config.items_size(); ++index) {
        const ::hiai::AIConfigItem& item = config.items(index);
        if (item.name() == "model_path") {
            std::string modelPath = item.value();
            if (modelPath.empty()) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngineGesture] The model_path does not exist in graph.config.");
                return HIAI_ERROR;
            }
            modelDesc.set_path(modelPath);
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
    modelDesc.set_name(modelName_);
    modelDescVec.push_back(modelDesc);
    // init model manager
    hiai::AIStatus ret = aiModelManager_->Init(config, modelDescVec);
    if (hiai::SUCCESS != ret) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngineGesture] Failed to call Init for AIModelManager.");
        return HIAI_ERROR;
    }

    ret = aiModelManager_->GetModelIOTensorDim(modelName_, inputTensorVec_, outputTensorVec_);
    if (hiai::SUCCESS != ret) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngineGesture] Failed to call GetModelIOTensorDim for AIModelManager.");
        return HIAI_ERROR;
    }

    batchSize_ = inputTensorVec_[INPUT_INDEX_0].n;
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngineGesture] End init!");
    return HIAI_OK;
}

/**
* @brief: handle the exceptions when the dataset batch failed
* @in: error_msg: the error message
* @return: HIAI_StatusT
*/
HIAI_StatusT MindInferenceEngineGesture::HandleExceptions(std::string errorMsg) {
    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, errorMsg.c_str());
    tranData_->status = false;
    tranData_->msg = errorMsg;
    return SendResult();
};

/**
* @brief: call ai model manager to do the prediction
* @return: HIAI_StatusT
*/
HIAI_StatusT MindInferenceEngineGesture::Predict() {
    //pre malloc OutData
    HIAI_StatusT hiaiRet = HIAI_OK;
    for (uint32_t index = 0; index < outputTensorVec_.size(); index++) {
        hiai::AITensorDescription outputTensorDesc = hiai::AINeuralNetworkBuffer::GetDescription();
        uint8_t* buf = nullptr;
        hiaiRet = hiai::HIAIMemory::HIAI_DMalloc(outputTensorVec_[index].size, (void *&)buf, 1000);
        if (hiaiRet != HIAI_OK || buf == nullptr) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngineGesture] Failed to call HIAI_DMalloc.");
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
//    cout << "predictInputData_:::::::" << predictInputData_.size() << endl;
//    cout << "predictInputData_:::::::" << predictInputData_ << endl;
    for (unsigned int i = 0; i < predictInputData_.size(); i++) {
        std::map<uint8_t *, int> tmp = predictInputData_[i];
        for (std::map<uint8_t *, int>::iterator it = tmp.begin();it != tmp.end(); ++it) {
            shared_ptr<hiai::IAITensor> inputTensor =
                hiai::AITensorFactory::GetInstance()->CreateTensor(inputTensorDesc, (void *)(it->first), it->second);
            inputDataVec_.push_back(inputTensor); // AIModelManager push input data
        }
    }

    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngineGesture] Start to call Process for AIModelManager.");

    hiai::AIContext aiContext;
    hiaiRet = aiModelManager_->Process(aiContext, inputDataVec_, outputDataVec_, 0);
    if (hiai::SUCCESS != hiaiRet) {
        ClearOutData();
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngineGesture] Failed to call Process for AIModelManager, error code: %u", hiaiRet);
        return HIAI_ERROR;
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
HIAI_StatusT MindInferenceEngineGesture::PrepareForInputBuffer1(uint8_t* buffer, const int imageCount, const int start, std::shared_ptr<hiai::BatchRawDataBuffer> multiInput1) {
    //the loop for each info
    for (int j = 0; j < batchSize_; j++) {
        if (start + j < imageCount) {
            hiai::RawDataBuffer rawDataBuffer = multiInput1->v_info[start + j];
            int size = rawDataBuffer.len_of_byte;
            if (memcpy_s(buffer + j * size, size, rawDataBuffer.data.get(), size)) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngineGesture] Failed to call memcpy_s for inputBuffer1 to copy image info.");
                return HIAI_ERROR;
            }
        } else {
            float info[3] = { 0.0, 0.0, 0.0 };
            int size = sizeof(info);
            if (memcpy_s(buffer + j * size, size, info, size)) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngineGesture] Failed to call memcpy_s for inputBuffer1 to fill zero.");
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
void MindInferenceEngineGesture::HandleBatchFailure(const int start, const int imageCount) {
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
HIAI_StatusT MindInferenceEngineGesture::SetOutputStruct(const int start) {
    for (unsigned int i = 0; i < outputDataVec_.size(); ++i) {
        std::shared_ptr<hiai::AINeuralNetworkBuffer> resultTensor = std::static_pointer_cast<hiai::AINeuralNetworkBuffer>(outputDataVec_[i]);
        auto tensorSize = resultTensor->GetSize();
        if (memcpy_s(tranData_->output_data_vec[i].data.get() + start / batchSize_ * tensorSize, tensorSize, resultTensor->GetBuffer(), tensorSize)) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngineGesture] Failed to call memcpy_s for tranData_->output_data_vec[%d].data.", i);
            return HIAI_ERROR;
        }
    }
    return HIAI_OK;
}

/**
* @brief: send the predicted result for one batch
* @return: HIAI_StatusT
*/
HIAI_StatusT MindInferenceEngineGesture::SendResult() {
    HIAI_StatusT hiaiRet = HIAI_OK;
    do {
        hiaiRet = SendData(0, "EngineTransT", std::static_pointer_cast<void>(tranData_));
        if (HIAI_QUEUE_FULL == hiaiRet) {
            HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngineGesture] The queue is full, sleep 200ms");
            cout << "qqqqqqqqqqqqqqqq" << endl;
            usleep(SEND_DATA_INTERVAL_MS);
        }
    } while (hiaiRet == HIAI_QUEUE_FULL);

    if (HIAI_OK != hiaiRet) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngineGesture] Failed to send data , error code: %d", hiaiRet);
    }
    return hiaiRet;
}

HIAI_StatusT MindInferenceEngineGesture::Sendcommand(int command){
  HIAI_StatusT hiaiRet = HIAI_OK;
  const int ECHOMAX = 255;
  string servAddress = "192.168.1.120";             // First arg: server address
  int echoString[3] = {command, 0, 0};                    // Second arg: string to echo
  int echoStringLen = 3;                            // Length of string to echo
  if (echoStringLen > ECHOMAX) {                    // Check input length
    cerr << "Echo string too long" << endl;
    exit(1);
  }
  unsigned short echoServPort = 43893;
  try {
    UDPSocket sock;
    // Send the string to the server
    sock.sendTo(echoString, echoStringLen, servAddress, echoServPort);
    // Receive a response
    char echoBuffer[ECHOMAX + 1];                   // Buffer for echoed string + \0
    int respStringLen;                              // Length of received response

  } catch (SocketException &e) {
    cerr << e.what() << endl;
    exit(1);
  }

  return hiaiRet;
}

/**
* @ingroup hiaiengine
* @brief HIAI_DEFINE_PROCESS : Realize the port input/output processing
* @[in]: Define an input port, an output port,
*        And the Engine is registered, its called "HIAIMultiEngineExample"
*/
HIAI_IMPL_ENGINE_PROCESS("MindInferenceEngineGesture", MindInferenceEngineGesture, INPUT_SIZE) {
//    std::cout << "GET!!!!!!!!!!!/!!\n";
//    clock_t start_time = clock();
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngineGesture] Start process!");
    HIAI_StatusT hiaiRet = HIAI_OK;
    std::lock_guard<std::mutex> lk(memoryRecursiveMutex_);
    if (tranData_ == nullptr) {
        tranData_ = std::make_shared<EngineTransT>();
    }
    // 1.PreProcess:Framework input data

    imageHandle_ = nullptr;
    inputQueue_.PushData(0, arg0);
#if (INPUT_SIZE == 1)
    if (!inputQueue_.PopAllData(imageHandle_)) {
        return HandleExceptions("[MindInferenceEngineGesture] Failed to call PopAllData.");
    }
#endif

    if (imageHandle_ == nullptr) {
        return HandleExceptions("[MindInferenceEngineGesture] The imageHandle_ is null.");
    }


    int inputBuffer0Size = 2*FRAME_LENGTH*14*sizeof(float);
//    int inputBuffer0Size = 2*30*14*sizeof(float);

    double *inputBuffer0 = nullptr;
    // 为inputBuffer0分配内存
    hiaiRet = hiai::HIAIMemory::HIAI_DMalloc(inputBuffer0Size, (void *&)inputBuffer0, 1000);
    if (hiaiRet != HIAI_OK || inputBuffer0 == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngineGesture] Failed to call HIAI_DMalloc for inputBuffer0.");
        return HIAI_ERROR;
    }
    // 1. 数据放在neural_buf里面
     shared_ptr<hiai::AINeuralNetworkBuffer> neural_buf = shared_ptr <hiai::AINeuralNetworkBuffer>
        (new hiai::AINeuralNetworkBuffer(), default_delete<hiai::AINeuralNetworkBuffer>());
     neural_buf->SetBuffer((void*) imageHandle_->data, inputBuffer0Size);

    // 2. input data
    shared_ptr<hiai::IAITensor> input_data = static_pointer_cast<hiai::IAITensor>(neural_buf);
    vector<shared_ptr<hiai::IAITensor>> input_data_vec;
    input_data.get();
    input_data_vec.push_back(input_data);

    // 3. create output tensor
    hiai::AIContext ai_context;
    vector<shared_ptr<hiai::IAITensor>> output_data_vec;
    hiai::AIStatus ret = aiModelManager_->CreateOutputTensor(input_data_vec, output_data_vec);
    shared_ptr<hiai::AISimpleTensor> result_tensor111 = static_pointer_cast <hiai::AISimpleTensor> (output_data_vec[0]);

    // create failed, need to deal next batch
    if (ret != hiai::SUCCESS) {
      HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT, "failed to create output tensor, ret=%u", ret);
      return HIAI_ERROR;
    }

    // 4. Process inference
    HIAI_ENGINE_LOG("begin to call AI Model Manager Process method.");
    ret = aiModelManager_->Process(ai_context, input_data_vec, output_data_vec, 0);
    // process failed, need to deal next batch
    if (ret != hiai::SUCCESS) {
      HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT, "failed to process AI Model, ret=%u", ret);
      return HIAI_ERROR;
    }
    HIAI_ENGINE_LOG("end to call AI Model Manager Process method.");

      shared_ptr<hiai::AISimpleTensor> result_tensor = static_pointer_cast <hiai::AISimpleTensor> (output_data_vec[0]);

      // copy data to float array
      int32_t size = result_tensor->GetSize() / sizeof(float);
      if(size <= 0){
        HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                        "the result tensor's size is not correct, size is %d", size);
        return HIAI_ERROR;
      }
      float result[size];
      errno_t mem_ret = memcpy_s(result, sizeof(result), result_tensor->GetBuffer(), result_tensor->GetSize());
//     cout << "---------result------image::" << IMAGE_NUM << "-------------" <<  endl;
//     float *b = max_element(result, result+6);
//    int maxPosition = max_element(result, result+6) - result;
//    int maxPosition = max_element(result, result+5) - result;
    int maxPosition = max_element(result, result+GESTURE_NUM) - result;
    cout << "======================" << endl;
//    for (int aa=0;aa<GESTURE_NUM;aa++){
//        cout << result[aa] << endl;
//    }
    float down = 1.4;
//    float result_total = pow(down, result[0]) + pow(down, result[1]) + pow(down, result[2]) + pow(down, result[3]) + pow(down, result[4]) + pow(down, result[5]);
    float result_total = pow(down, result[0]) + pow(down, result[1]) + pow(down, result[2]) + pow(down, result[3]) + pow(down, result[4]);
//    float result_total = pow(down, result[0]) + pow(down, result[1]) + pow(down, result[2]);
    result[0] = pow(down, result[0]) / result_total;
    result[1] = pow(down, result[1]) / result_total;
    result[2] = pow(down, result[2]) / result_total;
    result[3] = pow(down, result[3]) / result_total;
    result[4] = pow(down, result[4]) / result_total;
//    result[5] = pow(down, result[5]) / result_total;
//    for (int aa=0;aa<GESTURE_NUM;aa++){
//        cout << result[aa] << endl;
//    }
    cout << " 鼓掌" << result[0] << endl;
    cout << " 挥手" << result[1] << endl;
    cout << " 站立" << result[2] << endl;
    cout << "双手平举 " << result[3] << endl;
    cout << "踢腿 " << result[4] << endl;
//    cout << "======================" << result[maxPosition] << endl;

    if (result[maxPosition] >= 0.5){
        switch (maxPosition){
            case 0:
                if(result[maxPosition] > 0.9){
                    if (LAST_GES != 0){
                        cout << "==================鼓掌" << endl;
                        LAST_GES = 0;
                    }
                }
                break;
            case 1:
                if(result[maxPosition] > 0.8){
                    if (LAST_GES != 1){
                        cout << "==================挥手" << endl;
                        LAST_GES = 1;
                    }
                }
                break;
            case 2:
                if(result[maxPosition] > 0.5){
                    if (LAST_GES != 2){
                        cout << "==================站立" << endl;
                        LAST_GES = 2;
                    }
                }
                break;
            case 3:
                if(result[maxPosition] > 0.9){
                    if (LAST_GES != 3){
                        cout << "==================双手平举" << endl;
                        LAST_GES = 3;
                    }
                }
                break;
            case 4:
                if(result[maxPosition] > 0.9){
                    if (LAST_GES != 4){
                        cout << "==================踢腿" << endl;
                        LAST_GES = 4;
                    }
                    break;
                }
            default:
                cout << "max element==================nothing" << maxPosition << "     " << result[maxPosition] << endl;
                break;
         }
    }
    else{
    LAST_GES = -1;
    GES_TIMES = 0;
    if_sent = false;
//     cout << "max element==================无无无" << endl;
    }
//     cout << "max element==================" << maxPosition << "     " << result[maxPosition] << endl;

    IMAGE_NUM++;
    IMAGE_NUM = IMAGE_NUM%100;

    if (mem_ret != EOK) {
      HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                      "post process call memcpy_s() error=%d", mem_ret);
      return HIAI_ERROR;
    }
}