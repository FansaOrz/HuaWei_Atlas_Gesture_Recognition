/**
* @file DataInput.cpp
*
* Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include <ctime>
#include <memory>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>
#include <time.h>
#include <math.h>
#include <map>
#include "DataInput.h"
#include "hiaiengine/log.h"
#include "hiaiengine/data_type_reg.h"
#include "hiaiengine/ai_memory.h"
using namespace std;
const static std::string RC = "RC";
const int DVPP_BUFFER_ALIGN_SIZE = 128;

/**
* @brief: make datasetInfo_
*/
void DataInput::MakeDatasetInfo(int index) {
    string img_name = to_string(index) + ".jpg";
//    cout << "start read data!!" << endl;
    datasetInfo_.clear();
    ImageInfo imageInfo1;
    imageInfo1.id = 0;
    imageInfo1.path = path_ + img_name;
//    imageInfo1.width = 500;
    imageInfo1.width = 500;
//    imageInfo1.height = 374;
    imageInfo1.height = 500;

    struct stat statbuff;
    if(stat(imageInfo1.path.c_str(), &statbuff) < 0){
        return;
    }else{
        imageInfo1.size = statbuff.st_size;
    }
    // 测试size有什么作用
//    imageInfo1.size = 140054;
    imageInfo1.format = (int)IMAGE_TYPE_JPEG;
    datasetInfo_.push_back(imageInfo1);

//    ImageInfo imageInfo2;
//    imageInfo2.id = 1;
//    imageInfo2.path = path_ + "2008_000159.jpg";
//    imageInfo2.width = 500;
//    imageInfo2.height = 375;
////    imageInfo2.size = 91741;
//    imageInfo2.format = (int)IMAGE_TYPE_JPEG;
//    datasetInfo_.push_back(imageInfo2);
//    ImageInfo imageInfo3;
//    imageInfo3.id = 2;
//    imageInfo3.path = path_ + "2010_004236.jpg";
//    imageInfo3.width = 500;
//    imageInfo3.height = 375;
////    imageInfo3.size = 67517;
//    imageInfo3.format = (int)IMAGE_TYPE_JPEG;
//    datasetInfo_.push_back(imageInfo3);
}

// 初始化函数 将graph.conig传入该引擎
HIAI_StatusT DataInput::Init(const hiai::AIConfig& config, const  std::vector<hiai::AIModelDescription>& model_desc) {
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[DataInput] Start init!");
//    std::cout << "start init datainput" << std::endl;

    //read the config of dataset
    // 读取config文件中的配置参数，两个
    for (int index = 0; index < config.items_size(); ++index) {
        const ::hiai::AIConfigItem& item = config.items(index);
        std::string name = item.name();
        if (name == "target") {
            target_ = item.value();
        } else if (name == "path") {
            path_ = item.value();
        }
    }
    //get the dataset image info
    // 读取path路径下的图片文件，到Atlas上会变成~/HIAI_DATANDMODELSET/workspace_mind_studio/data路径
    // 更改为一直读取图片，放在runsameside函数内
//    MakeDatasetInfo();

    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[DataInput] End init!");
    return HIAI_OK;
}

/**
* @brief: get the image buffer
* @[in]: path, the image path;
* @[in]: imageBufferPtr, the point of image buffer;
* @[in]: imageBufferLen, the buffer length;
* @[in]: frameId, the start of file offset
* @[return]: bool, if success return true, else return false
*/
bool DataInput::GetImageBuffer(const char* path, uint8_t* imageBufferPtr, uint32_t imageBufferLen, uint32_t frameId){
    bool ret = false;
    FILE * file = fopen64(path, "r");
    if (file == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[DataInput] Failed to  open file %s.", path);
        return ret;
    }
    do {
        unsigned long imageFseek = ((unsigned  long)frameId)*((unsigned  long)imageBufferLen);
        if (0 != fseeko64(file, imageFseek, SEEK_SET)) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[DataInput] Failed to call fseeko64 for offset(%u)", frameId * imageBufferLen);
            break;
        }
//        cout << "IMAGE buffer length" << imageBufferLen << endl;
        if (imageBufferLen != fread(imageBufferPtr, 1, imageBufferLen, file)) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[DataInput] Failed to call fread for length(%u).", imageBufferLen);
            break;
        }
        ret = true;
    } while (0);

    fclose(file);
    return ret;
}

/**
* @brief free the buffer malloced by HIAI:MALLOC
*/
static void FreeImageBuffer(uint8_t* ptr){
    if (ptr == nullptr) {
        return;
    }
    HIAI_StatusT ret = HIAI_OK;
    #if defined(IS_RC)
        ret = hiai::HIAIMemory::HIAI_DVPP_DFree(ptr);
    #else
        ret = hiai::HIAIMemory::HIAI_DFree(ptr);
    #endif
    if (HIAI_OK != ret) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[DataInput] Failed to call DFree.");
    }
    ptr = nullptr;
}

/**
* @brief: convert image info to NewImageParaT
* @[in]: index, index of image in datasetInfo_
* @[out]: imgData, the point of data image
* @[return]: HIAI_StatusT
*/
HIAI_StatusT DataInput::makeImageInfo(NewImageParaT& imgData, int index) {
    if (index < 0 || (uint32_t)index >= datasetInfo_.size()) {
        return HIAI_ERROR;
    }
    imgData.img.format = (IMAGEFORMAT)datasetInfo_[index].format;
    imgData.img.width = datasetInfo_[index].width ;
    imgData.img.height = datasetInfo_[index].height;
    std::string imageFullPath = datasetInfo_[index].path;

    uint8_t * imageBufferPtr = nullptr;
    HIAI_StatusT mallocRet = HIAI_OK;
    #if defined(IS_RC)
        //run on same side with dvpp
        if ((ImageType)datasetInfo_[index].format == IMAGE_TYPE_JPEG) {
        // transfer jepg to imagepreprocess use dvpp jepgd need to add 8 bit for check
            imgData.img.size = datasetInfo_[index].size + 8;
        } else {
            imgData.img.size = datasetInfo_[index].size;
        }
        //run on same side with dvpp need to make the mem align to 128(dvpp need)
        int alignBufferSize = (int)ceil(1.0 * imgData.img.size / DVPP_BUFFER_ALIGN_SIZE) * DVPP_BUFFER_ALIGN_SIZE;
        mallocRet = hiai::HIAIMemory::HIAI_DVPP_DMalloc(alignBufferSize, (void*&)imageBufferPtr);
    #else
        imgData.img.size = datasetInfo_[index].size;
        mallocRet = hiai::HIAIMemory::HIAI_DMalloc(imgData.img.size, (void*&)imageBufferPtr);
    #endif

    if (HIAI_OK != mallocRet || imageBufferPtr == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[DataInput] Failed to call DMalloc.");
        return HIAI_ERROR;
    }

    if (!GetImageBuffer(imageFullPath.c_str(), imageBufferPtr, datasetInfo_[index].size, 0)) {
        FreeImageBuffer(imageBufferPtr);
        return HIAI_ERROR;
    }
    // free imageBufferPtr with function FreeImageBuffer()
    shared_ptr<uint8_t> data(imageBufferPtr, FreeImageBuffer);
    imgData.img.data = data;
    return HIAI_OK;
}

/**
* @brief: send batch for RC
* @[in]: batchId, batchId;
* @[in]: batchNum, the total number of batch;
* @[in]: imageInfoBatch, the send data;
* @[return]: HIAI_StatusT
*/
HIAI_StatusT DataInput::SendBatch(int batchId, int batchNum, std::shared_ptr<BatchImageParaWithScaleT> imageInfoBatch){
    HIAI_StatusT hiaiRet = HIAI_OK;
    imageInfoBatch->b_info.batch_size = imageInfoBatch->v_img.size();
    imageInfoBatch->b_info.max_batch_size = 1;
    imageInfoBatch->b_info.batch_ID = batchId;
    // 是否是第一张
    imageInfoBatch->b_info.is_first = (batchId == 0 ? true : false);
    // 是否是最后一张
    imageInfoBatch->b_info.is_last = (batchId == batchNum - 1 ? true : false);

    do {
        // 参数：：    制定从本引擎的哪一个端口出去           数据的类型        把发送的数据强制转换成void类型
        hiaiRet = SendData(DEFAULT_DATA_PORT, "BatchImageParaWithScaleT", std::static_pointer_cast<void>(imageInfoBatch));
        if (HIAI_QUEUE_FULL == hiaiRet) {
//            std::cout << "queue FULLLLLLLLLL!!!!!!!!\n";
            HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[DataInput] The queue is full, sleep 200ms.");
            usleep(SEND_DATA_INTERVAL_MS);
        }
    } while (hiaiRet == HIAI_QUEUE_FULL);

    if (HIAI_OK != hiaiRet) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[DataInput] Failed to send data for batch %u, error code: %u.", batchId, hiaiRet);
    }
    return hiaiRet;
}

/**
* @brief: run images on same side, all engine at same side
* @[return]: HIAI_StatusT
*/
HIAI_StatusT DataInput::RunOnSameSide(){
    HIAI_StatusT ret = HIAI_OK;
    // 要删除的文件名
    int before_index = 0;
    int totalCount = datasetInfo_.size();
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[DataInput] Run on %s for %u images", target_.c_str(), totalCount);
    // 一直读取路径下的图片，每一百组一个循环
//    for (int index = 0; ;index++) {
    for (int index = 0; ;) {
        index = index % 100;
        // if img exixt
//        std::cout << "当前处理第:" << index << "张图片"<<std::endl;

        string img_path = path_ + to_string(index) + ".jpg";
        const char* tmp = img_path.data();
        if((access(tmp, 0)) == -1){
            index --;
//            std::cout << "waiting " << index+1 << ".jpg" << std::endl;
            sleep(1);
            continue;
        }
        // 每次读取一张图片，图片名为index
        MakeDatasetInfo(index);

        // 删除之前的图片
        before_index = index - 50;
        if(before_index < 0){
            before_index += 100;
        }
        img_path = path_ + to_string(before_index) + ".jpg";
        const char * pre_img = img_path.c_str();
        // 删除图片文件
        unlink(pre_img);


        //convert batch image infos to BatchImageParaWithScaleT
        std::shared_ptr<BatchImageParaWithScaleT> imageInfoBatch = std::make_shared<BatchImageParaWithScaleT>();
        if (imageInfoBatch == nullptr) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[DataInput] Failed to call make_shared for BatchImageParaWithScaleT.");
            return HIAI_ERROR;
        }
        NewImageParaT imgData;

        // jiashi changed
        ret = makeImageInfo(imgData, 0);
        if (HIAI_OK != ret) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[DataInput] Failed to make image info for frame id %u. Stop to send images.",0);
//            cout << "UNKNOWN ERROR!!!!!!!!!!" << endl;
            continue;
            // Origin version
//            return ret;
        }
        imageInfoBatch->v_img.push_back(imgData);
        imageInfoBatch->b_info.frame_ID.push_back(datasetInfo_[0].id);

        //then send data
        ret = SendBatch(0, totalCount, imageInfoBatch);
        if (HIAI_OK != ret) {
            return ret;
        }
    }
    return  HIAI_OK;
}


/**
* @brief: Send Sentinel Image
*/
HIAI_StatusT DataInput::SendSentinelImage()
{
    HIAI_StatusT ret = HIAI_OK;

    shared_ptr<BatchImageParaWithScaleT> image_handle = std::make_shared<BatchImageParaWithScaleT>();
    if (image_handle == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[DataInput] Failed to call make_shared for BatchImageParaWithScaleT.");
        return HIAI_ERROR;
    }
    ret = SendBatch(-1, 1, image_handle);

    return ret;
}

/**
* @ingroup hiaiengine
* @brief HIAI_DEFINE_PROCESS : Overloading Engine Process processing logic
* @[in]: Define an input port, an output port
*/
// 类型名 类型 引擎的输入端口数
HIAI_IMPL_ENGINE_PROCESS("DataInput", DataInput, INPUT_SIZE)
{
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[DataInput] Start process!");
    std::static_pointer_cast<string>(arg0);
    HIAI_StatusT ret = HIAI_OK;
    clock_t start_time = clock();
    ret = RunOnSameSide();
    std::cout << "engine1 dataInput time::" << double(clock() - start_time) / CLOCKS_PER_SEC << std::endl;
    //send sentinel image 为了终止程序，为了连续识别应该不需要
    //HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[DataInput] Send sentinel image.");
    //ret = SendSentinelImage();
    //HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[DataInput] End process!");
    return ret;
}
