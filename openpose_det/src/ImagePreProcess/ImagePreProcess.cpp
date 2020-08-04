/**
* @file ImagePreProcess.cpp
*
* Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include <math.h>
#include <sstream>
#include <ctime>
#include "ImagePreProcess.h"
#include "hiaiengine/log.h"
#include "hiaiengine/ai_types.h"
#include "hiaiengine/c_graph.h"
#include "hiaiengine/ai_memory.h"

const static int SEND_DATA_SLEEP_MS = 100000;
const static int DVPP_SUPPORT_MAX_WIDTH = 4096;
const static int DVPP_SUPPORT_MIN_WIDTH = 16;
const static int DVPP_SUPPORT_MAX_HEIGHT = 4096;
const static int DVPP_SUPPORT_MIN_HEIGHT = 16;
const static int WIDTH_STRIDE = 16;
const static int HEIGHT_STRIDE = 2;

#define CHECK_ODD(NUM)      (((NUM) % 2 != 0) ? (NUM) : ((NUM) - 1))
#define CHECK_EVEN(NUM)     (((NUM) % 2 == 0) ? (NUM) : ((NUM) - 1))


ImagePreProcess::~ImagePreProcess() {
    if (nullptr != pidvppapi_) {
        DestroyDvppApi(pidvppapi_);
        pidvppapi_ = nullptr;
    }
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess] Destroy ImagePreProcess engine.");
}

HIAI_StatusT ImagePreProcess::Init(const hiai::AIConfig& config, const std::vector<hiai::AIModelDescription>& modelDesc) {
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess] Start init!");
    dvppConfig_ = std::make_shared<DvppConfig>();
    if (dvppConfig_ == nullptr || dvppConfig_.get() == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess] Failed to call make_shared for DvppConfig.");
        return HIAI_ERROR;
    }


    //get config from ImagePreProcess Property setting of user.
    std::stringstream ss;
    for (int index = 0; index < config.items_size(); ++index) {
        const ::hiai::AIConfigItem& item = config.items(index);
        std::string name = item.name();
        ss << item.value();
        if ("resize_height" == name) {
            ss >> dvppConfig_->resize_height;
        } else if ("resize_width" == name) {
            ss >> dvppConfig_->resize_width;
        }
        ss.clear();
    }

    if (DVPP_SUCCESS != CreateDvppApi(pidvppapi_)) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess] Failed to call CreateDvppApi.");
        return HIAI_ERROR;
    }
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess] End init!");
    return HIAI_OK;
}

//jpeg pic process flow:
//  1. DVPP_CTL_JPEGD_PROC
//  2. DVPP_CTL_TOOL_CASE_GET_RESIZE_PARAM
//  3. DVPP_CTL_VPC_PROC
int ImagePreProcess::HandleJpeg(const ImageData<u_int8_t> &img) {
    struct JpegdIn jpegdInData; // input data
    jpegdInData.jpegData = (unsigned char*)(img.data.get()); //the pointer addr of jpeg pic data
    jpegdInData.jpegDataSize = img.size; //the size of jpeg pic data
    jpegdInData.isYUV420Need = false; //(*dvppConfig_).yuv420_need;true:output yuv420 data, otherwize:raw format.
    jpegdInData.isVBeforeU = true; //currently, only support V before U, reserved

    struct JpegdOut jpegdOutData; //output data

    dvppapi_ctl_msg dvppApiCtlMsg; //create inputdata and outputdata for jpegd process
    dvppApiCtlMsg.in = (void *)&jpegdInData;
    dvppApiCtlMsg.in_size = sizeof(struct JpegdIn);
    dvppApiCtlMsg.out = (void *)&jpegdOutData;
    dvppApiCtlMsg.out_size = sizeof(struct JpegdOut);

    if (0 != DvppCtl(pidvppapi_, DVPP_CTL_JPEGD_PROC, &dvppApiCtlMsg)) { //if this single jpeg pic is processed with error, return directly, and then process next pic if there any.
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess] Failed to call DvppCtl for JPEGD, FrameID: %u", imageFrameID_);
        jpegdOutData.cbFree(); //release memory from caller.
        return HIAI_ERROR;
    }

    int ret = HandleVpcWithParam(jpegdOutData.yuvData, jpegdOutData.imgWidthAligned, jpegdOutData.imgHeightAligned,
        jpegdOutData.yuvDataSize, img, FILE_TYPE_PIC_JPEG, jpegdOutData.outFormat);
    jpegdOutData.cbFree(); //release memory from caller.

    return ret;
}

bool ImagePreProcess::SendPreProcessData() {
    if (nullptr == dvppOut_) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess] Nothing to send!");
        return false;
    }

    if (dvppOut_->v_img.size() > 0) {
        dvppOut_->b_info.batch_size = dvppOut_->v_img.size();
        HIAI_StatusT hiaiRet = HIAI_OK;
        do {
            hiaiRet = SendData(0, "BatchImageParaWithScaleT", std::static_pointer_cast<void>(dvppOut_));
            if (HIAI_QUEUE_FULL == hiaiRet) {
                HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess] The queue is full, sleep 200ms.");
                std::cout << "PPPPPPPPPPPPPPPPPPPPPPPP" << std::endl;
                usleep(SEND_DATA_INTERVAL_MS);
            }
        } while (hiaiRet == HIAI_QUEUE_FULL);

        if (HIAI_OK != hiaiRet) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess] Failed to send data, error code: %d", hiaiRet);
            return false;
        }
    }
    return true;
}

void ImagePreProcess::ClearData() {
    dvppIn_ = nullptr;
    dvppOut_ = nullptr;
}

void ImagePreProcess::ProcessCrop(VpcUserCropConfigure &area, const int &width, const int &height, const int &realWidth, const int &realHeight) {
    //default no crop
    int leftOffset = 0; //the left side of the cropped image
    int rightOffset = realWidth - 1; //the right side of the cropped image
    int upOffset = 0; //the top side of the cropped image
    uint32_t downOffset = realHeight - 1; //the bottom side of the cropped image

    //restriction: leftOffset and upOffset of inputputArea must be even, rightOffset and downOffset of inputputArea must be odd.
    area.leftOffset = CHECK_EVEN(leftOffset);
    area.rightOffset = CHECK_ODD(rightOffset);
    area.upOffset = CHECK_EVEN(upOffset);
    area.downOffset = CHECK_ODD(downOffset);
}

int ImagePreProcess::HandleVpcWithParam(const unsigned char* buffer, const int &width, const int &height, const long &bufferSize,
        const ImageData<u_int8_t> &img, const FILE_TYPE &type, const int &format) {
    int realWidth = img.width;
    int realHeight = img.height;

    HIAI_ENGINE_LOG("[ImagePreProcess] realWidth: %d, realHeight: %d, width: %d, height: %d", realWidth, realHeight, width, height);

    if (buffer == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess] The buffer is null.");
        return HIAI_ERROR;
    }

    VpcUserImageConfigure userImage;
    string paraSetPath[1];
    if (type == FILE_TYPE_PIC_JPEG) {
       switch(format) { //format is responding to color sapce after decoder, which is needed to match the input color space of vpc
            case DVPP_JPEG_DECODE_OUT_YUV444:
                userImage.inputFormat = INPUT_YUV444_SEMI_PLANNER_VU;
                break;
            case DVPP_JPEG_DECODE_OUT_YUV422_H2V1:
                userImage.inputFormat = INPUT_YUV422_SEMI_PLANNER_VU;
                break;
            case DVPP_JPEG_DECODE_OUT_YUV420:
                userImage.inputFormat = INPUT_YUV420_SEMI_PLANNER_VU;
                break;
            case DVPP_JPEG_DECODE_OUT_YUV400:
                userImage.inputFormat = INPUT_YUV400;
                break;
            default:
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess] Failed to get Jpegd out format[%d]", format);
                break;
       }
    }
    userImage.widthStride = width;
    userImage.heightStride = height;
    userImage.outputFormat = OUTPUT_YUV420SP_UV;
    userImage.bareDataAddr = (uint8_t *)buffer;
    userImage.bareDataBufferSize = bufferSize;

    VpcUserRoiConfigure roiConfigure;
    roiConfigure.next = nullptr;
    VpcUserRoiInputConfigure *inputConfigure = &roiConfigure.inputConfigure;
    inputConfigure->cropArea.leftOffset = 0;
    inputConfigure->cropArea.rightOffset = realWidth - 1;
    inputConfigure->cropArea.upOffset = 0;
    inputConfigure->cropArea.downOffset = realHeight - 1;

    uint32_t resizeWidth = dvppConfig_->resize_width;
    uint32_t resizeHeight = dvppConfig_->resize_height;
    if (0 == resizeWidth || 0 == resizeHeight) {
        HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess] The user does not need resize, resize width/height use real size of pic");
        resizeWidth = realWidth;
        resizeHeight = realHeight;
    }
    if (resizeWidth > DVPP_SUPPORT_MAX_WIDTH || resizeWidth < DVPP_SUPPORT_MIN_WIDTH
        || resizeHeight > DVPP_SUPPORT_MAX_HEIGHT || resizeHeight < DVPP_SUPPORT_MIN_HEIGHT) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess] Failed to check the resize range, resizeWidth:%u, resizeHeight:%u", resizeWidth, resizeHeight);
        return HIAI_ERROR;
    }

    VpcUserRoiOutputConfigure *outputConfigure = &roiConfigure.outputConfigure;
    //restriction: leftOffset and upOffset of outputArea must be even, rightOffset and downOffset of outputArea must be odd.
    ProcessCrop(inputConfigure->cropArea, width, height, realWidth, realHeight);

    outputConfigure->outputArea.leftOffset = 0;
    outputConfigure->outputArea.rightOffset = CHECK_ODD(resizeWidth - 1);
    outputConfigure->outputArea.upOffset = 0;
    outputConfigure->outputArea.downOffset = CHECK_ODD(resizeHeight - 1);
    outputConfigure->widthStride = ALIGN_UP(resizeWidth, WIDTH_STRIDE);
    outputConfigure->heightStride = ALIGN_UP(resizeHeight, HEIGHT_STRIDE);
    outputConfigure->bufferSize = outputConfigure->widthStride * outputConfigure->heightStride * 3 / 2;
    outputConfigure->addr = static_cast<uint8_t*>(HIAI_DVPP_DMalloc(outputConfigure->bufferSize));
    if (outputConfigure->addr == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess] The outputConfigure->addr is null.");
        return HIAI_ERROR;
    }
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess] inputConfigure cropArea:%u, %u, %u, %u, outputConfigure outputArea:%u, %u, %u, %u, stride:%u, %u, %u",
        inputConfigure->cropArea.leftOffset, inputConfigure->cropArea.rightOffset, inputConfigure->cropArea.upOffset, inputConfigure->cropArea.downOffset,
        outputConfigure->outputArea.leftOffset, outputConfigure->outputArea.rightOffset, outputConfigure->outputArea.upOffset, outputConfigure->outputArea.downOffset,
        outputConfigure->widthStride, outputConfigure->heightStride, outputConfigure->bufferSize);
 
    userImage.roiConfigure = &roiConfigure;

    dvppapi_ctl_msg dvppApiCtlMsg;
    dvppApiCtlMsg.in = (void *)&userImage;
    dvppApiCtlMsg.in_size = sizeof(VpcUserImageConfigure);
    if (0 != DvppCtl(pidvppapi_, DVPP_CTL_VPC_PROC, &dvppApiCtlMsg)) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess] Failed to call DvppCtl for VPC.");
        HIAI_DVPP_DFree(outputConfigure->addr);
        outputConfigure->addr = nullptr;
        return HIAI_ERROR;
    }

    if (dvppOut_ == nullptr) {
        dvppOut_ = std::make_shared<BatchImageParaWithScaleT>();
        if (dvppOut_ == nullptr || dvppOut_.get() == nullptr) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess] Failed to call make_shared for BatchImageParaWithScaleT.");
            HIAI_DVPP_DFree(outputConfigure->addr);
            outputConfigure->addr = nullptr;
            return HIAI_ERROR;
        }
        dvppOut_->b_info = dvppIn_->b_info;
        dvppOut_->b_info.frame_ID.clear();
    }

    std::shared_ptr<NewImageParaT> image = std::make_shared<NewImageParaT>();
    if (image == nullptr || image.get() == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess] Failed to call make_shared for NewImageParaT.");
        HIAI_DVPP_DFree(outputConfigure->addr);
        outputConfigure->addr = nullptr;
        return HIAI_ERROR;
    }

    image->img.width = outputConfigure->outputArea.rightOffset - outputConfigure->outputArea.leftOffset + 1;
    image->img.height = outputConfigure->outputArea.downOffset - outputConfigure->outputArea.upOffset + 1;
    image->img.size = outputConfigure->bufferSize;
    image->img.channel = img.channel;
    image->img.format = img.format;
    image->scale_info.scale_width = (1.0 * image->img.width)/img.width;
    image->scale_info.scale_height = (1.0 * image->img.height)/img.height;
    image->resize_info.resize_width = dvppConfig_->resize_width;
    image->resize_info.resize_height = dvppConfig_->resize_height;
    image->img.data.reset((uint8_t*)outputConfigure->addr, [](uint8_t* addr){hiai::HIAIMemory::HIAI_DVPP_DFree(addr);});
    dvppOut_->v_img.push_back(*image);
    dvppOut_->b_info.frame_ID.push_back(imageFrameID_);

    return HIAI_OK;
}

int ImagePreProcess::HandleDvpp() {
    if (dvppIn_ == nullptr || dvppIn_.get() == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess] The input data is null!");
        return HIAI_ERROR;
    }
    int i = 0;
    for (std::vector<NewImageParaT>::iterator iter = dvppIn_->v_img.begin(); iter != dvppIn_->v_img.end(); ++iter, i++) {
        imageFrameID_ = dvppIn_->b_info.frame_ID[i];
        if (IMAGE_TYPE_JPEG == (ImageTypeT)(*iter).img.format) {
            if (HIAI_OK != HandleJpeg((*iter).img)) {
                HIAI_ENGINE_LOG(HIAI_IDE_WARNING, "[ImagePreProcess] Failed to handle a image, continue next");
                continue;
            }
        }
    }

    return HIAI_OK;
}

/**
* @ingroup hiaiengine
* @brief HIAI_DEFINE_PROCESS: realize ImagePreProcess
* @[in]: define a input and output,
*        and register Engine named ImagePreProcess
*/
HIAI_IMPL_ENGINE_PROCESS("ImagePreProcess", ImagePreProcess, INPUT_SIZE) {
        clock_t start_time = clock();

    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess] Start process.");
    int errCode = HIAI_OK;
    std::shared_ptr<BatchImageParaWithScaleT> errorHandler = nullptr;
    if(arg0 != nullptr){
        dvppIn_ = std::static_pointer_cast<BatchImageParaWithScaleT>(arg0);
    } else {
        HIAI_ENGINE_LOG(HIAI_IDE_WARNING, "[ImagePreProcess] The arg0 is null.");
        return HIAI_ERROR;
    }
    //add sentinel image for showing this data in dataset are all sended, this is last step.
    if (isSentinelImage(dvppIn_)) {
        HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess] Send sentinel image, process over.");
        HIAI_StatusT hiaiRet = HIAI_OK;
        do {
            hiaiRet = SendData(0, "BatchImageParaWithScaleT", std::static_pointer_cast<void>(dvppIn_));
            if (HIAI_QUEUE_FULL == hiaiRet) {
                HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess] The queue is full, sleep 200ms");
                usleep(SEND_DATA_INTERVAL_MS);
            }
        } while (HIAI_QUEUE_FULL == hiaiRet);

        if (HIAI_OK != hiaiRet) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess] Failed to send sentinel image! error code: %u", hiaiRet);
        }
        return hiaiRet;
    }

    dvppOut_ = nullptr;

    // 1.check the pending data is jpeg, png, yuv or none
    // 2.preprocess before vpc(DVPP_CTL_JPEGD_PROC, DVPP_CTL_PNGD_PROC, or something else)
    // 3.calculate width, height, and so on, according to the user config of crop and resize, and DVPP_CTL_TOOL_CASE_GET_RESIZE_PARAM
    // 4.DVPP_CTL_VPC_PROC after create dvppapi_ctl_msg para
    // 5.dump yuv data to project dir out after dvpp, according to dump_value config(from device to IDE with IDE-deamon-client&IDE-deamon-hiai process)
    // 6.send data to next engine after the process of dvpp
    errCode = HandleDvpp();
    if (HIAI_ERROR == errCode) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess] Failed to call HandleDvpp.");
        goto ERROR;
    }

    if(!SendPreProcessData()) { // send to next engine after dvpp process
        goto ERROR;
    }
    ClearData();
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess] End process!");
//    std::cout << "engine2 ImagePreProcess time::" << double(clock() - start_time) / CLOCKS_PER_SEC << std::endl;

    return HIAI_OK;

ERROR:
    //send null to next node to avoid blocking when to encounter abnomal situation.
    SendData(0, "BatchImageParaWithScaleT", std::static_pointer_cast<void>(errorHandler));
    ClearData();
    return HIAI_ERROR;
}