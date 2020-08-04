#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import time
# 传输图片
import paramiko



def read_capture():
    num = 0
    IMG_NAME = 65
    send_time = 0
    # if_single = True
    #cap = cv2.VideoCapture("./output2.mp4")
    img_P = cv2.imread("")
    # cap = cv2.VideoCapture(2)
    # cap_R = cv2.VideoCapture(5)
    # success, frame = cap.read()
    # success_R, frame_R = cap_R.read()
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("192.168.1.2",  22, "HwHiAiUser", "Mind@123")
    # 打开SSH端口
    sftp = ssh.open_sftp()
    Atlas_path = "HIAI_DATANDMODELSET/workspace_mind_studio/data/"
    start = time.time()
    # save 60 frames
    while True:
        # 读取图片
        # img_np = frame.copy()

        # img_np_R = frame_R.copy()
        # crop
        #img_np = img_np[0:720, 300:1020]
        # 裁剪成500x500
        # img_np = cv2.resize(img_np, (500, 500))
        # img_np_R = cv2.resize(img_np_R, (500, 500))
        # 图片保存的位置
        img_path = '%s%d.jpg'%('./frames/test/', IMG_NAME)
        img_path_R = '%s%d.jpg'%('./frames/test/R_', IMG_NAME)
        img_np = cv2.imread(img_path)
        img_np_R = cv2.imread(img_path_R)
        # 旋转
        #(h, w) = img_np.shape[:2] #10
        #center = (w // 2, h // 2) #11
        #M = cv2.getRotationMatrix2D(center, -90, 1.0) #12
        #img_np = cv2.warpAffine(img_np, M, (w, h)) #13


        if num < 65:
            # 传输
            try:
                sftp.put(img_path, Atlas_path+'%d.jpg'%num)
                sftp.put(img_path_R, Atlas_path+'%d_R.jpg'%num)
                IMG_NAME += 1
                #print "11"
            except IOError:
                print "ERROR transport================================================================"
                # success, frame = cap.read()
                # success_R, frame_R = cap_R.read()
                continue
            print "发送第%d张图片"%num
        else:
            # sleep(7)
            time.sleep(8)
            num = -1
        num += 1


        cv2.imshow('capture face detection', img_np)
        cv2.imshow('capture face detection_R', img_np_R)
        # q key
        if cv2.waitKey(1) == 113:
            # print cv2.waitKey(1)
            break
        if num >= 65:
            # key a
            if cv2.waitKey(1) == 97:
                num = 0
        # success, frame = cap.read()
        # success_R, frame_R = cap_R.read()
        # 一秒发送25张
        time.sleep(0.01)
        # time.sleep(1)
        # print "目前用时%f"%(time.time()-start)
    cv2.destroyAllWindows()
    # cap.release()
    # cap_R.release()



if __name__ == '__main__':
    read_capture()

