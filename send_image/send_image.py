#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import time
# 传输图片
import paramiko



def read_capture():
    num = 0
    send_time = 0
    # if_single = True
    #cap = cv2.VideoCapture("./output2.mp4")
    cap = cv2.VideoCapture(2)
    success, frame = cap.read()
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("192.168.1.177",  22, "HwHiAiUser", "Mind@123")
    # 打开SSH端口
    sftp = ssh.open_sftp()
    Atlas_path = "HIAI_DATANDMODELSET/workspace_mind_studio/data/"
    start = time.time()
    while success:
        # 隔一帧发送一次，相当于15帧
        if send_time % 3 == 0:
            send_time += 1
            # if_single = False
            # print "跳过一张"
            continue
        else:
            send_time += 1
            if_single = True
            # 读取图片
            img_np = frame.copy()
            # crop
            #img_np = img_np[0:720, 300:1020]
            # 裁剪成500x500
            img_np = cv2.resize(img_np, (500, 500))
            # 图片保存的位置
            img_path = '%s%d.jpg'%('./frames/', num)
            # 旋转
            #(h, w) = img_np.shape[:2] #10
            #center = (w // 2, h // 2) #11
            #M = cv2.getRotationMatrix2D(center, -90, 1.0) #12
            #img_np = cv2.warpAffine(img_np, M, (w, h)) #13

            cv2.imwrite(img_path, img_np)
            # 传输
            try:
                sftp.put(img_path, Atlas_path+'%d.jpg'%num)
                #print "11"
            except IOError:
                print "ERROR transport================================================================"
                success, frame = cap.read()
                continue
            print "发送第%d张图片"%num
            num += 1
            # 一百张图片一次循环
            num = num % 100


            cv2.imshow('capture face detection', img_np)
            if cv2.waitKey(1) >= 0:
                break
            success, frame = cap.read()
            # 一秒发送25张
            time.sleep(0.028)
            # time.sleep(1)
            print "目前用时%f"%(time.time()-start)
    cv2.destroyAllWindows()
    cap.release()



if __name__ == '__main__':
    read_capture()

