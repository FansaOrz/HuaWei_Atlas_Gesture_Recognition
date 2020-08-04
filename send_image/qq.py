#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import time
# 传输图片
import paramiko



def read_capture():
    num = 0
    if_single = True
    cap = cv2.VideoCapture("/home/jiashi/pose_test.mp4")
    success, frame = cap.read()
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("192.168.1.2",  22, "HwHiAiUser", "Mind@123")
    # 打开SSH端口
    sftp = ssh.open_sftp()
    Atlas_path = "HIAI_DATANDMODELSET/workspace_mind_studio/data/"
    start = time.time()
    while success:
        # 隔一帧发送一次，相当于15帧
        if if_single:
            if_single = False
            # print "跳过一张"
            continue
        if_single = True
        # 读取图片
        img_np = frame.copy()
        # 裁剪成128x128
        img_np = cv2.resize(img_np, (500, 500))
        # 图片保存的位置
        img_path = '%s%d.jpg'%('./frames/', num)
        cv2.imwrite(img_path, img_np)
        # 传输
        sftp.put(img_path, Atlas_path+'%d.jpg'%num)
        print "发送第%d张图片"%num
        num += 1
        # 一百张图片一次循环
        num = num % 100


        cv2.imshow('capture face detection', frame)
        if cv2.waitKey(1) >= 0:
           break
        success, frame = cap.read()
        # 一秒发送16张
        time.sleep(0.016)
        # time.sleep(1)
        print "目前用时%f"%(time.time()-start)
    cv2.destroyAllWindows()
    cap.release()



if __name__ == '__main__':
    read_capture()

