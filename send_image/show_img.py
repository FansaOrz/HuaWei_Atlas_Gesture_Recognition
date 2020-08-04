#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import time
# 传输图片
import paramiko


def read_capture():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("192.168.1.2",  22, "HwHiAiUser", "Mind@123")
    # 打开SSH端口
    sftp = ssh.open_sftp()
    Atlas_path = "/home/HwHiAiUser/HIAI_PROJECTS/workspace_mind_studio/openpose_det_f06a50e5/out/result_files/0/SaveFilePostProcess/out1.jpg"
    local_path = "./out1.jpg"
    start = time.time()
    cv2.namedWindow('img', cv2.WINDOW_NORMAL)
    while True:
        sftp.get(Atlas_path, local_path)
        img = cv2.imread(local_path)
        try:
            cv2.imshow('img', img)
        except Exception:
            continue
        if cv2.waitKey(1) >= 0:
            break
        # 一秒发送16张
        time.sleep(0.006)
    cv2.destroyAllWindows()



if __name__ == '__main__':
    read_capture()

