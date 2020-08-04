#!/bin/expect

#设置变量
set user "HwHiAiUser"
set host "192.168.1.177"
set loginpass "Mind@123"
set cmd_prompt "]"

spawn ssh $user@$host
#设置超时时间，单位是秒
set timeout 30
# -re 匹配正则表达式
expect {
	-re "Are you sure you want to continue connecting (yes/no)?" {
		send "yes\r"
		} 
	-re "password: " {
		send "${loginpass}\r"
		} 
	-re "Permission denied, please try again." {
		exit
		}

}

expect { 
	-re $cmd_prompt {
		send "bash ./atlas_mapping.sh\n"
		send "exit\n"
	}

}

interact
