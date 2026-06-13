# 日常测试操作步骤：

1. 临时关闭xiaoyu进程，防止串口干扰（重启之后就会重新启动xiaoyu进程）
    ssh radxa@10.65.224.117     密码：radxa235    或者radxa
    ssh linaro@10.31.244.2      密码: linaro
    sudo systemctl stop xiaoyu

2. 把本项目下载到 ～ 目录下

3. 进入 文件夹TESTCOMMUNICATION，
‘’‘
   ~/TestCommunication/TEST_COMMUNICATION
’‘’

4. 如果需要修改参数，直接进config.ini修改
    
    












# （这个章节仅仅码农需要）如何修改代码变成 自己要发送的上下位机命令：
1. 设置上下位机通信要输入的消息内容如下：
‘’‘
unsigned char motor_buffer[] = {0x3b, 0x4f, 0x5d, 0x6e, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00};	//Speed=0 from smart to stm32 speed
unsigned char cmd_alarm_off[] = {0x3b, 0x4f, 0x5d, 0x6e, 0x07, 0x00, 0x04, 0x01, 0x00, 0x00, 0x00, 0x61};	//alarm_off command from smart to stm32  
unsigned char cut_height_query[] = {0x3b, 0x4f, 0x5d, 0x6e, 0x07, 0x00, 0x08, 0x64};	//cut_height_query command from smart to stm32  
‘’‘
增加程序中发送的对应的buffer如上面，修改对应发送的长度。


2. 修改 Makefile，按顺序往下添加。

‘’‘
TEST_COMMUNICATION: MotorTestDirect.cpp
‘’‘

修改之后进行编译
‘’‘
make TEST_COMMUNICATION 
‘’‘
生成的TEST_MOTOR_Direct 就是最终生成的可执行文件


3. 编译 
make TEST_COMMUNICATION


4. 如何检查是否已经正确往串口发送数据
timeout 5 strace -f ./TEST_COMMUNICATION /dev/ttyS6 2000 2000>/tmp/strace3.txt
grep "write(3" /tmp/strace3.txt




# 这个章节仅仅在新板子使用之前注意事项（镜像的板子请忽略）：

1. 先登录你的额机器，登录内网frp机器后查清楚 机器在实验室内的IP地址
ssh radxa@10.65.224.117     密码：radxa235    或者radxa
ssh linaro@10.31.244.2      密码: linaro

ssh linaro@192.168.1.*** 密码：linaro

2. 先检查系统的串口是否已经打开！！！！
   ls /dev/ttyS*


3. 确认设置当前使用客户的权限(必须有dialout)，否则需要执行和检查如下操作
radxa@xiaoyu:~/Final_Motor_Module_Test_yuxiao/TestMotorTiny$ sudo usermod -aG dialout xiaoyu
radxa@xiaoyu:~/Final_Motor_Module_Test_yuxiao/TestMotorTiny$ groups xiaoyu
radxa : radxa dialout sudo
sudo chmod 777 /dev/ttyS6
