# 使用之前注意事项：

1. 先检查系统的串口是否已经打开！！！！
   ls /dev/ttyS*


2. 确认设置当前使用客户的权限(必须有dialout)，否则需要执行和检查如下操作
radxa@xiaoyu:~/Final_Motor_Module_Test_yuxiao/TestMotorTiny$ sudo usermod -aG dialout wyf
radxa@xiaoyu:~/Final_Motor_Module_Test_yuxiao/TestMotorTiny$ groups wyf
radxa : radxa dialout sudo
sudo chmod 777 /dev/ttyS6


3. 临时关闭xiaoyu进程，防止串口干扰（重启之后就会重新启动xiaoyu进程）
sudo systemctl stop xiaoyu


# 如何修改程序使用的串口（不需要改写就跳过）：
1. 通过工具或者命令上传TestMotorTiny文件夹到目标板子的 /home/radxa/ 或者/home/linaro/  目录下
2. 进入 文件夹TestMotorTiny，修改文件端口。
    ssh radxa@10.65.224.117     密码：radxa235    或者radxa
    ssh linaro@10.31.244.2      密码: linaro
    
    cd /home/radxa/TestMotorTiny
    sudo nano ****.cpp
    找到 "/dev/ttyS*"; 这行

    把ttyS*改成你想要测试的串口。比如串口8 ，就修改成 /dev/ttyS8

    ctrl + o 保存编辑，然后ctrl + x 退出编辑界面到终端界面。


# 如何修改发送的上下位机命令（不需要改写就跳过）：
1. 设置上下位机通信要输入的消息内容如下：
参考 https://docs.qq.com/sheet/DQkV3TENIdmNHdk9X
unsigned char motor_buffer[] = {0x3b, 0x4f, 0x5d, 0x6e, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00};	//Speed=0 from smart to stm32 speed
unsigned char cmd_alarm_off[] = {0x3b, 0x4f, 0x5d, 0x6e, 0x07, 0x00, 0x04, 0x01, 0x00, 0x00, 0x00, 0x61};	//alarm_off command from smart to stm32  
unsigned char cut_height_query[] = {0x3b, 0x4f, 0x5d, 0x6e, 0x07, 0x00, 0x08, 0x64};	//cut_height_query command from smart to stm32  

增加程序中发送的对应的buffer如上面，修改对应发送的长度。


# 编译
1. 修改 Makefile，按顺序往下添加。

#-------------------------------MAKE TARGETS-------------------------------
TEST_YAW_CUT: YawCutTest.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -o $@ $^ $(LIB_PATH)	
TEST_MOTOR_Direct: MotorTestDirect.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -o $@ $^ $(LIB_PATH)

修改之后进行编译，举例：TEST_MOTOR_Direct
‘’‘
make TEST_MOTOR_Direct 
’‘’
生成的TEST_MOTOR_Direct 就是最终生成的可执行文件



# 如何运行可执行文件
1. 运行命令：
sudo ./TEST_MOTOR_Direct 10 100
    
上面10 代表左轮rpm速度， 100代表右轮rpm速度  代表每分钟轮子多少转，自己设定

【note】如果运行出现权限不够，运行 sudo chmod 777 TEST_MOTOR_Direct

# 日志显示：
发送速度，收到的内容，



# 如何检查是否已经正确往串口发送数据
timeout 5 strace -f ./TEST_COMMUNICATION 0 0 2>/tmp/strace3.txt
grep "write(3" /tmp/strace3.txt
