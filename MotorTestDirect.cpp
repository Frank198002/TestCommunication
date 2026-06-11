// 
// # Define Moter_HeTai        //use for BenMo moter communication process
// ./TEST_MOTOR_DIRECT 300 -300
// input： left: 300（mm/s） right: -300（mm/s）
// output： wheel_RPM*10     (Not motor RPM!!!!)

// 解析代码逻辑（需要处理read可能引起的多帧堆积）
// 各司其职：for 循环只管做一件事——把这次 read 到的所有原始数据，老老实实地全部 push_back 到 rev_buffer 的屁股后面。
// 全局扫描：数据塞完后，我们用一个 while 循环，从头到尾去扫描这个 rev_buffer，寻找 3C 86。
// 精准切除：一旦在中间某个位置找到了 3C 86，就把这一段切给 buffer 去解析，然后 erase 擦除掉。接着继续扫描剩下的数据，直到里面再也没有完整的帧为止。



// Third Motor
#define Moter_BenMo        //use for BenMo moter communication process

#ifdef Moter_BenMo
#define WHEEL_DIAMETER_MM 150.0f   //150mm
#endif

// ./TEST_MOTOR_DIRECT 0.3 -0.3
// input： left: 0.3（m/s） right: -0.3（m/s）
// output： wheel_RPM*10     (Not motor RPM!!!!)




// ======================================
// check stm32 Yaw value to smart borad
// ======================================
#include <iostream>
#include <thread>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdlib>
#include <csignal>
#include <atomic>

#include <vector>
#include <deque>

#include <cstdio>
#include <cstdint>

std::deque<uint8_t> rev_buffer;   //receive buffer, push_back each byte read from serial port, include 0x3c86, but not cut yet
std::vector<uint8_t> buffer;     //cut buffer, push_back each byte from rev_buffer until 0x3c86, then cut 3c86

std::atomic<bool> running(true);
int file = -1;

#define PI 3.1415926f

unsigned char motor_buffer[] = {0x3b, 0x4f, 0x5d, 0x6e, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00};	//Speed=0 from smart to stm32 speed
unsigned char cmd_alarm_off[] = {0x3b, 0x4f, 0x5d, 0x6e, 0x07, 0x00, 0x04, 0x01, 0x00, 0x00, 0x00, 0x61};	//alarm_off command from smart to stm32
unsigned char cut_height_query[] = {0x3b, 0x4f, 0x5d, 0x6e, 0x07, 0x00, 0x08, 0x64};	//cut_height_query command from smart to stm32
unsigned char pin_code_query[] = {0x3b, 0x4f, 0x5d, 0x6e, 0x07, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00}; //pin_code_query


unsigned char Set_THreshold[] = 
{
    0x3b, 0x4f, 0x5d, 0x6e,
    0x09,
    0x05,
    0x01,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00
};

unsigned char Set_recharge_voltage[] = 
{
    0x3b, 0x4f, 0x5d, 0x6e,
    0x07,
    0x05,
    0x02,
    0x00, 0x00,
    0x00, 0x00,
    0x00
};

unsigned char set_Control_Board_HD_version[] = 
{
    0x3b, 0x4f, 0x5d, 0x6e,
    0x07,
    0x05,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00
};

unsigned char Read_EEPROM[] = 
{
    0x3b, 0x4f, 0x5d, 0x6e,
    0x03,
    0x05,
    0x03,
    0x00
};



float g_leftVelocity = 0.0f;
float g_rightVelocity = 0.0f;

void signalHandler(int signum) {
    std::cout << "\n[Signal] Ctrl+C detected, stopping..." << std::endl;
    running = false;
}

// void SerialPort_1_Init(const char *devicePath) {
//     file = open(devicePath, O_RDWR | O_NOCTTY | O_NDELAY);
//     if (file == -1) {
//         perror("Failed to open serial port");
//         exit(1);
//     }

//     fcntl(file, F_SETFL, 0);  // 阻塞模式

//     struct termios options;
//     tcgetattr(file, &options);

//     cfsetispeed(&options, B115200);
//     cfsetospeed(&options, B115200);

//     options.c_cflag |= (CLOCAL | CREAD);
//     options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
//     options.c_iflag &= ~(IXON | IXOFF | IXANY);
//     options.c_oflag &= ~OPOST;

//     options.c_cc[VMIN] = 0;
//     options.c_cc[VTIME] = 1;  // 100ms

//     tcsetattr(file, TCSANOW, &options);
// }

//     //from claude
void SerialPort_1_Init(const char *devicePath) {
    file = open(devicePath, O_RDWR | O_NOCTTY | O_NDELAY);
    if (file == -1) {
        perror("Failed to open serial port");
        exit(1);
    }

    fcntl(file, F_SETFL, 0);

    struct termios options;
    // ★ 先清零，不要基于旧配置修改
    memset(&options, 0, sizeof(options));

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag = B115200 | CS8 | CREAD | CLOCAL;  // 无奇偶校验，1停止位
    options.c_iflag = 0;   // 完全清零，不做任何输入处理
    options.c_oflag = 0;   // 完全清零，不做任何输出处理
    options.c_lflag = 0;   // 完全清零，raw模式

    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 1;  // 100ms timeout

    tcflush(file, TCIOFLUSH);  // 清空收发缓冲区
    tcsetattr(file, TCSANOW, &options);
}



unsigned int CalBufferSum(unsigned char *buffer_, size_t len) {
    unsigned int sum = 0;
    for (size_t i = 0; i < len - 1; i++) {
        sum += buffer_[i];
    }
    return sum;
}

void send_loop() {
    int leftVelocity_ = 0;
    int rightVelocity_ = 0;

    while (running) {

        /********************************************************
         * motor command
         ********************************************************/

#ifdef Moter_BenMo
        // int convert maybe cause 0.8cm error
        leftVelocity_ = static_cast<int>(
            (g_leftVelocity * 1000.0f * 60.0f) /
            (WHEEL_DIAMETER_MM * PI));

        rightVelocity_ = static_cast<int>(
            (g_rightVelocity * 1000.0f * 60.0f) /
            (WHEEL_DIAMETER_MM * PI));
#endif

        memcpy(motor_buffer + 7, &leftVelocity_, 2);
        memcpy(motor_buffer + 9, &rightVelocity_, 2);

        unsigned int checkSum = CalBufferSum(motor_buffer, sizeof(motor_buffer));
        checkSum &= 0xFF;
        memcpy(motor_buffer + 13, &checkSum, 1);

        if (write(file, motor_buffer, sizeof(motor_buffer)) == -1) {
            std::cerr << "Send error (motor_buffer)" << std::endl;
        }

        std::cout << "leftVelocity_: " << leftVelocity_
                  << ", rightVelocity_: " << rightVelocity_
                  << std::endl;


        /********************************************************
         * pin code query
         ********************************************************/
        checkSum = CalBufferSum(pin_code_query, sizeof(pin_code_query));
        checkSum &= 0xFF;
        memcpy(pin_code_query + 11, &checkSum, 1);

        if (write(file, pin_code_query, sizeof(pin_code_query)) == -1) {
            std::cerr << "Send error (pin_code_query)" << std::endl;
        }


        /********************************************************
         * Set THreshold（RAIN，Tilt，Collision）
         ********************************************************/

        uint16_t RainTHHigh   = 3000;
        uint16_t TiltAngTH    = 30;
        uint16_t CollisionCoef = 10;

        Set_THreshold[7]  = (RainTHHigh >> 8) & 0xFF;
        Set_THreshold[8]  = RainTHHigh & 0xFF;

        Set_THreshold[9]  = (TiltAngTH >> 8) & 0xFF;
        Set_THreshold[10] = TiltAngTH & 0xFF;

        Set_THreshold[11] = (CollisionCoef >> 8) & 0xFF;
        Set_THreshold[12] = CollisionCoef & 0xFF;

        checkSum = CalBufferSum(Set_THreshold,sizeof(Set_THreshold));

        checkSum &= 0xFF;

        Set_THreshold[13] = checkSum;

        if (write(file, Set_THreshold, sizeof(Set_THreshold)) == -1)
        {
            std::cerr << "Send error (Set_THreshold)" << std::endl;
        }


        /********************************************************
         * Set recharge voltage
         ********************************************************/

        // uint16_t ReChargeV = 165; //yuhui
        // uint16_t FullV     = 195; //yuhui
        uint16_t ReChargeV = 150; //yuhui_test
        uint16_t FullV     = 202; //yuhui_test

        Set_recharge_voltage[7] = (ReChargeV >> 8) & 0xFF;
        Set_recharge_voltage[8] = ReChargeV & 0xFF;

        Set_recharge_voltage[9]  = (FullV >> 8) & 0xFF;
        Set_recharge_voltage[10] = FullV & 0xFF;

        checkSum = CalBufferSum(Set_recharge_voltage,sizeof(Set_recharge_voltage));

        checkSum &= 0xFF;

        Set_recharge_voltage[11] = checkSum;

        if (write(file,
                  Set_recharge_voltage,
                  sizeof(Set_recharge_voltage)) == -1)
        {
            std::cerr << "Send error (Set_recharge_voltage)" << std::endl;
        }


        /********************************************************
         * Set Control Board HD version
         ********************************************************/

        // uint32_t Hardware_VER = 0x01020304;

        // set_Control_Board_HD_version[7]  =
        //     (Hardware_VER >> 24) & 0xFF;

        // set_Control_Board_HD_version[8]  =
        //     (Hardware_VER >> 16) & 0xFF;

        // set_Control_Board_HD_version[9]  =
        //     (Hardware_VER >> 8) & 0xFF;

        // set_Control_Board_HD_version[10] =
        //     Hardware_VER & 0xFF;

        // checkSum = CalBufferSum(set_Control_Board_HD_version,sizeof(set_Control_Board_HD_version));

        // checkSum &= 0xFF;

        // set_Control_Board_HD_version[11] = checkSum;

        // if (write(file,
        //           set_Control_Board_HD_version,
        //           sizeof(set_Control_Board_HD_version)) == -1)
        // {
        //     std::cerr << "Send error (set_Control_Board_HD_version)"
        //               << std::endl;
        // }


        /********************************************************
         * READ EEPROM from stm32
         ********************************************************/

        // checkSum = CalBufferSum(Read_EEPROM,sizeof(Read_EEPROM));

        // checkSum &= 0xFF;

        // Read_EEPROM[7] = checkSum;

        // if (write(file,
        //           Read_EEPROM,
        //           sizeof(Read_EEPROM)) == -1)
        // {
        //     std::cerr << "Send error (Read_EEPROM)"
        //               << std::endl;
        // }




        //==================END==============
        usleep(500000);  // 500ms
    }
}


// ---------------------------------------------------------
// 全新重构的 receive_loop：基于滑窗扫描与擦除，不再漏数据
// ---------------------------------------------------------
void receive_loop() {
    unsigned char responseBuffer[1024];

    while (running) {
        int bytesRead = read(file, responseBuffer, sizeof(responseBuffer));

        if (bytesRead < 0) continue;

        if (bytesRead == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        // 可以解开注释查看底层收到的最原始的 16 进制流，用于极限 Debug
        printf("[RAW RECEIVE]: ");
        for(int k = 0; k < bytesRead; k++) {
            printf("%02X ", responseBuffer[k]);
        }
        printf(" !RAW END!\n");

        // 1. 将本次读取到的所有数据无脑推入队列尾部
        for (int k = 0; k < bytesRead; k++) {
            rev_buffer.push_back(responseBuffer[k]);
        }

        // 2. 滑动窗口扫描：只要队列里有超过 2 个字节，就去寻找 0x3C 0x86
        size_t i = 0;
        while (rev_buffer.size() >= 2 && i < (rev_buffer.size() - 1)) {
            
            // 如果发现了帧尾 0x3c 0x86
            if (rev_buffer[i] == 0x3c && rev_buffer[i + 1] == 0x86) {
                
                size_t frame_len = i + 2; // 整个完整帧的长度

                // 把 3C 86 之前的所有数据提取到 buffer 里进行解析
                buffer.clear();
                buffer.reserve(i);
                for (size_t j = 0; j < i; j++) {
                    buffer.push_back(rev_buffer[j]);
                }

                // 连同 3C 86 一起打印出来，看起来就是一个完全体的数据包
                printf("[VALID PACKET] size=%ld | Content: ", buffer.size() + 2);
                for (size_t j = 0; j < buffer.size(); j++) {
                    printf("%02X ", buffer[j]);
                }
                printf("3C 86\n"); // 手动把帧尾打印出来


                // ===== 数据解析阶段 =====
                if (buffer.size() >= 4) {

                    // datatype = 0x03 speed/imu/yaw
                    if (buffer[1] == 0x03) {
                        if (buffer.size() >= 32) {
                            int32_t left_rpm_raw =
                                (int32_t)buffer[2] | ((int32_t)buffer[3] << 8) |
                                ((int32_t)buffer[4] << 16) | ((int32_t)buffer[5] << 24);

                            int32_t right_rpm_raw =
                                (int32_t)buffer[6] | ((int32_t)buffer[7] << 8) |
                                ((int32_t)buffer[8] << 16) | ((int32_t)buffer[9] << 24);

#ifdef Moter_BenMo
                            float left_rpm = left_rpm_raw / 10.0f; 
                            float right_rpm = right_rpm_raw / 10.0f;
                            float left_speed_mps = (left_rpm * PI * WHEEL_DIAMETER_MM / 1000.0f) / 60.0f;
                            float right_speed_mps = (right_rpm * PI * WHEEL_DIAMETER_MM / 1000.0f) / 60.0f;
#endif                                
                            std::cout << "Left Wheel: " << left_rpm << " RPM, " << left_speed_mps << " m/s | ";
                            std::cout << "Right Wheel: " << right_rpm << " RPM, " << right_speed_mps << " m/s\n";

                            uint32_t raw_bits =
                                (uint32_t)buffer[27] | ((uint32_t)buffer[28] << 8) |
                                ((uint32_t)buffer[29] << 16) | ((uint32_t)buffer[30] << 24);

                            float f_yaw;
                            std::memcpy(&f_yaw, &raw_bits, sizeof(float));
                            std::cout << "Yaw: " << f_yaw << "\n";
                        }
                    }
                    // datatype = 0x07 cut height
                    else if (buffer[1] == 0x07) {
                        // cut height logic
                    }
                    // datatype = 0x05 pin code
                    else if (buffer[1] == 0x05) {
                        // pin code logic
                    }
                    // datatype = 0x06 version
                    else if (buffer[1] == 0x06) {
                        std::cout << "Version\n";
                    }
                    // datatype = 0x08 System Thresholds
                    else if (buffer[1] == 0x08) {
                        if (buffer.size() >= 16) {
                            uint16_t RainTHHigh = ((uint16_t)buffer[2] << 8) | (uint16_t)buffer[3];
                            uint16_t TiltAngTH = ((uint16_t)buffer[4] << 8) | (uint16_t)buffer[5];
                            uint16_t CollisionCoef = ((uint16_t)buffer[6] << 8) | (uint16_t)buffer[7];
                            uint16_t ReChargeV = ((uint16_t)buffer[8] << 8) | (uint16_t)buffer[9];
                            uint16_t FullV = ((uint16_t)buffer[10] << 8) | (uint16_t)buffer[11];
                            uint32_t Hardware_VER = ((uint32_t)buffer[12] << 24) | ((uint32_t)buffer[13] << 16) | ((uint32_t)buffer[14] << 8)  | (uint32_t)buffer[15];

                            std::cout << "RainTHHigh: " << RainTHHigh << "  "
                                      << "TiltAngTH: " << TiltAngTH << "  "
                                      << "CollisionCoef: " << CollisionCoef << std::endl;
                            std::cout << "ReChargeV: " << ReChargeV << std::endl;
                            std::cout << "FullV: " << FullV << "  "
                                      << "Hardware_VER: 0x" << std::hex << Hardware_VER << std::dec << std::endl;
                        }
                    } else {
                        // 打印未知类型的状态，防止漏看特殊握手或报错指令
                        printf("[UNKNOWN DATATYPE]: 0x%02X\n", buffer[1]);
                    }
                }

                // ===== 核心修改：安全擦除 =====
                // 只擦除从开头到 3C 86 结束的这 frame_len 个字节，保护后面的数据不被破坏
                rev_buffer.erase(rev_buffer.begin(), rev_buffer.begin() + frame_len);

                // 因为删除了数据，整体向前滑移，必须重置指针从 0 再次扫描可能紧跟着的下一帧
                i = 0; 
                continue;
            }

            // 如果当前不是 3C 86，指针往后走一格继续找
            i++;
        }

        // 3. 异常兜底保护：防止由于噪声导致永远找不到 3C 86，让内存撑爆
        // 这里把容忍阈值放大到 256（安全起见），超标时弹出最老的 1 个字节让窗口继续走
        while (rev_buffer.size() >= 256) {
            rev_buffer.pop_front();
        }
    }
}

int main(int argc, char *argv[]) {
    // const std::string motorPath = "/dev/ttyS6"; // for Radxa
    // const std::string motorPath = "/dev/ttyS3"; // for KickPi

    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <serial_port> <left_velocity(m/s)> <right_velocity>(m/s)\n";
        std::cerr << "Example: " << argv[0] << " /dev/ttyS6 0.3 0.3\n";
        return 1;
    }

    const std::string motorPath = argv[1];  // 从参数读取串口路径

    signal(SIGINT, signalHandler);

    SerialPort_1_Init(motorPath.c_str());

    // 获取速度参数（m/s）
    g_leftVelocity = atof(argv[2]);
    g_rightVelocity = atof(argv[3]);

    std::cout << "Motor control started. Press Ctrl+C to stop.\n";

    std::thread sender(send_loop);
    std::thread receiver(receive_loop);

    sender.join();
    receiver.join();

    close(file);

    std::cout << "Serial port closed. Program exited.\n";

    return 0;
}



// 时间同步
// sudo systemctl restart systemd-timesyncd