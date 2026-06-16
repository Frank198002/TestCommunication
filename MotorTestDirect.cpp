// 解析代码逻辑（需要处理read可能引起的多帧堆积）
// 各司其职：for 循环只管做一件事——把这次 read 到的所有原始数据，老老实实地全部 push_back 到 rev_buffer 的屁股后面。
// 全局扫描：数据塞完后，我们用一个 while 循环，从头到尾去扫描这个 rev_buffer，寻找 3C 86。
// 精准切除：一旦在中间某个位置找到了 3C 86，就把这一段切给 buffer 去解析，然后 erase 擦除掉。接着继续扫描剩下的数据，直到里面再也没有完整的帧为止。



// Test in KP_25


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

// ======================================
// config.ini 解析器（最小依赖，无需第三方库）
// ======================================
#include <fstream>
#include <sstream>
#include <string>
#include <map>

// -------------------------------------------------------
// 统一禁用标志：config.ini 中赋值 65535 表示该项不发送
// -------------------------------------------------------
#define PARAM_DISABLED  65535

// -------------------------------------------------------
// IniConfig: 轻量 INI 解析器
//   支持 [section] / key = value 格式
//   注释行以 # 或 ; 开头
// -------------------------------------------------------
struct IniConfig {
    // 存储结构：section -> (key -> value)
    std::map<std::string, std::map<std::string, std::string>> data;

    bool load(const std::string& path) {
        std::ifstream f(path);
        if (!f.is_open()) return false;

        std::string line, section;
        while (std::getline(f, line)) {
            // 去掉行首尾空白
            size_t s = line.find_first_not_of(" \t\r\n");
            if (s == std::string::npos) continue;
            line = line.substr(s);
            size_t e = line.find_last_not_of(" \t\r\n");
            if (e != std::string::npos) line = line.substr(0, e + 1);

            if (line.empty() || line[0] == '#' || line[0] == ';') continue;

            if (line[0] == '[') {
                size_t end = line.find(']');
                if (end != std::string::npos)
                    section = line.substr(1, end - 1);
                continue;
            }

            size_t eq = line.find('=');
            if (eq == std::string::npos) continue;

            std::string key = line.substr(0, eq);
            std::string val = line.substr(eq + 1);

            // trim key & val
            auto trim = [](std::string& str) {
                size_t a = str.find_first_not_of(" \t");
                size_t b = str.find_last_not_of(" \t");
                str = (a == std::string::npos) ? "" : str.substr(a, b - a + 1);
            };
            trim(key); trim(val);

            // 去掉行内注释（# 或 ; 之后的部分）
            for (char c : {'#', ';'}) {
                size_t pos = val.find(c);
                if (pos != std::string::npos) {
                    val = val.substr(0, pos);
                    trim(val);
                }
            }

            data[section][key] = val;
        }
        return true;
    }

    // 读取字符串，找不到返回 defaultVal
    std::string getString(const std::string& sec, const std::string& key,
                          const std::string& defaultVal = "") const {
        auto it = data.find(sec);
        if (it == data.end()) return defaultVal;
        auto it2 = it->second.find(key);
        if (it2 == it->second.end()) return defaultVal;
        return it2->second;
    }

    // 读取 int（支持十六进制 0x 前缀）
    int getInt(const std::string& sec, const std::string& key, int defaultVal = PARAM_DISABLED) const {
        std::string v = getString(sec, key, "");
        if (v.empty()) return defaultVal;
        try { return std::stoi(v, nullptr, 0); } catch (...) { return defaultVal; }
    }

    // 读取 float（float 值通过 *100 再截断存储为 int，保留精度）
    float getFloat(const std::string& sec, const std::string& key, float defaultVal = (float)PARAM_DISABLED) const {
        std::string v = getString(sec, key, "");
        if (v.empty()) return defaultVal;
        try { return std::stof(v); } catch (...) { return defaultVal; }
    }
};

// -------------------------------------------------------
// 全局配置对象（main() 解析后各线程只读）
// -------------------------------------------------------
IniConfig g_cfg;


std::deque<uint8_t> rev_buffer;   //receive buffer
std::vector<uint8_t> buffer;      //cut buffer

std::atomic<bool> running(true);
int file = -1;

#define PI 3.1415926f

unsigned char motor_buffer[] = {0x3b, 0x4f, 0x5d, 0x6e, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00};
unsigned char cmd_alarm_off[] = {0x3b, 0x4f, 0x5d, 0x6e, 0x07, 0x00, 0x04, 0x01, 0x00, 0x00, 0x00, 0x61};
unsigned char cut_height_query_buf[] = {0x3b, 0x4f, 0x5d, 0x6e, 0x07, 0x00, 0x08, 0x64};
unsigned char pin_code_query_buf[]   = {0x3b, 0x4f, 0x5d, 0x6e, 0x07, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00};

unsigned char Set_THreshold[] = 
{
    0x3b, 0x4f, 0x5d, 0x6e,
    0x09,
    0x05,
    0x01,
    0x00, 0x00,   // RainTHHigh   [7][8]
    0x00, 0x00,   // TiltAngTH    [9][10]
    0x00, 0x00,   // CollisionCoef[11][12]
    0x00          // checkSum     [13]
};

unsigned char Set_recharge_voltage[] = 
{
    0x3b, 0x4f, 0x5d, 0x6e,
    0x07,
    0x05,
    0x02,
    0x00, 0x00,   // ReChargeV [7][8]
    0x00, 0x00,   // FullV     [9][10]
    0x00          // checkSum  [11]
};

// Hardware VER: 4 字节，高位在前，低位在后，最后 checkSum
// 帧格式: 3B 4F 5D 6E | 07 | 05 | 00 | VER[3] VER[2] VER[1] VER[0] | checkSum
unsigned char set_Control_Board_HD_version[] = 
{
    0x3b, 0x4f, 0x5d, 0x6e,
    0x07,
    0x05,
    0x00,
    0x00,         // VER byte3 (最高位) [7]
    0x00,         // VER byte2          [8]
    0x00,         // VER byte1          [9]
    0x00,         // VER byte0 (最低位) [10]
    0x00          // checkSum           [11]
};

unsigned char Read_EEPROM[] = 
{
    0x3b, 0x4f, 0x5d, 0x6e,
    0x03,
    0x05,
    0x03,
    0x00          // checkSum [7]
};

unsigned char MowerOn_command[]  = {0x3b, 0x4f, 0x5d, 0x6e, 0x03, 0x03, 0x01, 0x00};  // [7]=checkSum
unsigned char MowerOff_command[] = {0x3b, 0x4f, 0x5d, 0x6e, 0x03, 0x03, 0x00, 0x00};  // [7]=checkSum



// GetRTC 命令帧（共15字节）：
// [0-5]  固定包头，[6-13] 全0占位，[14] checkSum
unsigned char getrtc_command[] = {0x3b, 0x4f, 0x5d, 0x6e, 0x0a, 0x01,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // [6-13] 占位
                                   0x00};                                             // [14] checkSum

// SetRTC 命令帧（共15字节）：
// [0-5]  固定包头
// [6]    hour, [7] min, [8] second, [9] day, [10] month, [11] weekday, [12] year(年份-2000)
// [13]   0x00 预留
// [14]   checkSum
unsigned char setrtc_command[] = {0x3b, 0x4f, 0x5d, 0x6e, 0x06, 0x06,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // [6-12] 运行时填入
                                   0x00,                                       // [13] 预留
                                   0x00};                                      // [14] checkSum


// sleep_command 帧格式（共 120 字节）：
// [0-6]  : 固定帧头  3B 4F 5D 6E 73 00 0F
// [7-118]: 28组 UTC 时间戳，每组4字节，小端序（低字节在前）
//          第i组 [7+i*4] ~ [7+i*4+3] = timer & 0xFF, (timer>>8)&0xFF, (timer>>16)&0xFF, (timer>>24)&0xFF
// [119]  : checkSum（前119字节累加之和的低8位）
//
// 时间戳由 config.ini [sleep_timestamps] 节的 ts_00 ~ ts_27 在运行时填充，
// 数组初始值仅保留帧头，其余字节全部清零。
unsigned char sleep_command[120] =
{
    // [0-6] 固定帧头
    0x3b, 0x4f, 0x5d, 0x6e, 0x73, 0x00, 0x0F,
    // [7-118] 28组 UTC 时间戳（运行时从 config.ini 读取后填入）
    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  // 组0  组1
    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  // 组2  组3
    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  // 组4  组5
    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  // 组6  组7
    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  // 组8  组9
    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  // 组10 组11
    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  // 组12 组13
    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  // 组14 组15
    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  // 组16 组17
    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  // 组18 组19
    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  // 组20 组21
    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  // 组22 组23
    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  // 组24 组25
    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  // 组26 组27
    // [119] checkSum（运行时计算填写）
    0x00
};


int g_leftVelocity  = 0;
int g_rightVelocity = 0;

void signalHandler(int signum) {
    std::cout << "\n[Signal] Ctrl+C detected, stopping..." << std::endl;
    running = false;
}

void SerialPort_1_Init(const char *devicePath, int baudRate) {
    file = open(devicePath, O_RDWR | O_NOCTTY | O_NDELAY);
    printf("open() returned fd = %d\n", file);
    if (file == -1) {
        perror("Failed to open serial port");
        exit(1);
    }

    fcntl(file, F_SETFL, 0);

    struct termios options;
    memset(&options, 0, sizeof(options));

    // 根据配置文件波特率选择对应的 B* 常量
    speed_t speed = B115200;
    switch (baudRate) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default:
            fprintf(stderr, "[WARN] Unsupported baud rate %d, fallback to 115200\n", baudRate);
            speed = B115200;
            break;
    }

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag = speed | CS8 | CREAD | CLOCAL;
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_lflag = 0;

    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 1;

    tcflush(file, TCIOFLUSH);
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
    int leftVelocity_  = 0;
    int rightVelocity_ = 0;

    // [system] 系统参数：启动时读一次，运行期间不变
    const int sendInterval = g_cfg.getInt("system", "send_interval_us", 500000);
    printf("[INFO] send_interval_us = %d us\n", sendInterval);

    while (running) {

        // -------------------------------------------------------
        // 每个发送周期重新读取 config.ini
        // 统一规则：值 == 65535 (PARAM_DISABLED) → 不发送该命令
        //           值 != 65535                  → 发送
        // -------------------------------------------------------
        g_cfg.load("config.ini");

        // const int sendInterval = g_cfg.getInt("system", "send_interval_us", 500000);  // 每周期实时生效

        // [motor]
        g_leftVelocity  = g_cfg.getInt("motor", "left_velocity",  0);
        g_rightVelocity = g_cfg.getInt("motor", "right_velocity", 0);

        // [commands] — 纯命令型，int: 1=发送, 65535=不发送
        const int pin_code_query   = g_cfg.getInt("commands", "pin_code_query",   PARAM_DISABLED);
        const int cut_height_query = g_cfg.getInt("commands", "cut_height_query", PARAM_DISABLED);
        const int alarm_off        = g_cfg.getInt("commands", "alarm_off",        PARAM_DISABLED);
        const int read_eeprom      = g_cfg.getInt("commands", "read_eeprom",      PARAM_DISABLED);
        const int mower_on         = g_cfg.getInt("commands", "mower_on",         PARAM_DISABLED);
        const int mower_off        = g_cfg.getInt("commands", "mower_off",        PARAM_DISABLED);

        // [thresholds] — 带值型，int: 65535=不发送, 0~65534=发送
        const int RainTHHigh    = (int)g_cfg.getFloat("thresholds", "RainTHHigh",    (float)PARAM_DISABLED);
        const int TiltAngTH     = (int)g_cfg.getFloat("thresholds", "TiltAngTH",     (float)PARAM_DISABLED);
        const int CollisionCoef = (int)g_cfg.getFloat("thresholds", "CollisionCoef", (float)PARAM_DISABLED);

        // [voltage] — 带值型，int: 65535=不发送, 0~65534=发送
        const int ReChargeV = g_cfg.getInt("voltage", "ReChargeV", PARAM_DISABLED);
        const int FullV     = g_cfg.getInt("voltage", "FullV",     PARAM_DISABLED);

        // [version] — 带值型，int (hex): 65535=不发送, 其他值=发送
        const int          Hardware_VER_raw = g_cfg.getInt("version", "Hardware_VER", PARAM_DISABLED);
        const unsigned int Hardware_VER     = (unsigned int)Hardware_VER_raw;

        // 推导各组命令的发送开关
        const int doSetTH      = (RainTHHigh    != PARAM_DISABLED ||
                                   TiltAngTH     != PARAM_DISABLED ||
                                   CollisionCoef != PARAM_DISABLED) ? 1 : 0;
        const int doSetVoltage = (ReChargeV != PARAM_DISABLED || FullV != PARAM_DISABLED) ? 1 : 0;

        const int doSetHdVer   = (Hardware_VER_raw != PARAM_DISABLED) ? 1 : 0;

        // [sleep] — 纯命令型，int: 65535=不发送, 其他值=发送
        const int send_sleep_command = g_cfg.getInt("commands", "sleep_command", PARAM_DISABLED);

        

        // 打印本轮参数摘要
        printf("\n========== CONFIG (this cycle) ==========\n");
        printf("  [motor]  left=%d  right=%d mm/s\n", g_leftVelocity, g_rightVelocity);
        printf("  [commands]  pin_code_query=%d  cut_height_query=%d  alarm_off=%d  read_eeprom=%d\n",
               pin_code_query, cut_height_query, alarm_off, read_eeprom);
        printf("  [thresholds]  RainTHHigh=%d  TiltAngTH=%d  CollisionCoef=%d  -> send=%d\n",
               RainTHHigh, TiltAngTH, CollisionCoef, doSetTH);
        printf("  [voltage]  ReChargeV=%d  FullV=%d  -> send=%d\n",
               ReChargeV, FullV, doSetVoltage);
        printf("  [version]  Hardware_VER=0x%08X  -> send=%d\n", Hardware_VER, doSetHdVer);
        printf("=========================================\n\n");

        /********************************************************
         * motor command（每帧必发）
         ********************************************************/
        leftVelocity_  = g_leftVelocity;
        rightVelocity_ = g_rightVelocity;

        memcpy(motor_buffer + 7, &leftVelocity_,  2);
        memcpy(motor_buffer + 9, &rightVelocity_, 2);

        unsigned int checkSum = CalBufferSum(motor_buffer, sizeof(motor_buffer));
        checkSum &= 0xFF;
        memcpy(motor_buffer + 13, &checkSum, 1);

        ssize_t ret = write(file, motor_buffer, sizeof(motor_buffer));
        printf("write fd=%d ret=%zd errno=%d %s\n", file, ret, errno, strerror(errno));

        printf("leftVelocity_: %d, rightVelocity_: %d\n", leftVelocity_, rightVelocity_);


        /********************************************************
         * pin_code_query  — [commands] pin_code_query != 65535
         ********************************************************/
        if (pin_code_query != PARAM_DISABLED) {
            checkSum = CalBufferSum(pin_code_query_buf, sizeof(pin_code_query_buf));
            checkSum &= 0xFF;
            memcpy(pin_code_query_buf + 11, &checkSum, 1);

            if (write(file, pin_code_query_buf, sizeof(pin_code_query_buf)) == -1) {
                std::cerr << "Send error (pin_code_query)" << std::endl;
            }
        }


        /********************************************************
         * cut_height_query  — [commands] cut_height_query != 65535
         ********************************************************/
        if (cut_height_query != PARAM_DISABLED) {
            if (write(file, cut_height_query_buf, sizeof(cut_height_query_buf)) == -1) {
                std::cerr << "Send error (cut_height_query)" << std::endl;
            }
        }


        /********************************************************
         * alarm_off  — [commands] alarm_off != 65535
         ********************************************************/
        if (alarm_off != PARAM_DISABLED) {
            if (write(file, cmd_alarm_off, sizeof(cmd_alarm_off)) == -1) {
                std::cerr << "Send error (cmd_alarm_off)" << std::endl;
            }
        }


        /********************************************************
         * Set_THreshold  — 任意阈值 != 65535 则整组发送
         * 未设置的单项保持为 65535（原样写入报文）
         ********************************************************/
        if (doSetTH) {
            int rain  = (RainTHHigh    != PARAM_DISABLED) ? RainTHHigh    : 0;
            int tilt  = (TiltAngTH     != PARAM_DISABLED) ? TiltAngTH     : 0;
            int coef  = (CollisionCoef != PARAM_DISABLED) ? CollisionCoef : 0;

            Set_THreshold[7]  = (rain >> 8) & 0xFF;
            Set_THreshold[8]  =  rain       & 0xFF;
            Set_THreshold[9]  = (tilt >> 8) & 0xFF;
            Set_THreshold[10] =  tilt       & 0xFF;
            Set_THreshold[11] = (coef >> 8) & 0xFF;
            Set_THreshold[12] =  coef       & 0xFF;

            checkSum = CalBufferSum(Set_THreshold, sizeof(Set_THreshold));
            checkSum &= 0xFF;
            Set_THreshold[13] = (unsigned char)checkSum;

            if (write(file, Set_THreshold, sizeof(Set_THreshold)) == -1) {
                std::cerr << "Send error (Set_THreshold)" << std::endl;
            }
        }


        /********************************************************
         * Set_recharge_voltage  — ReChargeV 或 FullV != 65535
         ********************************************************/
        if (doSetVoltage) {
            int rcv = (ReChargeV != PARAM_DISABLED) ? ReChargeV : 0;
            int fv  = (FullV     != PARAM_DISABLED) ? FullV     : 0;

            Set_recharge_voltage[7]  = (rcv >> 8) & 0xFF;
            Set_recharge_voltage[8]  =  rcv       & 0xFF;
            Set_recharge_voltage[9]  = (fv  >> 8) & 0xFF;
            Set_recharge_voltage[10] =  fv        & 0xFF;

            checkSum = CalBufferSum(Set_recharge_voltage, sizeof(Set_recharge_voltage));
            checkSum &= 0xFF;
            Set_recharge_voltage[11] = (unsigned char)checkSum;

            if (write(file, Set_recharge_voltage, sizeof(Set_recharge_voltage)) == -1) {
                std::cerr << "Send error (Set_recharge_voltage)" << std::endl;
            }
        }

        /********************************************************
         * MowerOn  — [commands] mower_on != 65535
         * 帧: 3B 4F 5D 6E 03 03 01 [checkSum]
         ********************************************************/
        if (mower_on != PARAM_DISABLED) {
            unsigned int checkSum_mow = CalBufferSum(MowerOn_command, sizeof(MowerOn_command));
            checkSum_mow &= 0xFF;
            memcpy(MowerOn_command + 7, &checkSum_mow, 1);

            printf("[MowerOn] ");
            for (int i = 0; i < (int)sizeof(MowerOn_command); i++) printf("%02X ", MowerOn_command[i]);
            printf("\n");

            if (write(file, MowerOn_command, sizeof(MowerOn_command)) == -1) {
                std::cerr << "Send error (MowerOn)" << std::endl;
            }
        }


        /********************************************************
         * MowerOff  — [commands] mower_off != 65535
         * 帧: 3B 4F 5D 6E 03 03 00 [checkSum]
         ********************************************************/
        if (mower_off != PARAM_DISABLED) {
            unsigned int checkSum_mow = CalBufferSum(MowerOff_command, sizeof(MowerOff_command));
            checkSum_mow &= 0xFF;
            memcpy(MowerOff_command + 7, &checkSum_mow, 1);

            printf("[MowerOff] ");
            for (int i = 0; i < (int)sizeof(MowerOff_command); i++) printf("%02X ", MowerOff_command[i]);
            printf("\n");

            if (write(file, MowerOff_command, sizeof(MowerOff_command)) == -1) {
                std::cerr << "Send error (MowerOff)" << std::endl;
            }
        }


        /********************************************************
         * set_Control_Board_HD_version  — Hardware_VER != 65535
         * 帧格式: [头4字节] 07 05 00 [VER高→低 4字节] [checkSum]
         ********************************************************/
        if (doSetHdVer) {
            set_Control_Board_HD_version[7]  = (Hardware_VER >> 24) & 0xFF;  // 最高字节
            set_Control_Board_HD_version[8]  = (Hardware_VER >> 16) & 0xFF;
            set_Control_Board_HD_version[9]  = (Hardware_VER >>  8) & 0xFF;
            set_Control_Board_HD_version[10] =  Hardware_VER        & 0xFF;  // 最低字节

            checkSum = CalBufferSum(set_Control_Board_HD_version, sizeof(set_Control_Board_HD_version));
            checkSum &= 0xFF;
            set_Control_Board_HD_version[11] = (unsigned char)checkSum;

            if (write(file, set_Control_Board_HD_version, sizeof(set_Control_Board_HD_version)) == -1) {
                std::cerr << "Send error (set_Control_Board_HD_version)" << std::endl;
            }
        }


        /********************************************************
         * GetRTC  — [commands] get_rtc != 65535
         * 帧: 3B 4F 5D 6E 0A 01 00 00 00 00 00 00 00 00 [checkSum]  共15字节
         ********************************************************/
        const int get_rtc = g_cfg.getInt("commands", "get_rtc", PARAM_DISABLED);
        if (get_rtc != PARAM_DISABLED) {
            unsigned int checkSum_grtc = CalBufferSum(getrtc_command, sizeof(getrtc_command));
            checkSum_grtc &= 0xFF;
            memcpy(getrtc_command + 14, &checkSum_grtc, 1);

            printf("[GetRTC] ");
            for (int i = 0; i < (int)sizeof(getrtc_command); i++) printf("%02X ", getrtc_command[i]);
            printf("\n");

            if (write(file, getrtc_command, sizeof(getrtc_command)) == -1) {
                std::cerr << "Send error (GetRTC)" << std::endl;
            }
        }


        /********************************************************
         * SetRTC  — [commands] set_rtc != 65535
         * 帧: 3B 4F 5D 6E 06 06 [hour][min][sec][day][month][weekday][year] 00 [checkSum]  共15字节
         * 时间取当前系统本地时间，year = 当前年份 - 2000
         ********************************************************/
        const int set_rtc = g_cfg.getInt("commands", "set_rtc", PARAM_DISABLED);
        if (set_rtc != PARAM_DISABLED) {
            time_t now = time(nullptr);
            struct tm *lt = localtime(&now);

            unsigned char hour    = (unsigned char)lt->tm_hour;
            unsigned char min     = (unsigned char)lt->tm_min;
            unsigned char second  = (unsigned char)lt->tm_sec;
            unsigned char day     = (unsigned char)lt->tm_mday;
            unsigned char month   = (unsigned char)(lt->tm_mon + 1);   // tm_mon: 0-11
            unsigned char weekday = (unsigned char)lt->tm_wday;        // 0=Sunday
            unsigned char year    = (unsigned char)(lt->tm_year + 1900 - 2000); // 年份-2000

            memcpy(setrtc_command + 6,  &hour,    1);
            memcpy(setrtc_command + 7,  &min,     1);
            memcpy(setrtc_command + 8,  &second,  1);
            memcpy(setrtc_command + 9,  &day,     1);
            memcpy(setrtc_command + 10, &month,   1);
            memcpy(setrtc_command + 11, &weekday, 1);
            memcpy(setrtc_command + 12, &year,    1);
            // [13] 保持 0x00 预留

            unsigned int checkSum_srtc = CalBufferSum(setrtc_command, sizeof(setrtc_command));
            checkSum_srtc &= 0xFF;
            memcpy(setrtc_command + 14, &checkSum_srtc, 1);

            printf("[SetRTC] %04d-%02d-%02d %02d:%02d:%02d weekday=%d year_field=%d\n",
                   lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
                   lt->tm_hour, lt->tm_min, lt->tm_sec,
                   lt->tm_wday, year);
            printf("[SetRTC] ");
            for (int i = 0; i < (int)sizeof(setrtc_command); i++) printf("%02X ", setrtc_command[i]);
            printf("\n");

            if (write(file, setrtc_command, sizeof(setrtc_command)) == -1) {
                std::cerr << "Send error (SetRTC)" << std::endl;
            }
        }



        /********************************************************
         * READ EEPROM  — [commands] read_eeprom != 65535
         ********************************************************/
        if (read_eeprom != PARAM_DISABLED) {
            checkSum = CalBufferSum(Read_EEPROM, sizeof(Read_EEPROM));
            checkSum &= 0xFF;
            Read_EEPROM[7] = (unsigned char)checkSum;

            if (write(file, Read_EEPROM, sizeof(Read_EEPROM)) == -1) {
                std::cerr << "Send error (Read_EEPROM)" << std::endl;
            }
        }


        /********************************************************
         * sleep_command  — [commands] sleep_command != 65535
         *
         * 帧结构（共 120 字节）：
         *   [0-6]   帧头: 3B 4F 5D 6E 73 00 0F
         *   [7-118] 28组 UTC 时间戳，每组 4 字节，低字节在前（little-endian）
         *   [119]   checkSum（前119字节之和的低8位）
         *
         * 28组时间规律（UTC）：
         *   起始日期: 2022-08-03，每天4个时间点，每半小时一个
         *   第1天(2022-08-03): 05:00, 05:30, 06:00, 06:30
         *   第2天(2022-08-04): 07:00, 07:30, 08:00, 08:30
         *   第3天(2022-08-05): 09:00, 09:30, 10:00, 10:30
         *   第4天(2022-08-06): 11:00, 11:30, 12:00, 12:30
         *   第5天(2022-08-07): 13:00, 13:30, 14:00, 14:30
         *   第6天(2022-08-08): 15:00, 15:30, 16:00, 16:30
         *   第7天(2022-08-09): 17:00, 17:30, 18:00, 18:30
         ********************************************************/
        if (send_sleep_command != PARAM_DISABLED) {

            // 从 config.ini [sleep_timestamps] 读取 28 组 UTC 时间戳（ts_00 ~ ts_27）
            // 并填充到帧缓冲区 sleep_command[7..118]（little-endian，低字节在前）
            static const char* ts_keys[28] = {
                "ts_00","ts_01","ts_02","ts_03","ts_04","ts_05","ts_06","ts_07",
                "ts_08","ts_09","ts_10","ts_11","ts_12","ts_13","ts_14","ts_15",
                "ts_16","ts_17","ts_18","ts_19","ts_20","ts_21","ts_22","ts_23",
                "ts_24","ts_25","ts_26","ts_27"
            };
            for (int n = 0; n < 28; n++) {
                uint32_t timer = (uint32_t)g_cfg.getInt("sleep_timestamps", ts_keys[n], 0);
                int base = 7 + n * 4;
                sleep_command[base]     = (uint8_t)( timer        & 0xFF);
                sleep_command[base + 1] = (uint8_t)((timer >> 8)  & 0xFF);
                sleep_command[base + 2] = (uint8_t)((timer >> 16) & 0xFF);
                sleep_command[base + 3] = (uint8_t)((timer >> 24) & 0xFF);
            }

            // 计算 checkSum：前119字节之和的低8位
            checkSum = CalBufferSum(sleep_command, sizeof(sleep_command));
            checkSum &= 0xFF;
            sleep_command[119] = (uint8_t)checkSum;

            // 打印发送内容（调试用）
            printf("[sleep_command] ");
            for (int i = 0; i < (int)sizeof(sleep_command); i++) {
                printf("%02X ", sleep_command[i]);
            }
            printf("\n");

            if (write(file, sleep_command, sizeof(sleep_command)) == -1) {
                std::cerr << "Send error (sleep_command)" << std::endl;
            }
          
        }

        //==================END==============
        usleep(sendInterval);   // 由 config.ini [system] send_interval_us 控制
    }
}


// ---------------------------------------------------------
// 全新重构的 receive_loop：基于滑窗扫描与擦除，不再漏数据
// ---------------------------------------------------------
void receive_loop() {
    unsigned char responseBuffer[1024];

    while (running) {
        int bytesRead = read(file, responseBuffer, sizeof(responseBuffer));
        printf("read ret=%d\n", bytesRead);

        if (bytesRead < 0) continue;

        if (bytesRead == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        printf("[RAW RECEIVE]: ");
        for (int k = 0; k < bytesRead; k++) {
            printf("%02X ", responseBuffer[k]);
        }
        printf(" !RAW END!\n");

        // 1. 将本次读取到的所有数据推入队列尾部
        for (int k = 0; k < bytesRead; k++) {
            rev_buffer.push_back(responseBuffer[k]);
        }

        // 2. 滑动窗口扫描：寻找 0x3C 0x86
        size_t i = 0;
        while (rev_buffer.size() >= 2 && i < (rev_buffer.size() - 1)) {
            
            if (rev_buffer[i] == 0x3c && rev_buffer[i + 1] == 0x86) {
                
                size_t frame_len = i + 2;

                buffer.clear();
                buffer.reserve(i);
                for (size_t j = 0; j < i; j++) {
                    buffer.push_back(rev_buffer[j]);
                }

                printf("[VALID PACKET] size=%ld | Content: ", buffer.size() + 2);
                for (size_t j = 0; j < buffer.size(); j++) {
                    printf("%02X ", buffer[j]);
                }
                printf("3C 86\n");


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
                            
                            printf("Left RPM: %d  Right RPM: %d\n", left_rpm_raw, right_rpm_raw);

                            uint32_t raw_bits =
                                (uint32_t)buffer[27] | ((uint32_t)buffer[28] << 8) |
                                ((uint32_t)buffer[29] << 16) | ((uint32_t)buffer[30] << 24);

                            float f_yaw;
                            std::memcpy(&f_yaw, &raw_bits, sizeof(float));
                            printf("Yaw: %.4f\n", f_yaw);
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
                        printf("Version\n");
                    }
                    // datatype = 0x08 System Thresholds
                    else if (buffer[1] == 0x08) {
                        if (buffer.size() >= 16) {
                            int RainTHHigh_r    = ((int)buffer[2] << 8) | (int)buffer[3];
                            int TiltAngTH_r     = ((int)buffer[4] << 8) | (int)buffer[5];
                            int CollisionCoef_r = ((int)buffer[6] << 8) | (int)buffer[7];
                            int ReChargeV_r     = ((int)buffer[8] << 8) | (int)buffer[9];
                            int FullV_r         = ((int)buffer[10] << 8) | (int)buffer[11];
                            unsigned int Hardware_VER_r =
                                ((unsigned int)buffer[12] << 24) | ((unsigned int)buffer[13] << 16) |
                                ((unsigned int)buffer[14] <<  8) |  (unsigned int)buffer[15];

                            printf("RainTHHigh: %d  TiltAngTH: %d  CollisionCoef: %d\n",
                                   RainTHHigh_r, TiltAngTH_r, CollisionCoef_r);
                            printf("ReChargeV: %d  FullV: %d\n", ReChargeV_r, FullV_r);
                            printf("Hardware_VER: 0x%08X\n", Hardware_VER_r);
                        }
                    } else {
                        printf("[UNKNOWN DATATYPE]: 0x%02X\n", buffer[1]);
                    }
                }

                // ===== 核心修改：安全擦除 =====
                rev_buffer.erase(rev_buffer.begin(), rev_buffer.begin() + frame_len);
                i = 0; 
                continue;
            }

            i++;
        }

        // 3. 异常兜底保护
        while (rev_buffer.size() >= 256) {
            rev_buffer.pop_front();
        }
    }
}

int main() {

    // ----------------------------------------------------------
    // 1. 加载 config.ini（与可执行文件同目录）
    // ----------------------------------------------------------
    const std::string cfgPath = "config.ini";
    if (!g_cfg.load(cfgPath)) {
        fprintf(stderr, "[ERROR] Cannot open config.ini at '%s'\n", cfgPath.c_str());
        fprintf(stderr, "        Please put config.ini in the same directory as the executable.\n");
        return 1;
    }
    printf("[INFO] Loaded config: %s\n", cfgPath.c_str());

    // ----------------------------------------------------------
    // 2. 从 config.ini 读取所有系统参数
    // ----------------------------------------------------------
    std::string serialPort = g_cfg.getString("system", "serial_port", "/dev/ttyS6");
    int         baudRate   = g_cfg.getInt   ("system", "baud_rate",    115200);

    g_leftVelocity  = g_cfg.getInt("motor", "left_velocity",  0);
    g_rightVelocity = g_cfg.getInt("motor", "right_velocity", 0);

    printf("[INFO] serial_port=%s  baud_rate=%d  left=%d  right=%d\n",
           serialPort.c_str(), baudRate, g_leftVelocity, g_rightVelocity);

    signal(SIGINT, signalHandler);

    SerialPort_1_Init(serialPort.c_str(), baudRate);

    std::cout << "Motor control started. Press Ctrl+C to stop.\n";

    std::thread sender(send_loop);
    std::thread receiver(receive_loop);

    sender.join();
    receiver.join();

    close(file);

    std::cout << "Serial port closed. Program exited.\n";

    return 0;
}


// ./TEST_COMMUNICATION    ← 直接运行，所有参数从 config.ini 读取

// 时间同步
// sudo systemctl restart systemd-timesyncd
// sudo systemctl stop systemd-timesyncd
// sudo systemctl start systemd-timesyncd

// sudo systemctl stop xiaoyu.service 
// sudo systemctl disable xiaoyu.service 

// scp -r /home/linaro/TestCommunication/TEST_COMMUNICATION linaro@10.31.244.6:/home/linaro/TestCommunication/
// scp -r /home/linaro/TestCommunication linaro@10.31.244.11:/home/linaro/

// git add *
// git commit -m
// git push