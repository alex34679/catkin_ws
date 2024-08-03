#include "sbus_wrapper/sbus_wrapper.h"
#include <asm-generic/termbits.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

namespace sbus_wrapper {
    // SBUS 通道的最大值
    const uint32_t SbusWrapper::CHANNEL_MAX = 512;
    // SBUS 通道的位数
    const uint32_t SbusWrapper::CHANNEL_SIZE = 16;
    // 一个字节的位数
    const uint8_t SbusWrapper::BYTE_SIZE = 8;
    // SBUS 帧的大小
    const uint8_t SbusWrapper::SBUS_FRAME_SIZE = 25;
    // SBUS 帧的开始字节
    const uint8_t SbusWrapper::SBUS_FRAME_BEGIN_BYTE = 0x0F;
    // SBUS 正常通道数
    const uint8_t SbusWrapper::SBUS_NORMAL_CHANS = 16;
    // SBUS 通道中心值
    const int SbusWrapper::SBUS_CHAN_CENTER = 992;
    // SBUS 通道值位数
    const uint8_t SbusWrapper::SBUS_CHAN_BITS = 11;
    // SBUS 通道位数
    const uint8_t SbusWrapper::SBUS_CH_BITS = 11;

    SbusWrapper::SbusWrapper(ros::NodeHandle &nh, ros::NodeHandle &pnh, bool proctection)
        : nh_(nh),
          pnh_(pnh),
          // 是否处于上锁状态
          barm_(false), 
          // 是否接收到位姿消息
          brecpose_(false),
          // 是否接收到控制指令的更新
          bcmdupdate_(false),
          // 是否启用保护模式
          b_protection_(proctection) {
        
        // 初始化参数
        initParams();

        // 初始化串口
        initSerial();

        std::vector<int> disarm_channels(CHANNEL_SIZE, -CHANNEL_MAX);
        disarm_channels[0] = 0;
        disarm_channels[1] = 0;
        disarm_channels[3] = 0;
        // 将通道值转换为帧
        cvtChannels2Frame(disarm_channels, disarm_frame_);

        cmd_sub_ = nh_.subscribe("control_command", 100, &SbusWrapper::cmdCallback, this);

        arm_sub_ = nh_.subscribe("arm", 100, &SbusWrapper::armCallback, this);


        pose_sub_ = nh_.subscribe("pose", 100, &SbusWrapper::poseCallback, this);


        timer_ = nh_.createTimer(ros::Duration(1.0 / freq_), &SbusWrapper::timerCallback, this);
    }

    // 进行参数的初始化
    void SbusWrapper::initParams() {

        bool bParams = true;

        bParams &= pnh_.getParam("frequency", freq_);
        bParams &= pnh_.getParam("serial_port", serial_port_);
        bParams &= pnh_.getParam("max_roll_rate", max_roll_rate_);
        bParams &= pnh_.getParam("max_pitch_rate", max_pitch_rate_);
        bParams &= pnh_.getParam("max_yaw_rate", max_yaw_rate_);
        bParams &= pnh_.getParam("max_thrust", max_thrust_);
        bParams &= pnh_.getParam("drone_mass", drone_mass_);

        bParams &= pnh_.getParam("thrust_k", thrust_k_);
        bParams &= pnh_.getParam("thrust_b", thrust_b_);

        ROS_ASSERT(bParams);
    }

    void SbusWrapper::initSerial() {
        // 打开串口设备，返回一个文件描述符serial_fd，用于后续的读写操作
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);

        // 清除串口配置
        fcntl(serial_fd_, F_SETFL, 0);
        // 利用fcntl函数设置串口为非阻塞读取模式
        fcntl(serial_fd_, F_SETFL, FNDELAY);

        struct termios2 uart_config;
        /* Fill the struct for the new configuration */
        // 获取当前串口配置，并将其存储在uart_config中
        ioctl(serial_fd_, TCGETS2, &uart_config);

        // Output flags - Turn off output processing
        // no CR to NL translation, no NL to CR-NL translation,
        // no NL to CR translation, no column 0 CR suppression,
        // no Ctrl-D suppression, no fill characters, no case mapping,
        // no local output processing
        /*         
        设置输出标志位，目的是关闭输出处理的各种特性，确保数据以原始格式输出
        无 回车到换行的转换，无 换行到回车换行的转换
        无换行到回车的转换，无列0回车抑制
        无Ctrl-D抑制，无填充字符，无大小写映射
        无本地输出处理 
        */
        uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

        //
        // No line processing:
        // echo off
        // echo newline off
        // canonical mode off,
        // extended input processing off
        // signal chars off
        //
        /*  
        设置输入标志位，无线路处理：关闭回显，关闭规范模式，关闭扩展输入处理，关闭信号字符
        */
        uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);


        // Turn off character processing
        // Turn off odd parity
        /*  
        设置字符处理标志位，关闭字符处理，关闭奇偶校验
        */
        uart_config.c_cflag &= ~(CSIZE | PARODD | CBAUD);

        // Enable parity generation on output and parity checking for input.
        // 启用输出奇偶校验，输入奇偶校验检查
        uart_config.c_cflag |= PARENB;
        // Set two stop bits, rather than one.
        // 设置两个停止位，而不是一个
        uart_config.c_cflag |= CSTOPB;
        // No output processing, force 8 bit input
        // 关闭输出处理，强制8位输入
        uart_config.c_cflag |= CS8;
        // Enable a non standard baud rate
        // 启用非标准波特率
        uart_config.c_cflag |= BOTHER;

        // Set custom baud rate of 100'000 bits/s necessary for sbus
        // 设置波特率为100000，这是sbus协议的波特率
        const speed_t spd = 100000;
        uart_config.c_ispeed = spd;
        uart_config.c_ospeed = spd;

        // 将更新后的串口配置写入串口
        ioctl(serial_fd_, TCSETS2, &uart_config);
    }

    // 发送数据，参数为一个包含要发送数据的字节的std::vector<uint8_t>类型的向量
    void SbusWrapper::sendData(const std::vector<uint8_t> &frame) const {
        // 将数据写入文件描述符serial_fd_所关联的文件(即串口设备)
        write(serial_fd_, frame.data(), SBUS_FRAME_SIZE);
    }

    void SbusWrapper::timerCallback(const ros::TimerEvent &event) {
        static std::vector<uint8_t> frame;
        if (!barm_) {
            sendData(disarm_frame_);
        } else {
            if (bcmdupdate_) {
                std::vector<int> channels;
                genNormalizedChannels(command_, channels);
                cvtChannels2Frame(channels, frame);
                bcmdupdate_ = false;
            }
            sendData(frame);
        }
    }

    // 将输入的通道值转换为符合 SBUS 格式要求的帧数据，并存储在 frame 向量中。
    void SbusWrapper::cvtChannels2Frame(const std::vector<int> &channels, std::vector<uint8_t> &frame) const {

        // 清空帧
        frame.clear();
        // 预留帧空间
        frame.reserve(SBUS_FRAME_SIZE);
        // 将同步字节SBUS_FRAME_BEGIN_BYTE加入帧frame
        frame.push_back(SBUS_FRAME_BEGIN_BYTE);

        // 声明一个变量bits，用于存储位数据
        uint32_t bits = 0;
        // 声明一个变量bitsavailable，用于表示可用的位数
        uint8_t bitsavailable = 0;

        // 遍历前SBUS_NORMAL_CHANS个通道
        // byte 1-22, channels 0..2047, limits not really clear (B
        for (int i = 0; i < SBUS_NORMAL_CHANS; i++) {
            int value = channels[i];

            // 将value转化为SBUS格式的位表示
            value = value * 8 / 5 + SBUS_CHAN_CENTER;

            // 将value添加到bits中的相应位置
            bits |= limit(0, value, 2047) << bitsavailable;
            // 更新bitsavailable
            bitsavailable += SBUS_CHAN_BITS;
            // 如果说明bitsavailable大于等于8，则将bits的低8位添加到帧frame中，并将bitsavailable减去8
            while (bitsavailable >= BYTE_SIZE) {
                frame.push_back((uint8_t) (bits & 0xff));
                bits >>= BYTE_SIZE;
                bitsavailable -= BYTE_SIZE;
            }
        }

        // flags，标志位
        uint8_t flags = 0;
        frame.push_back(flags);
        // last byte, always 0x0
        frame.push_back(0x00);
    }

    template<class T>
    T SbusWrapper::limit(T lowerbound, T val, T upperboud) const {
        return std::max(lowerbound, std::min(val, upperboud));
    }

    // 生成归一化的通道值，以便将控制指令转换为适合发送给SBUS设备的数据
    void SbusWrapper::genNormalizedChannels(const quadrotor_msgs::ControlCommand &msg, std::vector<int> &channels) const {
        ROS_ASSERT(barm_);
        double roll_rate = msg.bodyrates.x;
        double pitch_rate = msg.bodyrates.y;
        double yaw_rate = msg.bodyrates.z;
        // double yaw_rate = msg.bodyrates.z - 0.022;
        // thrust =  normalized thrust * mass
        //
        double thrust = msg.collective_thrust * drone_mass_;
        //double thrust = 9.7947* drone_mass_;

        if (b_protection_) {
            roll_rate = limit(-0.5, roll_rate, 0.5);
            pitch_rate = limit(-0.5, pitch_rate, 0.5);
            yaw_rate = limit(-0.5, yaw_rate, 0.5);
            thrust = limit((-1 + 9.7947) * drone_mass_, thrust, (1 + 9.7947) * drone_mass_);
        }

        // normalize input to -512 ~ 512
        int roll_rate_ratio = (roll_rate / max_roll_rate_ * CHANNEL_MAX);
        roll_rate_ratio = limit<int>(-CHANNEL_MAX, roll_rate_ratio, CHANNEL_MAX);
        int pitch_rate_ratio = pitch_rate / max_roll_rate_ * CHANNEL_MAX;
        pitch_rate_ratio = limit<int>(-CHANNEL_MAX, pitch_rate_ratio, CHANNEL_MAX);
        int yaw_rate_ratio = yaw_rate / max_roll_rate_ * CHANNEL_MAX;
        yaw_rate_ratio = limit<int>(-CHANNEL_MAX, yaw_rate_ratio, CHANNEL_MAX);

        /**
         * input thrust(g)
         * output percentage 0% ~100%
         */
        // auto f = [](float thrust) -> float { return 340 * thrust; };
        //auto f = [](float thrust) -> float { return -140.1 * thrust * thrust + 255.1 * thrust; };

        // 计算用于控制飞行器的油门比例
        // int throttle_ratio = f(thrust / 9.7947) * 10.24 - CHANNEL_MAX;
        // int throttle_ratio = 524.84 * thrust - CHANNEL_MAX;
        // int throttle_ratio = 469.15 * thrust  + 76.95 - CHANNEL_MAX;
        // int throttle_ratio = 395.42 * thrust  + 53.18 - CHANNEL_MAX;
        // ldarc
        // int throttle_ratio = 539.21 * thrust + 52.096 - CHANNEL_MAX;  
        // ldarcb
        // int throttle_ratio = 376.7 * thrust + 79.03 - CHANNEL_MAX; 
        // int throttle_ratio = 346 * thrust + 79.03 - CHANNEL_MAX;  
        // int throttle_ratio = 42.689 * thrust * thrust + 392.83 * thrust  + 93.017 - CHANNEL_MAX;

        //通用模板
        int throttle_ratio = thrust_k_ * thrust + thrust_b_ - CHANNEL_MAX;
  

        limit<int>(-CHANNEL_MAX, throttle_ratio, CHANNEL_MAX);
        channels.clear();
        channels.resize(CHANNEL_SIZE, -CHANNEL_MAX);
        channels[0] = roll_rate_ratio;
        channels[1] = pitch_rate_ratio;
        channels[2] = throttle_ratio;
        // the yaw is inverted
        channels[3] = -yaw_rate_ratio;
    }

    void SbusWrapper::cmdCallback(const quadrotor_msgs::ControlCommandConstPtr &msg) {
        if (!barm_) {
            ROS_WARN("[ sbus_wrapper ] arm the drone first");
            return;
        }
        if (msg->control_mode != quadrotor_msgs::ControlCommand::BODY_RATES) {
            ROS_WARN("[ sbus_wrapper ] invalid control command");
            barm_ = false;
            return;
        }
        if (!isfinite(msg->bodyrates.x) ||
            !isfinite(msg->bodyrates.y) ||
            !isfinite(msg->bodyrates.z)) {
            barm_ = false;
            ROS_WARN("[ sbus_wrapper ] control command infinite");
            return;
        }
        if (msg->collective_thrust * drone_mass_ > 1.1 * max_thrust_ ||
            abs(msg->bodyrates.x) > 1.1 * max_roll_rate_ ||
            abs(msg->bodyrates.y) > 1.1 * max_pitch_rate_ ||
            abs(msg->bodyrates.z) > 1.1 * max_yaw_rate_) {
            barm_ = false;
            ROS_WARN("[ sbus_wrapper ] control command exceed");
            ROS_WARN("[ sbus_wrapper ] input : %.3f %.3f %.3f %.3f", msg->collective_thrust, msg->bodyrates.x, msg->bodyrates.y, msg->bodyrates.z);
            //return;
        }

        command_ = *msg;
        bcmdupdate_ = true;
    }

    void SbusWrapper::armCallback(const std_msgs::BoolConstPtr &msg) {
        if (!brecpose_) {
            return;
        }
        if (msg->data) {
            ROS_INFO("[ sbus wrapper ] arm");
        } else {
            ROS_INFO("[ sbus wrapper ] disarm");
        }
        barm_ = msg->data;
    }

    void SbusWrapper::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
        
        if (!brecpose_) {
            stamp_ = msg->header.stamp;
            brecpose_ = true;
            return;
        }

        if (b_protection_) {

            float dt = (msg->header.stamp - stamp_).toSec();
            if (dt > 1e-1) {
                ROS_WARN("[ sbus wrapper ] protection activate : lose motion capture information");
                barm_ = false;
            }

            if (abs(msg->pose.position.x) > 3.0 ||
                abs(msg->pose.position.y) > 3.0 ||
                abs(msg->pose.position.z) > 2.5) {
                ROS_WARN("[ sbus wrapper ] protection activate : offside");
                barm_ = false;
            }
        }

        stamp_ = msg->header.stamp;
    }

}// namespace sbus_wrapper