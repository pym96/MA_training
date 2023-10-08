#include "serial.hpp"

#include <fcntl.h>  //文件控制选项头文件
#include <fmt/color.h>
#include <stdio.h>
#include <termios.h>  //linux串口相关的头文件
#include <unistd.h>  //Linux/Unix系统中内置头文件，包含了许多系统服务的函数原型
#include <chrono>
#include <mutex>
#include <thread>

namespace Devices
{
Serial::Serial(const std::string & name, std::mutex & mutex)
: name(name), fd(-1), serial_mutex(mutex)
{
    frame_header = 0xff;  //帧头
    frame_tail   = 0xfe;  //帧尾, 设为0x0A, 在串口调试助手中可以换行
    loss         = 0;

    tv.tv_sec  = 1;  //秒
    tv.tv_usec = 0;
}

bool Serial::openSerial()
{
    if (isOpen()) {
        fmt::print(fg(fmt::color::red) | fmt::emphasis::blink, "串口{:^15}已打开!!\n", name);
        return false;
    }
    //互斥信号量
    // std::lock_guard<std::mutex> l(serial_mutex);
    auto terminal_command = fmt::format("echo {}| sudo -S chmod 777 {}", "dji", name);
    int message           = system(terminal_command.c_str());
    if (message < 0) {
        fmt::print(
            fg(fmt::color::red) | fmt::emphasis::bold, "执行命令{}, 失败\n", terminal_command);
    }
    fd = open(name.c_str(), O_RDWR | O_NOCTTY);  //加O_NDELAY，为非阻塞式读取

    if (fd == -1) {
        fmt::print(fg(fmt::color::red) | fmt::emphasis::blink, "打开串口{:^15}失败!!\n", name);
        return false;
    }
    struct termios options;  // termios为类型名的结构
    tcgetattr(fd, &options);
    // 波特率115200, 8N1
    options.c_iflag = IGNPAR;                          //输入模式
    options.c_oflag = 0;                               //输出模式
    options.c_cflag = B460800 | CS8 | CLOCAL | CREAD;  //控制模式
    options.c_lflag = 0;                               //本地模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    //特殊控制模式
    //当设置为 阻塞模式时生效
    options.c_cc[VTIME] = 0;  //最少读取字符数
    options.c_cc[VMIN]  = 1;  //超时时间, 单位: 100ms
    // tcflush(fd, TCIFLUSH);             //清除缓冲区
    tcsetattr(fd, TCSANOW, &options);  //应用上面的设置

    return true;
}

bool Serial::reOpen()
{
    if (isOpen() || closeSerial()) {
        return false;
    }
    return openSerial();
}

bool Serial::closeSerial()
{
    //互斥信号量
    std::lock_guard<std::mutex> fd_lock(serial_mutex);
    int message = close(fd);
    if (message < 0) {
        return false;
    }
    fd = -1;
    return true;
}

bool Serial::isOpen()
{
    return fd > 0;
}

bool Serial::sendData(SendData & send_data)
{
    /*发送数据报
    帧头: 0;
    有无目标: 1;
    yaw轴角度: 2-5;
    pitch轴角度: 6-9;
    递增数据位: 10;
    帧尾: 11;
    */
    send_yaw.f   = send_data.send_yaw;
    send_pitch.f = send_data.send_pitch;
    //帧头、帧尾
    send_buffer_[0]  = frame_header;
    send_buffer_[11] = frame_tail;

    send_buffer_[1] = send_data.goal;

    {
        std::lock_guard<std::mutex> l(data_mutex);
        for (int i = 0; i < 4; i++) {
            send_buffer_[i + 2] = send_yaw.uchars[i];
            send_buffer_[i + 6] = send_pitch.uchars[i];
        }
    }
    send_buffer_[10] = loss;
    loss             = (loss + 1) % 0xff;

    if (write(fd, send_buffer_, 12) == 12) {
    } else {
        fmt::print(fg(fmt::color::red), "串口发送失败!\n");
        return false;
    }

    return true;
}

ReceiveData Serial::getData()
{
    std::lock_guard<std::mutex> data_lock{data_mutex};  //

    ReceiveData data{read_yaw.f, read_pitch.f, last_shoot_speed};
    // fmt::print("[get data]: yaw={}, pitch={}, shoot_speed={}\n", data.yaw, data.pitch, data.shoot_speed);
    return data;
}
bool Serial::noArmour()
{
    SendData send_data{};
    sendData(send_data);
    return true;
}

bool Serial::readSerial()
{
    /*
    1. 读取帧头
    2. 读取其余并判断帧尾是否符合
    */

    /*
    0: 0xff 帧头
    1-4: yaw 偏移角度
    5-8: pitch 偏移角度
    9-12: shoot_speed 射速
    13: 0xfe 帧尾
    */
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    //利用select函数，在tv时刻内检测串口是否有
    if (select(fd + 1, &fds, NULL, NULL, &tv) < 0) {
        return false;
    }
    // 串口句柄锁
    {
        std::lock_guard<std::mutex> fd_lock(serial_mutex);
        //head
        if (read(fd, read_buffer_, 1) != 1) {
            // 读取不到帧头
            // 当为阻塞模式时, 为超时
            return false;
        }
        if (read_buffer_[0] != frame_header) {
            // 帧头验证失败
            return false;
        }
        if (read(fd, read_buffer_ + 1, 13) != 13) {
            // 读取剩余内容失败
            return false;
        }
        if (read_buffer_[13] != frame_tail) {
            // 帧尾验证失败
            return false;
        }
    }
    // 赋值, 数据互斥锁
    {
        std::lock_guard<std::mutex> data_lock{data_mutex};
        for (int i = 0; i < 4; i++) {
            read_yaw.uchars[i]    = read_buffer_[1 + i];
            read_pitch.uchars[i]  = read_buffer_[5 + i];
            shoot_speed.uchars[i] = read_buffer_[9 + i];
        }
        updateShootSpeed(shoot_speed.f);
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(5));  //sleep 5ms
    return true;
}

float Serial::updateShootSpeed(float new_speed)
{
    if (new_speed <= 0) {
        return last_shoot_speed;
    }
    double a         = 0.99;
    new_speed        = a * last_shoot_speed + (1 - a) * new_speed;
    last_shoot_speed = new_speed;
    return last_shoot_speed;
}

}  // namespace Devices