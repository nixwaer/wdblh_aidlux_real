/*
 * DM-series IMU binary protocol parser (Linux termios backend).
 * Independent-frame protocol: 55 AA ID REG | 12B float[3] | CRC16 | term(0A/0D)
 * Verified against live IMU with sniff_imu.
 */
#include "dm_imu_raw.h"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <cmath>
#include <fcntl.h>
#include <iostream>
#include <iomanip>
#include <poll.h>
#include <stdexcept>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

// ── CRC16 lookup table ────────────────────────────────────────
static const uint16_t CRC16_TABLE[256] = {
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
    0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
    0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
    0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
    0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,
    0x5AF5,0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,0xDBFD,0xCBDC,0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,
    0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
    0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
    0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
    0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
    0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
    0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
    0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
    0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
    0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0,
};

uint16_t DmImuRaw::crc16(uint8_t* ptr, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i) {
        uint8_t idx = (crc >> 8) ^ ptr[i];
        crc = (crc << 1) ^ CRC16_TABLE[idx];
    }
    return crc;
}

void DmImuRaw::eulerToQuat(float roll, float pitch, float yaw, float q[4])
{
    double cr = std::cos(roll  / 180.0 * M_PI * 0.5);
    double sr = std::sin(roll  / 180.0 * M_PI * 0.5);
    double cp = std::cos(pitch / 180.0 * M_PI * 0.5);
    double sp = std::sin(pitch / 180.0 * M_PI * 0.5);
    double cy = std::cos(yaw   / 180.0 * M_PI * 0.5);
    double sy = std::sin(yaw   / 180.0 * M_PI * 0.5);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

DmImuRaw::DmImuRaw() = default;
DmImuRaw::~DmImuRaw() { stop(); }

namespace {

speed_t baudToTermios(int baud)
{
    switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default:
        throw std::runtime_error("unsupported IMU baud rate: " + std::to_string(baud));
    }
}

bool writeAll(int fd, const uint8_t* data, size_t length)
{
    size_t written = 0;
    while (written < length) {
        const ssize_t count = ::write(fd, data + written, length - written);
        if (count > 0) {
            written += static_cast<size_t>(count);
        } else if (count < 0 && errno == EINTR) {
            continue;
        } else {
            return false;
        }
    }
    return true;
}

} // namespace

void DmImuRaw::initSerial()
{
    const speed_t speed = baudToTermios(baud_);
    const int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd < 0)
        throw std::runtime_error("Unable to open imu serial port " + port_ + ": " + std::strerror(errno));

    termios tty{};
    if (::tcgetattr(fd, &tty) != 0) {
        const std::string error = std::strerror(errno);
        ::close(fd);
        throw std::runtime_error("Unable to read IMU serial settings: " + error);
    }

    ::cfmakeraw(&tty);
    if (::cfsetispeed(&tty, speed) != 0 || ::cfsetospeed(&tty, speed) != 0) {
        const std::string error = std::strerror(errno);
        ::close(fd);
        throw std::runtime_error("Unable to set IMU baud rate: " + error);
    }
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB);
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif
    // poll() supplies a 20 ms timeout in readThread(); reads then return
    // immediately with whatever bytes the UART driver has buffered.
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
        const std::string error = std::strerror(errno);
        ::close(fd);
        throw std::runtime_error("Unable to configure IMU serial port: " + error);
    }
    ::tcflush(fd, TCIOFLUSH);
    serial_fd_ = fd;
    std::cout << "[DmImu] Serial Port initialized on " << port_ << std::endl;
}

// ── Config commands: 55 AA 01 REG [data] CRC16 0A, each sent 5× ──
static void sendFrame(int fd, uint8_t reg, const uint8_t* payload, int payload_len)
{
    uint8_t buf[32];
    buf[0] = 0x55; buf[1] = 0xAA; buf[2] = 0x01; buf[3] = reg;
    if (payload && payload_len > 0) memcpy(buf+4, payload, payload_len);
    uint16_t crc = DmImuRaw::crc16(buf, 4 + payload_len);
    buf[4 + payload_len] = crc & 0xFF;
    buf[5 + payload_len] = crc >> 8;
    buf[6 + payload_len] = 0x0A;
    for (int i = 0; i < 5; i++) {
        if (!writeAll(fd, buf, static_cast<size_t>(7 + payload_len)))
            throw std::runtime_error("failed to write IMU configuration frame");
        usleep(10000);
    }
}

void DmImuRaw::enterSettingMode() { sendFrame(serial_fd_, 0x06, nullptr, 0); }
void DmImuRaw::turnOnAccel()      { uint8_t d[]={0x14}; sendFrame(serial_fd_, 0x01, d, 1); }
void DmImuRaw::turnOnGyro()       { uint8_t d[]={0x15}; sendFrame(serial_fd_, 0x01, d, 1); }
void DmImuRaw::turnOnEuler()      { uint8_t d[]={0x16}; sendFrame(serial_fd_, 0x01, d, 1); }
void DmImuRaw::turnOffQuat()      { uint8_t d[]={0x07}; sendFrame(serial_fd_, 0x01, d, 1); }
void DmImuRaw::setOutput1000Hz()  { uint8_t d[]={0x01,0x00}; sendFrame(serial_fd_, 0x02, d, 2); }
void DmImuRaw::saveImuPara()      { sendFrame(serial_fd_, 0x03, nullptr, 0); }
void DmImuRaw::exitSettingMode()  { sendFrame(serial_fd_, 0x06, nullptr, 0); }

bool DmImuRaw::init(const std::string& port, int baud)
{
    port_ = port;
    baud_ = baud;
    try { initSerial(); }
    catch (const std::exception& e) { std::cerr << "[DmImu] " << e.what() << std::endl; return false; }

    try {
        enterSettingMode();   usleep(20000);
        turnOnAccel();        usleep(20000);
        turnOnGyro();         usleep(20000);
        turnOnEuler();        usleep(20000);
        turnOffQuat();        usleep(20000);
        setOutput1000Hz();    usleep(20000);
        saveImuPara();        usleep(20000);
        exitSettingMode();    usleep(100000);
    } catch (const std::exception& e) {
        std::cerr << "[DmImu] " << e.what() << std::endl;
        stop();
        return false;
    }

    std::cout << "[DmImu] init complete on " << port_ << std::endl;
    return true;
}

void DmImuRaw::start()
{
    if (running_) return;
    stop_thread_ = false;
    running_ = true;
    read_thread_ = std::thread(&DmImuRaw::readThread, this);
}

void DmImuRaw::stop()
{
    stop_thread_ = true;
    if (read_thread_.joinable()) read_thread_.join();
    running_ = false;
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }
}

// ── Read thread: independent 19B frames (55 AA ID REG | 12B data | CRC16 | 0A/0D) ──
void DmImuRaw::readThread()
{
    try {
    uint8_t buf[256];
    int buf_pos = 0;
    uint64_t total_bytes = 0, valid_frames = 0;
    auto last_report = std::chrono::steady_clock::now();

    while (!stop_thread_) {
        pollfd pfd{serial_fd_, POLLIN, 0};
        const int poll_result = ::poll(&pfd, 1, 20);
        if (poll_result < 0) {
            if (errno == EINTR) continue;
            throw std::runtime_error("IMU serial poll failed: " + std::string(std::strerror(errno)));
        }
        if (poll_result == 0) continue;
        if (!(pfd.revents & (POLLIN | POLLPRI))) {
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
                throw std::runtime_error("IMU serial device disconnected");
            continue;
        }
        const ssize_t n = ::read(serial_fd_, buf + buf_pos, sizeof(buf) - static_cast<size_t>(buf_pos));
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            throw std::runtime_error("IMU serial read failed: " + std::string(std::strerror(errno)));
        }
        if (n == 0) continue;
        total_bytes += static_cast<uint64_t>(n);
        buf_pos += static_cast<int>(n);

        for (int i = 0; i <= buf_pos - 5; ) {
            if (buf[i] != 0x55) { i++; continue; }
            if (buf[i+1] != 0xAA) { i++; continue; }

            uint8_t reg = buf[i+3];
            uint8_t term = (i + 19 <= buf_pos) ? buf[i+18] : 0;
            int frame_len = 0;
            // Accept both 0x0A and 0x0D terminators
            if (i + 19 <= buf_pos && (buf[i+18] == 0x0A || buf[i+18] == 0x0D)) frame_len = 19;
            if (frame_len == 0) { i++; continue; }
            if (i + frame_len > buf_pos) break;

            // Verify CRC16 (over header 4B + payload 12B = 16B)
            uint16_t calc_crc = crc16(buf + i, 16);
            uint16_t frame_crc = buf[i+16] | (buf[i+17] << 8);
            if (calc_crc != frame_crc) { i++; continue; }

            valid_frames++;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                uint8_t* payload = buf + i + 4;
                if (reg == 0x01) memcpy(latest_data_.accel, payload, 12);
                else if (reg == 0x02) memcpy(latest_data_.gyro, payload, 12);
                else if (reg == 0x03) { memcpy(latest_data_.euler, payload, 12); eulerToQuat(latest_data_.euler[0], latest_data_.euler[1], latest_data_.euler[2], latest_data_.quat); }
                else if (reg == 0x04) memcpy(latest_data_.quat, payload, 12);
            }
            frame_count_++;
            i += frame_len;
        }

        if (buf_pos > 128) { memmove(buf, buf + buf_pos - 64, 64); buf_pos = 64; }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_report).count() >= 5000) {
            std::cout << "[DmImu] bytes=" << total_bytes
                      << " frames=" << valid_frames
                      << " gyro=[" << std::fixed << std::setprecision(2) << latest_data_.gyro[0] << "," << latest_data_.gyro[1] << "," << latest_data_.gyro[2] << "]"
                      << " euler=[" << latest_data_.euler[0] << "," << latest_data_.euler[1] << "," << latest_data_.euler[2] << "]"
                      << std::endl;
            last_report = now;
        }
    }
    } catch (const std::exception &e) {
        std::cerr << "[DmImu] readThread crashed: " << e.what() << std::endl;
    }
}

ImuData DmImuRaw::getLatestData()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_data_;
}
