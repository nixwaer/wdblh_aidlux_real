/*
 * DM-series IMU protocol driver.
 * Independent-frame protocol: 55 AA ID REG | 12B float[3] | CRC16 | term
 * The serial transport uses Linux termios directly and does not depend on
 * ROS or the wjwwood serial package.
 */
#ifndef DM_IMU_RAW_H
#define DM_IMU_RAW_H

#include <string>
#include <atomic>
#include <mutex>
#include <thread>
#include <cstdint>

struct ImuData
{
    float quat[4]  = {1.0f, 0.0f, 0.0f, 0.0f};
    float gyro[3]  = {0.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, 0.0f, 0.0f};
    float euler[3] = {0.0f, 0.0f, 0.0f};
};

class DmImuRaw
{
public:
    DmImuRaw();
    ~DmImuRaw();

    bool init(const std::string &port, int baud = 921600);
    void start();
    void stop();
    bool isRunning() const { return running_; }
    ImuData getLatestData();

    static uint16_t crc16(uint8_t *ptr, uint16_t len);
    static void eulerToQuat(float roll, float pitch, float yaw, float q[4]);

private:
    void initSerial();
    void readThread();

    void enterSettingMode();
    void turnOnAccel();
    void turnOnGyro();
    void turnOnEuler();
    void turnOffQuat();
    void setOutput1000Hz();
    void saveImuPara();
    void exitSettingMode();

    std::string port_;
    int        baud_ = 921600;
    int        serial_fd_ = -1;

    std::thread        read_thread_;
    std::atomic<bool>  running_{false};
    std::atomic<bool>  stop_thread_{false};

    mutable std::mutex data_mutex_;
    ImuData            latest_data_;
    uint64_t frame_count_ = 0;
};

#endif
