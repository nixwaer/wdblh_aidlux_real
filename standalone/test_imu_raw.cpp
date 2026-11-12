#include "dm_imu_raw.h"
#include <iostream>
#include <csignal>
#include <unistd.h>

static volatile bool quit = false;
static void sigHandler(int) { quit = true; }

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <port> [baudrate]\n";
        return 1;
    }
    std::string port = argv[1];
    int baud = (argc >= 3) ? std::stoi(argv[2]) : 921600;
    signal(SIGINT, sigHandler);

    DmImuRaw imu;
    if (!imu.init(port, baud)) {
        std::cerr << "Failed to init IMU\n";
        return 1;
    }
    imu.start();
    std::cout << "IMU started. Move the IMU around...\n\n";

    while (!quit) {
        ImuData d = imu.getLatestData();
        std::cout << "[IMU] quat: " << d.quat[0] << " " << d.quat[1] << " " << d.quat[2] << " " << d.quat[3]
                  << " | gyro: " << d.gyro[0] << " " << d.gyro[1] << " " << d.gyro[2]
                  << " | euler: " << d.euler[0] << " " << d.euler[1] << " " << d.euler[2]
                  << std::endl;
        usleep(500000);  // 2Hz
    }
    imu.stop();
    std::cout << "Done.\n";
    return 0;
}
