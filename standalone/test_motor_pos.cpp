/*
 * Motor position-control test — POSIX serial (no libserial).
 *
 * Build:
 *   g++ -std=c++17 -o test_motor_pos test_motor_pos.cpp pc_protocol.cpp -lpthread
 *
 * Usage:
 *   ./test_motor_pos /dev/ttyACM1 4000000 <motor_id> <target_pos> [kp] [kd]
 *   ./test_motor_pos /dev/ttyACM1 4000000 3 0.0 20 2
 */

#include "pc_protocol.h"
#include <iostream>
#include <csignal>
#include <unistd.h>
#include <chrono>
#include <fcntl.h>
#include <termios.h>

static volatile bool quit = false;
static void sigHandler(int) { quit = true; }

static int openSerial(const std::string &port, int baud)
{
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "Failed to open: " << port << std::endl;
        return -1;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) { close(fd); return -1; }

    cfmakeraw(&tio);
    tio.c_cflag |= CREAD | CLOCAL;
    tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS | HUPCL);
    tio.c_cflag |= CS8;

    speed_t speed = B4000000;
    switch (baud) {
        case 921600:  speed = B921600;  break;
        case 1000000: speed = B1000000; break;
        case 2000000: speed = B2000000; break;
        case 3000000: speed = B3000000; break;
        case 4000000: speed = B4000000; break;
        default:      speed = B4000000; break;
    }
    cfsetospeed(&tio, speed);
    cfsetispeed(&tio, speed);

    if (tcsetattr(fd, TCSANOW, &tio) != 0) { close(fd); return -1; }
    return fd;
}

int main(int argc, char **argv)
{
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <port> <baud> <motor_id> <target_rad> [kp] [kd]\n"
                  << "Example: " << argv[0] << " /dev/ttyACM1 4000000 3 0.0 20 2\n";
        return 1;
    }

    std::string port = argv[1];
    int baud         = std::stoi(argv[2]);
    int motor_id     = std::stoi(argv[3]);
    float target     = std::stof(argv[4]);
    float kp         = (argc >= 6) ? std::stof(argv[5]) : 20.0f;
    float kd         = (argc >= 7) ? std::stof(argv[6]) : 2.0f;

    signal(SIGINT, sigHandler);

    int fd = openSerial(port, baud);
    if (fd < 0) return 1;

    std::cout << "[PosTest] Motor " << motor_id << " → target=" << target
              << " rad  Kp=" << kp << "  Kd=" << kd
              << "\nPress Ctrl-C to stop.\n\n";

    MotorCommand cmd[PC_MOTOR_NUM];
    pc_make_idle(cmd);

    cmd[motor_id].q  = target;
    cmd[motor_id].kp = kp;
    cmd[motor_id].kd = kd;

    uint8_t cmd_buf[PC_CMD_PACKET_SIZE];
    uint8_t pkt_buf[PC_STATUS_PACKET_SIZE];
    int pkt_pos = 0;
    bool synced = false;

    MotorState latest[PC_MOTOR_NUM] = {};

    auto t_last_disp = std::chrono::steady_clock::now();
    uint64_t cmd_cnt = 0, pkt_cnt = 0;

    while (!quit) {
        // Send command first
        pc_build_command(cmd_buf, cmd);
        write(fd, cmd_buf, PC_CMD_PACKET_SIZE);
        cmd_cnt++;

        // Read status
        uint8_t buf[4096];
        ssize_t n = read(fd, buf, sizeof(buf));
        for (ssize_t i = 0; i < n; i++) {
            if (!synced) {
                if (buf[i] == 0xBB) { synced = true; pkt_pos = 0; pkt_buf[pkt_pos++] = buf[i]; }
                continue;
            }
            pkt_buf[pkt_pos++] = buf[i];
            if (pkt_pos >= PC_STATUS_PACKET_SIZE) {
                pkt_pos = 0; synced = false;
                MotorState parsed[PC_MOTOR_NUM];
                if (pc_parse_status(pkt_buf, parsed) == 0) {
                    pkt_cnt++;
                    for (int j = 0; j < PC_MOTOR_NUM; j++) latest[j] = parsed[j];
                }
            }
        }

        // Display every 200 ms
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - t_last_disp).count() >= 200) {
            auto &m = latest[motor_id];
            float err = target - m.q;
            printf("\r  ID=%d  mode=%d  pos=%.4f rad  vel=%.3f rad/s  err=%.4f  torque=%.3f  pkts=%lu  ",
                   motor_id, m.mode, m.q, m.dq, err, m.tau_est, pkt_cnt);
            std::cout << std::flush;
            t_last_disp = now;
        }
    }

    // Stop motor
    pc_make_idle(cmd);
    pc_build_command(cmd_buf, cmd);
    for (int i = 0; i < 5; i++) { write(fd, cmd_buf, PC_CMD_PACKET_SIZE); usleep(2000); }
    close(fd);
    std::cout << "\nDone.\n";
    return 0;
}
