/*
 * Test program for STM32 UART7 motor bus — POSIX serial (no libserial).
 *
 * Build:
 *   g++ -std=c++17 -o test_motor_raw test_motor_raw.cpp pc_protocol.cpp -lpthread
 *
 * Usage:
 *   ./test_motor_raw <port> [baud]
 *   ./test_motor_raw /dev/ttyUSB0 4000000
 */

#include "pc_protocol.h"
#include <iostream>
#include <csignal>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <sys/file.h>

static volatile bool quit = false;
static void sigHandler(int) { quit = true; }

static int openSerial(const std::string &port, int baud)
{
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) { std::cerr << "Failed to open: " << port << std::endl; return -1; }

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
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <motor_port> [baud]\n"
                  << "Example: " << argv[0] << " /dev/ttyUSB0 4000000\n";
        return 1;
    }

    std::string port = argv[1];
    int baud = (argc >= 3) ? std::stoi(argv[2]) : 4000000;

    signal(SIGINT, sigHandler);

    int fd = openSerial(port, baud);
    if (fd < 0) return 1;

    // 排他锁：控制器 wdblh.service 在跑时绝不能同时打开同一串口。
    // 两个进程抢同一个 tty 会把字节流撕碎(crc_err 飙到 70~90%)，
    // 并打满内核发送缓冲导致对方 write 卡 2 秒、甚至 CH343 USB 掉线重枚举。
    if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
        std::cerr << "\n[FATAL] " << port << " 已被其他进程占用（wdblh.service 在跑？）\n"
                  << "        请先执行: sudo systemctl stop wdblh\n" << std::endl;
        close(fd);
        return 1;
    }

    std::cout << "[Motor] Serial opened on " << port << " @ " << baud << " bps\n";

    // Send initial idle command (5x)
    uint8_t cmd_buf[PC_CMD_PACKET_SIZE];
    MotorCommand cmd[PC_MOTOR_NUM];
    pc_make_idle(cmd);
    pc_build_command(cmd_buf, cmd);

    for (int i = 0; i < 5; i++) {
        write(fd, cmd_buf, PC_CMD_PACKET_SIZE);
        usleep(2000);
    }
    std::cout << "[Motor] Idle command sent, waiting for status...\n";

    // Read + display loop
    uint8_t buf[4096];
    uint8_t pkt_buf[PC_STATUS_PACKET_SIZE];
    int pkt_pos = 0;
    bool synced = false;

    uint64_t total_bytes = 0, valid_pkts = 0, crc_err = 0;
    uint64_t cmd_sent = 0;
    MotorState latest[PC_MOTOR_NUM] = {};

    auto t_start = std::chrono::steady_clock::now();
    auto t_last_display = t_start;
    auto t_last_cmd = t_start;
    auto t_last_valid = t_start;

    std::cout << "\nMonitoring motor bus (Ctrl-C to stop)...\n\n";

    while (!quit)
    {
        // Read
        ssize_t n = read(fd, buf, sizeof(buf));
        if (n > 0) {
            total_bytes += n;
            for (ssize_t i = 0; i < n; i++) {
                uint8_t byte = buf[i];

                if (!synced) {
                    if (byte == 0xBB) {
                        synced = true;
                        pkt_pos = 0;
                        pkt_buf[pkt_pos++] = byte;
                    }
                    continue;
                }

                pkt_buf[pkt_pos++] = byte;
                if (pkt_pos >= PC_STATUS_PACKET_SIZE) {
                    pkt_pos = 0;
                    synced = false;

                    MotorState parsed[PC_MOTOR_NUM];
                    if (pc_parse_status(pkt_buf, parsed) == 0) {
                        valid_pkts++;
                        memcpy(latest, parsed, sizeof(latest));
                        t_last_valid = std::chrono::steady_clock::now();
                    } else {
                        crc_err++;
                    }
                }
            }
        }

        // Stall detector: warn if no valid packet for >1s
        auto now = std::chrono::steady_clock::now();
        double stall_sec = std::chrono::duration<double>(now - t_last_valid).count();
        static double last_stall_warn = 0;
        static int stall_count = 0;
        if (stall_sec > 1.0 && stall_sec - last_stall_warn > 1.0) {
            stall_count++;
            printf("\n*** STALL #%d! No motor data for %.1fs ***\n", stall_count, stall_sec);
            last_stall_warn = stall_sec;
        }

        // Send command at 1kHz (fixed schedule)
        if (now - t_last_cmd >= std::chrono::microseconds(1000)) {
            write(fd, cmd_buf, PC_CMD_PACKET_SIZE);
            cmd_sent++;
            t_last_cmd += std::chrono::microseconds(1000);
        }

        // Display every 200ms
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - t_last_display).count() >= 200) {
            double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_start).count() / 1000.0;
            double rx_rate = valid_pkts / elapsed;
            double tx_rate = cmd_sent / elapsed;

            std::cout << "\033[2J\033[H";
            std::cout << "=== WDBLH Motor Bus ===  " << port << " @ " << baud/1000 << "kbps  "
                      << "RX: " << rx_rate << " Hz  TX: " << tx_rate << " Hz  "
                      << "pkts=" << valid_pkts << "  crc_err=" << crc_err
                      << "  elapsed=" << elapsed << "s\n\n";

            constexpr float GR = 6.33f;
            // Joint mapping for WDBLH (training order FL,FR,RL,RR)
            const char* motor_joint[PC_MOTOR_NUM] = {
                "RL_calf",    // m0
                "FR_calf",    // m1
                "RL_hip",     // m2
                "FR_thigh",   // m3
                "FL_calf",    // m4
                "RR_thigh",   // m5
                "FL_hip",     // m6
                "FL_thigh",   // m7
                "RL_thigh",   // m8
                "FR_hip",     // m9
                "RR_hip",     // m10
                "RR_calf",    // m11
            };

            std::cout << "    Motor Shaft (raw)                     Joint Output (/6.33)\n";
            std::cout << "ID  Joint      md  Pos(rad)      Vel(rad/s)   Torque(Nm)  │  Pos(rad)      Vel(rad/s)   Torque(Nm)\n";
            std::cout << "──  ─────────  ──  ────────────  ───────────  ───────────  │  ────────────  ───────────  ───────────\n";

            for (int i = 0; i < PC_MOTOR_NUM; i++) {
                printf(" %02d  %-9s  %1d  %+11.6f  %+11.6f  %+11.6f  │  %+11.6f  %+11.6f  %+11.6f\n",
                       i,
                       motor_joint[i],
                       latest[i].mode,
                       latest[i].q, latest[i].dq, latest[i].tau_est,
                       latest[i].q / GR, latest[i].dq / GR, latest[i].tau_est * GR);
            }

            std::cout << "\nPress Ctrl-C to stop." << std::flush;
            t_last_display = now;
        }
    }

    // Cleanup
    pc_make_idle(cmd);
    pc_build_command(cmd_buf, cmd);
    write(fd, cmd_buf, PC_CMD_PACKET_SIZE);
    usleep(50000);
    close(fd);
    std::cout << "\nDone.\n";
    return 0;
}
