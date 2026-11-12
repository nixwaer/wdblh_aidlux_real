/*
 * Interactive motor debug tool — single-joint control + full-state display.
 *
 * Build:
 *   g++ -std=c++17 -O2 -o test_motor_debug test_motor_debug.cpp pc_protocol.cpp -lpthread
 *
 * Usage:
 *   ./test_motor_debug <port> [baud]
 *   ./test_motor_debug /dev/ttyUSB0 4000000
 *
 * Interactive commands:
 *   s<N>       select motor N (0-11), e.g. "s5"
 *   p<value>   set target position (rad) for selected motor, e.g. "p0.5"
 *   k<Kp>,<Kd> set gains for selected motor, e.g. "k20,2"
 *   d<dir>     set motor direction override: + or -, e.g. "d+" (applied to Pos)
 *   i          idle selected motor (release control)
 *   A          idle ALL motors
 *   h          show help
 *   q / Ctrl-C quit
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
#include <sys/select.h>
#include <sstream>
#include <iomanip>

static volatile bool quit = false;
static void sigHandler(int) { quit = true; }

// ── Terminal raw mode helpers ──────────────────────────────────
static struct termios orig_tio;

static void term_raw()
{
    tcgetattr(STDIN_FILENO, &orig_tio);
    struct termios raw = orig_tio;
    cfmakeraw(&raw);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    setbuf(stdin, NULL);
}

static void term_restore()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_tio);
}

static int kbhit()
{
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    struct timeval tv = {0, 0};
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
}

// ── Serial ─────────────────────────────────────────────────────

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

// ── Joint name mapping ─────────────────────────────────────────

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

// ── Help ────────────────────────────────────────────────────────

static void print_help()
{
    std::cout << "\n"
              << "══════ Interactive Motor Debug ══════\n"
              << "  s<N>        select motor,  e.g. s5\n"
              << "  p<rad>      set target pos, e.g. p0.5\n"
              << "  k<Kp>,<Kd>  set gains,      e.g. k20,2\n"
              << "  d<+/->      direction override\n"
              << "  i           idle this motor\n"
              << "  A           idle ALL motors\n"
              << "  h           this help\n"
              << "  q / Ctrl-C  quit\n"
              << "══════════════════════════════════════\n"
              << std::endl;
}

// ── Main ───────────────────────────────────────────────────────

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <port> [baud]\n"
                  << "Example: " << argv[0] << " /dev/ttyUSB0 4000000\n";
        return 1;
    }

    std::string port = argv[1];
    int baud = (argc >= 3) ? std::stoi(argv[2]) : 4000000;

    signal(SIGINT, sigHandler);
    term_raw();
    atexit(term_restore);

    int fd = openSerial(port, baud);
    if (fd < 0) { term_restore(); return 1; }

    std::cout << "[Motor Debug] " << port << " @ " << baud/1000 << "kbps\n";
    print_help();

    // ── State ───────────────────────────────────────────────
    MotorCommand cmd[PC_MOTOR_NUM];
    pc_make_idle(cmd);

    int   selected_motor = 0;
    float target_pos     = 0.0f;
    float gain_kp        = 20.0f;
    float gain_kd        = 2.0f;
    float dir_override   = 1.0f;  // +1 or -1

    // Send initial idle
    uint8_t cmd_buf[PC_CMD_PACKET_SIZE];
    pc_build_command(cmd_buf, cmd);
    for (int i = 0; i < 5; i++) {
        write(fd, cmd_buf, PC_CMD_PACKET_SIZE);
        usleep(2000);
    }

    // ── Read loop ───────────────────────────────────────────
    uint8_t buf[4096];
    uint8_t pkt_buf[PC_STATUS_PACKET_SIZE];
    int pkt_pos = 0;
    bool synced = false;

    MotorState latest[PC_MOTOR_NUM] = {};
    std::string input_buf;

    auto t_start      = std::chrono::steady_clock::now();
    auto t_last_disp  = t_start;
    auto t_last_cmd   = t_start;
    uint64_t valid_pkts = 0, cmd_sent = 0, crc_err = 0;

    constexpr float GR = 6.33f;

    while (!quit) {
        // ── Read serial ──────────────────────────────────
        ssize_t n = read(fd, buf, sizeof(buf));
        if (n > 0) {
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
                    } else {
                        crc_err++;
                    }
                }
            }
        }

        // ── Read keyboard ─────────────────────────────────
        while (kbhit()) {
            char c;
            if (read(STDIN_FILENO, &c, 1) <= 0) break;

            if (c == '\n' || c == '\r') {
                // ── Parse command ─────────────────────────
                std::string cmd_line = input_buf;
                input_buf.clear();

                if (cmd_line.empty()) continue;

                char op = cmd_line[0];
                std::string arg = cmd_line.substr(1);

                switch (op) {
                case 's': case 'S': {
                    int id = std::atoi(arg.c_str());
                    if (id >= 0 && id < PC_MOTOR_NUM) {
                        selected_motor = id;
                        std::cout << "\n[Motor " << id << " (" << motor_joint[id]
                                  << ") selected]\n";
                    } else {
                        std::cout << "\n[Invalid motor ID: " << id << "]\n";
                    }
                    break;
                }
                case 'p': case 'P': {
                    if (!arg.empty()) {
                        target_pos = std::stof(arg);
                        float cmd_pos = target_pos * dir_override;
                        cmd[selected_motor].q  = cmd_pos;
                        cmd[selected_motor].kp = gain_kp;
                        cmd[selected_motor].kd = gain_kd;
                        std::cout << "\n[Motor " << selected_motor << " ("
                                  << motor_joint[selected_motor]
                                  << ") → pos=" << target_pos << " rad"
                                  << "  Kp=" << gain_kp << "  Kd=" << gain_kd
                                  << "  dir=" << (dir_override > 0 ? '+' : '-')
                                  << "]\n";
                    }
                    break;
                }
                case 'k': case 'K': {
                    // Parse kp,kd
                    size_t comma = arg.find(',');
                    if (comma != std::string::npos) {
                        gain_kp = std::stof(arg.substr(0, comma));
                        gain_kd = std::stof(arg.substr(comma + 1));
                    } else if (!arg.empty()) {
                        gain_kp = std::stof(arg);
                    }
                    // Apply to selected motor
                    cmd[selected_motor].kp = gain_kp;
                    cmd[selected_motor].kd = gain_kd;
                    std::cout << "\n[Gains: Kp=" << gain_kp << "  Kd=" << gain_kd << "]\n";
                    break;
                }
                case 'd': case 'D': {
                    if (!arg.empty() && (arg[0] == '+' || arg[0] == '-')) {
                        dir_override = (arg[0] == '+') ? 1.0f : -1.0f;
                        // Re-apply with new direction
                        cmd[selected_motor].q  = target_pos * dir_override;
                        cmd[selected_motor].kp = gain_kp;
                        cmd[selected_motor].kd = gain_kd;
                        std::cout << "\n[Direction override: "
                                  << (dir_override > 0 ? '+' : '-') << "]\n";
                    }
                    break;
                }
                case 'i': case 'I': {
                    cmd[selected_motor].q  = 0.0f;
                    cmd[selected_motor].kp = 0.0f;
                    cmd[selected_motor].kd = 0.0f;
                    target_pos = 0.0f;
                    std::cout << "\n[Motor " << selected_motor << " ("
                              << motor_joint[selected_motor] << ") idled]\n";
                    break;
                }
                case 'a': {
                    // Idle ALL motors (capital A only)
                    break;
                    // fall through handled below
                }
                case 'A': {
                    pc_make_idle(cmd);
                    target_pos = 0.0f;
                    std::cout << "\n[ALL motors idled]\n";
                    break;
                }
                case 'h': case 'H': {
                    print_help();
                    break;
                }
                case 'q': case 'Q': {
                    quit = true;
                    break;
                }
                default:
                    std::cout << "\n[Unknown command: '" << cmd_line
                              << "'. Press h for help.]\n";
                    break;
                }
            } else {
                input_buf += c;
            }
        }

        // ── Send command at 1kHz ──────────────────────────
        auto now = std::chrono::steady_clock::now();
        if (now - t_last_cmd >= std::chrono::microseconds(1000)) {
            pc_build_command(cmd_buf, cmd);
            write(fd, cmd_buf, PC_CMD_PACKET_SIZE);
            cmd_sent++;
            t_last_cmd += std::chrono::microseconds(1000);
        }

        // ── Display every 200ms ────────────────────────────
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - t_last_disp).count() >= 200) {
            double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_start).count() / 1000.0;
            double rx_rate = valid_pkts / elapsed;

            std::cout << "\033[2J\033[H";  // clear screen

            std::cout << "=== WDBLH Motor Debug ===  " << port << " @ " << baud/1000 << "kbps  "
                      << "RX: " << std::fixed << std::setprecision(0) << rx_rate << " Hz"
                      << "  pkts=" << valid_pkts << "  crc_err=" << crc_err
                      << "  elapsed=" << elapsed << "s\n\n";

            // Full joint table
            std::cout << "    Motor Shaft (raw)                     Joint Output (/6.33)\n";
            std::cout << "ID  Joint      md  Pos(rad)      Vel(rad/s)   Torque(Nm)  │  Pos(rad)      Vel(rad/s)   Torque(Nm)\n";
            std::cout << "──  ─────────  ──  ────────────  ───────────  ───────────  │  ────────────  ───────────  ───────────\n";

            for (int i = 0; i < PC_MOTOR_NUM; i++) {
                // Highlight selected motor
                const char *cursor = (i == selected_motor) ? ">" : " ";
                printf("%s%02d  %-9s  %1d  %+11.6f  %+11.6f  %+11.6f  │  %+11.6f  %+11.6f  %+11.6f\n",
                       cursor,
                       i,
                       motor_joint[i],
                       latest[i].mode,
                       latest[i].q, latest[i].dq, latest[i].tau_est,
                       latest[i].q / GR, latest[i].dq / GR, latest[i].tau_est * GR);
            }

            // Status & command line
            std::cout << "\n── Selected ──────────────────────────────────────────────────────────────────\n";
            std::cout << "Motor " << selected_motor << " (" << motor_joint[selected_motor]
                      << ")  target=" << target_pos << " rad  Kp=" << gain_kp << "  Kd=" << gain_kd
                      << "  dir=" << (dir_override > 0 ? '+' : '-') << "\n";
            std::cout << "Cmd: " << input_buf << "_" << "\n";
            std::cout << "s<id> p<rad> k<kp>,<kd> d<+/-> i=idle A=idle-all q=quit h=help"
                      << std::flush;

            t_last_disp = now;
        }
    }

    // ── Cleanup ─────────────────────────────────────────────
    pc_make_idle(cmd);
    pc_build_command(cmd_buf, cmd);
    for (int i = 0; i < 5; i++) {
        write(fd, cmd_buf, PC_CMD_PACKET_SIZE);
        usleep(2000);
    }
    close(fd);
    term_restore();
    std::cout << "\nDone.\n";
    return 0;
}
