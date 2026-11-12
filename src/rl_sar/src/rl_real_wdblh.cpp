// Modified 2026-11-11: WDBLH-only extraction and portable configuration.
/*
 * rl_real_wdblh — WDBLH 四足机器人 RL 实物控制器
 * ==============================================
 * 架构: 1kHz 控制循环(RobotControl) + 50Hz RL推理(RunModel)
 * 硬件: DM-IMU(921600bps) + STM32电机总线(4Mbps,12电机) + Web Gamepad Bridge
 * 启动: sudo ./rl_real_wdblh [imu_串口] [电机_串口]
 *       不传端口 = 自动检测: /dev/serial/by-id 优先, 回退扫描 ttyACM/USB
 * 手柄: gamepad_bridge/server.py 通过 localhost UDP 9877 转发浏览器 Gamepad API
 */

#include "rl_real_wdblh.hpp"
#include <cerrno>              // errno
#include <cstring>             // strerror (串口写错误诊断)
#include <cmath>               // isfinite, atan2, asin (倾倒检测)
#include <csignal>             // signal(), _exit()
#include <algorithm>           // std::clamp (关节限位钳制)
#include <filesystem>          // 遍历 /dev/serial/by-id/
#include <sys/mman.h>          // mlockall (锁内存防缺页)
#include <unistd.h>            // usleep, access, readlink
#include <chrono>              // 计时 (steady_clock)
#include <fcntl.h>             // open(), O_RDONLY, O_NONBLOCK
#include <termios.h>           // 串口配置 (cfmakeraw, cfsetospeed)
#include <sys/file.h>          // flock 串口排他锁
#include <poll.h>              // poll() (电机非阻塞读)
#include <sys/socket.h>        // socket (UDP导航)
#include <netinet/in.h>        // sockaddr_in, htons
#include <arpa/inet.h>         // INADDR_ANY
#include <sys/ioctl.h>         // ioctl
#include <linux/serial.h>      // ASYNC_LOW_LATENCY (USB低延迟)
#include <fstream>             // 读 /sys/class 设备描述
#include <iomanip>             // setprecision
#include <string>

namespace {
constexpr uint16_t WEB_GAMEPAD_PORT = 9877;
constexpr int64_t WEB_GAMEPAD_TIMEOUT_MS = 1200;

#pragma pack(push, 1)
struct WebGamepadPacket {
    char magic[4];       // "GPB1"
    uint32_t sequence;
    float axes[4];       // leftX, leftY, rightX, rightY
    uint32_t buttons;    // standard Gamepad mapping, bits 0..16
};
#pragma pack(pop)
static_assert(sizeof(WebGamepadPacket) == 28, "Unexpected WebGamepadPacket size");

constexpr uint32_t GP_A      = 1u << 0;
constexpr uint32_t GP_B      = 1u << 1;
constexpr uint32_t GP_X      = 1u << 2;
constexpr uint32_t GP_Y      = 1u << 3;
constexpr uint32_t GP_LB     = 1u << 4;
constexpr uint32_t GP_RB     = 1u << 5;
constexpr uint32_t GP_DPAD_U = 1u << 12;
constexpr uint32_t GP_DPAD_D = 1u << 13;

int64_t SteadyNowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

float ClampGamepadAxis(float value)
{
    // Match the original /dev/input/js* path: js.value / 32767.0f, with no
    // additional software deadzone or response-curve remapping.
    return std::clamp(value, -1.0f, 1.0f);
}
} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// 1. 构造函数 — 初始化所有硬件, YAML配置, 状态机, 控制线程
// ═══════════════════════════════════════════════════════════════════════════
RL_Real::RL_Real(int argc, char **argv)
{
    // 锁定所有内存页, 防止缺页中断导致控制周期抖动
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // ── 1.1 命令行参数: IMU/电机端口可选, 波特率可选 ──
    if (argc >= 2) imu_port_   = argv[1];
    if (argc >= 3) motor_port_ = argv[2];
    if (argc >= 4) imu_baud_   = std::stoi(argv[3]);
    if (argc >= 5) motor_baud_ = std::stoi(argv[4]);

    // ── 1.2 自动检测 IMU/电机 USB 串口 ──
    {
        auto detect = [&](std::string &port, bool is_imu) -> bool {
            if (!port.empty()) return true; // 命令行已指定, 跳过

            // 策略A: /dev/serial/by-id/ 精确匹配 (不受USB口变化影响)
            const char *byid = "/dev/serial/by-id";
            if (access(byid, F_OK) == 0) {
                for (auto const &entry : std::filesystem::directory_iterator(byid)) {
                    std::string path = entry.path().string();
                    std::string name = entry.path().filename().string();
                    char target[256];
                    ssize_t n = readlink(path.c_str(), target, sizeof(target) - 1);
                    if (n <= 0) continue;
                    target[n] = '\0';
                    std::string dev = std::filesystem::path(target).filename().string();

                    // IMU = DM-Tech DM-IMU-L1
                    if (is_imu && (name.find("DM-IMU") != std::string::npos ||
                                   name.find("DM-Tech") != std::string::npos)) {
                        port = "/dev/" + dev;
                        std::cout << LOGGER::INFO << "[WDBLH] IMU  -> " << port
                                  << "  (" << name << ")" << std::endl;
                        return true;
                    }
                    // 电机 = QinHeng USB Single Serial (1a86)
                    if (!is_imu && (name.find("USB_Single_Serial") != std::string::npos ||
                                    name.find("STM") != std::string::npos ||
                                    name.find("1a86") != std::string::npos)) {
                        port = "/dev/" + dev;
                        std::cout << LOGGER::INFO << "[WDBLH] Motor -> " << port
                                  << "  (" << name << ")" << std::endl;
                        return true;
                    }
                }
            }

            // 策略B: 扫描 /dev/ttyACM* 和 /dev/ttyUSB*, 读 sysfs 描述符
            for (auto prefix : {"/dev/ttyACM", "/dev/ttyUSB"}) {
                for (int i = 0; i < 8; i++) {
                    char tty[32]; snprintf(tty, sizeof(tty), "%s%d", prefix, i);
                    if (access(tty, F_OK) != 0) continue;
                    char clsname[32];
                    if (strstr(prefix, "ACM")) snprintf(clsname, sizeof(clsname), "ttyACM%d", i);
                    else                        snprintf(clsname, sizeof(clsname), "ttyUSB%d", i);
                    std::string desc;
                    for (auto p : {"device/interface", "device/../product"}) {
                        char pp[64]; snprintf(pp, sizeof(pp), "/sys/class/tty/%s/%s", clsname, p);
                        std::ifstream f(pp);
                        if (f.good()) { std::getline(f, desc); break; }
                    }
                    bool is_imu_dev = (desc.find("IMU") != std::string::npos ||
                                       desc.find("DM-") != std::string::npos);
                    if (is_imu == is_imu_dev) {
                        port = tty;
                        std::cout << LOGGER::INFO << "[WDBLH] "
                                  << (is_imu ? "IMU  -> " : "Motor -> ")
                                  << tty << "  (" << desc << ")" << std::endl;
                        return true;
                    }
                }
            }
            return false;
        };

        // 开机时USB可能未就绪, 重试5次 (每次等2秒)
        for (int retry = 0; retry < 5; retry++) {
            if (imu_port_.empty())   detect(imu_port_,   true);
            if (motor_port_.empty()) detect(motor_port_, false);
            if (!imu_port_.empty() && !motor_port_.empty()) break;
            if (retry < 4) {
                std::cout << LOGGER::INFO << "[WDBLH] Waiting for devices... (retry "
                          << retry+1 << ")" << std::endl;
                sleep(2);
            }
        }

        if (imu_port_.empty() || motor_port_.empty())
            throw std::runtime_error("Port detection failed");
    }

    // ── 1.4 ROS (可选) ──

    // ── 1.5 读取 YAML 配置 (base.yaml + config.yaml) ──
    this->ang_vel_axis = "body";
    this->robot_name = "wdblh";
    this->ReadYaml(this->robot_name, "base.yaml");

    // ── 1.6 加载 FSM 状态机 ──
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name)) {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr) this->fsm = *fsm_ptr;
    } else {
        throw std::runtime_error("FSM not found for " + this->robot_name);
    }

    // ── 1.7 初始化 RL 内部结构 ──
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();

    // ── 1.8 初始化 DM IMU (重试3次; 校验四元数范数≈1) ──
    for (int imu_retry = 0; imu_retry < 3; imu_retry++) {
        if (!dm_imu.init(imu_port_, imu_baud_))
            throw std::runtime_error("DM IMU init failed");
        dm_imu.start();
        sleep(1);
        ImuData test = dm_imu.getLatestData();
        float qn = test.quat[0]*test.quat[0] + test.quat[1]*test.quat[1]
                 + test.quat[2]*test.quat[2] + test.quat[3]*test.quat[3];
        if (qn > 0.5f && qn < 1.5f) break;  // 有效的单位四元数
        dm_imu.stop();
        if (!motor_port_.empty()) std::swap(imu_port_, motor_port_); // 可能端口反了
        sleep(2);
    }

    // ── 1.9 初始化电机总线 ──
    MotorInit();

    // ── Web 手柄接收: 仅绑定回环地址, 避免局域网伪造控制包 ──
    gamepad_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (gamepad_fd_ >= 0) {
        struct sockaddr_in gamepad_addr = {};
        gamepad_addr.sin_family = AF_INET;
        gamepad_addr.sin_port = htons(WEB_GAMEPAD_PORT);
        gamepad_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (bind(gamepad_fd_, reinterpret_cast<struct sockaddr*>(&gamepad_addr),
                 sizeof(gamepad_addr)) == 0) {
            gamepad_running_ = true;
            gamepad_thread_ = std::thread(&RL_Real::WebGamepadRecvThread, this);
            std::cout << LOGGER::INFO << "[WDBLH] Web gamepad receiver on 127.0.0.1:"
                      << WEB_GAMEPAD_PORT << std::endl;
        } else {
            std::cout << LOGGER::ERROR << "[WDBLH] Cannot bind web gamepad port "
                      << WEB_GAMEPAD_PORT << ": " << strerror(errno) << std::endl;
            close(gamepad_fd_);
            gamepad_fd_ = -1;
        }
    }

    // ── 1.10 启动控制线程 ──
    this->loop_control = std::make_shared<LoopFunc>(
        "loop_control", this->params.Get<float>("dt"),
        std::bind(&RL_Real::RobotControl, this), ConfiguredCpu("WDBLH_CONTROL_CPU", 4));  // 绑到CPU4, 1kHz控制循环
    this->loop_rl = std::make_shared<LoopFunc>(
        "loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"),
        std::bind(&RL_Real::RunModel, this), ConfiguredCpu("WDBLH_RL_CPU", 5));  // 绑到CPU5, 50Hz RL推理
    this->loop_control->start();
    this->loop_rl->start();

    // ── 1.11 启动 UDP 导航接收 (端口9876, 收ROS Twist格式: 24字节=6*float) ──
    udp_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (udp_fd_ >= 0) {
        struct sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(9876);
        addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(udp_fd_, (struct sockaddr*)&addr, sizeof(addr)) == 0) {
            udp_running_ = true;
            udp_thread_ = std::thread([this]() {
                uint8_t buf[32];
                while (udp_running_) {
                    struct pollfd pfd = {udp_fd_, POLLIN, 0};
                    if (poll(&pfd, 1, 100) > 0) {
                        ssize_t n = recv(udp_fd_, buf, sizeof(buf), 0);
                        if (n >= 24) {  // ROS Twist消息
                            float *f = (float*)buf;
                            udp_vx_.store(f[0], std::memory_order_relaxed);   // linear.x
                            udp_vy_.store(f[1], std::memory_order_relaxed);   // linear.y
                            udp_vyaw_.store(f[5], std::memory_order_relaxed);  // angular.z
                        }
                    }
                }
            });
            std::cout << LOGGER::INFO << "[WDBLH] UDP nav receiver on port 9876" << std::endl;
        } else { close(udp_fd_); udp_fd_ = -1; }
    }

    // ── 1.13 开机预加载 RL 模型 (后台线程) ──
    //    躺地(被动模式)时就把模型加载完, 起立进RL运动不再卡顿
    //    (加载期大量malloc/CUDA初始化会锁竞争卡住1kHz控制环60~90ms)
    {
        std::thread preload_thread([this]() {
            try {
                this->InitRL(this->robot_name + "/himloco");
                std::cout << LOGGER::INFO << "[WDBLH] Model preloaded at startup" << std::endl;
            } catch (const std::exception &e) {
                std::cout << LOGGER::ERROR << "[WDBLH] Model preload failed (will load on demand): "
                          << e.what() << std::endl;
            }
        });
        preload_thread.detach();
    }

    std::cout << LOGGER::INFO << "[WDBLH] RL_Real start complete" << std::endl;
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. 析构函数 — 关闭所有线程和硬件
// ═══════════════════════════════════════════════════════════════════════════
RL_Real::~RL_Real()
{
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    dm_imu.stop();
    MotorDeinit();
    if (gamepad_running_) { gamepad_running_ = false; if (gamepad_thread_.joinable()) gamepad_thread_.join(); }
    if (gamepad_fd_ >= 0) close(gamepad_fd_);
    if (udp_running_) { udp_running_ = false; if (udp_thread_.joinable()) udp_thread_.join(); }
    if (udp_fd_ >= 0) close(udp_fd_);
    std::cout << LOGGER::INFO << "[WDBLH] RL_Real exit" << std::endl;
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. 传感器读取 — IMU(带数据验证) + 电机状态
// ═══════════════════════════════════════════════════════════════════════════
void RL_Real::GetState(RobotState<float> *state)
{
    ImuData imu_data = dm_imu.getLatestData();

    // ── IMU 数据验证 ──
    static ImuData last_valid_imu;
    static bool first_imu = true;
    bool ok = true;
    // NaN/Inf 检查
    for (int i = 0; i < 4; i++) ok = ok && std::isfinite(imu_data.quat[i]);
    for (int i = 0; i < 3; i++) ok = ok && std::isfinite(imu_data.gyro[i]);
    // 四元数范数检查 (有效范围 0.5~1.5)
    float q_norm = imu_data.quat[0]*imu_data.quat[0] + imu_data.quat[1]*imu_data.quat[1]
                 + imu_data.quat[2]*imu_data.quat[2] + imu_data.quat[3]*imu_data.quat[3];
    if (q_norm < 0.5f || q_norm > 1.5f) ok = false;

    if (!ok) {
        static int nan_cnt = 0;
        if (++nan_cnt <= 3)
            std::cout << LOGGER::WARNING << "[IMU] Bad data (q_norm=" << q_norm << "), reusing last valid" << std::endl;
        imu_data = last_valid_imu;
    } else {
        // ── IMU 冻结检测: gyro=0 且 euler 不变 >3秒 → 重连 ──
        static ImuData last_ok = imu_data;
        static int frozen_cnt = 0;
        bool gyro_zero = (imu_data.gyro[0] == 0.0f && imu_data.gyro[1] == 0.0f && imu_data.gyro[2] == 0.0f);
        bool euler_same = (imu_data.euler[0] == last_ok.euler[0] &&
                           imu_data.euler[1] == last_ok.euler[1] &&
                           imu_data.euler[2] == last_ok.euler[2]);
        if (gyro_zero && euler_same && !first_imu) {
            frozen_cnt++;
            bool in_rl = this->rl_init_done;
            if (frozen_cnt > 3000) {  // 3000次×1ms=3秒
                if (!in_rl) {
                    std::cout << LOGGER::WARNING << "[IMU] Frozen 3s, reconnecting..." << std::endl;
                    dm_imu.stop(); usleep(200000);
                    if (dm_imu.init(imu_port_, imu_baud_)) dm_imu.start();
                } else {
                    std::cout << LOGGER::WARNING << "[IMU] Frozen in RL, using last valid" << std::endl;
                    imu_data = last_valid_imu;
                }
                frozen_cnt = 0; first_imu = true;
            }
        } else { frozen_cnt = 0; }
        last_ok = imu_data;
        last_valid_imu = imu_data;
        first_imu = false;
    }

    // 填充机器人状态 (四元数 w,x,y,z + 陀螺仪 rad/s)
    state->imu.quaternion[0] = imu_data.quat[0];
    state->imu.quaternion[1] = imu_data.quat[1];
    state->imu.quaternion[2] = imu_data.quat[2];
    state->imu.quaternion[3] = imu_data.quat[3];
    state->imu.gyroscope[0] = imu_data.gyro[0];
    state->imu.gyroscope[1] = imu_data.gyro[1];
    state->imu.gyroscope[2] = imu_data.gyro[2];

    MotorGetState(state);  // 读电机编码器
}

void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    MotorSetCommand(command);
}

// ── Web Gamepad UDP 接收线程 ──
//    线程只更新原子快照; 按键边沿和控制量在 1kHz 控制线程中处理。
void RL_Real::WebGamepadRecvThread()
{
    while (gamepad_running_.load()) {
        struct pollfd pfd = {gamepad_fd_, POLLIN, 0};
        if (poll(&pfd, 1, 100) <= 0) continue;

        WebGamepadPacket packet{};
        ssize_t n = recv(gamepad_fd_, &packet, sizeof(packet), 0);
        if (n != static_cast<ssize_t>(sizeof(packet)) ||
            std::memcmp(packet.magic, "GPB1", 4) != 0) {
            continue;
        }
        bool valid = true;
        for (float axis : packet.axes) valid = valid && std::isfinite(axis);
        if (!valid) continue;

        std::lock_guard<std::mutex> lock(gamepad_mutex_);
        gamepad_lx_.store(std::clamp(packet.axes[0], -1.0f, 1.0f), std::memory_order_relaxed);
        gamepad_ly_.store(std::clamp(packet.axes[1], -1.0f, 1.0f), std::memory_order_relaxed);
        gamepad_rx_.store(std::clamp(packet.axes[2], -1.0f, 1.0f), std::memory_order_relaxed);
        gamepad_buttons_.store(packet.buttons, std::memory_order_relaxed);
        gamepad_last_rx_ms_.store(SteadyNowMs(), std::memory_order_release);
    }
}

namespace {
Input::Gamepad FaceButtonEvent(uint32_t rising, uint32_t held, uint32_t bit,
                               Input::Gamepad plain, Input::Gamepad with_lb,
                               Input::Gamepad with_rb)
{
    if ((rising & bit) == 0) return Input::Gamepad::None;
    if (held & GP_LB) return with_lb;
    if (held & GP_RB) return with_rb;
    return plain;
}
} // namespace

// 将一帧 Web Gamepad 快照映射成原有 RL/FSM 控制输入。
// 这里只处理按下边沿, 避免浏览器心跳包重复触发状态切换。
static void ApplyWebGamepadButtons(RL_Real *self, uint32_t buttons,
                                    uint32_t rising, bool chord_rising)
{
    Input::Gamepad event = Input::Gamepad::None;
    for (Input::Gamepad candidate : {
            FaceButtonEvent(rising, buttons, GP_A, Input::Gamepad::A,
                            Input::Gamepad::LB_A, Input::Gamepad::RB_A),
            FaceButtonEvent(rising, buttons, GP_B, Input::Gamepad::B,
                            Input::Gamepad::LB_B, Input::Gamepad::RB_B),
            FaceButtonEvent(rising, buttons, GP_X, Input::Gamepad::X,
                            Input::Gamepad::LB_X, Input::Gamepad::RB_X),
            FaceButtonEvent(rising, buttons, GP_Y, Input::Gamepad::Y,
                            Input::Gamepad::LB_Y, Input::Gamepad::RB_Y)}) {
        if (candidate != Input::Gamepad::None) event = candidate;
    }
    if (chord_rising) event = Input::Gamepad::LB_RB;
    if (event != Input::Gamepad::None) self->control.SetGamepad(event);
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. 控制循环 — 1kHz (RobotControl)
//    每毫秒: 读手柄 → 读传感器 → 通信保护 → 状态机 → 发电指令
// ═══════════════════════════════════════════════════════════════════════════
void RL_Real::RobotControl()
{
    this->GetState(&this->robot_state);

    // Snapshot a single gamepad packet; stale snapshots cannot release/arm A.
    int64_t pad_received_ms;
    uint32_t buttons;
    float lx, ly, rx;
    {
        std::lock_guard<std::mutex> lock(gamepad_mutex_);
        pad_received_ms = gamepad_last_rx_ms_.load(std::memory_order_relaxed);
        buttons = gamepad_buttons_.load(std::memory_order_relaxed);
        lx = ClampGamepadAxis(gamepad_lx_.load(std::memory_order_relaxed));
        ly = ClampGamepadAxis(gamepad_ly_.load(std::memory_order_relaxed));
        rx = ClampGamepadAxis(gamepad_rx_.load(std::memory_order_relaxed));
    }
    const int64_t now_ms = SteadyNowMs();
    const int64_t age_ms = now_ms - pad_received_ms;
    const bool connected = gamepad_fd_ >= 0 && pad_received_ms > 0 &&
                           age_ms >= 0 && age_ms <= WEB_GAMEPAD_TIMEOUT_MS;
    bool authorized, just_authorized;
    wdblh::MotorFeedbackGate::Phase phase;
    {
        std::lock_guard<std::mutex> lock(motor_mutex_);
        motor_gate_.Update(now_ms);
        just_authorized = motor_gate_.ObserveGamepad(connected, (buttons & GP_A) != 0,
            pad_received_ms, (buttons & (GP_LB | GP_RB)) != 0);
        authorized = motor_gate_.authorized();
        phase = motor_gate_.phase();
    }
    const bool was_authorized = motor_motion_authorized_.exchange(authorized);
    static auto last_phase = wdblh::MotorFeedbackGate::Phase::WaitingFeedback;
    if (phase != last_phase)
    {
        if (phase == wdblh::MotorFeedbackGate::Phase::WaitingFeedback)
            printf("[MOTOR-WAIT] Feedback lost/invalid; motion revoked, waiting in Passive.\n");
        else if (phase == wdblh::MotorFeedbackGate::Phase::WaitingRelease)
            printf("[MOTOR-READY] Feedback stable 300ms. Passive: release A, then press A again.\n");
        else if (phase == wdblh::MotorFeedbackGate::Phase::WaitingPress)
            printf("[MOTOR-ARM] A released. Passive: press A to GetUp.\n");
        else
            printf("[MOTOR-ARM] Fresh A press accepted; starting from current joint feedback.\n");
        last_phase = phase;
    }
    if ((!authorized && was_authorized) || just_authorized)
    {
        // Drop old RL targets; recovery must never replay a pre-fault trajectory.
        std::vector<float> discarded;
        while (output_dof_pos_queue.try_pop(discarded)) {}
        while (output_dof_vel_queue.try_pop(discarded)) {}
    }

    // ── 4.1 Web 手柄输入 ──
    static uint32_t previous_buttons = 0;
    static bool previous_lb_rb = false;
    static bool was_connected = false;

    if (connected && authorized) {
        uint32_t rising = buttons & ~previous_buttons;
        bool lb_rb = (buttons & (GP_LB | GP_RB)) == (GP_LB | GP_RB);

        this->control.x = -ly;
        this->control.y = -lx;
        this->control.yaw = -rx * 2.0f;
        this->control.lb_pressed = (buttons & GP_LB) != 0;
        this->control.rb_pressed = (buttons & GP_RB) != 0;
        ApplyWebGamepadButtons(this, buttons, rising, lb_rb && !previous_lb_rb);
        if (just_authorized)
        {
            // Only A is accepted on this cycle, even if other buttons are held.
            this->control.current_gamepad = Input::Gamepad::A;
            this->control.current_keyboard = this->control.last_keyboard = Input::Keyboard::None;
            this->control.x = this->control.y = this->control.yaw = 0.0f;
        }

        // 十字键按住时连续调节机身高度, 速度 0.1m/s。
        if (buttons & GP_DPAD_U)      this->control.height += 0.0001f;
        else if (buttons & GP_DPAD_D) this->control.height -= 0.0001f;
        this->control.height = std::clamp(this->control.height, 0.18f, 0.35f);

        previous_buttons = buttons;
        previous_lb_rb = lb_rb;
        was_connected = true;
    } else {
        if (was_connected && !connected) {
            printf("[WEB-GAMEPAD-TIMEOUT] %lldms 无数据, 清零速度指令 "
                   "(x=%.2f y=%.2f yaw=%.2f, state=%s)\n",
                   static_cast<long long>(age_ms), this->control.x, this->control.y,
                   this->control.yaw, this->fsm.GetCurrentStateName().c_str());
        }
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->control.lb_pressed = false;
        this->control.rb_pressed = false;
        // Consume held buttons during motor wait without executing them.
        previous_buttons = connected ? buttons : 0;
        previous_lb_rb = connected && (buttons & (GP_LB | GP_RB)) == (GP_LB | GP_RB);
        was_connected = connected;
    }

    // ── 4.2 通信断连保护 ──
    {
        // Missing feedback is handled by the motor gate, without exiting.
        // 发送看门狗: 读线程正常但连续 2 秒写不出去 → 电机收不到指令，
        // 此时"通信正常"是假象，必须重启而不是继续运行。
        {
            int64_t tx_ok = motor_last_tx_ok_ms_.load(std::memory_order_relaxed);
            if (tx_ok > 0) {
                int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch()).count();
                double tx_stale = (now_ms - tx_ok) / 1000.0;
                if (tx_stale > 2.0) {
                    printf("[FATAL] Motor TX stalled (%.1fs), exiting for restart...\n", tx_stale);
                    fflush(stdout); _exit(1);
                }
            }
        }
        if (!dm_imu.isRunning() && !imu_port_.empty()) {  // IMU线程崩溃→退出
            printf("[FATAL] IMU thread died, exiting for restart...\n");
            fflush(stdout); _exit(1);
        }
    }

    if (!authorized)
    {
        this->control.current_gamepad = this->control.last_gamepad = Input::Gamepad::None;
        this->control.current_keyboard = this->control.last_keyboard = Input::Keyboard::None;
        this->control.model_switch_pending = false;
        this->control.navigation_mode = false;
        this->control.x = this->control.y = this->control.yaw = 0.0f;
        udp_vx_.store(0); udp_vy_.store(0); udp_vyaw_.store(0);
        const int count = this->params.Get<int>("num_of_dofs");
        for (int i = 0; i < count; ++i)
        {
            auto& cmd = this->robot_command.motor_command;
            cmd.q[i] = cmd.dq[i] = cmd.tau[i] = cmd.kp[i] = cmd.kd[i] = 0.0f;
        }
        // Send idle BEFORE resetting FSM (RL Exit may join a loading thread).
        this->SetCommand(&this->robot_command);
        wdblh_fsm::ResetMotorWaitToPassive(*this, &this->robot_state, &this->robot_command);
        static int64_t last_wait_log = -5000;
        if (phase == wdblh::MotorFeedbackGate::Phase::WaitingFeedback && now_ms - last_wait_log >= 5000)
        {
            printf("[MOTOR-WAIT] Waiting for usable 12-motor feedback; zero output, early inputs discarded.\n");
            last_wait_log = now_ms;
        }
        return;
    }

    // ── 4.3 X键切换模型 (himloco1 ↔ himloco2) ──
    if (this->control.current_gamepad == Input::Gamepad::X &&
        this->control.current_gamepad != this->control.last_gamepad) {
        this->control.model_switch_pending = true;
    }

    // ── 4.4 倾倒保护: roll/pitch >60° 持续50ms → 强制趴下 ──
    {
        static int tip_cnt = 0;
        float qw = this->robot_state.imu.quaternion[0];
        float qx = this->robot_state.imu.quaternion[1];
        float qy = this->robot_state.imu.quaternion[2];
        float qz = this->robot_state.imu.quaternion[3];
        // 四元数 → 欧拉角
        float roll  = std::abs(std::atan2(2.0f*(qw*qx + qy*qz), 1.0f - 2.0f*(qx*qx + qy*qy))) * 180.0f / M_PI;
        float pitch = std::abs(std::asin(2.0f*(qw*qy - qz*qx))) * 180.0f / M_PI;
        if (roll > 60.0f || pitch > 60.0f) {
            tip_cnt++;
            if (tip_cnt > 50) {  // 持续50ms才触发(防止误报)
                printf("[TIP-OVER] roll=%.0f pitch=%.0f -- forcing GetDown!\n", roll, pitch);
                fflush(stdout);
                this->fsm.RequestStateChange("RLFSMStateGetDown");
                tip_cnt = 0;
            }
        } else { tip_cnt = 0; }
    }

    // ── 4.5 状态机 (手柄/键盘指令 → FSM 状态切换) ──
    float js_x = this->control.x, js_y = this->control.y, js_yaw = this->control.yaw, js_h = this->control.height;
    this->StateController(&this->robot_state, &this->robot_command);

    // ── FSM 卡住检测: GetUp/GetDown 超15秒无进展 → 大概率电机未上电/机械卡住 ──
    {
        static std::string last_fsm = "";
        static auto last_fsm_change = std::chrono::steady_clock::now();
        std::string cur = this->fsm.GetCurrentStateName();
        if (cur != last_fsm) { last_fsm = cur; last_fsm_change = std::chrono::steady_clock::now(); }
        else if (cur.find("GetUp") != std::string::npos || cur.find("GetDown") != std::string::npos) {
            double dur = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_fsm_change).count();
            if (dur > 15.0) {
                printf("[FSM-STUCK] %s 已 %.0fs 无进展! 检查电机电源/机械\n", cur.c_str(), dur);
                last_fsm_change = std::chrono::steady_clock::now();  // 每15s一条
            }
        }
    }
    // 手柄摇杆覆盖键盘速度值
    this->control.x = js_x; this->control.y = js_y; this->control.yaw = js_yaw; this->control.height = js_h;
    this->control.ClearInput();
    this->SetCommand(&this->robot_command);  // 发送电机指令(292字节, ~1kHz)
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. RL 推理循环 — 50Hz (RunModel)
//    每20ms: 构造观测(46维×6帧历史=276维) → 策略推理 → 输出12维action
// ═══════════════════════════════════════════════════════════════════════════
void RL_Real::RunModel()
{
    rl_heartbeat_.store(std::chrono::steady_clock::now().time_since_epoch().count(), std::memory_order_relaxed);

    // ── RL 频率监控 ──
    static auto last_rl = std::chrono::steady_clock::now();
    auto now_rl = std::chrono::steady_clock::now();
    double rl_dt = std::chrono::duration<double>(now_rl - last_rl).count();
    // RL 亚阈值停摆统计 (>25ms 的推理滑移, 周期20ms; 随RL-FREQ每10s报告)
    static int rl_stall_cnt = 0; static double rl_stall_max = 0.0;
    if (rl_dt > 0.025) { rl_stall_cnt++; rl_stall_max = std::max(rl_stall_max, rl_dt); }
    if (rl_dt > 0.030) {  // >30ms: 限频报警
        static auto last_rl_slow = std::chrono::steady_clock::time_point();
        if (std::chrono::duration<double>(now_rl - last_rl_slow).count() > 10.0) {
            printf("[SLOW-RL] dt=%.0fms\n", rl_dt*1000);
            last_rl_slow = now_rl;
        }
    }
    if (rl_dt > 0.500) printf("[FREEZE-RL] %.1fs!\n", rl_dt);
    last_rl = now_rl;

    // ── RL 频率计数 (每10秒报告) ──
    static int rl_freq_cnt = 0;
    static auto rl_freq_t0 = std::chrono::steady_clock::now();
    rl_freq_cnt++;
    double rl_freq_elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - rl_freq_t0).count();
    if (rl_freq_elapsed >= 10.0) {
        printf("[RL-FREQ] %.1f Hz (elapsed=%.1fs, stalls>25ms: %d, max=%.1fms)\n",
               rl_freq_cnt/rl_freq_elapsed, rl_freq_elapsed, rl_stall_cnt, rl_stall_max*1000);
        rl_freq_cnt = 0; rl_stall_cnt = 0; rl_stall_max = 0.0; rl_freq_t0 = std::chrono::steady_clock::now();
    }

    if (motor_motion_authorized_.load() && this->rl_init_done)
    {
        this->episode_length_buf += 1;

        // ── 构造观测 ──
        this->obs.ang_vel  = this->robot_state.imu.gyroscope;                  // 角速度(3)
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw}; // 速度指令(3)
        this->obs.height   = {this->control.height};                           // 高度指令(1)

        // 导航模式: 手柄指令 → UDP收到的 /cmd_vel 指令
        if (this->control.navigation_mode && udp_fd_ >= 0) {
            float vx = udp_vx_.load(std::memory_order_relaxed);
            float vy = udp_vy_.load(std::memory_order_relaxed);
            float vyaw = udp_vyaw_.load(std::memory_order_relaxed);
            this->obs.commands = {vx, vy, vyaw};
        }

        // 终端显示最终送入 himloco 的速度/高度指令。放在导航覆盖之后，
        // 所以手柄模式和导航模式显示的都是模型真正收到的值。
        static auto last_himloco_cmd_print = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - last_himloco_cmd_print).count() >= 1000) {
            printf("[HIMLOCO-CMD] x:%+.2f y:%+.2f yaw:%+.2f h:%.3f\n",
                   this->obs.commands[0], this->obs.commands[1],
                   this->obs.commands[2], this->obs.height[0]);
            last_himloco_cmd_print = std::chrono::steady_clock::now();
        }

        this->obs.base_quat = this->robot_state.imu.quaternion;  // 姿态四元数(4)
        this->obs.dof_pos   = this->robot_state.motor_state.q;   // 关节位置(12)
        this->obs.dof_vel   = this->robot_state.motor_state.dq;  // 关节速度(12)

        // 首帧: 预热历史缓冲区 (6步×46维=276维)
        if (this->episode_length_buf == 1) {
            auto warmup_obs = this->ComputeObservation();
            for (int i = 0; i < 6; i++) this->history_obs_buf.insert(warmup_obs);
        }

        // ── 策略推理 ──
        this->obs.actions = this->Forward();

        // ── 模型输出冻结检测 ──
        static std::vector<float> last_actions;
        static int frozen_cnt = 0;
        if (last_actions.size() == this->obs.actions.size()) {
            bool same = true;
            for (size_t i = 0; i < this->obs.actions.size(); i++)
                if (std::fabs(this->obs.actions[i] - last_actions[i]) > 0.0001f) { same = false; break; }
            if (same) frozen_cnt++; else frozen_cnt = 0;
            if (frozen_cnt > 10) printf("[FROZEN-OUT] Model output unchanged for %d cycles!\n", frozen_cnt);
        }
        last_actions = this->obs.actions;

        // ── 计算输出 (action → 关节目标位置/速度/力矩) ──
        this->ComputeOutput(this->obs.actions, this->output_dof_pos,
                            this->output_dof_vel, this->output_dof_tau);

        // NaN/Inf 保护
        bool tau_safe = true;
        for (auto v : this->output_dof_tau) { if (!std::isfinite(v)) { tau_safe = false; break; } }
        if (tau_safe && motor_motion_authorized_.load()) {
            if (!this->output_dof_pos.empty()) output_dof_pos_queue.push(this->output_dof_pos);
            if (!this->output_dof_vel.empty()) output_dof_vel_queue.push(this->output_dof_vel);
            // ── 队列积压检测: RL在推但FSM没消费 → 指令滞后/堆积 ──
            static size_t last_q_sz = 0;
            static auto last_q_warn = std::chrono::steady_clock::time_point();
            size_t qsz = this->output_dof_pos_queue.unsafe_size();
            if (qsz > 500 && qsz > last_q_sz &&
                std::chrono::duration<double>(std::chrono::steady_clock::now() - last_q_warn).count() > 10.0) {
                printf("[QUEUE-BACKLOG] pos_queue=%zu (RL 50Hz在推但FSM未消费!)\n", qsz);
                last_q_warn = std::chrono::steady_clock::now();
            }
            last_q_sz = qsz;
        } else if (!tau_safe) {
            static int bad_cnt = 0;
            if (++bad_cnt <= 3) std::cout << LOGGER::WARNING << "[RL] NaN/Inf in output, skip" << std::endl;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. 模型前向 — AidLite/QNN 或显式启用的 ONNX 推理
// ═══════════════════════════════════════════════════════════════════════════
std::vector<float> RL_Real::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);
    if (!lock.owns_lock()) {  // 模型正在重新加载中
        std::cout << LOGGER::WARNING << "Model reinitializing, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();
    std::vector<float> actions;

    try {  // 策略推理异常捕获: 不catch会静默终止整个进程
        if (!this->params.Get<std::vector<int>>("observations_history").empty()) {
            this->history_obs_buf.insert(clamped_obs);
            this->history_obs = this->history_obs_buf.get_obs_vec(
                this->params.Get<std::vector<int>>("observations_history"));
            actions = this->model->forward({this->history_obs});  // 策略推理
        } else {
            actions = this->model->forward({clamped_obs});
        }
    } catch (const std::exception &e) {
        static auto last_inf_err = std::chrono::steady_clock::time_point();
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_inf_err).count() > 10.0) {
            std::cout << LOGGER::ERROR << "[RL-INF-ERR] 推理异常: " << e.what()
                      << " (沿用上一帧动作; 连续出现请查推理后端/模型)" << std::endl;
            last_inf_err = std::chrono::steady_clock::now();
        }
        return this->obs.actions;
    }

    // Action 钳制到 YAML 配置的上下限
    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() &&
        !this->params.Get<std::vector<float>>("clip_actions_lower").empty()) {
        return clamp(actions,
                     this->params.Get<std::vector<float>>("clip_actions_lower"),
                     this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    return actions;
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. 电机通信 — STM32 UART7 协议 (4Mbps, 292字节指令包, 124字节状态包)
// ═══════════════════════════════════════════════════════════════════════════

// GO-M8010-6 减速比: 转子6.33转 = 关节1转
static constexpr float GEAR_RATIO = 6.33f;

void RL_Real::MotorInit()
{
    // ── 打开串口 ──
    // O_NONBLOCK: 4Mbps 下 USB 偶发背压会让阻塞 write 卡住 2 秒(内核 tty
    // write_wait 上限)，直接把 1ms 控制循环冻结 —— 实机上等于"失控 2 秒"。
    // 非阻塞写失败时丢一帧即可，控制器下一毫秒就重发，绝不能让控制环等 IO。
    motor_fd_ = open(motor_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (motor_fd_ < 0) throw std::runtime_error("Motor serial open failed: " + motor_port_);

    // 排他锁：防止 test_motor_raw / 另一个控制器实例抢同一串口。
    // 实测两个进程同时读写同一 tty：字节流被撕碎 → crc_err 73%，
    // 内核 tty 发送缓冲被打满 → 阻塞 write 卡 2 秒 → 控制环冻结，
    // 极端时 CH343 直接 USB 掉线重枚举。所以这里必须硬失败，不能继续。
    if (flock(motor_fd_, LOCK_EX | LOCK_NB) != 0) {
        printf("[FATAL] Motor serial %s is locked by another process "
               "(test_motor_raw? another controller?), exiting.\n", motor_port_.c_str());
        fflush(stdout);
        _exit(1);
    }

    // ── 配置 termios: 原始模式, 8N1 ──
    struct termios tio;
    tcgetattr(motor_fd_, &tio);
    cfmakeraw(&tio);
    tio.c_cflag |= CREAD | CLOCAL;
    tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS | HUPCL);
    tio.c_cflag |= CS8;

    // ── 波特率: 默认4Mbps ──
    speed_t speed = B4000000;
    switch (motor_baud_) {
        case 921600:  speed = B921600;  break;
        case 1000000: speed = B1000000; break;
        case 2000000: speed = B2000000; break;
        case 3000000: speed = B3000000; break;
        default:      speed = B4000000; break;
    }
    cfsetospeed(&tio, speed); cfsetispeed(&tio, speed);
    tcsetattr(motor_fd_, TCSANOW, &tio);

    // USB CDC ACM 低延迟 (默认16ms缓冲 → ~1ms)
    struct serial_struct serial_info;
    if (ioctl(motor_fd_, TIOCGSERIAL, &serial_info) == 0) {
        serial_info.flags |= ASYNC_LOW_LATENCY;
        ioctl(motor_fd_, TIOCSSERIAL, &serial_info);
    }

    // 清空收发缓冲：上一个进程/上次断连残留的半帧会被当成新帧，
    // 启动瞬间刷出几千个假 crc_err，同步器还要花时间重新对齐。
    tcflush(motor_fd_, TCIOFLUSH);


    // ── 发送初始空闲指令 (5次确保收到) ──
    pc_make_idle(motor_command_);
    uint8_t cmd_buf[PC_CMD_PACKET_SIZE];  // 292字节
    pc_build_command(cmd_buf, motor_command_);
    for (int i = 0; i < 5; i++) { write(motor_fd_, cmd_buf, PC_CMD_PACKET_SIZE); usleep(2000); }

    // ── 启动状态读取线程 ──
    motor_running_ = true;
    motor_thread_ = std::thread(&RL_Real::MotorReadThread, this);

    // Do not pretend motors are connected or block gamepad startup. The control
    // loop sends idle while the reader gathers a continuous usable window.
    std::cout << LOGGER::INFO << "[MOTOR-WAIT] Bus opened; waiting for feedback in Passive." << std::endl;
}

// ── 电机状态读取线程 (独立线程, poll非阻塞, 解析0xBB帧头+CRC8) ──
void RL_Real::MotorReadThread()
{
    uint8_t buf[8192]; int buf_head = 0, buf_tail = 0;
    uint8_t pkt_buf[PC_STATUS_PACKET_SIZE];  // 124字节
    int pkt_pos = 0; bool synced = false;
    uint64_t total_bytes = 0, valid_packets = 0, crc_errors = 0;
    auto last_report = std::chrono::steady_clock::now();

    // 电机数据冻结检测 (电机未上电时位置/速度不变)
    auto last_data_change = std::chrono::steady_clock::now();
    float prev_q[PC_MOTOR_NUM] = {}, prev_dq[PC_MOTOR_NUM] = {}, prev_tau[PC_MOTOR_NUM] = {};
    bool prev_init = false;
    // 单电机卡死检测: 每个电机自己的"最后数据变化时刻"
    auto last_q_change = std::chrono::steady_clock::now();  // 占位初始化
    std::chrono::steady_clock::time_point motor_change_t[PC_MOTOR_NUM];
    for (int j = 0; j < PC_MOTOR_NUM; j++) motor_change_t[j] = last_q_change;

    while (motor_running_) {
        // poll() 2ms超时, 释放fd避免和写线程冲突
        int space = (int)sizeof(buf) - buf_tail;
        if (space > 0) {
            struct pollfd pfd; pfd.fd = motor_fd_; pfd.events = POLLIN;
            if (poll(&pfd, 1, 2) > 0 && (pfd.revents & POLLIN)) {
                ssize_t n = read(motor_fd_, buf + buf_tail, space);
                if (n > 0) { total_bytes += n; buf_tail += n; }
            }
        }

        // ── 解析: 0xBB帧头 → 124字节 → CRC8 ──
        while (buf_head < buf_tail) {
            uint8_t byte = buf[buf_head++];
            if (!synced) {
                if (byte == 0xBB) { synced = true; pkt_pos = 0; pkt_buf[pkt_pos++] = byte; }
                continue;
            }
            pkt_buf[pkt_pos++] = byte;
            if (pkt_pos >= PC_STATUS_PACKET_SIZE) {
                pkt_pos = 0; synced = false;
                MotorState parsed[PC_MOTOR_NUM]{};
                if (wdblh::ParseCompleteMotorFeedback(pkt_buf, parsed)) {
                    valid_packets++;
                    const bool usable = wdblh::UsableMotorFeedback(parsed);
                    {
                        std::lock_guard<std::mutex> lock(motor_mutex_);
                        motor_gate_.RecordFeedback(usable, SteadyNowMs());
                        if (usable)
                            for (int j = 0; j < PC_MOTOR_NUM; ++j) motor_state_[j] = parsed[j];
                    }
                    if (!usable) continue;
                    // 冻结检测: 任何电机的位置/速度/力矩变化?
                    bool changed = false;
                    if (prev_init) {
                        for (int j = 0; j < PC_MOTOR_NUM && !changed; j++)
                            if (std::abs(parsed[j].q - prev_q[j]) > 0.001f ||
                                std::abs(parsed[j].dq - prev_dq[j]) > 0.0001f ||
                                std::abs(parsed[j].tau_est - prev_tau[j]) > 0.001f)
                                changed = true;
                        // 单电机级变化跟踪 (卡死检测用)
                        for (int j = 0; j < PC_MOTOR_NUM; j++)
                            if (std::abs(parsed[j].q - prev_q[j]) > 0.001f ||
                                std::abs(parsed[j].dq - prev_dq[j]) > 0.0001f ||
                                std::abs(parsed[j].tau_est - prev_tau[j]) > 0.001f)
                                motor_change_t[j] = std::chrono::steady_clock::now();
                    }
                    if (changed || !prev_init) { last_data_change = std::chrono::steady_clock::now(); prev_init = true; }
                    for (int j = 0; j < PC_MOTOR_NUM; j++)
                        { prev_q[j]=parsed[j].q; prev_dq[j]=parsed[j].dq; prev_tau[j]=parsed[j].tau_est; }
                } else { crc_errors++; }
            }
        }
        // 缓冲压缩
        if (buf_head > 0 && buf_head >= buf_tail) buf_head = buf_tail = 0;
        else if (buf_head > 4096) { memmove(buf, buf+buf_head, buf_tail-buf_head); buf_tail-=buf_head; buf_head=0; }

        // ── 周期状态 (每10秒) ──
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_report).count() >= 10000) {
            static uint64_t last_pkts = 0;
            double frozen_s = std::chrono::duration<double>(now - last_data_change).count();
            // 当前下发指令里的最大 kp / |tau| —— 用来区分"电机该动却没动"和"本来就没使能"
            float active_kp_max = 0.0f, active_tau_max = 0.0f;
            for (int j = 0; j < PC_MOTOR_NUM; j++) {
                active_kp_max  = std::max(active_kp_max,  motor_command_[j].kp);
                active_tau_max = std::max(active_tau_max, std::abs(motor_command_[j].tau));
            }
            // 只报本窗口增量 + 百分比：累计值在启动对齐期会有一次性噪声，
            // 稳态时增量应为 0，这才是判断链路是否健康的指标。
            static uint64_t last_crc = 0;
            uint64_t crc_delta = crc_errors - last_crc;
            last_crc = crc_errors;
            double crc_pct = (valid_packets + crc_errors) > 0
                           ? 100.0 * crc_errors / (valid_packets + crc_errors) : 0.0;
            std::cout << "[WDBLH Motor] bytes=" << total_bytes << " pkts=" << valid_packets
                      << " crc_err=" << crc_errors << " (+" << crc_delta
                      << ", " << std::fixed << std::setprecision(2) << crc_pct << "%)";
            // 机器人静止(未使能/站立不动)时编码器不变是正常现象，只在
            // 控制器正在出力(有 kp 或 tau)时才把"数据不变"当成冻结故障。
            const bool commanding = (active_kp_max > 1e-4f) || (active_tau_max > 1e-4f);
            if (frozen_s > 0.2 && commanding)
                std::cout << " ** FROZEN-" << std::fixed << std::setprecision(0) << frozen_s << "s **";
            if (valid_packets == last_pkts && valid_packets > 0)
                std::cout << " ** USB DISCONNECTED? **";
            // 单电机卡死: 整体在动但个别电机数据>2s不变 → 逐号报出
            if (frozen_s < 2.0) {
                int stuck_list[PC_MOTOR_NUM]; int n_stuck = 0;
                for (int j = 0; j < PC_MOTOR_NUM; j++) {
                    double mf = std::chrono::duration<double>(now - motor_change_t[j]).count();
                    if (mf > 2.0) stuck_list[n_stuck++] = j;
                }
                if (n_stuck > 0) {
                    std::cout << " ** MOTOR-STUCK:";
                    for (int k = 0; k < n_stuck; k++) std::cout << " m" << stuck_list[k];
                    std::cout << " (数据不变>2s, 其他电机在动 — 查该电机供电/接线)";
                }
            }
            std::cout << "\n";
            last_pkts = valid_packets; last_report = now;
        }
    }
}

// ── 电机状态 → 关节值 (减速比 + 方向) ──
void RL_Real::MotorGetState(RobotState<float> *state)
{
    int num_dofs = this->params.Get<int>("num_of_dofs");
    const auto &joint_map = this->params.Get<std::vector<int>>("joint_mapping");
    const auto &motor_dir = this->params.Get<std::vector<int>>("motor_direction");
    std::lock_guard<std::mutex> lock(motor_mutex_);
    for (int i = 0; i < num_dofs; i++) {
        int m = joint_map[i]; float dir = (float)motor_dir[i];
        state->motor_state.q[i]       = motor_state_[m].q       / GEAR_RATIO * dir;
        state->motor_state.dq[i]      = motor_state_[m].dq      / GEAR_RATIO * dir;
        state->motor_state.tau_est[i] = motor_state_[m].tau_est * GEAR_RATIO * dir;
    }
}

// ── 关节指令 → 电机原始值 (带关节限位保护) ──
void RL_Real::MotorSetCommand(const RobotCommand<float> *command)
{
    // ── CMD 频率监控 ──
    static auto last_cmd = std::chrono::steady_clock::now();
    auto now_cmd = std::chrono::steady_clock::now();
    double cmd_dt = std::chrono::duration<double>(now_cmd - last_cmd).count();
    // 亚阈值停摆统计 (>5ms 的周期滑移计数+最大时长, 随CMD-FREQ每10s报告)
    static int stall_cnt = 0; static double stall_max = 0.0;
    if (cmd_dt > 0.005) { stall_cnt++; stall_max = std::max(stall_max, cmd_dt); }
    if (cmd_dt > 0.050) {  // >50ms: 限频报警 (每10s至多1条, 持续性问题持续可见)
        static auto last_slow_print = std::chrono::steady_clock::time_point();
        if (std::chrono::duration<double>(now_cmd - last_slow_print).count() > 10.0) {
            printf("[SLOW-CMD] dt=%.0fms\n", cmd_dt*1000);
            last_slow_print = now_cmd;
        }
    }
    if (cmd_dt > 0.500) printf("[FREEZE-CMD] %.1fs!\n", cmd_dt);
    last_cmd = now_cmd;

    static int freq_cnt = 0; static auto freq_t0 = std::chrono::steady_clock::now();
    freq_cnt++;
    double freq_elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - freq_t0).count();
    if (freq_elapsed >= 10.0) {
        printf("[CMD-FREQ] %.0f Hz (elapsed=%.1fs, count=%d, stalls>5ms: %d, max=%.1fms)\n",
               freq_cnt/freq_elapsed, freq_elapsed, freq_cnt, stall_cnt, stall_max*1000);
        freq_cnt = 0; stall_cnt = 0; stall_max = 0.0; freq_t0 = std::chrono::steady_clock::now();
    }

    int num_dofs = this->params.Get<int>("num_of_dofs");
    const auto &joint_map = this->params.Get<std::vector<int>>("joint_mapping");
    const auto &motor_dir = this->params.Get<std::vector<int>>("motor_direction");
    const auto &def = this->params.Get<std::vector<float>>("default_dof_pos");

    // ── 关节限位: 超出安全范围→钳制到边界 (不冻结上一帧, 避免关节卡死) ──
    std::vector<float> clamped_q(num_dofs);
    for (int i = 0; i < num_dofs; i++) {
        bool is_hip = (i%3==0), is_thigh = (i%3==1);
        float lim = is_hip ? 0.8f : (is_thigh ? 1.5f : 1.8f);  // 髋/大腿/小腿
        float lo = def[i] - lim, hi = def[i] + lim;
        float tgt = command->motor_command.q[i];
        clamped_q[i] = std::clamp(tgt, lo, hi);
        // 只在"真的会出力"时才告警：passive 态 kp=0 且 tau=0，钳制后电机不出力，
        // 此时 target=0 落在 default±lim 之外属于正常现象，不是故障。
        const bool drives_actuator = (command->motor_command.kp[i] > 1e-4f) ||
                                     (std::abs(command->motor_command.tau[i]) > 1e-4f);
        if (drives_actuator && (tgt < lo || tgt > hi)) {
            static long clamp_cnt = 0;
            if (++clamp_cnt <= 10 || clamp_cnt % 1000 == 0)
                std::cout << LOGGER::WARNING << "[CLAMP] joint " << i << " tgt=" << tgt
                          << " -> " << clamped_q[i] << " (lim " << lo << "~" << hi << ")" << std::endl;
        }
    }

    // ── 关节值 → 电机原始值 (减速比6.33 + 方向) ──
    for (int i = 0; i < num_dofs; i++) {
        int m = joint_map[i]; float dir = (float)motor_dir[i];
        if (m < PC_MOTOR_NUM) {
            motor_command_[m].q   = clamped_q[i] * GEAR_RATIO * dir;
            motor_command_[m].dq  = command->motor_command.dq[i] * GEAR_RATIO * dir;
            motor_command_[m].tau = command->motor_command.tau[i] / GEAR_RATIO * dir;
            motor_command_[m].kp  = command->motor_command.kp[i] / (GEAR_RATIO * GEAR_RATIO);
            motor_command_[m].kd  = command->motor_command.kd[i] / (GEAR_RATIO * GEAR_RATIO);
        }
    }

    // ── 构建292字节包并发送 ──
    // Final output interlock, including any stale FSM/RL commands on recovery.
    if (!motor_motion_authorized_.load()) pc_make_idle(motor_command_);
    uint8_t cmd_buf[PC_CMD_PACKET_SIZE];
    pc_build_command(cmd_buf, motor_command_);

    auto t_write0 = std::chrono::steady_clock::now();
    errno = 0;  // 清残留errno (UDP非阻塞收包会留下EAGAIN, 避免误报)
    ssize_t written = write(motor_fd_, cmd_buf, PC_CMD_PACKET_SIZE);
    auto t_write1 = std::chrono::steady_clock::now();
    double write_ms = std::chrono::duration<double, std::milli>(t_write1 - t_write0).count();

    // 写耗时告警 (>2ms 说明USB反压; 限频每10s一条)
    if (write_ms > 2.0) {
        static auto last_ws_print = std::chrono::steady_clock::time_point();
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_ws_print).count() > 10.0) {
            printf("[SLOW-WRITE] %.1fms (ret=%zd, errno=%d)\n", write_ms, written, errno);
            last_ws_print = std::chrono::steady_clock::now();
        }
    }

    if (written == PC_CMD_PACKET_SIZE) {
        motor_last_tx_ok_ms_.store(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count(),
            std::memory_order_relaxed);
    }

    // 写失败处理。
    // 非阻塞 fd 下 EAGAIN/EWOULDBLOCK = 内核 tty 发送缓冲满(USB 背压)，
    // 这是"这一毫秒写不进去"的正常反馈，丢这一帧即可 —— 下一毫秒立刻重发，
    // 绝对不能退回阻塞等待(会冻结 1ms 控制环 2 秒)。只统计、不刷屏。
    if (written != PC_CMD_PACKET_SIZE) {
        static int eagain_cnt = 0, real_err_cnt = 0;
        if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            if (++eagain_cnt <= 3 || eagain_cnt % 1000 == 0)
                printf("[TX-BACKPRESSURE] 发送缓冲满，丢帧 #%d (控制环不受影响)\n", eagain_cnt);
        } else {
            if (++real_err_cnt <= 5)
                printf("[WRITE-ERR] ret=%zd, expected=%d, errno=%d (%s)\n",
                       written, PC_CMD_PACKET_SIZE, errno, strerror(errno));
        }
    }
}

// ── 电机关闭: 发空闲指令后关串口 ──
void RL_Real::MotorDeinit()
{
    motor_running_ = false;
    if (motor_thread_.joinable()) motor_thread_.join();
    if (motor_fd_ >= 0) {
        pc_make_idle(motor_command_);
        uint8_t cmd_buf[PC_CMD_PACKET_SIZE];
        pc_build_command(cmd_buf, motor_command_);
        write(motor_fd_, cmd_buf, PC_CMD_PACKET_SIZE); usleep(50000);
        close(motor_fd_); motor_fd_ = -1;
    }
    motor_motion_authorized_ = false;
}

// ═══════════════════════════════════════════════════════════════════════════
// 8. 主函数 — 信号处理 + 事件循环
// ═══════════════════════════════════════════════════════════════════════════

volatile sig_atomic_t g_shutdown_requested = 0;
void signalHandler(int signum) {
    std::cout << LOGGER::INFO << "Received signal " << signum << ", shutting down..." << std::endl;
    g_shutdown_requested = 1;
}

int main(int argc, char **argv)
{
#ifndef WDBLH_INFERENCE_CONFIGURED
    std::cerr << "Offline build: no inference backend configured. Rebuild with ENABLE_AIDLITE=ON for the supplied AIDEM policy. No devices opened.\n";
    return 2;
#endif
    signal(SIGINT, signalHandler);
    // 终端窗口/管道断开时忽略 SIGPIPE/SIGHUP — 控制器必须继续运行
    // (实测: 窗口关闭会沿 bash→sudo→程序 静默杀死控制器)
    signal(SIGPIPE, SIG_IGN);
    signal(SIGHUP, SIG_IGN);
    // stdout/stderr 非阻塞: 终端/管道背压时丢弃日志, 绝不让打印卡住1kHz控制环
    // (否则 每2秒的IMU打印 在终端渲染慢时 会阻塞5~8ms 造成电机间歇顿挫)
    fcntl(STDOUT_FILENO, F_SETFL, fcntl(STDOUT_FILENO, F_GETFL) | O_NONBLOCK);
    fcntl(STDERR_FILENO, F_SETFL, fcntl(STDERR_FILENO, F_GETFL) | O_NONBLOCK);
    RL_Real rl_sar(argc, argv);  // 构造函数阻塞直到所有硬件就绪
    while (!g_shutdown_requested) { sleep(1); }  // 主线程等待 Ctrl+C
    std::cout << LOGGER::INFO << "Exiting..." << std::endl;
    return 0;
}
