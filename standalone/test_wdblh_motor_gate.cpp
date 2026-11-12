// Offline regression checks: synthetic feedback/buttons, no serial or controller.
#include "wdblh_motor_gate.hpp"
#include <array>
#include <iostream>
#include <limits>
#include <stdexcept>

using Gate = wdblh::MotorFeedbackGate;
static void require(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}
static void feedback_until(Gate& gate, int64_t first, int64_t last)
{
    for (int64_t t = first; t <= last; t += 10)
    {
        gate.RecordFeedback(true, t);
        gate.Update(t);
    }
}
static void arm(Gate& gate, int64_t now)
{
    require(!gate.ObserveGamepad(true, false, now + 1), "release must not authorize");
    require(gate.ObserveGamepad(true, true, now + 2), "new A press must authorize");
    require(gate.authorized(), "authorized after release and press");
}

static void test_wait_and_rearm()
{
    Gate gate;
    // An hour without motor power, including early presses, must keep waiting.
    for (int64_t t = 0; t <= 3600000; t += 100)
    {
        gate.Update(t);
        gate.ObserveGamepad(true, t % 200 == 0, t);
        require(!gate.ready() && !gate.authorized(), "no feedback must stay blocked");
    }
    feedback_until(gate, 3600010, 3600300);
    require(!gate.ready(), "less than 300ms feedback must not be ready");
    feedback_until(gate, 3600310, 3600310);
    require(gate.ready() && !gate.authorized(), "stable static feedback is ready but not armed");
    gate.ObserveGamepad(true, false, 3600309);
    require(gate.phase() == Gate::Phase::WaitingRelease, "old released snapshot cannot arm");
    for (int i = 1; i <= 3; ++i) gate.ObserveGamepad(true, true, 3600310 + i);
    require(!gate.authorized(), "held A through power-up must not authorize");
    arm(gate, 3600320);

    // Feedback timeout cancels motion, and resumed feedback cannot replay A.
    gate.Update(3600411);
    require(!gate.authorized() && !gate.ready(), "100ms feedback loss must revoke motion");
    feedback_until(gate, 3600420, 3600710);
    require(!gate.ready(), "recovery also needs 300ms");
    feedback_until(gate, 3600720, 3600720);
    gate.ObserveGamepad(true, true, 3600721);
    require(!gate.authorized(), "old held A cannot resume motion");
    arm(gate, 3600730);

    gate.RecordFeedback(false, 3600740);
    gate.Update(3600740);
    require(!gate.authorized(), "placeholder frame must revoke motion immediately");
    feedback_until(gate, 3600750, 3601050);
    gate.ObserveGamepad(true, false, 3601051);
    gate.ObserveGamepad(false, false, 3601052);
    gate.ObserveGamepad(true, true, 3601053);
    require(!gate.authorized(), "disconnect must not count as an A release");
    gate.ObserveGamepad(true, false, 3601054);
    gate.ObserveGamepad(true, true, 3601055, true);
    require(!gate.authorized(), "A with LB/RB must not arm");
    arm(gate, 3601060);
}

static void test_hidden_gap()
{
    Gate gate;
    feedback_until(gate, 0, 300);
    arm(gate, 300);
    // Even if the control loop missed a loss and recovery, the reader epoch
    // prevents the old authorization from surviving the interrupted window.
    for (int64_t t = 500; t <= 800; t += 10) gate.RecordFeedback(true, t);
    gate.Update(800);
    require(gate.ready() && !gate.authorized(), "reader-observed gap must reset authorization");
}

static void test_protocol()
{
    PC_StatusPacket packet{};
    packet.head = 0xBB; packet.cmd = 0x02; packet.num = PC_MOTOR_NUM;
    for (int i = 0; i < PC_MOTOR_NUM; ++i)
    {
        packet.motors[i].id = i;
        packet.motors[i].Pos = (i + 1) * 1000;
    }
    auto bytes = reinterpret_cast<uint8_t*>(&packet);
    auto crc = [&] { packet.crc = pc_crc8(bytes, sizeof(packet) - 1); };
    MotorState parsed[PC_MOTOR_NUM]{};
    crc();
    require(wdblh::ParseCompleteMotorFeedback(bytes, parsed), "complete packet must parse");
    require(wdblh::UsableMotorFeedback(parsed), "static nonzero feedback must be usable");
    packet.crc ^= 1;
    require(!wdblh::ParseCompleteMotorFeedback(bytes, parsed), "bad CRC must be rejected");
    packet.motors[1].id = 0; crc();
    require(!wdblh::ParseCompleteMotorFeedback(bytes, parsed), "duplicate/missing motor ID must be rejected");
    packet.motors[1].id = 12; crc();
    require(!wdblh::ParseCompleteMotorFeedback(bytes, parsed), "out-of-range ID must be rejected");
    packet.motors[1].id = 1; packet.num = 11; crc();
    require(!wdblh::ParseCompleteMotorFeedback(bytes, parsed), "wrong motor count must be rejected");
    packet.num = 12; packet.cmd = 3; crc();
    require(!wdblh::ParseCompleteMotorFeedback(bytes, parsed), "wrong packet type must be rejected");
    packet.cmd = 2;
    for (auto& motor : packet.motors) motor.Pos = 0;
    crc();
    require(wdblh::ParseCompleteMotorFeedback(bytes, parsed), "zero packet can have valid CRC");
    require(!wdblh::UsableMotorFeedback(parsed), "zero placeholder must not count as motor ready");
    parsed[0].q = 1;
    require(wdblh::UsableMotorFeedback(parsed), "individual zero position is legitimate");
    parsed[2].dq = std::numeric_limits<float>::quiet_NaN();
    require(!wdblh::UsableMotorFeedback(parsed), "NaN must be rejected");
    parsed[2].dq = std::numeric_limits<float>::infinity();
    require(!wdblh::UsableMotorFeedback(parsed), "Inf must be rejected");
}

int main()
{
    try
    {
        test_wait_and_rearm(); test_hidden_gap(); test_protocol();
        std::cout << "PASS: motor wait, static feedback, release/new A, recovery, packet validation\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
