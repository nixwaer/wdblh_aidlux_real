#ifndef WDBLH_MOTOR_GATE_HPP
#define WDBLH_MOTOR_GATE_HPP

#include "pc_protocol.h"
#include <cmath>
#include <cstdint>

namespace wdblh
{
// The wire protocol has no per-motor heartbeat. These checks reject malformed
// and all-zero placeholder frames, but cannot detect cached nonzero feedback.
inline bool UsableMotorFeedback(const MotorState* motors)
{
    bool any_nonzero = false;
    for (int i = 0; i < PC_MOTOR_NUM; ++i)
    {
        if (!std::isfinite(motors[i].q) || !std::isfinite(motors[i].dq) ||
            !std::isfinite(motors[i].tau_est)) return false;
        any_nonzero = any_nonzero || motors[i].q != 0.0f ||
                      motors[i].dq != 0.0f || motors[i].tau_est != 0.0f;
    }
    return any_nonzero;
}

inline bool ParseCompleteMotorFeedback(const uint8_t* packet, MotorState* motors)
{
    // pc_parse_status alone accepts missing/duplicate IDs and ignores cmd/num.
    if (packet[0] != 0xBB || packet[1] != 0x02 || packet[2] != PC_MOTOR_NUM)
        return false;
    uint16_t ids = 0;
    for (int i = 0; i < PC_MOTOR_NUM; ++i)
    {
        const auto id = packet[3 + i * sizeof(MotorStatusRaw)];
        if (id >= PC_MOTOR_NUM || (ids & (1u << id))) return false;
        ids |= 1u << id;
    }
    return pc_parse_status(packet, motors) == 0;
}

// Caller serializes access. Millisecond times are monotonic, never wall time.
class MotorFeedbackGate
{
public:
    static constexpr int64_t kStableMs = 300;
    static constexpr int64_t kMaxGapMs = 100;
    enum class Phase { WaitingFeedback, WaitingRelease, WaitingPress, Authorized };

    void RecordFeedback(bool usable, int64_t now)
    {
        if (!usable)
        {
            start_ms_ = last_ms_ = -1;
            ++epoch_;
            return;
        }
        if (last_ms_ < 0 || now - last_ms_ > kMaxGapMs)
        {
            start_ms_ = now;
            ++epoch_;
        }
        last_ms_ = now;
    }

    void Update(int64_t now)
    {
        const bool ready = start_ms_ >= 0 && last_ms_ >= 0 &&
                           now - last_ms_ <= kMaxGapMs && now - start_ms_ >= kStableMs;
        if (!ready || epoch_ != observed_epoch_)
        {
            phase_ = Phase::WaitingFeedback;
            last_pad_ms_ = -1;
            previous_a_ = false;
        }
        observed_epoch_ = epoch_;
        if (ready && phase_ == Phase::WaitingFeedback)
        {
            phase_ = Phase::WaitingRelease;
            ready_ms_ = now;
        }
    }

    // Require a NEW released packet after readiness, followed by a NEW press.
    // Disconnection must not count as a physical release of A.
    bool ObserveGamepad(bool connected, bool a_pressed, int64_t received_ms,
                        bool modifiers = false)
    {
        if (phase_ == Phase::WaitingFeedback || phase_ == Phase::Authorized) return false;
        if (!connected)
        {
            phase_ = Phase::WaitingRelease;
            previous_a_ = false;
            return false;
        }
        if (received_ms <= ready_ms_ || received_ms <= last_pad_ms_) return false;
        last_pad_ms_ = received_ms;
        if (phase_ == Phase::WaitingRelease)
        {
            if (!a_pressed) phase_ = Phase::WaitingPress;
        }
        else if (a_pressed && !previous_a_ && !modifiers)
        {
            phase_ = Phase::Authorized;
            return true;
        }
        previous_a_ = a_pressed;
        return false;
    }

    bool ready() const { return phase_ != Phase::WaitingFeedback; }
    bool authorized() const { return phase_ == Phase::Authorized; }
    Phase phase() const { return phase_; }

private:
    int64_t start_ms_ = -1, last_ms_ = -1, ready_ms_ = -1, last_pad_ms_ = -1;
    uint64_t epoch_ = 0, observed_epoch_ = 0;
    bool previous_a_ = false;
    Phase phase_ = Phase::WaitingFeedback;
};
} // namespace wdblh
#endif
