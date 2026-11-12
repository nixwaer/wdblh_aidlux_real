// Emit synthetic packets only: compare Python daemon output against the C++ wire ABI.
#include "pc_protocol.h"
#include <iostream>

int main()
{
    MotorCommand motors[PC_MOTOR_NUM]{};
    uint8_t packet[PC_CMD_PACKET_SIZE]{};
    pc_make_idle(motors);
    pc_build_command(packet, motors);
    std::cout.write(reinterpret_cast<const char*>(packet), sizeof(packet));
    constexpr float gear = 6.33f;
    for (auto& motor : motors) motor.kd = 3.0f / (gear * gear);
    pc_build_command(packet, motors);
    std::cout.write(reinterpret_cast<const char*>(packet), sizeof(packet));
}
