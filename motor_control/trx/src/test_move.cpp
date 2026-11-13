#include <chrono>
#include <thread>
#include <iostream>
#include "trx/seed_command.hpp"

int main()
{
    SeedCommand motor("/dev/ttyACM0", 115200);

    if (!motor.COM_Open()) {
        std::cout << "COM Open Failed" << std::endl;
        return -1;
    }

    const uint8_t id1 = 1;
    const uint8_t id2 = 2;

    // サーボON
    motor.onServo(id1, 1);
    motor.onServo(id2, 1);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ID1, ID2を同時に+2000 pulse
    std::cout << "Move ID1, ID2 +2000 pulse" << std::endl;

    motor.actuateRelativePositionBySpeed(id1, 2000, 80000);
    motor.actuateRelativePositionBySpeed(id2, 2000, 80000);

    std::this_thread::sleep_for(std::chrono::seconds(3));

    // ID1, ID2を同時に-2000 pulse
    std::cout << "Move ID1, ID2 -2000 pulse" << std::endl;

    // motor.actuateRelativePositionBySpeed(id1, 2000, -80000);
    // motor.actuateRelativePositionBySpeed(id2, 2000, -80000);

    std::this_thread::sleep_for(std::chrono::seconds(3));

    // サーボOFF
    motor.onServo(id1, 0);
    motor.onServo(id2, 0);

    motor.COM_Close();

    return 0;
}