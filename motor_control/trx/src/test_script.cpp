#include "trx/seed_command.hpp"

#include <chrono>
#include <iostream>
#include <thread>

int main()
{
    SeedCommand motor("/dev/ttyACM0", 115200);

    if (!motor.COM_Open())
    {
        std::cerr << "Failed to open serial port." << std::endl;
        return 1;
    }

    std::cout << "Serial opened." << std::endl;

    // 実行前の位置を取得
    auto pos1_id1 = motor.getPosition(1);
    auto pos1_id2 = motor.getPosition(2);

    std::cout << "ID1 Before : " << pos1_id1[2] << std::endl;
    std::cout << "ID2 Before : " << pos1_id2[2] << std::endl;

    std::cout << "Run Script No.6" << std::endl;



    // ID1・ID2に同時にスクリプト実行命令を送る
    motor.Script_Go(1, 5);
    motor.Script_Go(2, 5);

    // ID1・ID2のスクリプト終了を待つ
    bool finish_id1 = motor.waitForScriptEnd(1, 10.0);
    bool finish_id2 = motor.waitForScriptEnd(2, 10.0);

    if (finish_id1)
    {
        std::cout << "ID1 Script finished." << std::endl;
    }
    else
    {
        std::cout << "ID1 Script timeout." << std::endl;
    }

    if (finish_id2)
    {
        std::cout << "ID2 Script finished." << std::endl;
    }
    else
    {
        std::cout << "ID2 Script timeout." << std::endl;
    }

    // 実行後の位置を取得
    auto pos2_id1 = motor.getPosition(1);
    auto pos2_id2 = motor.getPosition(2);

    std::cout << "ID1 After : " << pos2_id1[2] << std::endl;
    std::cout << "ID2 After : " << pos2_id2[2] << std::endl;


    motor.COM_Close();

    return 0;
}