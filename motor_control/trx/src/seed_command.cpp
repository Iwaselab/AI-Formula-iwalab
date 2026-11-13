#include "trx/seed_command.hpp"
#include <boost/asio.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <array>
#include <functional>
#include <chrono>

using namespace std::placeholders;
std::string num2str(int value, int byte = 1) {
    if (value < 0) value = 0xFFFFFF + 1 + value;
    std::stringstream ss;
    ss << std::uppercase << std::hex << (value & 0xFFFFFF);
    std::string s = ss.str();
    size_t width = 1 << byte;
    if (s.length() < width)
        s = std::string(width - s.length(), '0') + s;
    return s.substr(s.length() - width, width);
}

int SeedCommand::str2int(std::string _data)
{
  return std::strtol(_data.c_str(),NULL,16);
}

SeedCommand::SeedCommand(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate), check_sum_(0), count_(0), length_(0), fd_(-1) {
          send_data_.resize(6);
    }

SeedCommand::~SeedCommand() {
    COM_Close();
}

bool SeedCommand::COM_Open() {
    std::lock_guard<std::mutex> lock(mtx_);
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        std::cerr << "Failed to open port: " << port_ << std::endl;
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "tcgetattr error\n";
        return false;
    }
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr error\n";
        return false;
    }

    write(fd_, "S8\r", 3);
    write(fd_, "O\r", 2);

    std::cout << "Serial port opened: " << port_ << std::endl;
    return true;
}

void SeedCommand::writeSerialCommand(uint8_t _id, uint8_t *_data)
{
  std::vector<char> send_char;
  char convert[3]={0};

  sprintf(convert,"%01X",_id);

  length_ = 22;
  send_char.resize(length_);
  std::fill(send_char.begin(),send_char.end(),0);

  send_char[0] = 't';
  send_char[1] = '3';
  send_char[2] = '0';
  send_char[3] = convert[0];
  send_char[4] = '8';
  send_char[5] = 'F';
  send_char[6] = convert[0];
  send_char[7] = '0';
  send_char[8] = '0';

  for(uint8_t i=0;i<6;i++){
    sprintf(convert,"%02X",_data[i]);
    send_char[9+i*2] = convert[0];
    send_char[10+i*2] = convert[1];
  }

  send_char[21] = '\r';

  ssize_t ret = write(fd_, send_char.data(), send_char.size());
  if(ret < 0) perror("write");
}

bool SeedCommand::readSerialCommand(std::vector<uint8_t>& _receive_data, uint16_t _timeout)
{
    (void)_timeout; 
    char buf[64];
    ssize_t n = read(fd_, buf, sizeof(buf));
    if (n <= 0) {
        std::cerr << "[readSerialCommand] read failed or timeout" << std::endl;
        return false;
    }
    _receive_data.assign(buf, buf + n);
    return true;
}

void SeedCommand::setPositionPulse(int _speed ,int target_pulse, int _id)
{
    uint8_t id = _id;
    uint8_t cmd = 0x68;   
    uint8_t time_H = (_speed  >> 8) & 0xFF;
    uint8_t time_L = _speed  & 0xFF;
    uint8_t pos_H = (target_pulse >> 16) & 0xFF;
    uint8_t pos_M = (target_pulse >> 8) & 0xFF;
    uint8_t pos_L = target_pulse & 0xFF;
    setPosition(id, cmd, time_H, time_L, pos_H, pos_M, pos_L);
}

void SeedCommand::setPosition(uint8_t id, uint8_t cmd, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5)
{
    uint8_t data[6];
    data[0] = cmd;
    data[1] = c1;
    data[2] = c2;
    data[3] = c3;
    data[4] = c4;
    data[5] = c5;

    writeSerialCommand(id, data);
}

void SeedCommand::Script_Go(int id_num, int s_num)
{
    if (s_num > 0x00 && s_num < 0x0F) {
        uint8_t data[6] = {0};
        data[0] = 0x5F;
        data[1] = static_cast<uint8_t>(id_num);
        data[2] = static_cast<uint8_t>(s_num);
        writeSerialCommand(static_cast<uint8_t>(id_num), data);
    }
}

bool SeedCommand::waitForScriptEnd(int id_num, double timeout_sec)
{
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < timeout_sec) {
        std::vector<uint8_t> receive_data;
        if (!readSerialCommand(receive_data, 500)) {
            continue;
        }
        std::string hex_ascii(receive_data.begin(), receive_data.end());
        size_t t_pos = hex_ascii.find('t');
        if (t_pos == std::string::npos) {
            continue;
        }
        std::string packet = hex_ascii.substr(t_pos);
        if (packet.length() < 15) {
            continue;
        }

        int id = str2int(packet.substr(5, 1));
        int signal = str2int(packet.substr(13, 2));
        if (id == id_num && signal == 0xFF) {
            return true;
        }
    }
    return false;
}
// 絶対位置取得
std::array<int, 3> SeedCommand::getPosition(uint8_t _id)
{
  std::string target_packet = "";
  int count = 0;
  while(true){
    fill(send_data_.begin(), send_data_.end(), 0);
    send_data_[0] = 0x42;
    send_data_[1] = _id;
    writeSerialCommand(_id, send_data_.data());

    std::vector<uint8_t> receive_data;
    std::string id = "";
    std::string command = "";
    std::string velocity = "";
    std::string position = "";

    if (!readSerialCommand(receive_data,50)) {
        std::cout << "[getPosition] receive failed." << std::endl;
        continue;
    }

    std::string hex_ascii(receive_data.begin(), receive_data.end());
    size_t t_pos = hex_ascii.find('t');
    if (t_pos == std::string::npos) {
        if(count == count_limit){
            std::cout << "[getPosition] No valid packet found " << std::endl;
            return{0,0,0};
        } else{
            std::cout << "[getPosition] No valid packet found. retry... " << std::endl;
            count++;
            continue;
        }
    }
    std::string main_packet = hex_ascii.substr(t_pos);

    if (main_packet.length() < 26) {
        std::cout << "[getPosition] main_packet too short: " << main_packet.length() << std::endl;
        continue;
    }

    int pre_id_val;
    try {
        std::string pre_id_str = main_packet.substr(5, 1);
        pre_id_val = std::stoi(pre_id_str, nullptr, 16);
    } catch (const std::exception& e) {
        std::cout << "[getPosition] failed to parse packet, skip: " << e.what() << std::endl;
        continue;
    }

    if(pre_id_val != _id) {
        if(count == count_limit){
            std::cout << "[getPosition] Can not find target_id " << std::endl;
            return{0,0,0};
        } else{
            std::cout << "[getPosition] No match target_id. retry... " << std::endl;
            count++;
            continue;
        }
    }

    target_packet = main_packet;
    break;
  }

  try {
    std::string id_str   = target_packet.substr(5, 1);
    std::string cmd_str  = target_packet.substr(9, 2);
    std::string pos_str  = target_packet.substr(15, 6);

    int id_val  = std::stoi(id_str, nullptr, 16);
    int cmd = std::stoi(cmd_str, nullptr, 16);
    int pos = std::stoi(pos_str, nullptr, 16);
    return {id_val, cmd, pos};
  } catch (const std::exception& e) {
    std::cout << "[getPosition] failed to parse final packet: " << e.what() << std::endl;
    return {0, 0, 0};
  }
}

//////////////////////////////////////////////////
// Servo ON/OFF
//////////////////////////////////////////////////
void SeedCommand::onServo(uint8_t _id, uint8_t _state)
{
    fill(send_data_.begin(), send_data_.end(), 0);

    send_data_[0] = 0x50;
    send_data_[1] = _id;
    send_data_[2] = _state;

    writeSerialCommand(_id, send_data_.data());
}

//////////////////////////////////////////////////
// Motor Stop
//////////////////////////////////////////////////
void SeedCommand::stopMotor(uint8_t _id)
{
    fill(send_data_.begin(), send_data_.end(), 0);

    send_data_[0] = 0x51;
    send_data_[1] = _id;

    writeSerialCommand(_id, send_data_.data());
}

//////////////////////////////////////////////////
// Relative Position (Speed)
//////////////////////////////////////////////////
void SeedCommand::actuateRelativePositionBySpeed(
        uint8_t _id,
        int16_t _speed,
        int32_t _position)
{
    fill(send_data_.begin(), send_data_.end(), 0);

    send_data_[0] = 0x63;

    send_data_[1] = (_speed >> 8) & 0xFF;
    send_data_[2] = _speed & 0xFF;

    send_data_[3] = (_position >> 16) & 0xFF;
    send_data_[4] = (_position >> 8) & 0xFF;
    send_data_[5] = _position & 0xFF;

    writeSerialCommand(_id, send_data_.data());
}

//////////////////////////////////////////////////
// Absolute Position (Speed)
//////////////////////////////////////////////////
void SeedCommand::actuateAbsolutePositionBySpeed(
        uint8_t _id,
        int16_t _speed,
        int32_t _position)
{
    fill(send_data_.begin(), send_data_.end(), 0);

    send_data_[0] = 0x66;

    send_data_[1] = (_speed >> 8) & 0xFF;
    send_data_[2] = _speed & 0xFF;

    send_data_[3] = (_position >> 16) & 0xFF;
    send_data_[4] = (_position >> 8) & 0xFF;
    send_data_[5] = _position & 0xFF;

    writeSerialCommand(_id, send_data_.data());
}

//////////////////////////////////////////////////
// Speed Control
//////////////////////////////////////////////////
void SeedCommand::actuateBySpeed(
        uint8_t _id,
        int16_t _speed)
{
    fill(send_data_.begin(), send_data_.end(), 0);

    send_data_[0] = 0x6C;

    if (_speed < 0)
    {
        send_data_[1] = ((-_speed) >> 8) & 0xFF;
        send_data_[2] = (-_speed) & 0xFF;
        send_data_[3] = 1;
    }
    else
    {
        send_data_[1] = (_speed >> 8) & 0xFF;
        send_data_[2] = _speed & 0xFF;
        send_data_[3] = 0;
    }

    send_data_[4] = 0;
    send_data_[5] = 0;

    writeSerialCommand(_id, send_data_.data());
}

void SeedCommand::COM_Close() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
        std::cout << "Serial port closed" << std::endl;
    }
}
