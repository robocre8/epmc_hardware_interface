#ifndef EPMC_HPP
#define EPMC_HPP

#include <iostream>
#include <iomanip>
#include <vector>
#include <tuple>
#include <chrono>
#include <thread>
#include <cstring>  // memcpy
#include <stdexcept> // For standard exception types
#include <cmath> // Required for std::round, std::ceil, std::floor
#include <libserial/SerialPort.h>

double round_to_dp(double value, int decimal_places) {
    const double multiplier = std::pow(10.0, decimal_places);
    return std::round(value * multiplier) / multiplier;
}

// Serial Protocol Command IDs -------------
const uint8_t START_BYTE = 0xAA;
const uint8_t WRITE_VEL = 0x01;
const uint8_t WRITE_PWM = 0x02;
const uint8_t READ_POS = 0x03;
const uint8_t READ_VEL = 0x04;
const uint8_t READ_UVEL = 0x05;
const uint8_t GET_MAX_VEL = 0x14;
const uint8_t SET_PID_MODE = 0x15;
const uint8_t GET_PID_MODE = 0x16;
const uint8_t SET_CMD_TIMEOUT = 0x17;
const uint8_t GET_CMD_TIMEOUT = 0x18;
const uint8_t READ_MOTOR_DATA = 0x2A;
const uint8_t CLEAR_DATA_BUFFER = 0x2C;
//---------------------------------------------

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  case 460800:
    return LibSerial::BaudRate::BAUD_460800;
  case 921600:
    return LibSerial::BaudRate::BAUD_921600;
  default:
    std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
    return LibSerial::BaudRate::BAUD_57600;
  }
}

class EPMC
{

public:
  EPMC() = default;

  void connect(const std::string &serial_device, int32_t baud_rate = 57600, int32_t timeout_ms = 100)
  {
    try {
      timeout_ms_ = timeout_ms;
      serial_conn_.Open(serial_device);
      serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    } catch (const LibSerial::OpenFailed&) {
        std::cerr << "Failed to open serial port!" << std::endl;
    }
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  void writePWM(int pwm0, int pwm1)
  {
    write_data2(WRITE_PWM, (float)pwm0, (float)pwm1);
  }

  void writeSpeed(float v0, float v1)
  {
    write_data2(WRITE_VEL, v0, v1);
  }

  std::tuple<bool, float, float> readPos()
  {
    bool success; float pos0, pos1;
    std::tie(success, pos0, pos1) = read_data2(READ_POS);
    pos0 = round_to_dp(pos0, 4);
    pos1 = round_to_dp(pos1, 4);
    return std::make_tuple(success, pos0, pos1);
  }

  std::tuple<bool, float, float> readVel()
  {
    bool success; float v0, v1;
    std::tie(success, v0, v1) = read_data2(READ_VEL);
    v0 = round_to_dp(v0, 4);
    v1 = round_to_dp(v1, 4);
    return std::make_tuple(success, v0, v1);
  }

  std::tuple<bool, float, float, float, float> readMotorData()
  {
    bool success; float pos0, pos1, v0, v1;
    std::tie(success, pos0, pos1, v0, v1) = read_data4(READ_MOTOR_DATA);
    pos0 = round_to_dp(pos0, 4);
    pos1 = round_to_dp(pos1, 4);
    v0 = round_to_dp(v0, 4);
    v1 = round_to_dp(v1, 4);
    return std::make_tuple(success, pos0, pos1, v0, v1);
  }

  std::tuple<bool, float> getMaxVel(int motor_no)
  {
    bool success; float max_vel;
    std::tie(success, max_vel) = read_data1(GET_MAX_VEL, (uint8_t)motor_no);
    max_vel = round_to_dp(max_vel, 4);
    return std::make_tuple(success, max_vel);
  }

  void setCmdTimeout(int timeout_ms)
  {
    write_data1(SET_CMD_TIMEOUT, (float)timeout_ms);
  }

  std::tuple<bool, int> getCmdTimeout()
  {
    bool success; float timeout_ms;
    std::tie(success, timeout_ms) = read_data1(GET_CMD_TIMEOUT);
    return std::make_tuple(success, (int)timeout_ms);
  }

  void setPidMode(int mode, int motor_no)
  {
    write_data1(SET_PID_MODE, (float)mode, (uint8_t)motor_no);
  }

  std::tuple<bool, int> getPidMode(int motor_no)
  {
    bool success; float mode;
    std::tie(success, mode) = read_data1(GET_PID_MODE, (uint8_t)motor_no);
    return std::make_tuple(success, (int)mode);
  }

  bool clearDataBuffer()
  {
    bool success;
    std::tie(success, std::ignore) = read_data1(CLEAR_DATA_BUFFER);
    return success;
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;

  uint8_t calcChecksum(const std::vector<uint8_t>& packet) {
    uint32_t sum = 0;
    for (auto b : packet) sum += b;
    return sum & 0xFF;
  }

  void send_packet_without_payload(uint8_t cmd, uint8_t len=0) {
    std::vector<uint8_t> packet = {START_BYTE, cmd, len}; // no payload
    uint8_t checksum = calcChecksum(packet);
    packet.push_back(checksum);
    serial_conn_.Write(packet);
    serial_conn_.DrainWriteBuffer();
  }

  void send_packet_with_payload(uint8_t cmd, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> packet = {START_BYTE, cmd, (uint8_t)payload.size()};
    packet.insert(packet.end(), payload.begin(), payload.end());
    uint8_t checksum = calcChecksum(packet);
    packet.push_back(checksum);
    serial_conn_.Write(packet);
    serial_conn_.DrainWriteBuffer();
  }

  std::tuple<bool, float> read_packet1() {
    std::vector<uint8_t> payload;
    float val;
    try
    {
      serial_conn_.Read(payload, 4, timeout_ms_);
      if (payload.size() < 4) {
        // std::cerr << "[EPMC SERIAL ERROR]: Timeout while reading 1 values" << std::endl;
        return std::make_tuple(false, 0.0);
      }
      std::memcpy(&val, payload.data(), sizeof(float)); // little-endian assumed
      return std::make_tuple(true, val);
    }
    catch(const LibSerial::ReadTimeout &e)
    {
      // std::cerr << "[LIB SERIAL ERROR]: ReadTimeout" << std::endl;
      return std::make_tuple(false, 0.0);
    }
  }

  std::tuple<bool, float, float> read_packet2() {
    std::vector<uint8_t> payload;
    float val0, val1;
    try
    {
      serial_conn_.Read(payload, 8, timeout_ms_);
      if (payload.size() < 8) {
        // std::cerr << "[EPMC SERIAL ERROR]: Timeout while reading 2 values" << std::endl;
        return std::make_tuple(false, 0.0, 0.0);
      }
      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
      return std::make_tuple(true, val0, val1);
    }
    catch(const LibSerial::ReadTimeout &e)
    {
      // std::cerr << "[LIB SERIAL ERROR]: ReadTimeout" << std::endl;
      return std::make_tuple(false, 0.0, 0.0);
    }
  }

  std::tuple<bool, float, float, float, float> read_packet4() {
    std::vector<uint8_t> payload;
    float val0, val1, val2, val3;
    try
    {
      serial_conn_.Read(payload, 16, timeout_ms_);
      if (payload.size() < 16) {
        // std::cerr << "[EPMC SERIAL ERROR]: Timeout while reading 4 values" << std::endl;
        return std::make_tuple(false, 0.0, 0.0, 0.0, 0.0);
      }
      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
      std::memcpy(&val2, payload.data() + 8, sizeof(float));
      std::memcpy(&val3, payload.data() + 12, sizeof(float));
      return std::make_tuple(true, val0, val1, val2, val3);
    }
    catch(const LibSerial::ReadTimeout &e)
    {
      // std::cerr << "[LIB SERIAL ERROR]: ReadTimeout" << std::endl;
      return std::make_tuple(false, 0.0, 0.0, 0.0, 0.0);
    }

      
  }

  // ------------------- High-Level Wrappers -------------------
  void write_data1(uint8_t cmd, float val, uint8_t pos=100) {
      std::vector<uint8_t> payload(sizeof(uint8_t) + sizeof(float));
      payload[0] = pos;
      std::memcpy(&payload[1], &val, sizeof(float));
      send_packet_with_payload(cmd, payload);
  }

  void write_data2(uint8_t cmd, float a, float b) {
      std::vector<uint8_t> payload(2 * sizeof(float));
      std::memcpy(&payload[0],  &a, sizeof(float));
      std::memcpy(&payload[4],  &b, sizeof(float));
      send_packet_with_payload(cmd, payload);
  }

  std::tuple<bool, float> read_data1(uint8_t cmd, uint8_t pos=100) {
      std::vector<uint8_t> payload(sizeof(uint8_t) + sizeof(float));
      payload[0] = pos;
      send_packet_with_payload(cmd, payload);
      bool success; float val;
      std::tie(success, val) = read_packet1();
      return std::make_tuple(success, val);
  }

  std::tuple<bool, float, float> read_data2(uint8_t cmd) {
      send_packet_without_payload(cmd);
      bool success; float a, b;
      std::tie(success, a, b) = read_packet2();
      return std::make_tuple(success, a, b);
  }

  std::tuple<bool, float, float, float, float> read_data4(uint8_t cmd) {
      send_packet_without_payload(cmd);
      bool success; float a, b, c, d;
      std::tie(success, a, b, c, d) = read_packet4();
      return std::make_tuple(success, a, b, c, d);
  }

};

#endif