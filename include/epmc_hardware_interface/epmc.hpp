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

class EPMCSerialError : public std::exception
{
public:
    EPMCSerialError(const std::string& msg) : m_msg(msg) {}

   ~EPMCSerialError()
   {
        std::cout << "EPMCSerialError::~EPMCSerialError" << std::endl;
   }

   virtual const char* what() const throw () 
   {
        return m_msg.c_str();
   }

   const std::string m_msg;
};

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

  void connect(const std::string &serial_device, int32_t baud_rate = 115200, int32_t timeout_ms = 100)
  {
    try {
      timeout_ms_ = timeout_ms;
      serial_conn_.Open(serial_device);
      serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
      serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
      serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
      serial_conn_.SetParity(LibSerial::Parity::PARITY_NONE);
      serial_conn_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
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

  void readPos(float &pos0, float& pos1)
  {
    read_data2(READ_POS, pos0, pos1);
    pos0 = round_to_dp(pos0, 4);
    pos1 = round_to_dp(pos1, 4);
  }

  void readVel(float &v0, float& v1)
  {
    read_data2(READ_VEL, v0, v1);
    v0 = round_to_dp(v0, 4);
    v1 = round_to_dp(v1, 4);
  }

  void readUVel(float &v0, float& v1)
  {
    read_data2(READ_UVEL, v0, v1);
    v0 = round_to_dp(v0, 4);
    v1 = round_to_dp(v1, 4);
  }

  void readMotorData(float &pos0, float& pos1, float &v0, float& v1)
  {
    read_data4(READ_MOTOR_DATA, pos0, pos1, v0, v1);
    pos0 = round_to_dp(pos0, 4);
    pos1 = round_to_dp(pos1, 4);
    v0 = round_to_dp(v0, 4);
    v1 = round_to_dp(v1, 4);
  }

  float getMaxVel(int motor_no)
  {
    float max_vel = read_data1(GET_MAX_VEL, motor_no);
    return round_to_dp(max_vel, 4);
  }

  bool setCmdTimeout(int timeout_ms)
  {
    float res = write_data1(SET_CMD_TIMEOUT, 100, (float)timeout_ms);
    return ((int)res == 1);
  }

  int getCmdTimeout()
  {
    float timeout_ms = read_data1(GET_CMD_TIMEOUT, 100);
    return (int)timeout_ms;
  }

  bool setPidMode(int motor_no, int mode)
  {
    float res = write_data1(SET_PID_MODE, (uint8_t)motor_no, (float)mode);
    return ((int)res == 1);
  }

  int getPidMode(int motor_no)
  {
    float mode = read_data1(GET_PID_MODE, (uint8_t)motor_no);
    return (int)mode;
  }

  bool clearDataBuffer()
  {
    float res = write_data1(CLEAR_DATA_BUFFER, 100, 0.0);
    return ((int)res == 1);
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;

  uint8_t calcChecksum(const std::vector<uint8_t>& packet) {
      uint32_t sum = 0;
      for (auto b : packet) sum += b;
      return sum & 0xFF;
  }

  void send_packet_without_payload(uint8_t cmd) {
      std::vector<uint8_t> packet = {START_BYTE, cmd, 0}; // no payload
      uint8_t checksum = calcChecksum(packet);
      packet.push_back(checksum);
      serial_conn_.Write(packet);
  }

  void send_packet_with_payload(uint8_t cmd, const std::vector<uint8_t>& payload) {
      std::vector<uint8_t> packet = {START_BYTE, cmd, (uint8_t)payload.size()};
      packet.insert(packet.end(), payload.begin(), payload.end());
      uint8_t checksum = calcChecksum(packet);
      packet.push_back(checksum);
      serial_conn_.Write(packet);
  }

  void read_packet1(float &val) {
      std::vector<uint8_t> payload;
      serial_conn_.Read(payload, 4, timeout_ms_);
      if (payload.size() < 4) {
        std::cerr << "[EPMC SERIAL ERROR]: Timeout while reading 1 values" << std::endl;
        throw EPMCSerialError("[EPMC SERIAL ERROR]: Timeout while reading 1 values");
      }
      std::memcpy(&val, payload.data(), sizeof(float)); // little-endian assumed
  }

  void read_packet2(float &val0, float &val1) {
      std::vector<uint8_t> payload;
      serial_conn_.Read(payload, 8, timeout_ms_);
      if (payload.size() < 8) {
        std::cerr << "[EPMC SERIAL ERROR]: Timeout while reading 2 values" << std::endl;
        throw EPMCSerialError("[EPMC SERIAL ERROR]: Timeout while reading 2 values");
      }
      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
  }

  void read_packet4(float &val0, float &val1, float &val2, float &val3) {
      std::vector<uint8_t> payload;
      serial_conn_.Read(payload, 16, timeout_ms_);
      if (payload.size() < 16) {
        std::cerr << "[EPMC SERIAL ERROR]: Timeout while reading 4 values" << std::endl;
        throw EPMCSerialError("[EPMC SERIAL ERROR]: Timeout while reading 4 values");
      }
      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
      std::memcpy(&val2, payload.data() + 8, sizeof(float));
      std::memcpy(&val3, payload.data() + 12, sizeof(float));
  }

  // ------------------- High-Level Wrappers -------------------
  float write_data1(uint8_t cmd, uint8_t pos, float val) {
      std::vector<uint8_t> payload(sizeof(uint8_t) + sizeof(float));
      payload[0] = pos;
      std::memcpy(&payload[1], &val, sizeof(float));
      send_packet_with_payload(cmd, payload);
      float data;
      read_packet1(data);
      return data;
  }

  float read_data1(uint8_t cmd, uint8_t pos) {
      float zero = 0.0f;
      std::vector<uint8_t> payload(sizeof(uint8_t) + sizeof(float));
      payload[0] = pos;
      std::memcpy(&payload[1], &zero, sizeof(float));
      send_packet_with_payload(cmd, payload);
      float val;
      read_packet1(val);
      return val;
  }

  void write_data2(uint8_t cmd, float a, float b) {
      std::vector<uint8_t> payload(2 * sizeof(float));
      std::memcpy(&payload[0],  &a, sizeof(float));
      std::memcpy(&payload[4],  &b, sizeof(float));
      send_packet_with_payload(cmd, payload);
  }

  void read_data2(uint8_t cmd, float &a, float &b) {
      send_packet_without_payload(cmd);
      read_packet2(a, b);
  }

  void read_data4(uint8_t cmd, float &a, float &b, float &c, float &d) {
      send_packet_without_payload(cmd);
      read_packet4(a, b, c, d);
  }

};

#endif