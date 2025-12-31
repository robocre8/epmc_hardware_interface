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
const int WRITE_SPEED = 10;
const int READ_SPEED = 11;
const int READ_TSPEED = 12;
const int READ_POS = 13;
const int WRITE_PWM = 14;
const int SET_KP = 15;
const int GET_KP = 16;
const int SET_KI = 17;
const int GET_KI = 18;
const int SET_KD = 19;
const int GET_KD = 20;
const int SET_PPR = 21;
const int GET_PPR = 22;
const int SET_CF = 23;
const int GET_CF = 24;
const int SET_RDIR = 25;
const int GET_RDIR = 26;
const int SET_PID_MODE = 27;
const int GET_PID_MODE = 28;
const int SET_CMD_TIMEOUT = 29;
const int GET_CMD_TIMEOUT = 30;
const int SET_I2C_ADDR = 31;
const int GET_I2C_ADDR = 32;
const int SET_MAX_SPEED = 33;
const int GET_MAX_SPEED = 34;
const int RESET = 35;
const int CLEAR = 36;
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
      serial_conn_.FlushIOBuffers();
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
    send((float)WRITE_PWM, (float)pwm0, (float)pwm1);
  }

  void writeSpeed(float v0, float v1)
  {
    send((float)WRITE_SPEED, v0, v1);
  }

  std::tuple<bool, float, float> readPos()
  {
    bool success; float pos0, pos1;
    std::tie(success, pos0, pos1) = recv((float)READ_POS);
    return std::make_tuple(success, pos0, pos1);
  }

  std::tuple<bool, float, float> readSpeed()
  {
    bool success; float v0, v1;
    std::tie(success, v0, v1) = recv((float)READ_SPEED);
    return std::make_tuple(success, v0, v1);
  }

  std::tuple<bool, float> getMaxSpeed(int motor_no)
  {
    bool success; float max_vel;
    std::tie(success, max_vel, std::ignore) = recv((float)GET_MAX_SPEED, (float)motor_no);
    return std::make_tuple(success, max_vel);
  }

  void setCmdTimeout(int timeout_ms)
  {
    send((float)SET_CMD_TIMEOUT, 0.0, (float)timeout_ms);
  }

  std::tuple<bool, int> getCmdTimeout()
  {
    bool success; float timeout_ms;
    std::tie(success, timeout_ms, std::ignore) = recv((float)GET_CMD_TIMEOUT);
    return std::make_tuple(success, (int)timeout_ms);
  }

  void setPidMode(int motor_no, int mode)
  {
    send((float)SET_PID_MODE, (float)motor_no, (float)mode);
  }

  std::tuple<bool, int> getPidMode(int motor_no)
  {
    bool success; float mode;
    std::tie(success, mode, std::ignore) = recv((float)GET_PID_MODE, (float)motor_no);
    return std::make_tuple(success, (int)mode);
  }

  bool clearDataBuffer()
  {
    bool success;
    std::tie(success, std::ignore, std::ignore) = recv((float)CLEAR);
    return success;
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;

  void send(float cmd, float arg1=0.0, float arg2=0.0)
  {
    std::ostringstream ss;
    ss << cmd << " " << round_to_dp(arg1,4) << " " << round_to_dp(arg2,4) << "\r";

    serial_conn_.Write(ss.str());
  }

  std::tuple<bool, float, float> recv(float cmd, float arg1=0.0)
  {
    bool success;
    float data1, data2;
    try {
      // Send request
      send(cmd, arg1);

      // Read response line (terminated by '\n')
      std::string line;
      serial_conn_.ReadLine(line, '\n');

      // Convert using strtof (robust & locale-safe)
      char* ptr = line.data();
      char* end;

      data1 = strtof(ptr, &end);
      if (ptr == end) {
        success = false;
        data1 = 0.0f;
        data2 = 0.0f;
        return std::make_tuple(success, data1, data2);
      }

      data2 = strtof(end, &end);
      if (ptr == end){
        success = false;
        data1 = 0.0f;
        data2 = 0.0f;
        return std::make_tuple(success, data1, data2);
      }

      success = true;
      return std::make_tuple(success, round_to_dp(data1,4), round_to_dp(data2,4));
    }
    catch (...) {
      success = false;
      data1 = 0.0f;
      data2 = 0.0f;
      // serial_conn_.FlushIOBuffers();
      return std::make_tuple(success, data1, data2);
    }
  }


};

#endif