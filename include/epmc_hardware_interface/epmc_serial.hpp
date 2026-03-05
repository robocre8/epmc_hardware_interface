#pragma once

#include <libserial/SerialPort.h>

#include <iostream>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <tuple>
#include <utility>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <cmath>

/* =======================
   Protocol Definitions
   ======================= */

namespace 
{

static constexpr uint8_t START_BYTE        = 0xAA;

// Command IDs
static constexpr uint8_t WRITE_VEL         = 0x01;
static constexpr uint8_t WRITE_PWM         = 0x02;
static constexpr uint8_t READ_POS          = 0x03;
static constexpr uint8_t READ_VEL          = 0x04;
static constexpr uint8_t READ_UVEL         = 0x05;
static constexpr uint8_t READ_TVEL         = 0x06;

static constexpr uint8_t SET_PPR           = 0x07;
static constexpr uint8_t GET_PPR           = 0x08;

static constexpr uint8_t SET_KP            = 0x09;
static constexpr uint8_t GET_KP            = 0x0A;
static constexpr uint8_t SET_KI            = 0x0B;
static constexpr uint8_t GET_KI            = 0x0C;
static constexpr uint8_t SET_KD            = 0x0D;
static constexpr uint8_t GET_KD            = 0x0E;

static constexpr uint8_t SET_RDIR          = 0x0F;
static constexpr uint8_t GET_RDIR          = 0x10;

static constexpr uint8_t SET_CUT_FREQ      = 0x11;
static constexpr uint8_t GET_CUT_FREQ      = 0x12;

static constexpr uint8_t SET_MAX_VEL       = 0x13;
static constexpr uint8_t GET_MAX_VEL       = 0x14;

static constexpr uint8_t SET_PID_MODE      = 0x15;
static constexpr uint8_t GET_PID_MODE      = 0x16;

static constexpr uint8_t SET_CMD_TIMEOUT   = 0x17;
static constexpr uint8_t GET_CMD_TIMEOUT   = 0x18;

static constexpr uint8_t SET_I2C_ADDR      = 0x19;
static constexpr uint8_t GET_I2C_ADDR      = 0x1A;

static constexpr uint8_t RESET_PARAMS      = 0x1B;
static constexpr uint8_t READ_MOTOR_DATA   = 0x2A;
static constexpr uint8_t CLEAR_DATA_BUFFER = 0x2C;
static constexpr uint8_t GET_NUM_OF_MOTORS = 0x2D;
  
}


/* =======================
   EPMC Serial Client
   ======================= */
namespace epmc_serial
{

inline float round_to_dp(float value, int decimal_places) {
    const float multiplier = std::pow(10.0, decimal_places);
    return std::round(value * multiplier) / multiplier;
}

inline LibSerial::BaudRate convert_baud_rate(int baud_rate)
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

}


namespace epmc_serial
{

enum class SupportedNumOfMotors: int {
  TWO = 2,
  FOUR = 4
};

class EPMCSerialClient
{
public:
    EPMCSerialClient() = default;

    ~EPMCSerialClient() { 
      disconnect();
    }

    /* ---------- Connection ---------- */

    void connect(const std::string& port,
                 int baudrate = 115200,
                 int timeout_ms = 100)
    {
        if (serial.IsOpen())
        serial.Close();

        serial.Open(port);

        serial.SetBaudRate(convert_baud_rate(baudrate));
        serial.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial.SetParity(LibSerial::Parity::PARITY_NONE);
        serial.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

        timeout_ms_ = timeout_ms;

        // ---- Allow MCU to boot / flush startup bytes ----
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));

        // ---- Retry confirm motors ----
        constexpr int max_attempts = 10;

        for (int i = 0; i < max_attempts; ++i)
        {
            if (confirmNumOfMotors()){
              std::cout << "EPMC Connected Successfully" << std::endl;
              return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        disconnect();
        throw std::runtime_error("EPMC supported number of motors mismatch");
    }

    void disconnect()
    {
        if (serial.IsOpen())
            serial.Close();
    }

    bool connected() const
    {
        return serial.IsOpen();
    }

    void supportedNumOfMotors(SupportedNumOfMotors supported_num_of_motors) {
      switch (supported_num_of_motors)
      {
      case SupportedNumOfMotors::TWO :
        num_of_motors = 2;
        break;
      
      case SupportedNumOfMotors::FOUR :
        num_of_motors = 4;
        break;
      }
    };

    /* ------------------------------------ */

    bool confirmNumOfMotors()
    {
        bool success; float motor_num = 0.0f;
        std::tie(success, motor_num) = read_data1(GET_NUM_OF_MOTORS);
        return success && ((int)motor_num == num_of_motors);
    }

    /* ---------- High-Level API ---------- */

    // Motion
    void writeSpeed(float v0, float v1, float v2=0.0, float v3=0.0) {
      if(num_of_motors == 2) write_data2(WRITE_VEL, v0, v1); 
      else if(num_of_motors == 4) write_data4(WRITE_VEL, v0, v1, v2, v3); 
    }

    void writePWM(float p0, float p1, float p2=0.0, float p3=0.0)   {
      if(num_of_motors == 2) write_data2(WRITE_PWM, p0, p1);
      else if(num_of_motors == 4) write_data4(WRITE_PWM, p0, p1, p2, p3);
    }

    std::tuple<bool, std::vector<float>> readPos() { return _read_motor_array(READ_POS); }
    std::tuple<bool, std::vector<float>> readVel() { return _read_motor_array(READ_VEL); }
    std::tuple<bool, std::vector<float>> readUVel(){ return _read_motor_array(READ_UVEL); }
    std::tuple<bool, std::vector<float>> readTVel(){ return _read_motor_array(READ_TVEL); }

    std::tuple<bool, std::vector<float>>
    readMotorData() {
      if(num_of_motors == 2){
        return read_data4(READ_MOTOR_DATA);
      }
      else if(num_of_motors == 4){
        return read_data8(READ_MOTOR_DATA);
      }
      else {
        std::vector<float> values;
        return {false, values};
      }
    }

    // PID & Control
    void setKP(float v, uint8_t motor) { write_data1(SET_KP, v, motor); }
    void setKI(float v, uint8_t motor) { write_data1(SET_KI, v, motor); }
    void setKD(float v, uint8_t motor) { write_data1(SET_KD, v, motor); }

    std::tuple<bool, float> getKP(uint8_t motor){ return read_data1(GET_KP, motor); }
    std::tuple<bool, float> getKI(uint8_t motor){ return read_data1(GET_KI, motor); }
    std::tuple<bool, float> getKD(uint8_t motor){ return read_data1(GET_KD, motor); }

    // Encoder
    void setPPR(float ppr, uint8_t motor){ write_data1(SET_PPR, ppr, motor); }
    std::tuple<bool, float> getPPR(uint8_t motor){ return read_data1(GET_PPR, motor); }

    // Velocity limits
    void setMaxVel(float max_vel, uint8_t motor){ write_data1(SET_MAX_VEL, max_vel, motor); }
    std::tuple<bool, float> getMaxVel(uint8_t motor){ return read_data1(GET_MAX_VEL, motor); }

    // Direction & filters
    void setRDir(int rdir, uint8_t motor){ write_data1(SET_RDIR, (float)rdir, motor); }
    std::tuple<bool, float> getRDir(uint8_t motor){ return read_data1(GET_RDIR, motor); }

    void setCutoffFreq(float f, uint8_t motor){ write_data1(SET_CUT_FREQ, f, motor); }
    std::tuple<bool, float> getCutoffFreq(uint8_t motor){ return read_data1(GET_CUT_FREQ, motor); }

    // Modes & system
    void setPidMode(int mode){ write_data1(SET_PID_MODE, (float)mode); }
    std::tuple<bool, float> getPidMode(){ return read_data1(GET_PID_MODE); }

    void setCmdTimeout(int t){ write_data1(SET_CMD_TIMEOUT, (float)t); }
    std::tuple<bool, float> getCmdTimeout(){ return read_data1(GET_CMD_TIMEOUT); }

    void setI2cAddress(int address) { write_data1(SET_I2C_ADDR, (float)address); }
    std::tuple<bool, float> getI2cAddress(){ return read_data1(GET_I2C_ADDR); }

    bool resetParams(){ 
      bool success;
      std::tie(success, std::ignore) = read_data1(RESET_PARAMS);
      return success;
    }
    bool clearDataBuffer(){ 
      bool success;
      std::tie(success, std::ignore) = read_data1(CLEAR_DATA_BUFFER);
      return success;
    }

private:
    LibSerial::SerialPort serial;
    int timeout_ms_;
    int num_of_motors = 2;

    /* ---------- Packet Helpers ---------- */

    void flush_rx()
    {
        if (!serial.IsOpen()) return;
        try {
            serial.FlushInputBuffer();  // clears RX
        } catch (...) {
        }
    }

    void flush_tx()
    {
        if (!serial.IsOpen()) return;
        try {
            serial.DrainWriteBuffer();  // clears TX
        } catch (...) {
        }
    }

    void sendPacket(uint8_t cmd,
                    const std::vector<uint8_t>& payload = {})
    {
        if (!serial.IsOpen()) {
          throw std::runtime_error("Serial port not connected");
        }
        flush_rx();
        
        std::vector<uint8_t> packet;
        packet.reserve(4 + payload.size());

        packet.push_back(START_BYTE);
        packet.push_back(cmd);
        packet.push_back(static_cast<uint8_t>(payload.size()));
        packet.insert(packet.end(), payload.begin(), payload.end());

        uint8_t checksum = 0;
        for (uint8_t b : packet)
            checksum += b;

        packet.push_back(checksum);

        serial.Write(packet);
        serial.DrainWriteBuffer();
    }

    std::pair<bool, std::vector<float>>
    readFloats(size_t count)
    {
        const size_t bytes_needed = count * sizeof(float);
        std::vector<uint8_t> buf(bytes_needed);

        try {
            serial.Read(buf, bytes_needed, timeout_ms_);
            if (buf.size() != bytes_needed){
              flush_rx();
              return {false, std::vector<float>(count, 0.0f)};
            }
        }
        catch (const LibSerial::ReadTimeout&) {
            flush_rx();   // critical for stream resync
            return {false, std::vector<float>(count, 0.0f)};
        }
        catch (...) {
            flush_rx();
            return {false, std::vector<float>(count, 0.0f)};
        }

        std::vector<float> values(count);
        std::memcpy(values.data(), buf.data(), bytes_needed);
        return {true, values};
    }

    /* ---------- Generic Data ---------- */

    void write_data1(uint8_t cmd, float val, uint8_t pos = 0)
    {
        std::vector<uint8_t> payload(1 + sizeof(float));
        payload[0] = pos;
        std::memcpy(&payload[1], &val, sizeof(float));
        sendPacket(cmd, payload);
    }

    void write_data2(uint8_t cmd, float a, float b)
    {
        std::vector<uint8_t> payload(2 * sizeof(float));
        std::memcpy(&payload[0], &a, sizeof(float));
        std::memcpy(&payload[4], &b, sizeof(float));
        sendPacket(cmd, payload);
    }

    void write_data4(uint8_t cmd, float a, float b, float c, float d)
    {
        std::vector<uint8_t> payload(4 * sizeof(float));
        std::memcpy(&payload[0], &a, sizeof(float));
        std::memcpy(&payload[4], &b, sizeof(float));
        std::memcpy(&payload[8], &c, sizeof(float));
        std::memcpy(&payload[12], &d, sizeof(float));
        sendPacket(cmd, payload);
    }

    std::tuple<bool, float>
    read_data1(uint8_t cmd, uint8_t pos = 0)
    {
        float dummy = 0.0f;
        std::vector<uint8_t> payload(1 + sizeof(float));
        payload[0] = pos;
        std::memcpy(&payload[1], &dummy, sizeof(float));

        sendPacket(cmd, payload);

        auto [ok, vals] = readFloats(1);
        return {ok, round_to_dp(vals[0],4)};
    }

    std::tuple<bool, std::vector<float>>
    read_data2(uint8_t cmd)
    {
        sendPacket(cmd);
        auto [ok, vals] = readFloats(2);
        std::vector<float> value(2);
        value[0] = round_to_dp(vals[0],4);
        value[1] = round_to_dp(vals[1],4);
        return {ok, value};
    }

    std::tuple<bool, std::vector<float>>
    read_data4(uint8_t cmd)
    {
        sendPacket(cmd);
        auto [ok, vals] = readFloats(4);
        std::vector<float> value(4);
        value[0] = round_to_dp(vals[0],4);
        value[1] = round_to_dp(vals[1],4);
        value[2] = round_to_dp(vals[2],4);
        value[3] = round_to_dp(vals[3],4);
        return {ok, value};
    }

    std::tuple<bool, std::vector<float>>
    read_data8(uint8_t cmd)
    {
        sendPacket(cmd);
        auto [ok, vals] = readFloats(8);
        std::vector<float> value(8);
        value[0] = round_to_dp(vals[0],4);
        value[1] = round_to_dp(vals[1],4);
        value[2] = round_to_dp(vals[2],4);
        value[3] = round_to_dp(vals[3],4);
        value[4] = round_to_dp(vals[4],4);
        value[5] = round_to_dp(vals[5],4);
        value[6] = round_to_dp(vals[6],4);
        value[7] = round_to_dp(vals[7],4);
        return {ok, value};
    }

    std::tuple<bool, std::vector<float>>
    _read_motor_array(uint8_t cmd){
      if(num_of_motors == 2){
        return read_data2(cmd);
      }
      else if(num_of_motors == 4){
        return read_data4(cmd);
      }
      else {
        std::vector<float> values;
        return {false, values};
      }
    }
};

}