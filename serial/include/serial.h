#ifndef SERIAL_H
#define SERIAL_H

#include <boost/asio.hpp>
#include <thread>

#pragma pack(push, 1)
struct timestamp{
    uint32_t sec;
    uint16_t msec;
};
struct attitude{
    uint16_t pitch;
    uint16_t roll;
    uint16_t yaw;
};


struct serial_data{
    uint8_t header[2];
    timestamp ts;
    attitude rpy;
};
#pragma pack(pop)

class SerialConn {
private:
    static constexpr uint8_t HEADER[2] = {0xFE, 0xED};
    boost::asio::io_service m_io;
    boost::asio::serial_port m_port;
    mutable std::mutex m_data_mx;
    char m_raw_buf[100]{};
    void ConvertToHost();
    serial_data m_data{};
public:
    SerialConn(const std::string& port, unsigned int baud_rate);
    ~SerialConn();
    void SetupReading();
    void ReadData();
    void Loop();
    std::thread ThreadFn();
    void Close();
    [[nodiscard]] serial_data GetData() const;
};

std::ostream& operator<< (std::ostream& os, const serial_data& data);
#endif //SERIAL_H
