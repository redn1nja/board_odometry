#include <iomanip>
#include <serial.h>
#include <boost/regex.hpp>

SerialConn::SerialConn(const std::string& port, unsigned int baud_rate) : m_port(m_io, port) {
    m_port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
}

SerialConn::~SerialConn() {
    Close();
}

void SerialConn::SetupReading() {
    boost::asio::streambuf s;
    read_until(m_port, s, boost::regex(reinterpret_cast<const char*>(HEADER)));
}


void SerialConn::ReadData() {
    boost::asio::read(m_port, boost::asio::buffer(m_raw_buf, sizeof(serial_data)));
    {
        std::lock_guard lk{m_data_mx};
        memcpy(&m_data, m_raw_buf, sizeof(serial_data));
        ConvertToHost();
    }
}

void SerialConn::Close() {
    Disable();
    m_port.close();
}

serial_data SerialConn::GetData() const {
    std::lock_guard lk{m_data_mx};
    return m_data;
}

void SerialConn::ConvertToHost() {
    m_data.ts.sec = ntohl(m_data.ts.sec);
    m_data.ts.msec = ntohs(m_data.ts.msec);
    m_data.rpy.pitch = ntohs(m_data.rpy.pitch);
    m_data.rpy.roll = ntohs(m_data.rpy.roll);
    m_data.rpy.yaw = ntohs(m_data.rpy.yaw);
}

void SerialConn::Loop() {
    SetupReading();
    while (m_run) {
        ReadData();
    }
}

std::thread SerialConn::ThreadFn() {

    return std::thread{&SerialConn::Loop, this};
}


std::ostream &operator<<(std::ostream &os, const serial_data &data) {
    os << "[ " << data.ts.sec << "." << std::setfill('0') << std::setw(4) << data.ts.msec << " ]\n{\n" << \
    "Roll: " << static_cast<double>(data.rpy.roll) / 100 << "\n" << \
    "Pitch: " << static_cast<double>(data.rpy.pitch) / 100 << "\n" << \
    "Yaw: " << static_cast<double>(data.rpy.yaw) / 100 << "\n}\n";
    return os;
}

void SerialConn::Disable() {
    m_run = false;
}

