#include "app.h"

#include <types.h>

App::App(const std::string &dev, unsigned int baud) : m_serial{dev, baud} {
    m_serial.SetupReading();
    m_read_thread = m_serial.ThreadFn();
}

App::~App() {
    m_serial.Close();
    if (m_read_thread.joinable()) {
        m_read_thread.join();
    }
}

void App::run(cv::Mat frame) {
    auto data = m_serial.GetData().rpy;
    double altitude = 10;
    nav::ImageCorrection::Attitude attitude = {static_cast<double>(data.roll) / 100.0,
                                                static_cast<double>(data.pitch) / 100.0};

    m_processor.calclulate_offsets(frame, attitude, altitude);
}

