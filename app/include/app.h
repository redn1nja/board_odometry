#ifndef APP_H
#define APP_H
#include <processor.h>
#include <serial.h>

class App{
    nav::ImageProc m_processor;
    SerialConn m_serial;
    std::thread m_read_thread;
public:
    App(const std::string& dev, unsigned int baud);
    ~App();
    void run(cv::Mat frame);


};

#endif //APP_H
