#include "Camera.h"
#include "Stream.h"
#include "VideoOutput.h"
#include "RGA.h"
#include "Common.h"
#include "Display.h"
#include "Overlay.h"
#include "DrawingOverlay.h"

#include "StateManager.h"
#include <iostream>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <functional>
#include <string>
#include <opencv2/opencv.hpp>

cv::Mat frame;

volatile sig_atomic_t run = 1;
std::pair resolution = {720, 480};

void signalHandler(int signum) {
    std::cout << "\nSIGINT received. Shutting down gracefully.\n";
    run = 0;
}

void frame_cb(void* frame, size_t size, uint64_t time_stamp, int is_iframe) {
    cv::Mat img(resolution.first, resolution.second, CV_8UC3, frame);
}

int main() {
    std::signal(SIGINT, signalHandler);
    vision::InitRkmedia();
    vision::Camera cam(0, 1920, 1080, 30);
    vision::VideoOutput vo(720, 480);
    cam.Start();
    vo.Start();

    vision::RGA rga_camera_nv12_to_rgb888;
    rga_camera_nv12_to_rgb888.GetSrcParamsFrom(cam);
    rga_camera_nv12_to_rgb888.SetDstParams(resolution.first, resolution.second, 0, 0, IMAGE_TYPE_RGB888);
    rga_camera_nv12_to_rgb888.Start();

    vision::Stream stream_vi_rga3("VI <-> RGA3", cam, rga_camera_nv12_to_rgb888);
    vision::Stream stream_rga3_vo("RGA3 <-> VO", rga_camera_nv12_to_rgb888, vo);
    stream_vi_rga3.StartFrameReceive();
    stream_rga3_vo.SetDestinationCb(rga_camera_nv12_to_rgb888, frame_cb);

    vision::Display display;
    display.SetDestination(vo);
    display.Init();

    while(run) {
    }

    display.Stop();
    stream_vi_rga3.Unbind();
    stream_rga3_vo.Unbind();
    rga_camera_nv12_to_rgb888.Stop();
    vo.Stop();
    cam.Stop();
    return 0;
}
