#include <fstream>
#include <processor.h>
#include <csv.h>
#include <structs.h>

int main(int argc, char** argv) {
    nav::ImageCorrection correction;
    correction.set_modes(nav::ROLL_MODE::ROLL_NEGATIVE, nav::PITCH_MODE::PITCH_POSITIVE);

    cv::Mat frame;
    cv::VideoCapture cap(argv[1]);
    std::ifstream file(argv[2]);
    Data data;
    double _;
    while (cap.read(frame)) {
        CommaSeparatedReader::read(file,
            Data::id,
            _,
            data.attitude.roll,
            data.attitude.pitch)
        ;
        if (Data::id == 242) {
            auto corrected = correction.transform_frame(frame, data.attitude);
            cv::hconcat(frame, corrected, frame);
            cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
            cv::imshow("Frame", frame);
            cv::waitKey(0);
            cv::imwrite("output.png", frame);
        }
    }

    return 0;


}