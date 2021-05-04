#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <vector>

class Camera{
public:
    Camera();
    ~Camera();

    Spinnaker::SystemPtr system;
    void print_spinnaker_version();
    bool camera_ready;

    std::vector<cv::Mat> acquire_image();
    void set_camera();


private:
    std::string cam_1_serial;
    std::string cam_2_serial;

    Spinnaker::CameraPtr cam_1;
    Spinnaker::CameraPtr cam_2;
    Spinnaker::CameraList camList;

};