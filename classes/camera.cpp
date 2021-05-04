#include "camera.h"
#include <chrono>


Camera::Camera(){
    std::cout<<"WTF"<<std::endl;
    system = Spinnaker::System::GetInstance();

    cam_1_serial = "20515474"; // Primary Camera
    cam_2_serial = "21102408"; // Secondary Camera
    camera_ready = false;
    cam_1 = nullptr;
    cam_2 = nullptr;
    set_camera();
}
Camera::~Camera(){

    if(camera_ready){
        cam_1->EndAcquisition();
        cam_2->EndAcquisition();
    }
    camList.Clear();

}

void Camera::print_spinnaker_version(){

    const Spinnaker::LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
    std::cout << "Spinnaker library version: " << spinnakerLibraryVersion.major << "." << spinnakerLibraryVersion.minor
         << "." << spinnakerLibraryVersion.type << "." << spinnakerLibraryVersion.build << std::endl
         << std::endl;
}


void Camera::set_camera(){
    camList = system->GetCameras();
    const unsigned int numCameras = camList.GetSize();
    std::cout<<"Number of cameras detected: "<<numCameras<<std::endl;

    if(numCameras != 2) {
        camera_ready = false;
    } else {
        camera_ready = true;
        cam_1 = camList.GetBySerial(cam_1_serial);
        cam_2 = camList.GetBySerial(cam_2_serial);
        cam_1->Init();
        cam_1->LineSelector.SetValue(Spinnaker::LineSelector_Line2);
        cam_1->V3_3Enable.SetValue(true);

        cam_2->Init();
        cam_2->TriggerMode.SetValue(Spinnaker::TriggerMode_Off);
        cam_2->TriggerSource.SetValue(Spinnaker::TriggerSource_Line3);
        cam_2->TriggerOverlap.SetValue(Spinnaker::TriggerOverlap_ReadOut);
        cam_2->TriggerMode.SetValue(Spinnaker::TriggerMode_On);

        cam_1->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);
        cam_2->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);

        Spinnaker::GenApi::INodeMap& sNodeMap_1 = cam_1->GetTLStreamNodeMap();
        Spinnaker::GenApi::INodeMap& sNodeMap_2 = cam_2->GetTLStreamNodeMap();


        Spinnaker::GenApi::CEnumerationPtr ptrHandlingMode_1 = sNodeMap_1.GetNode("StreamBufferHandlingMode");
        if (!IsAvailable(ptrHandlingMode_1) || !IsWritable(ptrHandlingMode_1))
        {
            std::cout << "Unable to set Buffer Handling mode (node retrieval). Aborting..." << std::endl;
        }

        Spinnaker::GenApi::CEnumEntryPtr ptrHandlingModeEntry_1 = ptrHandlingMode_1->GetCurrentEntry();
        if (!IsAvailable(ptrHandlingModeEntry_1) || !IsReadable(ptrHandlingModeEntry_1))
        {
            std::cout << "Unable to set Buffer Handling mode (Entry retrieval). Aborting..." << std::endl;
        }

        Spinnaker::GenApi::CEnumerationPtr ptrHandlingMode_2 = sNodeMap_2.GetNode("StreamBufferHandlingMode");
        if (!IsAvailable(ptrHandlingMode_2) || !IsWritable(ptrHandlingMode_2))
        {
            std::cout << "Unable to set Buffer Handling mode (node retrieval). Aborting..." << std::endl;
        }

        Spinnaker::GenApi::CEnumEntryPtr ptrHandlingModeEntry_2 = ptrHandlingMode_2->GetCurrentEntry();
        if (!IsAvailable(ptrHandlingModeEntry_2) || !IsReadable(ptrHandlingModeEntry_2))
        {
            std::cout << "Unable to set Buffer Handling mode (Entry retrieval). Aborting..." << std::endl;
        }


        ptrHandlingModeEntry_1 = ptrHandlingMode_1->GetEntryByName("NewestFirst");
        ptrHandlingMode_1->SetIntValue(ptrHandlingModeEntry_1->GetValue());
        std::cout << std::endl
                << std::endl
                << "Buffer Handling Mode has been set to " << ptrHandlingModeEntry_1->GetDisplayName() << std::endl;

        ptrHandlingModeEntry_2 = ptrHandlingMode_2->GetEntryByName("NewestFirst");
        ptrHandlingMode_2->SetIntValue(ptrHandlingModeEntry_2->GetValue());
        std::cout << std::endl
                << std::endl
                << "Buffer Handling Mode has been set to " << ptrHandlingModeEntry_2->GetDisplayName() << std::endl;




        cam_2->BeginAcquisition();
        cam_1->BeginAcquisition();






    }
}


std::vector<cv::Mat> Camera::acquire_image(){
    auto t0 = std::chrono::high_resolution_clock::now();

    Spinnaker::ImagePtr img1 = cam_1->GetNextImage();
    Spinnaker::ImagePtr img2 = cam_2->GetNextImage();

    auto t1 = std::chrono::high_resolution_clock::now();

    std::vector<cv::Mat> image_vector;
    image_vector.resize(2);

    if (img1->IsIncomplete())
    {
        std::cout << "Image 1 incomplete: " << Spinnaker::Image::GetImageStatusDescription(img1->GetImageStatus())<< std::endl;
        return image_vector;
    }
    if (img2->IsIncomplete())
    {
        std::cout << "Image 2 incomplete: " << Spinnaker::Image::GetImageStatusDescription(img2->GetImageStatus())<< std::endl;
        return image_vector;
    }

    const size_t width_1 = img1->GetWidth();
    const size_t height_1 = img1->GetHeight();
    Spinnaker::ImagePtr convertedImage_1 = img1->Convert(Spinnaker::PixelFormat_BGR8, Spinnaker::BILINEAR);
    cv::Mat imgMat_1 = cv::Mat(cv::Size(width_1, height_1), CV_8UC3, convertedImage_1->GetData());
    cv::resize(imgMat_1, imgMat_1, cv::Size(imgMat_1.cols/6, imgMat_1.rows/6));


    const size_t width_2 = img2->GetWidth();
    const size_t height_2 = img2->GetHeight();
    Spinnaker::ImagePtr convertedImage_2 = img2->Convert(Spinnaker::PixelFormat_BGR8, Spinnaker::BILINEAR);
    cv::Mat imgMat_2 = cv::Mat(cv::Size(width_2, height_2), CV_8UC3, convertedImage_2->GetData());
    cv::resize(imgMat_2, imgMat_2, cv::Size(imgMat_2.cols/6, imgMat_2.rows/6));


    image_vector[0] = imgMat_1;
    image_vector[1] = imgMat_2;

    auto t2 = std::chrono::high_resolution_clock::now();

    cv::Mat concat;
    cv::hconcat(imgMat_1, imgMat_2, concat);
    // cv::resize(concat, concat, cv::Size(concat.cols/6, concat.rows/6));
    // cv::imshow("Fuck", concat);
    // cv::waitKey(1);

    img1->Release();
    img2->Release();

    auto t01 = 1.e-3*std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();
    auto t12 = 1.e-3*std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();

    std::cout<<"t01 : "<<t01<<std::endl;
    std::cout<<"t12 : "<<t12<<std::endl;


    return image_vector;
}