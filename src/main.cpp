#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "camera.h"

int main(int argc, char ** argv) {
    Camera cam;
    // ros::init(argc, argv, "send_image");

    // ros::NodeHandle nh;
    // image_transport::ImageTransport it_1(nh);
    // image_transport::ImageTransport it_2(nh);

    // image_transport::Publisher pub_1 = it_1.advertise("camera1/image", 1);
    // image_transport::Publisher pub_2 = it_2.advertise("camera2/image", 1);


    // ros::Rate loop_rate(2000);


    // while(ros::ok()){
    while(1){
        std::vector<cv::Mat> acquired_image =  cam.acquire_image();
        // sensor_msgs::ImagePtr msg_1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", acquired_image[0]).toImageMsg();
        // sensor_msgs::ImagePtr msg_2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", acquired_image[1]).toImageMsg();

        // pub_1.publish(msg_1);
        // pub_1.publish(msg_2);

        // ROS_INFO("image sent!");
        // ros::spinOnce();
        // loop_rate.sleep();
        std::cout<<"FUCK MAN"<<std::endl;


    }

    return 0;
}