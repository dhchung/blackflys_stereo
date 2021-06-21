#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "camera.h"

int main(int argc, char ** argv) {
    Camera cam;
    ros::init(argc, argv, "send_image");

    ros::NodeHandle nh;

    cv_bridge::CvImage img_bridge;

    sensor_msgs::Image img1;
    sensor_msgs::Image img2;
    std_msgs::Header header;

    ros::Publisher pub_img1 = nh.advertise<sensor_msgs::Image>("camera1/image", 1);
    ros::Publisher pub_img2 = nh.advertise<sensor_msgs::Image>("camera2/image", 1);

    ros::Rate loop_rate(20);
    int count = 0;
    while(ros::ok()){
    // while(1){

        std::vector<cv::Mat> acquired_image =  cam.acquire_image();

        header.seq = count;
        header.stamp = ros::Time::now();
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, acquired_image[0]);
        img_bridge.toImageMsg(img1);
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, acquired_image[1]);
        img_bridge.toImageMsg(img2);

        pub_img1.publish(img1);
        pub_img2.publish(img2);


        cv::Mat concat;
        cv::hconcat(acquired_image[0], acquired_image[1], concat);
        cv::resize(concat, concat, cv::Size(concat.cols/2, concat.rows/2));
        cv::imshow("Stereo Image", concat);
        cv::waitKey(1);



        ROS_INFO("image sent!");
        ros::spinOnce();
        loop_rate.sleep();
        ++count;

    }
    return 0;
}