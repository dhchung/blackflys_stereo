#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>


#include <image_transport/image_transport.h>

#include "camera.h"

int main(int argc, char ** argv) {
    Camera cam;
    ros::init(argc, argv, "send_image");

    ros::NodeHandle nh("~");

    bool image_show = false;
    bool ros_param_image_show;
    bool params_passed = nh.getParam("image_show", ros_param_image_show);    

    if(!params_passed) {
        std::cout<<"Input should be either true or false"<<std::endl;
        std::cout<<"ex) $ rosrun blackfly_stereo_rosmsg send_img"<<std::endl;
        std::cout<<"ex) $ rosrun blackfly_stereo_rosmsg send_img _image_show:=true"<<std::endl;
        std::cout<<"ex) $ rosrun blackfly_stereo_rosmsg send_img _image_show:=false"<<std::endl;
        std::cout<<"Image show is set to false by default"<<std::endl;
        image_show = false;
    } else {
        if(ros_param_image_show) {
            std::cout<<"Image show is set to true"<<std::endl;
            image_show = true;
        } else {
            std::cout<<"Image show is set to false"<<std::endl;
            image_show = false;
        }
    }



    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera1/image",1 );

    cv_bridge::CvImage img_bridge;

    // sensor_msgs::Image img1;
    // sensor_msgs::Image img2;
    std_msgs::Header header;

    // ros::Publisher pub_img1 = nh.advertise<sensor_msgs::Image>("camera1/image", 1);
    // ros::Publisher pub_img2 = nh.advertise<sensor_msgs::Image>("camera2/image", 1);

    ros::Rate loop_rate(20);
    int count = 0;
    while(ros::ok()){
    // while(1){
        bool img_ok = false;
        double time;
        std::vector<cv::Mat> acquired_image =  cam.acquire_image(time, img_ok);
        if(img_ok) {
            header.seq = count;
            header.stamp.fromSec(time);
            // img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, acquired_image[0]);
            // img_bridge.toImageMsg(img1);
            // img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, acquired_image[1]);
            // img_bridge.toImageMsg(img2);

            // pub_img1.publish(img1);
            // pub_img2.publish(img2);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", acquired_image[0]).toImageMsg();
            pub.publish(msg);

            // cv::Mat concat;
            // cv::hconcat(acquired_image[0], acquired_image[1], concat);
            // cv::resize(concat, concat, cv::Size(concat.cols/2, concat.rows/2));
            // cv::imshow("Stereo Image", concat);
            // cv::waitKey(1);

            if(image_show) {
                cv::Mat disp_img;
                acquired_image[0].copyTo(disp_img);
                cv::resize(disp_img, disp_img, cv::Size(disp_img.cols/2, disp_img.rows/2));
                cv::imshow("Left image", disp_img);
                cv::waitKey(1);
            }

            ROS_INFO("image sent!");
            ++count;
        }
        ros::spinOnce();
        // loop_rate.sleep();
    }
    return 0;
}