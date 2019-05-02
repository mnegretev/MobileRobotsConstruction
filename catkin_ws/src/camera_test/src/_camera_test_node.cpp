#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

int main(int argc, char** argv)
{
    std::cout << "Initalizing camera-test node..." << std::endl;
    ros::init(argc, argv, "camera_test");
    ros::NodeHandle n;
    ros::Publisher pubImage = n.advertise<sensor_msgs::Image>("/minirobot/hardware/image", 1);
    ros::Rate loop(30);

    cv::VideoCapture cap(0);
    if(!cap.isOpened())
    {
        std::cout << "Cannot open camera ... :'(" << std::endl;
        return -1;
    }
    sensor_msgs::Image msgImage;
    msgImage.header.frame_id = "camera_link";
    cv::waitKey(100);
    cv::Mat firstFrame;
    cap >> firstFrame;
    std::cout << "Rows: "<< firstFrame.rows<<"  Cols: "<< firstFrame.cols << "  ElemSize: "<< firstFrame.elemSize()<< std::endl;
    int imageSize = firstFrame.rows*firstFrame.cols*firstFrame.elemSize();
    msgImage.data.resize(imageSize);
    msgImage.height = firstFrame.rows;
    msgImage.width  = firstFrame.cols;

    while(ros::ok() && cv::waitKey(15) != 27)
    {
        cv::Mat frame;
        cap >> frame;
        msgImage.header.stamp = ros::Time::now();
        memcpy(msgImage.data.data(), frame.data, imageSize);
        pubImage.publish(msgImage);
        ros::spinOnce();
    }
    return 0;
}
