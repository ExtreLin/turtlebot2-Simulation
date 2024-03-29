#ifndef CKINECT_LISTENER_H
#define CKINECT_LISTENER_H
#include<opencv/cv.h>
 #include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "cv_bridge.h"
#include<qobject.h>
#include<QImage>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/CameraInfo.h>
#include<tf/transform_listener.h>

class CKinectListener:public QObject
{
    Q_OBJECT
public:
    CKinectListener();
    //image
    image_transport::Subscriber sub_ ;  
    image_transport::Subscriber depthSub_;

    ros::Subscriber ptSub_;
    ros::Subscriber  camSub_;
    ros::Subscriber runSub_;

    ros::Publisher  runPub_;
    ros::Publisher  goalPub_;

    tf::TransformListener  tflistener_;
    geometry_msgs::Twist cmd_vel_;
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void depthCb(const sensor_msgs::ImageConstPtr& msg);
    void pointsCb(const sensor_msgs::PointCloud2& msg);
    void cameraCb(const sensor_msgs::CameraInfo& msg);
    void velocityCb(const geometry_msgs::Twist& cmd_vel);
signals:
    void  sigCvImage(const cv_bridge::CvImagePtr );
    void  sigCvImageDepth(const cv_bridge::CvImagePtr);
    void  sigCameraInfo(const sensor_msgs::CameraInfo& msg);
};
#endif