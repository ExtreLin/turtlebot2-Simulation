#ifndef GET_ROS_DATA_H
#define GET_ROS_DATA_H
#include<opencv/cv.h>
 #include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include<qobject.h>
#include<QImage>
#include<sensor_msgs/PointCloud2.h>

class CKinectListener:public QObject
{
    Q_OBJECT
public:
    CKinectListener();
    image_transport::Subscriber sub ;  
    image_transport::Subscriber depthSub;
    ros::Subscriber ptSub;
    ros::Publisher  runPub;
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    QVector<QRgb> colorTable;
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void depthCb(const sensor_msgs::ImageConstPtr& msg);
    void pointsCb(const sensor_msgs::PointCloud2& msg);
signals:
    void  sigCvImage(const QImage& qImage);
    void  sigCvImageDepth(const QImage& qImage);
};
#endif