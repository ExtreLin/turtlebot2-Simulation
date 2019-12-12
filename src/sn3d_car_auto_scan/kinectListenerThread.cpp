#include "kinectListenerThread.h"
#include<geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include<tf/LinearMath/Vector3.h>
#include<nav_msgs/OccupancyGrid.h>
#include<move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

const tf::StampedTransform& CKinectListenerThread::get_current_rt()
{
    if(kl_ != nullptr)
        kl_->tflistener_ .lookupTransform("odom", "base_footprint",ros::Time(0), stamped_transform_);  
    return stamped_transform_;
}

void CKinectListenerThread::slotCvImage(const cv_bridge::CvImagePtr qImage)
{
    emit sigCvImage(qImage); 
}


void CKinectListenerThread::slotCvImageDepth(const cv_bridge::CvImagePtr qImage)
{
    if(!isInit_)
    {
        original_rt = get_current_rt();
        isInit_ = true;
    }
    emit sigCvImageDepth(qImage);
}

void CKinectListenerThread::slotCarRun(const float& liner,const float& angler)
{
   if(kl_ == nullptr)
        return;
    geometry_msgs::Twist speed;
    speed = geometry_msgs::Twist();
    speed.linear.x = liner;
    speed.angular.z = angler;  
    if(isnan(liner)||isnan(angler))
       return;
    kl_->runPub_.publish(speed);   
} 

void CKinectListenerThread::slotNavigation(const Eigen::Matrix<float,7,1>& pose)
{
     if(kl_==nullptr)
        return;
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = pose[0];
    ps.pose.position.y = pose[1];
    ps.pose.position.z = pose[2];
    ps.pose.orientation.x = pose[3];
    ps.pose.orientation.y = pose[4];
    ps.pose.orientation.z = pose[5];
    ps.pose.orientation.w = pose[6];
    kl_->goalPub_.publish(ps);
    emit sigWaitMovingFinished();
}

void CKinectListenerThread::slotWaitMovingFinished()
{
    timer_.start(1000);
}

void CKinectListenerThread::slotTimeOut()
{
    if(kl_->cmd_vel_.angular.z == 0 && kl_->cmd_vel_.linear.x == 0)
    {
        if(waitNum_ == 0)
        {
              waitNum_++;//多等一个周期
              return;
        }
        waitNum_ = 0;
        tf::StampedTransform st =  get_current_rt();
        tf::Vector3  t =  st.getOrigin();
        timer_.stop();
        emit sigMoveFinished(Eigen::Vector2f(t.x(),t.y()));
    }
    else{
        waitNum_ = 0;
    }
       //timer_.start(1000);
}

void CKinectListenerThread::slotCameraInfo(const sensor_msgs::CameraInfo& msg)
{
    emit sigCameraInfo(msg);
}

void CKinectListenerThread::run()
{
    ros::init(argc_,argv_,"my_Kinect_listener");
    kl_ = new  CKinectListener();
    ros::AsyncSpinner spinner(8); // Use8 threads
    connect(kl_,SIGNAL(sigCvImage(const cv_bridge::CvImagePtr)),this,SLOT(slotCvImage(const cv_bridge::CvImagePtr)));
    connect(kl_,SIGNAL(sigCvImageDepth(const cv_bridge::CvImagePtr )),this,SLOT(slotCvImageDepth(const cv_bridge::CvImagePtr)));
    connect(kl_,SIGNAL(sigCameraInfo(const sensor_msgs::CameraInfo&)),this,SLOT(slotCameraInfo(const sensor_msgs::CameraInfo&)));
    spinner.start();
    ros::waitForShutdown();       
}