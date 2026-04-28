/**
* This file is part of ORB-SLAM3
*
* RViz debug extension:
*   1. /orb_slam3/camera_pose
*   2. /orb_slam3/camera_path
*   3. /orb_slam3/map_active_points
*   4. /orb_slam3/map_history_points
*   5. /orb_slam3/map_reference_points
*   6. /orb_slam3/map_local_covisible_points
*   7. /orb_slam3/map_current_frame_points
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>
#include<cstdint>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>

#include<sensor_msgs/Imu.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/point_cloud2_iterator.h>
#include<sensor_msgs/image_encodings.h>

#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Path.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM,
                 ImuGrabber *pImuGb,
                 ros::NodeHandle &nh,
                 const bool bRect,
                 const bool bClahe)
        : mpSLAM(pSLAM),
          mpImuGb(pImuGb),
          do_rectify(bRect),
          mbClahe(bClahe),
          mWorldFrameId("map"),
          mnPointCloudStride(5),
          mnPointCloudCounter(0)
    {
        pubPose = nh.advertise<geometry_msgs::PoseStamped>("/orb_slam3/camera_pose", 10);
        pubPath = nh.advertise<nav_msgs::Path>("/orb_slam3/camera_path", 10);

        pubActiveMapPoints =
            nh.advertise<sensor_msgs::PointCloud2>("/orb_slam3/map_active_points", 2);
        pubHistoricalMapPoints =
            nh.advertise<sensor_msgs::PointCloud2>("/orb_slam3/map_history_points", 2);
        pubReferenceMapPoints =
            nh.advertise<sensor_msgs::PointCloud2>("/orb_slam3/map_reference_points", 2);
        pubLocalMapPoints =
            nh.advertise<sensor_msgs::PointCloud2>("/orb_slam3/map_local_covisible_points", 2);
        pubTrackedMapPoints =
            nh.advertise<sensor_msgs::PointCloud2>("/orb_slam3/map_current_frame_points", 2);

        pathMsg.header.frame_id = mWorldFrameId;
    }

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    void PublishPoseAndPath(const Sophus::SE3f &Tcw, double timestamp);

    void PublishColoredMapPointCloud(const std::vector<ORB_SLAM3::MapPoint*> &vpMPs,
                                     ros::Publisher &pub,
                                     double timestamp,
                                     uint8_t r,
                                     uint8_t g,
                                     uint8_t b);

    void PublishAllMapLayers(double timestamp);

    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft,mBufMutexRight;

    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    ros::Publisher pubPose;
    ros::Publisher pubPath;

    ros::Publisher pubActiveMapPoints;
    ros::Publisher pubHistoricalMapPoints;
    ros::Publisher pubReferenceMapPoints;
    ros::Publisher pubLocalMapPoints;
    ros::Publisher pubTrackedMapPoints;

    nav_msgs::Path pathMsg;
    std::string mWorldFrameId;

    // 点云发布降频。图像 20Hz 时，stride=5 大约 4Hz。
    int mnPointCloudStride;
    size_t mnPointCloudCounter;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo_Inertial");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    bool bEqual = false;

    if(argc < 4 || argc > 5)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM3 Stereo_Inertial "
             << "path_to_vocabulary path_to_settings do_rectify [do_equalize]"
             << endl;
        ros::shutdown();
        return 1;
    }

    std::string sbRect(argv[3]);
    if(argc==5)
    {
        std::string sbEqual(argv[4]);
        if(sbEqual == "true")
            bEqual = true;
    }

    ORB_SLAM3::System SLAM(argv[1],
                           argv[2],
                           ORB_SLAM3::System::IMU_STEREO,
                           true);

    ImuGrabber imugb;
    ImageGrabber igb(&SLAM, &imugb, n, sbRect == "true", bEqual);

    if(igb.do_rectify)
    {
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() ||
           P_l.empty() || P_r.empty() ||
           R_l.empty() || R_r.empty() ||
           D_l.empty() || D_r.empty() ||
           rows_l==0 || rows_r==0 ||
           cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,
                                    D_l,
                                    R_l,
                                    P_l.rowRange(0,3).colRange(0,3),
                                    cv::Size(cols_l,rows_l),
                                    CV_32F,
                                    igb.M1l,
                                    igb.M2l);

        cv::initUndistortRectifyMap(K_r,
                                    D_r,
                                    R_r,
                                    P_r.rowRange(0,3).colRange(0,3),
                                    cv::Size(cols_r,rows_r),
                                    CV_32F,
                                    igb.M1r,
                                    igb.M2r);
    }

    ros::Subscriber sub_imu =
        n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);

    ros::Subscriber sub_img_left =
        n.subscribe("/camera/left/image_raw", 100, &ImageGrabber::GrabImageLeft, &igb);

    ros::Subscriber sub_img_right =
        n.subscribe("/camera/right/image_raw", 100, &ImageGrabber::GrabImageRight, &igb);

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    if(sync_thread.joinable())
        sync_thread.join();

    return 0;
}

void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
    std::lock_guard<std::mutex> lock(mBufMutexLeft);

    if(!imgLeftBuf.empty())
        imgLeftBuf.pop();

    imgLeftBuf.push(img_msg);
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
    std::lock_guard<std::mutex> lock(mBufMutexRight);

    if(!imgRightBuf.empty())
        imgRightBuf.pop();

    imgRightBuf.push(img_msg);
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return cv::Mat();
    }

    return cv_ptr->image.clone();
}

void ImageGrabber::PublishPoseAndPath(const Sophus::SE3f &Tcw, double timestamp)
{
    // 避免系统尚未初始化时发布无意义的默认位姿。
    std::vector<ORB_SLAM3::MapPoint*> tracked = mpSLAM->GetTrackedMapPoints();
    if(tracked.empty())
        return;

    Sophus::SE3f Twc = Tcw.inverse();

    Eigen::Vector3f twc = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();

    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.stamp = ros::Time(timestamp);
    poseMsg.header.frame_id = mWorldFrameId;

    poseMsg.pose.position.x = twc.x();
    poseMsg.pose.position.y = twc.y();
    poseMsg.pose.position.z = twc.z();

    poseMsg.pose.orientation.x = q.x();
    poseMsg.pose.orientation.y = q.y();
    poseMsg.pose.orientation.z = q.z();
    poseMsg.pose.orientation.w = q.w();

    pubPose.publish(poseMsg);

    pathMsg.header.stamp = poseMsg.header.stamp;
    pathMsg.poses.push_back(poseMsg);
    pubPath.publish(pathMsg);
}

void ImageGrabber::PublishColoredMapPointCloud(
    const std::vector<ORB_SLAM3::MapPoint*> &vpMPs,
    ros::Publisher &pub,
    double timestamp,
    uint8_t r,
    uint8_t g,
    uint8_t b)
{
    if(pub.getNumSubscribers() == 0)
        return;

    size_t nValid = 0;
    for(ORB_SLAM3::MapPoint* pMP : vpMPs)
    {
        if(pMP && !pMP->isBad())
            ++nValid;
    }

    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ros::Time(timestamp);
    msg.header.frame_id = mWorldFrameId;
    msg.height = 1;
    msg.is_bigendian = false;
    msg.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(nValid);

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg, "b");

    for(ORB_SLAM3::MapPoint* pMP : vpMPs)
    {
        if(!pMP || pMP->isBad())
            continue;

        Eigen::Vector3f pos = pMP->GetWorldPos();

        *iter_x = pos.x();
        *iter_y = pos.y();
        *iter_z = pos.z();

        *iter_r = r;
        *iter_g = g;
        *iter_b = b;

        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_r;
        ++iter_g;
        ++iter_b;
    }

    pub.publish(msg);
}

void ImageGrabber::PublishAllMapLayers(double timestamp)
{
    // 点云不需要每帧都发，否则 RViz 和 ORB-SLAM3 后端都会有额外负担。
    if((mnPointCloudCounter++ % static_cast<size_t>(mnPointCloudStride)) != 0)
        return;

    const bool hasAnySubscriber =
        pubActiveMapPoints.getNumSubscribers() > 0 ||
        pubHistoricalMapPoints.getNumSubscribers() > 0 ||
        pubReferenceMapPoints.getNumSubscribers() > 0 ||
        pubLocalMapPoints.getNumSubscribers() > 0 ||
        pubTrackedMapPoints.getNumSubscribers() > 0;

    if(!hasAnySubscriber)
        return;

    // 颜色语义：
    // 活跃地图：绿色
    // 历史地图：灰色
    // 参考地图：黄色
    // 当前帧局部 / 共视地图：青色
    // 当前帧真实跟踪到的地图点：红色
    PublishColoredMapPointCloud(mpSLAM->GetActiveMapPoints(),
                                pubActiveMapPoints,
                                timestamp,
                                0, 255, 0);

    PublishColoredMapPointCloud(mpSLAM->GetHistoricalMapPoints(),
                                pubHistoricalMapPoints,
                                timestamp,
                                160, 160, 160);

    PublishColoredMapPointCloud(mpSLAM->GetReferenceMapPoints(),
                                pubReferenceMapPoints,
                                timestamp,
                                255, 255, 0);

    PublishColoredMapPointCloud(mpSLAM->GetLocalMapPoints(),
                                pubLocalMapPoints,
                                timestamp,
                                0, 255, 255);

    PublishColoredMapPointCloud(mpSLAM->GetTrackedMapPoints(),
                                pubTrackedMapPoints,
                                timestamp,
                                255, 0, 0);
}

void ImageGrabber::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while(ros::ok())
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0;
        double tImRight = 0;

        if(!imgLeftBuf.empty() && !imgRightBuf.empty() && !mpImuGb->imuBuf.empty())
        {
            tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            tImRight = imgRightBuf.front()->header.stamp.toSec();

            mBufMutexRight.lock();
            while((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1)
            {
                imgRightBuf.pop();
                tImRight = imgRightBuf.front()->header.stamp.toSec();
            }
            mBufMutexRight.unlock();

            mBufMutexLeft.lock();
            while((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1)
            {
                imgLeftBuf.pop();
                tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            }
            mBufMutexLeft.unlock();

            if((tImLeft - tImRight) > maxTimeDiff ||
               (tImRight - tImLeft) > maxTimeDiff)
            {
                continue;
            }

            if(tImLeft > mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;

            mBufMutexLeft.lock();
            imLeft = GetImage(imgLeftBuf.front());
            imgLeftBuf.pop();
            mBufMutexLeft.unlock();

            mBufMutexRight.lock();
            imRight = GetImage(imgRightBuf.front());
            imgRightBuf.pop();
            mBufMutexRight.unlock();

            if(imLeft.empty() || imRight.empty())
                continue;

            vector<ORB_SLAM3::IMU::Point> vImuMeas;

            mpImuGb->mBufMutex.lock();

            if(!mpImuGb->imuBuf.empty())
            {
                vImuMeas.clear();

                while(!mpImuGb->imuBuf.empty() &&
                      mpImuGb->imuBuf.front()->header.stamp.toSec() <= tImLeft)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();

                    cv::Point3f acc(
                        mpImuGb->imuBuf.front()->linear_acceleration.x,
                        mpImuGb->imuBuf.front()->linear_acceleration.y,
                        mpImuGb->imuBuf.front()->linear_acceleration.z);

                    cv::Point3f gyr(
                        mpImuGb->imuBuf.front()->angular_velocity.x,
                        mpImuGb->imuBuf.front()->angular_velocity.y,
                        mpImuGb->imuBuf.front()->angular_velocity.z);

                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));

                    mpImuGb->imuBuf.pop();
                }
            }

            mpImuGb->mBufMutex.unlock();

            if(mbClahe)
            {
                mClahe->apply(imLeft, imLeft);
                mClahe->apply(imRight, imRight);
            }

            if(do_rectify)
            {
                cv::remap(imLeft, imLeft, M1l, M2l, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r, M2r, cv::INTER_LINEAR);
            }

            Sophus::SE3f Tcw =
                mpSLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

            PublishPoseAndPath(Tcw, tImLeft);
            PublishAllMapLayers(tImLeft);
        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    imuBuf.push(imu_msg);
}
