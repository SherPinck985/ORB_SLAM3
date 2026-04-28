/**
* Reusable RViz visualizer for ORB-SLAM3 ROS examples.
*
* Put this file at:
*   ORB_SLAM3/Examples/ROS/ORB_SLAM3/include/RvizVisualizer.h
*/

#ifndef ORB_SLAM3_RVIZ_VISUALIZER_H
#define ORB_SLAM3_RVIZ_VISUALIZER_H

#include <string>
#include <vector>
#include <cstdint>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>

#include "../../../include/System.h"
#include "../../../include/MapPoint.h"

namespace ORB_SLAM3
{

class RvizVisualizer
{
public:
    /**
     * @brief 构造通用 RViz 可视化器。
     * @param pSLAM ORB-SLAM3 System 指针。
     * @param nh ROS NodeHandle，用于创建 publisher。
     * @param worldFrame RViz Fixed Frame，默认 map。
     * @param pointCloudStride 点云发布降频步长，默认每 5 帧发布一次。
     * @param topicPrefix RViz 调试 topic 前缀，默认 /orb_slam3。
     */
    RvizVisualizer(System* pSLAM,
                   ros::NodeHandle& nh,
                   const std::string& worldFrame = "map",
                   int pointCloudStride = 5,
                   const std::string& topicPrefix = "/orb_slam3");

    /**
     * @brief 发布当前相机位姿和轨迹。
     * @param Tcw ORB-SLAM3 返回的世界到相机位姿。
     * @param timestamp 当前图像时间戳。
     */
    void PublishPoseAndPath(const Sophus::SE3f& Tcw, double timestamp);

    /**
     * @brief 发布多层地图点云。
     * @param timestamp 当前图像时间戳。
     */
    void PublishMapLayers(double timestamp);

    /**
     * @brief 一次性发布位姿、轨迹和地图层。
     */
    void PublishAll(const Sophus::SE3f& Tcw, double timestamp);

    /**
     * @brief 清空 RViz 轨迹缓存。
     */
    void ResetPath();

private:
    std::string Topic(const std::string& name) const;

    void PublishColoredMapPointCloud(const std::vector<MapPoint*>& vpMPs,
                                     ros::Publisher& pub,
                                     double timestamp,
                                     uint8_t r,
                                     uint8_t g,
                                     uint8_t b);

    bool HasAnyMapSubscriber() const;
    bool HasValidTracking() const;

private:
    System* mpSLAM;

    std::string mWorldFrameId;
    std::string mTopicPrefix;

    ros::Publisher mPubPose;
    ros::Publisher mPubPath;

    ros::Publisher mPubActiveMapPoints;
    ros::Publisher mPubHistoricalMapPoints;
    ros::Publisher mPubReferenceMapPoints;
    ros::Publisher mPubLocalMapPoints;
    ros::Publisher mPubTrackedMapPoints;

    nav_msgs::Path mPathMsg;

    int mnPointCloudStride;
    size_t mnPointCloudCounter;
};

} // namespace ORB_SLAM3

#endif // ORB_SLAM3_RVIZ_VISUALIZER_H
