/**
* Reusable RViz visualizer for ORB-SLAM3 ROS examples.
*
* Put this file at:
*   ORB_SLAM3/Examples/ROS/ORB_SLAM3/src/RvizVisualizer.cc
*/

#include "../include/RvizVisualizer.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Point.h>

namespace ORB_SLAM3
{

RvizVisualizer::RvizVisualizer(System* pSLAM,
                               ros::NodeHandle& nh,
                               const std::string& worldFrame,
                               int pointCloudStride,
                               const std::string& topicPrefix)
    : mpSLAM(pSLAM),
      mWorldFrameId(worldFrame),
      mTopicPrefix(topicPrefix),
      mnPointCloudStride(pointCloudStride > 0 ? pointCloudStride : 1),
      mnPointCloudCounter(0)
{
    mPubPose = nh.advertise<geometry_msgs::PoseStamped>(Topic("camera_pose"), 10);
    mPubPath = nh.advertise<nav_msgs::Path>(Topic("camera_path"), 10);
    mPubRobotAxes = nh.advertise<visualization_msgs::MarkerArray>(Topic("camera_axes"), 10);

    mPubActiveMapPoints =
        nh.advertise<sensor_msgs::PointCloud2>(Topic("map_active_points"), 2);
    mPubHistoricalMapPoints =
        nh.advertise<sensor_msgs::PointCloud2>(Topic("map_history_points"), 2);
    mPubReferenceMapPoints =
        nh.advertise<sensor_msgs::PointCloud2>(Topic("map_reference_points"), 2);
    mPubLocalMapPoints =
        nh.advertise<sensor_msgs::PointCloud2>(Topic("map_local_covisible_points"), 2);
    mPubTrackedMapPoints =
        nh.advertise<sensor_msgs::PointCloud2>(Topic("map_current_frame_points"), 2);

    mPathMsg.header.frame_id = mWorldFrameId;
}

std::string RvizVisualizer::Topic(const std::string& name) const
{
    if(mTopicPrefix.empty())
        return name;

    if(mTopicPrefix[mTopicPrefix.size()-1] == '/')
        return mTopicPrefix + name;

    return mTopicPrefix + "/" + name;
}

void RvizVisualizer::PublishAll(const Sophus::SE3f& Tcw, double timestamp)
{
    PublishPoseAndPath(Tcw, timestamp);
    PublishMapLayers(timestamp);
}

bool RvizVisualizer::HasValidTracking() const
{
    if(!mpSLAM)
        return false;

    // 避免系统尚未初始化时发布无意义的默认位姿。
    // 对 mono / stereo / rgbd / inertial 均适用：只要当前帧没有关联地图点，就先不追加轨迹。
    return !mpSLAM->GetTrackedMapPoints().empty();
}

void RvizVisualizer::PublishPoseAndPath(const Sophus::SE3f& Tcw, double timestamp)
{
    if(!HasValidTracking())
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

    mPubPose.publish(poseMsg);
    PublishRobotAxes(Twc, timestamp);

    mPathMsg.header.stamp = poseMsg.header.stamp;
    mPathMsg.poses.push_back(poseMsg);
    mPubPath.publish(mPathMsg);
}

visualization_msgs::Marker RvizVisualizer::CreateAxisMarker(int id,
                                                            const ros::Time& stamp,
                                                            const Eigen::Vector3f& origin,
                                                            const Eigen::Vector3f& direction,
                                                            float r,
                                                            float g,
                                                            float b) const
{
    visualization_msgs::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = mWorldFrameId;
    marker.ns = "robot_axes";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    geometry_msgs::Point start;
    start.x = origin.x();
    start.y = origin.y();
    start.z = origin.z();

    geometry_msgs::Point end;
    end.x = origin.x() + direction.x();
    end.y = origin.y() + direction.y();
    end.z = origin.z() + direction.z();

    marker.points.push_back(start);
    marker.points.push_back(end);

    marker.scale.x = 0.03;  // shaft diameter
    marker.scale.y = 0.08;  // head diameter
    marker.scale.z = 0.12;  // head length

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0.0);

    return marker;
}

void RvizVisualizer::PublishRobotAxes(const Sophus::SE3f& Twc, double timestamp)
{
    if(mPubRobotAxes.getNumSubscribers() == 0)
        return;

    const ros::Time stamp(timestamp);
    const Eigen::Vector3f origin = Twc.translation();
    const Eigen::Matrix3f Rwc = Twc.rotationMatrix();
    const float axisLength = 0.35f;

    visualization_msgs::MarkerArray axes;
    axes.markers.push_back(CreateAxisMarker(0, stamp, origin, Rwc.col(0) * axisLength, 1.0f, 0.0f, 0.0f));
    axes.markers.push_back(CreateAxisMarker(1, stamp, origin, Rwc.col(1) * axisLength, 0.0f, 1.0f, 0.0f));
    axes.markers.push_back(CreateAxisMarker(2, stamp, origin, Rwc.col(2) * axisLength, 0.0f, 0.2f, 1.0f));

    mPubRobotAxes.publish(axes);
}

bool RvizVisualizer::HasAnyMapSubscriber() const
{
    return mPubActiveMapPoints.getNumSubscribers() > 0 ||
           mPubHistoricalMapPoints.getNumSubscribers() > 0 ||
           mPubReferenceMapPoints.getNumSubscribers() > 0 ||
           mPubLocalMapPoints.getNumSubscribers() > 0 ||
           mPubTrackedMapPoints.getNumSubscribers() > 0;
}

void RvizVisualizer::PublishMapLayers(double timestamp)
{
    if(!mpSLAM)
        return;

    // 点云不需要每帧都发，否则 RViz 和 ORB-SLAM3 后端都会有额外负担。
    if((mnPointCloudCounter++ % static_cast<size_t>(mnPointCloudStride)) != 0)
        return;

    if(!HasAnyMapSubscriber())
        return;

    // 颜色语义：
    // 活跃地图：绿色
    // 历史地图：灰色
    // 参考地图：黄色
    // 当前帧局部 / 共视地图：青色
    // 当前帧真实跟踪到的地图点：红色
    PublishColoredMapPointCloud(mpSLAM->GetActiveMapPoints(),
                                mPubActiveMapPoints,
                                timestamp,
                                0, 255, 0);

    PublishColoredMapPointCloud(mpSLAM->GetHistoricalMapPoints(),
                                mPubHistoricalMapPoints,
                                timestamp,
                                160, 160, 160);

    PublishColoredMapPointCloud(mpSLAM->GetReferenceMapPoints(),
                                mPubReferenceMapPoints,
                                timestamp,
                                255, 255, 0);

    PublishColoredMapPointCloud(mpSLAM->GetLocalMapPoints(),
                                mPubLocalMapPoints,
                                timestamp,
                                0, 255, 255);

    PublishColoredMapPointCloud(mpSLAM->GetTrackedMapPoints(),
                                mPubTrackedMapPoints,
                                timestamp,
                                255, 0, 0);
}

void RvizVisualizer::PublishColoredMapPointCloud(const std::vector<MapPoint*>& vpMPs,
                                                 ros::Publisher& pub,
                                                 double timestamp,
                                                 uint8_t r,
                                                 uint8_t g,
                                                 uint8_t b)
{
    if(pub.getNumSubscribers() == 0)
        return;

    size_t nValid = 0;
    for(MapPoint* pMP : vpMPs)
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

    for(MapPoint* pMP : vpMPs)
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

void RvizVisualizer::ResetPath()
{
    mPathMsg.poses.clear();
    mPathMsg.header.frame_id = mWorldFrameId;
}

} // namespace ORB_SLAM3
