/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "ImuTypes.h"

#include "GeometricCamera.h"
#include "SerializationUtils.h"

#include <mutex>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>


namespace ORB_SLAM3
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class GeometricCamera;

/**
 * @brief 关键帧类
 *
 * KeyFrame 是 ORB-SLAM3 地图中的核心结点，既保存了一帧图像中提取出的 ORB 特征，
 * 也维护了与地图点、其他关键帧、IMU 状态之间的关系。
 *
 * 它在系统中的主要职责包括：
 * 1. 保存来自 Frame 的关键点、描述子、金字塔尺度、相机参数和位姿；
 * 2. 保存关键帧对 MapPoint 的观测关系；
 * 3. 维护共视图、生成树、闭环边和地图融合边；
 * 4. 支持 BoW 查询、局部建图、闭环、重定位和序列化存储；
 * 5. 在惯性模式下保存速度、偏置和 IMU 预积分信息。
 *
 * 需要注意：
 * KeyFrame 本身并不重新执行 ORB 特征提取。ORB 关键点、方向和描述子已经在 Frame
 * 构造阶段由 ORBextractor 计算完成，KeyFrame 只是把这些结果固化到地图节点中。
 */
class KeyFrame
{
    friend class boost::serialization::access;

    /**
     * @brief Boost 序列化接口
     * @tparam Archive 序列化归档类型
     * @param ar       归档对象
     * @param version  版本号
     *
     * 该函数负责把关键帧中可持久化的数据写入/读出归档，包括：
     * 1. 基本编号与时间戳；
     * 2. 相机参数、关键点、描述子和 BoW；
     * 3. 位姿、尺度信息和图像网格；
     * 4. 关键帧图结构关系；
     * 5. IMU 相关数据。
     *
     * 注意：大量运行时临时变量不会序列化，而是通过 PreSave / PostLoad 组合恢复。
     */
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & mnId;
        ar & const_cast<long unsigned int&>(mnFrameId);
        ar & const_cast<double&>(mTimeStamp);
        // Grid
        ar & const_cast<int&>(mnGridCols);
        ar & const_cast<int&>(mnGridRows);
        ar & const_cast<float&>(mfGridElementWidthInv);
        ar & const_cast<float&>(mfGridElementHeightInv);

        // Variables of tracking
        //ar & mnTrackReferenceForFrame;
        //ar & mnFuseTargetForKF;
        // Variables of local mapping
        //ar & mnBALocalForKF;
        //ar & mnBAFixedForKF;
        //ar & mnNumberOfOpt;
        // Variables used by KeyFrameDatabase
        //ar & mnLoopQuery;
        //ar & mnLoopWords;
        //ar & mLoopScore;
        //ar & mnRelocQuery;
        //ar & mnRelocWords;
        //ar & mRelocScore;
        //ar & mnMergeQuery;
        //ar & mnMergeWords;
        //ar & mMergeScore;
        //ar & mnPlaceRecognitionQuery;
        //ar & mnPlaceRecognitionWords;
        //ar & mPlaceRecognitionScore;
        //ar & mbCurrentPlaceRecognition;
        // Variables of loop closing
        //serializeMatrix(ar,mTcwGBA,version);
        //serializeMatrix(ar,mTcwBefGBA,version);
        //serializeMatrix(ar,mVwbGBA,version);
        //serializeMatrix(ar,mVwbBefGBA,version);
        //ar & mBiasGBA;
        //ar & mnBAGlobalForKF;
        // Variables of Merging
        //serializeMatrix(ar,mTcwMerge,version);
        //serializeMatrix(ar,mTcwBefMerge,version);
        //serializeMatrix(ar,mTwcBefMerge,version);
        //serializeMatrix(ar,mVwbMerge,version);
        //serializeMatrix(ar,mVwbBefMerge,version);
        //ar & mBiasMerge;
        //ar & mnMergeCorrectedForKF;
        //ar & mnMergeForKF;
        //ar & mfScaleMerge;
        //ar & mnBALocalForMerge;

        // Scale
        ar & mfScale;
        // Calibration parameters
        ar & const_cast<float&>(fx);
        ar & const_cast<float&>(fy);
        ar & const_cast<float&>(invfx);
        ar & const_cast<float&>(invfy);
        ar & const_cast<float&>(cx);
        ar & const_cast<float&>(cy);
        ar & const_cast<float&>(mbf);
        ar & const_cast<float&>(mb);
        ar & const_cast<float&>(mThDepth);
        serializeMatrix(ar, mDistCoef, version);
        // Number of Keypoints
        ar & const_cast<int&>(N);
        // KeyPoints
        serializeVectorKeyPoints<Archive>(ar, mvKeys, version);
        serializeVectorKeyPoints<Archive>(ar, mvKeysUn, version);
        ar & const_cast<vector<float>& >(mvuRight);
        ar & const_cast<vector<float>& >(mvDepth);
        serializeMatrix<Archive>(ar,mDescriptors,version);
        // BOW
        ar & mBowVec;
        ar & mFeatVec;
        // Pose relative to parent
        serializeSophusSE3<Archive>(ar, mTcp, version);
        // Scale
        ar & const_cast<int&>(mnScaleLevels);
        ar & const_cast<float&>(mfScaleFactor);
        ar & const_cast<float&>(mfLogScaleFactor);
        ar & const_cast<vector<float>& >(mvScaleFactors);
        ar & const_cast<vector<float>& >(mvLevelSigma2);
        ar & const_cast<vector<float>& >(mvInvLevelSigma2);
        // Image bounds and calibration
        ar & const_cast<int&>(mnMinX);
        ar & const_cast<int&>(mnMinY);
        ar & const_cast<int&>(mnMaxX);
        ar & const_cast<int&>(mnMaxY);
        ar & boost::serialization::make_array(mK_.data(), mK_.size());
        // Pose
        serializeSophusSE3<Archive>(ar, mTcw, version);
        // MapPointsId associated to keypoints
        ar & mvBackupMapPointsId;
        // Grid
        ar & mGrid;
        // Connected KeyFrameWeight
        ar & mBackupConnectedKeyFrameIdWeights;
        // Spanning Tree and Loop Edges
        ar & mbFirstConnection;
        ar & mBackupParentId;
        ar & mvBackupChildrensId;
        ar & mvBackupLoopEdgesId;
        ar & mvBackupMergeEdgesId;
        // Bad flags
        ar & mbNotErase;
        ar & mbToBeErased;
        ar & mbBad;

        ar & mHalfBaseline;

        ar & mnOriginMapId;

        // Camera variables
        ar & mnBackupIdCamera;
        ar & mnBackupIdCamera2;

        // Fisheye variables
        ar & mvLeftToRightMatch;
        ar & mvRightToLeftMatch;
        ar & const_cast<int&>(NLeft);
        ar & const_cast<int&>(NRight);
        serializeSophusSE3<Archive>(ar, mTlr, version);
        serializeVectorKeyPoints<Archive>(ar, mvKeysRight, version);
        ar & mGridRight;

        // Inertial variables
        ar & mImuBias;
        ar & mBackupImuPreintegrated;
        ar & mImuCalib;
        ar & mBackupPrevKFId;
        ar & mBackupNextKFId;
        ar & bImu;
        ar & boost::serialization::make_array(mVw.data(), mVw.size());
        ar & boost::serialization::make_array(mOwb.data(), mOwb.size());
        ar & mbHasVelocity;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 默认构造函数
     *
     * 主要用于反序列化或先创建空关键帧对象、后续再填充内容的场景。
     */
    KeyFrame();

    /**
     * @brief 由 Frame 构造关键帧
     * @param F      当前帧对象
     * @param pMap   关键帧所属地图
     * @param pKFDB  关键帧数据库
     *
     * 该构造函数会把 Frame 中已经计算好的 ORB 特征、描述子、位姿、相机参数、
     * 网格索引、IMU 数据等复制到关键帧中，使其成为可以长期保存在地图里的节点。
     */
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    /**
     * @brief 设置关键帧位姿
     * @param Tcw 相机位姿，表示从世界坐标系到当前相机坐标系的变换
     *
     * 该函数会同步更新：
     * 1. mTcw：世界到相机位姿；
     * 2. mTwc：相机到世界逆位姿；
     * 3. mRcw / mRwc：对应旋转矩阵；
     * 4. 若启用 IMU，则同步更新 IMU 在世界系下的位置。
     */
    void SetPose(const Sophus::SE3f &Tcw);

    /**
     * @brief 设置关键帧速度
     * @param Vw_ 世界坐标系下的速度
     */
    void SetVelocity(const Eigen::Vector3f &Vw_);

    /**
     * @brief 获取世界到相机位姿
     * @return Tcw
     */
    Sophus::SE3f GetPose();

    /**
     * @brief 获取相机到世界位姿
     * @return Twc
     */
    Sophus::SE3f GetPoseInverse();

    /**
     * @brief 获取相机中心在世界坐标系下的位置
     * @return 相机中心坐标
     */
    Eigen::Vector3f GetCameraCenter();

    /**
     * @brief 获取 IMU 在世界坐标系下的位置
     * @return IMU 位置
     */
    Eigen::Vector3f GetImuPosition();

    /**
     * @brief 获取 IMU 的世界系旋转
     * @return IMU 旋转矩阵
     */
    Eigen::Matrix3f GetImuRotation();

    /**
     * @brief 获取 IMU 在世界系下的完整位姿
     * @return IMU 位姿
     */
    Sophus::SE3f GetImuPose();

    /**
     * @brief 获取世界到相机的旋转矩阵
     * @return 旋转矩阵
     */
    Eigen::Matrix3f GetRotation();

    /**
     * @brief 获取世界到相机位姿中的平移部分
     * @return 平移向量
     */
    Eigen::Vector3f GetTranslation();

    /**
     * @brief 获取关键帧速度
     * @return 世界系下速度
     */
    Eigen::Vector3f GetVelocity();

    /**
     * @brief 查询关键帧是否保存了有效速度
     * @return true 表示速度有效
     */
    bool isVelocitySet();

    /**
     * @brief 计算关键帧的 BoW 表示
     *
     * 该函数会将当前关键帧的 ORB 描述子映射到词袋向量和特征向量中，
     * 供闭环检测、重定位和地点识别使用。
     */
    void ComputeBoW();

    /**
     * @brief 添加一条共视连接
     * @param pKF    另一关键帧
     * @param weight 两帧共享 MapPoint 的权重
     */
    void AddConnection(KeyFrame* pKF, const int &weight);

    /**
     * @brief 删除一条共视连接
     * @param pKF 待删除的关键帧连接
     */
    void EraseConnection(KeyFrame* pKF);

    /**
     * @brief 重新统计当前关键帧与其他关键帧的共视关系
     * @param upParent 是否允许在首次连接时自动设置父关键帧
     */
    void UpdateConnections(bool upParent=true);

    /**
     * @brief 按共视权重重排相连关键帧
     */
    void UpdateBestCovisibles();

    /**
     * @brief 获取所有与当前关键帧相连的关键帧
     * @return 连接关键帧集合
     */
    std::set<KeyFrame *> GetConnectedKeyFrames();

    /**
     * @brief 获取按权重排序后的共视关键帧列表
     * @return 共视关键帧列表
     */
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();

    /**
     * @brief 获取前 N 个最强共视关键帧
     * @param N 希望返回的关键帧数量
     * @return 最强共视关键帧
     */
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);

    /**
     * @brief 获取权重大于等于给定阈值的共视关键帧
     * @param w 权重阈值
     * @return 满足条件的关键帧列表
     */
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);

    /**
     * @brief 获取与某关键帧的共视权重
     * @param pKF 目标关键帧
     * @return 共视权重
     */
    int GetWeight(KeyFrame* pKF);

    /**
     * @brief 在生成树中添加一个子关键帧
     * @param pKF 子关键帧
     */
    void AddChild(KeyFrame* pKF);

    /**
     * @brief 在生成树中删除一个子关键帧
     * @param pKF 子关键帧
     */
    void EraseChild(KeyFrame* pKF);

    /**
     * @brief 修改当前关键帧在生成树中的父关键帧
     * @param pKF 新父关键帧
     */
    void ChangeParent(KeyFrame* pKF);

    /**
     * @brief 获取所有子关键帧
     * @return 子关键帧集合
     */
    std::set<KeyFrame*> GetChilds();

    /**
     * @brief 获取父关键帧
     * @return 父关键帧
     */
    KeyFrame* GetParent();

    /**
     * @brief 判断指定关键帧是否是当前关键帧的子结点
     * @param pKF 目标关键帧
     * @return true 表示是子结点
     */
    bool hasChild(KeyFrame* pKF);

    /**
     * @brief 设置“是否为首次建立连接”标志
     * @param bFirst true 表示还未完成首次父子连接
     */
    void SetFirstConnection(bool bFirst);

    /**
     * @brief 添加一条闭环边
     * @param pKF 闭环相连关键帧
     */
    void AddLoopEdge(KeyFrame* pKF);

    /**
     * @brief 获取所有闭环边
     * @return 闭环边集合
     */
    std::set<KeyFrame*> GetLoopEdges();

    /**
     * @brief 添加一条地图融合边
     * @param pKF 融合相连关键帧
     */
    void AddMergeEdge(KeyFrame* pKF);

    /**
     * @brief 获取所有地图融合边
     * @return 融合边集合
     */
    set<KeyFrame*> GetMergeEdges();

    /**
     * @brief 获取当前关键帧中已关联 MapPoint 的数量
     * @return MapPoint 数量
     */
    int GetNumberMPs();

    /**
     * @brief 将某个 MapPoint 挂到指定特征点索引上
     * @param pMP 地图点
     * @param idx 特征点索引
     */
    void AddMapPoint(MapPoint* pMP, const size_t &idx);

    /**
     * @brief 按索引删除一个特征点与 MapPoint 的匹配关系
     * @param idx 特征点索引
     */
    void EraseMapPointMatch(const int &idx);

    /**
     * @brief 按 MapPoint 指针删除与当前关键帧的匹配关系
     * @param pMP 地图点
     */
    void EraseMapPointMatch(MapPoint* pMP);

    /**
     * @brief 用新的 MapPoint 替换指定索引上的匹配
     * @param idx 特征点索引
     * @param pMP 新地图点
     */
    void ReplaceMapPointMatch(const int &idx, MapPoint* pMP);

    /**
     * @brief 获取当前关键帧观测到的所有有效地图点
     * @return 地图点集合
     */
    std::set<MapPoint*> GetMapPoints();

    /**
     * @brief 获取与每个特征点索引对应的地图点数组
     * @return 地图点向量
     */
    std::vector<MapPoint*> GetMapPointMatches();

    /**
     * @brief 统计满足观测次数条件的地图点数量
     * @param minObs 最小观测次数阈值
     * @return 满足条件的地图点数量
     */
    int TrackedMapPoints(const int &minObs);

    /**
     * @brief 获取指定索引上的地图点
     * @param idx 特征点索引
     * @return 地图点指针
     */
    MapPoint* GetMapPoint(const size_t &idx);

    /**
     * @brief 在图像指定区域内查询特征点
     * @param x      查询中心 x
     * @param y      查询中心 y
     * @param r      查询半径
     * @param bRight 是否在右目网格中查询
     * @return 落在区域内的特征点索引
     *
     * 该函数依赖关键帧内部的规则网格 mGrid / mGridRight 来加速局部特征搜索。
     */
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const bool bRight = false) const;

    /**
     * @brief 将带深度的特征点反投影到世界坐标系
     * @param i   特征点索引
     * @param x3D 输出三维点坐标
     * @return true 表示反投影成功
     */
    bool UnprojectStereo(int i, Eigen::Vector3f &x3D);

    /**
     * @brief 判断像素是否落在图像有效边界内
     * @param x 像素 x
     * @param y 像素 y
     * @return true 表示在图像内部
     */
    bool IsInImage(const float &x, const float &y) const;

    /**
     * @brief 禁止关键帧被立即删除
     *
     * 常用于关键帧已参与闭环边或融合边的情况。
     */
    void SetNotErase();

    /**
     * @brief 取消“禁止删除”标志
     */
    void SetErase();

    /**
     * @brief 将当前关键帧标记为坏关键帧并从图结构中移除
     */
    void SetBadFlag();

    /**
     * @brief 查询关键帧是否已被标坏
     * @return true 表示坏关键帧
     */
    bool isBad();

    /**
     * @brief 计算当前关键帧观测到地图点的场景深度统计量
     * @param q 分位因子，常用 q=2 表示中值深度
     * @return 深度统计量
     *
     * 单目初始化与建图中常用该值来评估基线是否足够。
     */
    float ComputeSceneMedianDepth(const int q);

    /**
     * @brief 共视权重排序比较函数
     */
    static bool weightComp( int a, int b){
        return a>b;
    }

    /**
     * @brief 按关键帧 id 升序比较
     */
    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }

    /**
     * @brief 获取当前关键帧所属地图
     * @return 地图对象
     */
    Map* GetMap();

    /**
     * @brief 更新当前关键帧所属地图
     * @param pMap 新地图
     */
    void UpdateMap(Map* pMap);

    /**
     * @brief 设置新的 IMU 偏置
     * @param b 新偏置
     */
    void SetNewBias(const IMU::Bias &b);

    /**
     * @brief 获取陀螺仪偏置
     * @return 陀螺仪偏置向量
     */
    Eigen::Vector3f GetGyroBias();

    /**
     * @brief 获取加速度计偏置
     * @return 加速度计偏置向量
     */
    Eigen::Vector3f GetAccBias();

    /**
     * @brief 获取完整 IMU 偏置结构
     * @return IMU 偏置
     */
    IMU::Bias GetImuBias();

    /**
     * @brief 将地图点投影到带畸变图像平面
     * @param pMP 地图点
     * @param kp  输出像素点
     * @param u   输出像素 x
     * @param v   输出像素 y
     * @return true 表示投影成功
     */
    bool ProjectPointDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v);

    /**
     * @brief 将地图点投影到去畸变图像平面
     * @param pMP 地图点
     * @param kp  输出像素点
     * @param u   输出像素 x
     * @param v   输出像素 y
     * @return true 表示投影成功
     */
    bool ProjectPointUnDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v);

    /**
     * @brief 序列化前预保存指针关系
     * @param spKF 允许保存的关键帧集合
     * @param spMP 允许保存的地图点集合
     * @param spCam 允许保存的相机集合
     *
     * 该函数会把无法直接序列化的指针关系转成 id，便于随后写盘。
     */
    void PreSave(set<KeyFrame*>& spKF,set<MapPoint*>& spMP, set<GeometricCamera*>& spCam);

    /**
     * @brief 反序列化后恢复指针关系与运行时状态
     * @param mpKFid 关键帧 id 到指针的映射
     * @param mpMPid 地图点 id 到指针的映射
     * @param mpCamId 相机 id 到指针的映射
     */
    void PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<long unsigned int, MapPoint*>& mpMPid, map<unsigned int, GeometricCamera*>& mpCamId);


    /**
     * @brief 设置 ORB 词典
     * @param pORBVoc ORB 词典
     */
    void SetORBVocabulary(ORBVocabulary* pORBVoc);

    /**
     * @brief 设置关键帧数据库
     * @param pKFDB 关键帧数据库
     */
    void SetKeyFrameDatabase(KeyFrameDatabase* pKFDB);

    /// 标记该关键帧是否已经具备惯性信息。
    bool bImu;

public:

    /// 全局自增关键帧 id。
    static long unsigned int nNextId;
    /// 当前关键帧 id。
    long unsigned int mnId;
    /// 生成该关键帧的原始 Frame id。
    const long unsigned int mnFrameId;

    /// 时间戳。
    const double mTimeStamp;

    /// 图像网格列数，用于加速局部特征搜索。
    const int mnGridCols;
    /// 图像网格行数。
    const int mnGridRows;
    /// x 方向网格宽度倒数。
    const float mfGridElementWidthInv;
    /// y 方向网格高度倒数。
    const float mfGridElementHeightInv;

    /// Tracking 中该关键帧上次作为参考关键帧的帧 id。
    long unsigned int mnTrackReferenceForFrame;
    /// Tracking / LocalMapping 融合时的访问标记。
    long unsigned int mnFuseTargetForKF;

    /// Local BA 中“已作为局部优化关键帧”的标记。
    long unsigned int mnBALocalForKF;
    /// Local BA 中“已作为固定关键帧”的标记。
    long unsigned int mnBAFixedForKF;

    /// 记录该关键帧被 BA 优化的次数。
    long unsigned int mnNumberOfOpt;

    /// 闭环查询标记。
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;
    long unsigned int mnMergeQuery;
    int mnMergeWords;
    float mMergeScore;
    long unsigned int mnPlaceRecognitionQuery;
    int mnPlaceRecognitionWords;
    float mPlaceRecognitionScore;

    /// 当前是否正在参与地点识别流程。
    bool mbCurrentPlaceRecognition;


    /// 全局 BA 后的关键帧位姿。
    Sophus::SE3f mTcwGBA;
    /// 全局 BA 前的关键帧位姿。
    Sophus::SE3f mTcwBefGBA;
    /// 全局 BA 后的速度。
    Eigen::Vector3f mVwbGBA;
    /// 全局 BA 前的速度。
    Eigen::Vector3f mVwbBefGBA;
    /// 全局 BA 后的偏置。
    IMU::Bias mBiasGBA;
    /// 记录最近一次更新该关键帧 GBA 状态的关键帧 id。
    long unsigned int mnBAGlobalForKF;

    /// 地图融合时使用的位姿与速度缓存。
    Sophus::SE3f mTcwMerge;
    Sophus::SE3f mTcwBefMerge;
    Sophus::SE3f mTwcBefMerge;
    Eigen::Vector3f mVwbMerge;
    Eigen::Vector3f mVwbBefMerge;
    IMU::Bias mBiasMerge;
    long unsigned int mnMergeCorrectedForKF;
    long unsigned int mnMergeForKF;
    float mfScaleMerge;
    long unsigned int mnBALocalForMerge;

    /// 关键帧整体尺度因子。
    float mfScale;

    /// 相机内参与双目几何参数。
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
    /// 畸变参数。
    cv::Mat mDistCoef;

    /// 关键点总数。
    const int N;

    /// 左目原始关键点。
    const std::vector<cv::KeyPoint> mvKeys;
    /// 去畸变后的关键点。
    const std::vector<cv::KeyPoint> mvKeysUn;
    /// 对应右目坐标，单目点为负值。
    const std::vector<float> mvuRight; // negative value for monocular points
    /// 对应深度，单目点为负值。
    const std::vector<float> mvDepth; // negative value for monocular points
    /// ORB 描述子矩阵。
    const cv::Mat mDescriptors;

    /// 词袋向量。
    DBoW2::BowVector mBowVec;
    /// 特征向量。
    DBoW2::FeatureVector mFeatVec;

    /// 当前关键帧相对父关键帧的位姿，在被标坏时计算。
    Sophus::SE3f mTcp;

    /// ORB 金字塔层数。
    const int mnScaleLevels;
    /// 相邻层尺度因子。
    const float mfScaleFactor;
    /// 尺度因子的对数。
    const float mfLogScaleFactor;
    /// 每层尺度因子。
    const std::vector<float> mvScaleFactors;
    /// 每层 sigma^2。
    const std::vector<float> mvLevelSigma2;
    /// 每层 sigma^2 的倒数。
    const std::vector<float> mvInvLevelSigma2;

    /// 去畸变后图像有效区域边界。
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;

    /// 时间链中的前一关键帧和后一关键帧。
    KeyFrame* mPrevKF;
    KeyFrame* mNextKF;

    /// 从前一关键帧到当前关键帧的 IMU 预积分量。
    IMU::Preintegrated* mpImuPreintegrated;
    /// IMU 与相机外参标定。
    IMU::Calib mImuCalib;

    /// 原始所属地图 id。
    unsigned int mnOriginMapId;

    /// 对应图像文件名。
    string mNameFile;

    /// 数据集编号。
    int mnDataset;

    /// 回环候选关键帧。
    std::vector <KeyFrame*> mvpLoopCandKFs;
    /// 地图融合候选关键帧。
    std::vector <KeyFrame*> mvpMergeCandKFs;

    //bool mbHasHessian;
    //cv::Mat mHessianPose;

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:
    /// 世界到相机位姿。
    Sophus::SE3<float> mTcw;
    /// 世界到相机旋转矩阵。
    Eigen::Matrix3f mRcw;
    /// 相机到世界位姿。
    Sophus::SE3<float> mTwc;
    /// 相机到世界旋转矩阵。
    Eigen::Matrix3f mRwc;

    /// IMU 在世界坐标系下的位置。
    Eigen::Vector3f mOwb;
    /// 世界系速度。
    Eigen::Vector3f mVw;
    /// 是否保存了有效速度。
    bool mbHasVelocity;

    /// 双目鱼眼中左到右、右到左相机的相对位姿。
    Sophus::SE3<float> mTlr;
    Sophus::SE3<float> mTrl;

    /// 当前 IMU 偏置。
    IMU::Bias mImuBias;

    /// 与每个特征点索引关联的地图点。
    std::vector<MapPoint*> mvpMapPoints;
    /// 保存/加载时使用的地图点 id 备份。
    std::vector<long long int> mvBackupMapPointsId;

    /// 关键帧数据库。
    KeyFrameDatabase* mpKeyFrameDB;
    /// ORB 词典。
    ORBVocabulary* mpORBvocabulary;

    /// 左目图像网格，加速局部特征搜索。
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    /// 共视图中与其他关键帧的连接及权重。
    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    /// 按权重排序后的共视关键帧列表。
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    /// 与上面列表对应的权重。
    std::vector<int> mvOrderedWeights;
    /// 保存/加载时使用的共视连接备份。
    std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights;

    /// 是否还未建立首次父子连接。
    bool mbFirstConnection;
    /// 生成树父结点。
    KeyFrame* mpParent;
    /// 生成树子结点集合。
    std::set<KeyFrame*> mspChildrens;
    /// 闭环边集合。
    std::set<KeyFrame*> mspLoopEdges;
    /// 地图融合边集合。
    std::set<KeyFrame*> mspMergeEdges;
    /// 保存/加载时使用的图结构备份。
    long long int mBackupParentId;
    std::vector<long unsigned int> mvBackupChildrensId;
    std::vector<long unsigned int> mvBackupLoopEdgesId;
    std::vector<long unsigned int> mvBackupMergeEdgesId;

    /// 不允许立即删除。
    bool mbNotErase;
    /// 需要在允许时删除。
    bool mbToBeErased;
    /// 是否为坏关键帧。
    bool mbBad;    

    /// 半基线，主要用于可视化。
    float mHalfBaseline; // Only for visualization

    /// 当前关键帧所属地图。
    Map* mpMap;

    /// 保存/加载时使用的惯性备份。
    long long int mBackupPrevKFId;
    long long int mBackupNextKFId;
    IMU::Preintegrated mBackupImuPreintegrated;

    /// 保存/加载时使用的相机 id 备份。
    unsigned int mnBackupIdCamera, mnBackupIdCamera2;

    /// 内参矩阵。
    Eigen::Matrix3f mK_;

    /// 保护位姿、速度与偏置。
    std::mutex mMutexPose; // for pose, velocity and biases
    /// 保护共视图、生成树、坏关键帧状态。
    std::mutex mMutexConnections;
    /// 保护特征点与地图点关联关系。
    std::mutex mMutexFeatures;
    /// 保护地图指针。
    std::mutex mMutexMap;

public:
    /// 主相机和次相机指针。
    GeometricCamera* mpCamera, *mpCamera2;

    /// 双目匹配索引：左目到右目、右目到左目。
    std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

    /**
     * @brief 获取右相机到左相机的相对位姿
     * @return Trl
     */
    Sophus::SE3f GetRelativePoseTrl();

    /**
     * @brief 获取左相机到右相机的相对位姿
     * @return Tlr
     */
    Sophus::SE3f GetRelativePoseTlr();

    /// 右目关键点，仅双目鱼眼场景使用。
    const std::vector<cv::KeyPoint> mvKeysRight;

    /// 左目和右目关键点数量。
    const int NLeft, NRight;

    /// 右目图像网格。
    std::vector< std::vector <std::vector<size_t> > > mGridRight;

    /**
     * @brief 获取右相机位姿
     * @return 右相机的 Tcw
     */
    Sophus::SE3<float> GetRightPose();

    /**
     * @brief 获取右相机逆位姿
     * @return 右相机的 Twc
     */
    Sophus::SE3<float> GetRightPoseInverse();

    /**
     * @brief 获取右相机中心
     * @return 右相机中心世界坐标
     */
    Eigen::Vector3f GetRightCameraCenter();

    /**
     * @brief 获取右相机旋转矩阵
     * @return 旋转矩阵
     */
    Eigen::Matrix<float,3,3> GetRightRotation();

    /**
     * @brief 获取右相机平移向量
     * @return 平移向量
     */
    Eigen::Vector3f GetRightTranslation();

    /**
     * @brief 打印当前关键帧左右图地图点分布
     *
     * 主要用于调试双目场景下左右视图的地图点分布情况。
     */
    void PrintPointDistribution(){
        int left = 0, right = 0;
        int Nlim = (NLeft != -1) ? NLeft : N;
        for(int i = 0; i < N; i++){
            if(mvpMapPoints[i]){
                if(i < Nlim) left++;
                else right++;
            }
        }
        cout << "Point distribution in KeyFrame: left-> " << left << " --- right-> " << right << endl;
    }


};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
