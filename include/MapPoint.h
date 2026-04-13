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


#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "Converter.h"

#include "SerializationUtils.h"

#include <opencv2/core/core.hpp>
#include <mutex>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>

namespace ORB_SLAM3
{

class KeyFrame;
class Map;
class Frame;

/**
 * @brief 地图中的三维路标点
 *
 * MapPoint 是 ORB-SLAM3 地图层的基本元素之一，用来表示一个被多个关键帧共同观测到的
 * 三维空间点。它主要维护三类信息：
 * 1. 几何信息：世界坐标、平均观测方向、尺度不变距离范围；
 * 2. 外观信息：由多个观测中筛选出来的代表性 ORB 描述子；
 * 3. 关联信息：哪些关键帧观测到了它、是否已失效、是否被其他 MapPoint 替换。
 *
 * 其中外观信息与 ORB 特征提取器直接相关。ORB 的工作流程是：
 * 1. 在图像金字塔各层中提取 FAST 角点；
 * 2. 用灰度质心法估计关键点方向；
 * 3. 在该方向下旋转 BRIEF 采样模板，得到 256 bit 的 ORB 描述子。
 *
 * MapPoint 不负责“提取”ORB，但会利用关键帧/帧中已经提取好的 ORB 描述子来：
 * 1. 选出一个最具代表性的描述子用于快速匹配；
 * 2. 根据关键点所在的金字塔层级估计该点的可观测距离范围；
 * 3. 根据当前观测距离反推最可能出现的尺度层。
 */
class MapPoint
{

    friend class boost::serialization::access;

    /**
     * @brief       序列化 MapPoint 的持久化状态
     * @param ar    Boost 序列化归档对象
     * @param version 序列化版本号
     *
     * 这里只保存“可恢复地图结构”的核心状态，例如：
     * 1. 点的 ID 与首次创建来源；
     * 2. 位置、法向、代表性描述子；
     * 3. 观测关系的备份 ID；
     * 4. 参考关键帧、被替换关系、尺度距离信息。
     *
     * 像跟踪线程中的临时投影、局部 BA 的中间标记等瞬时状态不会写入磁盘，
     * 因为这些状态在地图重新加载后可以重新计算或重新初始化。
     */
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & mnId;
        ar & mnFirstKFid;
        ar & mnFirstFrame;
        ar & nObs;
        // Variables used by the tracking
        //ar & mTrackProjX;
        //ar & mTrackProjY;
        //ar & mTrackDepth;
        //ar & mTrackDepthR;
        //ar & mTrackProjXR;
        //ar & mTrackProjYR;
        //ar & mbTrackInView;
        //ar & mbTrackInViewR;
        //ar & mnTrackScaleLevel;
        //ar & mnTrackScaleLevelR;
        //ar & mTrackViewCos;
        //ar & mTrackViewCosR;
        //ar & mnTrackReferenceForFrame;
        //ar & mnLastFrameSeen;

        // Variables used by local mapping
        //ar & mnBALocalForKF;
        //ar & mnFuseCandidateForKF;

        // Variables used by loop closing and merging
        //ar & mnLoopPointForKF;
        //ar & mnCorrectedByKF;
        //ar & mnCorrectedReference;
        //serializeMatrix(ar,mPosGBA,version);
        //ar & mnBAGlobalForKF;
        //ar & mnBALocalForMerge;
        //serializeMatrix(ar,mPosMerge,version);
        //serializeMatrix(ar,mNormalVectorMerge,version);

        // Protected variables
        ar & boost::serialization::make_array(mWorldPos.data(), mWorldPos.size());
        ar & boost::serialization::make_array(mNormalVector.data(), mNormalVector.size());
        //ar & BOOST_SERIALIZATION_NVP(mBackupObservationsId);
        //ar & mObservations;
        ar & mBackupObservationsId1;
        ar & mBackupObservationsId2;
        serializeMatrix(ar,mDescriptor,version);
        ar & mBackupRefKFId;
        //ar & mnVisible;
        //ar & mnFound;

        ar & mbBad;
        ar & mBackupReplacedId;

        ar & mfMinDistance;
        ar & mfMaxDistance;

    }


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 默认构造函数
     *
     * 主要供序列化/反序列化等场景使用。此时只初始化基础状态，
     * 真正的几何与观测关系会在后续恢复过程中补齐。
     */
    MapPoint();

    /**
     * @brief       根据已知三维坐标创建地图点
     * @param Pos   地图点世界坐标
     * @param pRefKF 参考关键帧
     * @param pMap  所属地图
     *
     * 这是最常见的构造方式，通常用于：
     * 1. 初始化阶段三角化出的点；
     * 2. 局部建图中新生成的路标点。
     */
    MapPoint(const Eigen::Vector3f &Pos, KeyFrame* pRefKF, Map* pMap);

    /**
     * @brief         以逆深度形式创建地图点
     * @param invDepth 初始逆深度
     * @param uv_init  主机关键帧中的初始像素坐标
     * @param pRefKF   当前参考关键帧
     * @param pHostKF  主机关键帧
     * @param pMap     所属地图
     *
     * 该构造方式用于某些需要保留逆深度参数化的优化场景。
     * 此时地图点的世界坐标可能暂时未知，因此不会立即设置 mWorldPos。
     */
    MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame* pRefKF, KeyFrame* pHostKF, Map* pMap);

    /**
     * @brief       根据普通帧中的近距离特征创建临时地图点
     * @param Pos   地图点世界坐标
     * @param pMap  所属地图
     * @param pFrame 产生该点的普通帧
     * @param idxF  特征在帧中的索引
     *
     * 这种点常见于 Tracking 阶段的临时视觉里程计点。构造时会直接：
     * 1. 从对应特征复制 ORB 描述子；
     * 2. 根据该特征所在尺度层估计可观测距离范围；
     * 3. 用当前观测射线初始化平均观测方向。
     */
    MapPoint(const Eigen::Vector3f &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    /**
     * @brief       设置地图点世界坐标
     * @param Pos   新的世界坐标
     *
     * 该函数会同时持有全局互斥量和位置互斥量，
     * 防止多线程下位置更新与全局优化等过程发生竞争。
     */
    void SetWorldPos(const Eigen::Vector3f &Pos);

    /**
     * @brief 获取地图点世界坐标
     * @return 地图点的三维世界坐标
     */
    Eigen::Vector3f GetWorldPos();

    /**
     * @brief 获取地图点平均观测方向
     * @return 单位化的平均观测方向向量
     *
     * 该方向由所有观测该点的相机中心到该点的视线方向平均得到，
     * 常用于视角一致性判断。
     */
    Eigen::Vector3f GetNormal();

    /**
     * @brief         直接设置平均观测方向
     * @param normal  新的法向/平均观测方向
     */
    void SetNormalVector(const Eigen::Vector3f& normal);

    /**
     * @brief 获取参考关键帧
     * @return 当前参考关键帧指针
     *
     * 参考关键帧通常是该点最早或最稳定的观测来源，
     * 并被用于更新尺度不变距离范围。
     */
    KeyFrame* GetReferenceKeyFrame();

    /**
     * @brief 获取全部观测关系
     * @return 关键帧到索引对的映射
     *
     * `tuple<int,int>` 的含义为：
     * 1. `first`：左目/单目特征索引；
     * 2. `second`：右目特征索引；
     * 3. 若某一侧不存在观测，则为 `-1`。
     */
    std::map<KeyFrame*,std::tuple<int,int>> GetObservations();

    /**
     * @brief 获取总观测次数
     * @return 当前累计观测数
     *
     * 注意这里统计的是“有效测量数”而不只是关键帧数：
     * 1. 单目或 RGB-D 的一次观测计为 1；
     * 2. 传统双目中若左目特征有合法右目匹配，会计为 2；
     * 3. 双鱼眼/双相机模型下左右特征会分别单独计数。
     */
    int Observations();

    /**
     * @brief       为地图点增加一个关键帧观测
     * @param pKF   观测该点的关键帧
     * @param idx   地图点在该关键帧中的特征索引
     */
    void AddObservation(KeyFrame* pKF,int idx);

    /**
     * @brief       删除某个关键帧对该点的观测
     * @param pKF   要删除观测的关键帧
     *
     * 若删除后该点剩余有效观测过少，则会进一步被标记为坏点。
     */
    void EraseObservation(KeyFrame* pKF);

    /**
     * @brief       获取地图点在指定关键帧中的索引
     * @param pKF   指定关键帧
     * @return      `(左目索引, 右目索引)`；若不存在则返回 `(-1,-1)`
     */
    std::tuple<int,int> GetIndexInKeyFrame(KeyFrame* pKF);

    /**
     * @brief       判断地图点是否被指定关键帧观测
     * @param pKF   指定关键帧
     * @return      若存在观测则返回 true
     */
    bool IsInKeyFrame(KeyFrame* pKF);

    /**
     * @brief 将地图点标记为坏点并从地图关联中移除
     *
     * 坏点不会立刻释放内存，但会：
     * 1. 清空自身观测表；
     * 2. 从所有关键帧中删除对应的匹配关系；
     * 3. 从所属地图的点集合中移除。
     */
    void SetBadFlag();

    /**
     * @brief 判断当前地图点是否已失效
     * @return 若为坏点则返回 true
     */
    bool isBad();

    /**
     * @brief       用另一个地图点替换当前点
     * @param pMP   用于替换当前点的目标点
     *
     * 典型场景是融合重复地图点。替换后：
     * 1. 当前点会被标记为坏点；
     * 2. 其观测会尽量迁移到目标点；
     * 3. 目标点会重新统计代表性描述子。
     */
    void Replace(MapPoint* pMP);    

    /**
     * @brief 获取替换当前点的地图点
     * @return 替换者指针；若未被替换则为 NULL
     */
    MapPoint* GetReplaced();

    /**
     * @brief       增加“被投影到视野内”的次数
     * @param n     增量，默认为 1
     */
    void IncreaseVisible(int n=1);

    /**
     * @brief       增加“成功匹配/找到”的次数
     * @param n     增量，默认为 1
     */
    void IncreaseFound(int n=1);

    /**
     * @brief 获取成功匹配比例
     * @return `mnFound / mnVisible`
     *
     * 该值用于衡量地图点的稳定性。比例越低，说明该点虽然经常被预测到视野内，
     * 但实际匹配成功率不高，可靠性较差。
     */
    float GetFoundRatio();

    /**
     * @brief 获取成功匹配次数
     * @return 当前 `mnFound`
     */
    inline int GetFound(){
        return mnFound;
    }

    /**
     * @brief 计算最具代表性的 ORB 描述子
     *
     * MapPoint 会收集所有观测到它的 ORB 描述子，然后计算它们之间的汉明距离，
     * 选出“到其他描述子中值距离最小”的那个描述子作为代表。
     *
     * 之所以这样做，是因为 ORB 描述子来自不同视角、不同尺度层，但都描述同一个物理点。
     * 中值最小的那个描述子更接近“群体中心”，用于后续匹配会更稳健。
     */
    void ComputeDistinctiveDescriptors();

    /**
     * @brief 获取代表性 ORB 描述子
     * @return 描述子的深拷贝
     */
    cv::Mat GetDescriptor();

    /**
     * @brief 更新平均观测方向和尺度不变距离范围
     *
     * 该函数会：
     * 1. 对所有观测视线做单位化平均，得到 `mNormalVector`；
     * 2. 以参考关键帧中的尺度层为基准，估计 `mfMinDistance` 和 `mfMaxDistance`。
     *
     * 这里的尺度距离信息直接依赖 ORB 金字塔设计：
     * 参考关键点若出现在较高层，说明它在原图中更小，因此该地图点允许在更远距离被稳定观测。
     */
    void UpdateNormalAndDepth();

    /**
     * @brief 获取尺度不变的最小观测距离
     * @return 带有保守系数的最小距离
     */
    float GetMinDistanceInvariance();

    /**
     * @brief 获取尺度不变的最大观测距离
     * @return 带有保守系数的最大距离
     */
    float GetMaxDistanceInvariance();

    /**
     * @brief             根据当前距离预测在关键帧中的最可能尺度层
     * @param currentDist 当前相机到地图点的距离
     * @param pKF         目标关键帧
     * @return            预测的 ORB 金字塔层级
     *
     * 公式的核心思想是：
     * 距离越远，地图点在图像中看起来越小，应当匹配到越高层（分辨率更低）的特征；
     * 距离越近，则更可能出现在低层（分辨率更高）。
     */
    int PredictScale(const float &currentDist, KeyFrame*pKF);

    /**
     * @brief             根据当前距离预测在普通帧中的最可能尺度层
     * @param currentDist 当前相机到地图点的距离
     * @param pF          目标普通帧
     * @return            预测的 ORB 金字塔层级
     */
    int PredictScale(const float &currentDist, Frame* pF);

    /**
     * @brief 获取所属地图
     * @return 当前地图指针
     */
    Map* GetMap();

    /**
     * @brief       更新所属地图
     * @param pMap  新的地图指针
     *
     * 多地图/地图融合场景下，一个 MapPoint 可能会被迁移到新的 Map 中。
     */
    void UpdateMap(Map* pMap);

    /**
     * @brief 打印当前地图点的观测信息
     *
     * 主要用于调试，输出地图点 ID 以及它在哪些关键帧/地图中被观测到。
     */
    void PrintObservations();

    /**
     * @brief             序列化前预处理
     * @param spKF        将被保存的关键帧集合
     * @param spMP        将被保存的地图点集合
     *
     * 该函数把指针关系转换为 ID 关系，便于后续写盘。
     */
    void PreSave(set<KeyFrame*>& spKF,set<MapPoint*>& spMP);

    /**
     * @brief              反序列化后恢复指针关系
     * @param mpKFid       关键帧 ID 到指针的映射
     * @param mpMPid       地图点 ID 到指针的映射
     */
    void PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<long unsigned int, MapPoint*>& mpMPid);

public:
    /// 地图点唯一 ID。
    long unsigned int mnId;
    /// 下一个可分配的地图点 ID。
    static long unsigned int nNextId;
    /// 首次观测到该点的关键帧 ID；若来自普通帧则可能为 -1。
    long int mnFirstKFid;
    /// 首次创建该点的帧 ID。
    long int mnFirstFrame;
    /// 地图点累计有效观测数。
    int nObs;

    // Variables used by the tracking
    /// 当前帧预测投影到左目图像的 x 坐标。
    float mTrackProjX;
    /// 当前帧预测投影到左目图像的 y 坐标。
    float mTrackProjY;
    /// 当前帧下该点的深度估计。
    float mTrackDepth;
    /// 右目/第二相机下的深度估计。
    float mTrackDepthR;
    /// 当前帧预测投影到右目图像的 x 坐标。
    float mTrackProjXR;
    /// 当前帧预测投影到右目图像的 y 坐标。
    float mTrackProjYR;
    /// 左目/右目中是否落入视野内。
    bool mbTrackInView, mbTrackInViewR;
    /// 左目/右目中预测到的金字塔尺度层。
    int mnTrackScaleLevel, mnTrackScaleLevelR;
    /// 左目/右目观测方向与平均法向之间的夹角余弦。
    float mTrackViewCos, mTrackViewCosR;
    /// 上一次使用该点进行跟踪的参考帧 ID。
    long unsigned int mnTrackReferenceForFrame;
    /// 最近一次看到该点的帧 ID。
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    /// 本地点 BA 访问标记。
    long unsigned int mnBALocalForKF;
    /// 融合候选标记。
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    /// 闭环线程处理标记。
    long unsigned int mnLoopPointForKF;
    /// 最近一次修正该点的关键帧 ID。
    long unsigned int mnCorrectedByKF;
    /// 闭环修正参考关键帧 ID。
    long unsigned int mnCorrectedReference;    
    /// 全局 BA 后的位置备份。
    Eigen::Vector3f mPosGBA;
    /// 全局 BA 访问标记。
    long unsigned int mnBAGlobalForKF;
    /// 地图融合阶段的本地 BA 标记。
    long unsigned int mnBALocalForMerge;

    // Variable used by merging
    /// 融合阶段的位置备份。
    Eigen::Vector3f mPosMerge;
    /// 融合阶段的法向备份。
    Eigen::Vector3f mNormalVectorMerge;


    // Fopr inverse depth optimization
    /// 逆深度参数。
    double mInvDepth;
    /// 主机关键帧中的初始像素 u。
    double mInitU;
    /// 主机关键帧中的初始像素 v。
    double mInitV;
    /// 逆深度参数化所依赖的主机关键帧。
    KeyFrame* mpHostKF;

    /// 保护跨地图点的全局位置写操作。
    static std::mutex mGlobalMutex;

    /// 该点最初所属地图的 ID。
    unsigned int mnOriginMapId;

protected:    

     // Position in absolute coordinates
     /// 地图点在世界坐标系中的三维位置。
     Eigen::Vector3f mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     /// 观测表：记录哪些关键帧看到了该点，以及左右目对应的特征索引。
     std::map<KeyFrame*,std::tuple<int,int> > mObservations;
     // For save relation without pointer, this is necessary for save/load function
     /// 保存左目/单目索引的备份表，键为关键帧 ID。
     std::map<long unsigned int, int> mBackupObservationsId1;
     /// 保存右目索引的备份表，键为关键帧 ID。
     std::map<long unsigned int, int> mBackupObservationsId2;

     // Mean viewing direction
     /// 多个观测视线平均得到的单位方向向量。
     Eigen::Vector3f mNormalVector;

     // Best descriptor to fast matching
     /// 代表性 ORB 描述子，用于后续快速匹配。
     cv::Mat mDescriptor;

     // Reference KeyFrame
     /// 参考关键帧。
     KeyFrame* mpRefKF;
     /// 参考关键帧 ID 备份，用于序列化恢复。
     long unsigned int mBackupRefKFId;

     // Tracking counters
     /// 被预测到可见区域内的次数。
     int mnVisible;
     /// 实际匹配成功的次数。
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     /// 坏点标记。
     bool mbBad;
     /// 若当前点被融合掉，则指向替换它的地图点。
     MapPoint* mpReplaced;
     // For save relation without pointer, this is necessary for save/load function
     /// 替换关系的 ID 备份。
     long long int mBackupReplacedId;

     // Scale invariance distances
     /// 该点可稳定匹配的最小距离。
     float mfMinDistance;
     /// 该点可稳定匹配的最大距离。
     float mfMaxDistance;

     /// 当前所属地图。
     Map* mpMap;

     // Mutex
     /// 保护位置、法向、尺度距离等几何信息。
     std::mutex mMutexPos;
     /// 保护观测表、描述子、坏点状态等特征信息。
     std::mutex mMutexFeatures;
     /// 保护所属地图指针。
     std::mutex mMutexMap;

};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
