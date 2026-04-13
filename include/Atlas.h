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

#ifndef ATLAS_H
#define ATLAS_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

#include <set>
#include <mutex>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>


namespace ORB_SLAM3
{
class Viewer;
class Map;
class MapPoint;
class KeyFrame;
class KeyFrameDatabase;
class Frame;
class KannalaBrandt8;
class Pinhole;

//BOOST_CLASS_EXPORT_GUID(Pinhole, "Pinhole")
//BOOST_CLASS_EXPORT_GUID(KannalaBrandt8, "KannalaBrandt8")

/**
 * @brief Atlas，多地图管理器
 *
 * ORB-SLAM3 在跟踪失败后可能重新初始化并生成新地图，或者在地图合并前同时维护多张地图。
 * Atlas 就是所有 Map 的上层容器，负责：
 * 1. 维护当前正在使用的地图；
 * 2. 管理 Atlas 内所有地图与坏地图；
 * 3. 管理相机模型对象，避免重复保存相同的相机参数；
 * 4. 为存档/读档准备和重建跨对象引用关系。
 *
 * 需要特别注意：
 * Atlas 本身不参与 ORB 特征提取。ORB 特征提取发生在 ORBextractor 中，
 * 其流程是“金字塔 -> FAST 角点 -> 四叉树均匀化 -> 方向/描述子”。
 * Atlas 只负责管理这些特征后来用于建图和定位时所依附的地图容器。
 */
class Atlas
{
    friend class boost::serialization::access;

    /**
     * @brief Boost 序列化接口
     * @param ar      Boost 提供的归档对象，负责读写数据
     * @param version 序列化版本号，当前实现中未显式使用
     *
     * 序列化时主要保存 Atlas 的“可重建状态”：
     * 1. 备份地图数组和相机数组；
     * 2. 各类静态 ID 计数器，保证重新加载后新对象 ID 不冲突；
     * 3. 最近一次初始化地图的起始关键帧 ID。
     */
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        // 注册具体相机类型，保证反序列化 GeometricCamera* 时能恢复为正确子类。
        ar.template register_type<Pinhole>();
        ar.template register_type<KannalaBrandt8>();

        // 由于旧版 Boost 在特定环境下对 set 的序列化存在问题，这里改用 vector 作为备份容器。
        //ar & mspMaps;
        ar & mvpBackupMaps;
        ar & mvpCameras;
        // 保存全局静态 ID 计数器，避免读档后继续创建对象时发生 ID 冲突。
        ar & Map::nNextId;
        ar & Frame::nNextId;
        ar & KeyFrame::nNextId;
        ar & MapPoint::nNextId;
        ar & GeometricCamera::nNextId;
        ar & mnLastInitKFidMap;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 默认构造函数
     *
     * 仅初始化 Atlas 的基本状态，不自动创建第一张地图。
     */
    Atlas();

    /**
     * @brief 带初始关键帧 ID 的构造函数
     * @param initKFid 第一张地图初始化时使用的关键帧起始 ID
     *
     * 构造后会立即创建一张新地图，并把它设为当前地图。
     */
    Atlas(int initKFid); // When its initialization the first map is created

    /**
     * @brief 析构函数
     *
     * 负责释放 Atlas 当前维护的所有 Map 对象。
     */
    ~Atlas();

    /**
     * @brief 创建一张新地图并切换为当前地图
     *
     * 若当前已有地图，则先把旧地图标记为“已存储/未使用”，然后新建 Map。
     */
    void CreateNewMap();

    /**
     * @brief 切换当前地图
     * @param pMap 要切换到的目标地图
     */
    void ChangeMap(Map* pMap);

    /**
     * @brief 获取下一张新地图初始化时应使用的关键帧起始 ID
     * @return 最近维护的初始化关键帧 ID
     */
    unsigned long int GetLastInitKFid();

    /**
     * @brief 设置 Viewer 指针
     * @param pViewer 外部 Viewer 实例
     */
    void SetViewer(Viewer* pViewer);

    /**
     * @brief 将关键帧加入其所属地图
     * @param pKF 待加入的关键帧
     *
     * 注意这里不是强行加入当前地图，而是根据关键帧自身记录的 Map 指针分发。
     */
    void AddKeyFrame(KeyFrame* pKF);

    /**
     * @brief 将地图点加入其所属地图
     * @param pMP 待加入的地图点
     */
    void AddMapPoint(MapPoint* pMP);
    //void EraseMapPoint(MapPoint* pMP);
    //void EraseKeyFrame(KeyFrame* pKF);

    /**
     * @brief 向 Atlas 注册一个相机模型
     * @param pCam 待注册的相机对象
     * @return 若已存在参数完全相同的相机，则返回已存在对象；否则返回新加入对象
     *
     * 该接口用于避免多个关键帧重复持有内容相同的相机模型实例，便于序列化和引用重建。
     */
    GeometricCamera* AddCamera(GeometricCamera* pCam);

    /**
     * @brief 获取 Atlas 中登记的全部相机
     * @return 相机对象数组
     */
    std::vector<GeometricCamera*> GetAllCameras();

    /**
     * @brief 设置当前地图的参考地图点集合
     * @param vpMPs 参考地图点数组
     */
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

    /**
     * @brief 通知当前地图发生了一次“大变化”
     *
     * 通常用于回环闭合、全局 BA 等会显著改变地图结构的场景。
     */
    void InformNewBigChange();

    /**
     * @brief 获取当前地图最近一次大变化的编号
     * @return 大变化索引
     */
    int GetLastBigChangeIdx();

    /**
     * @brief 获取当前地图中的地图点数量
     * @return 地图点数
     */
    long unsigned int MapPointsInMap();

    /**
     * @brief 获取当前地图中的关键帧数量
     * @return 关键帧数
     */
    long unsigned KeyFramesInMap();

    /**
     * @brief 获取当前地图中的全部关键帧
     * @return 关键帧数组
     */
    std::vector<KeyFrame*> GetAllKeyFrames();

    /**
     * @brief 获取当前地图中的全部地图点
     * @return 地图点数组
     */
    std::vector<MapPoint*> GetAllMapPoints();

    /**
     * @brief 获取当前地图中的参考地图点
     * @return 参考地图点数组
     */
    std::vector<MapPoint*> GetReferenceMapPoints();

    /**
     * @brief 获取 Atlas 中的全部地图
     * @return 按地图 ID 升序排列的地图数组
     */
    vector<Map*> GetAllMaps();

    /**
     * @brief 统计 Atlas 中当前维护的地图数量
     * @return 地图数
     */
    int CountMaps();

    /**
     * @brief 清空当前地图中的关键帧、地图点等内容
     *
     * 该接口不删除 Atlas 中的 Map 对象本身，只清空当前 Map 内部数据。
     */
    void clearMap();

    /**
     * @brief 清空整个 Atlas 的地图集合状态
     *
     * 该函数会重置当前地图指针和初始关键帧计数信息。
     */
    void clearAtlas();

    /**
     * @brief 获取当前地图
     * @return 当前正在使用的 Map 指针
     */
    Map* GetCurrentMap();

    /**
     * @brief 将一张地图标记为坏地图并移出主地图集合
     * @param pMap 待标记的地图
     */
    void SetMapBad(Map* pMap);

    /**
     * @brief 清空坏地图暂存集合
     *
     * 当前实现只从集合中移除记录，不在这里真正 delete 地图对象。
     */
    void RemoveBadMaps();

    /**
     * @brief 判断当前地图是否为惯性地图
     * @return true 表示当前地图使用 IMU / 惯性信息
     */
    bool isInertial();

    /**
     * @brief 将当前地图标记为惯性地图
     */
    void SetInertialSensor();

    /**
     * @brief 将当前地图标记为 IMU 已初始化
     */
    void SetImuInitialized();

    /**
     * @brief 查询当前地图的 IMU 初始化状态
     * @return true 表示 IMU 已完成初始化
     */
    bool isImuInitialized();

    /**
     * @brief 序列化前预处理
     *
     * 负责把 set 中的地图整理到可序列化的备份 vector，并让每张地图先完成自己的预保存工作。
     */
    void PreSave();

    /**
     * @brief 反序列化后重建 Atlas 内部引用关系
     *
     * 主要恢复地图集合、相机查找表，以及每张地图内部对数据库、词袋和相机的引用。
     */
    void PostLoad();

    /**
     * @brief 获取备份地图中所有关键帧的 ID 到指针映射
     * @return 关键帧 ID 映射表
     */
    map<long unsigned int, KeyFrame*> GetAtlasKeyframes();

    /**
     * @brief 设置关键帧数据库指针
     * @param pKFDB 关键帧数据库
     */
    void SetKeyFrameDababase(KeyFrameDatabase* pKFDB);

    /**
     * @brief 获取关键帧数据库指针
     * @return 关键帧数据库
     */
    KeyFrameDatabase* GetKeyFrameDatabase();

    /**
     * @brief 设置 ORB 词袋字典指针
     * @param pORBVoc ORB 词袋字典
     */
    void SetORBVocabulary(ORBVocabulary* pORBVoc);

    /**
     * @brief 获取 ORB 词袋字典指针
     * @return ORB 词袋字典
     */
    ORBVocabulary* GetORBVocabulary();

    /**
     * @brief 统计 Atlas 中所有地图当前存活的关键帧总数
     * @return 关键帧总数
     */
    long unsigned int GetNumLivedKF();

    /**
     * @brief 统计 Atlas 中所有地图当前存活的地图点总数
     * @return 地图点总数
     */
    long unsigned int GetNumLivedMP();

protected:

    /// Atlas 当前维护的全部地图对象集合。
    std::set<Map*> mspMaps;
    /// 被标记为坏地图、等待后续清理的地图集合。
    std::set<Map*> mspBadMaps;
    /// 由于旧版 Boost 对 set 序列化有兼容性问题，这里用 vector 作为保存/加载时的备份容器。
    std::vector<Map*> mvpBackupMaps;

    /// 当前正在被系统使用的地图。
    Map* mpCurrentMap;

    /// Atlas 内登记过的相机模型对象，用于共享和去重。
    std::vector<GeometricCamera*> mvpCameras;

    /// 最近一次新地图初始化时使用的起始关键帧 ID。
    unsigned long int mnLastInitKFidMap;

    /// 可选的 Viewer 指针，用于可视化模块交互。
    Viewer* mpViewer;
    /// 是否已经绑定了 Viewer。
    bool mHasViewer;

    /// 读档后用于恢复关键帧数据库引用。
    KeyFrameDatabase* mpKeyFrameDB;
    /// 读档后用于恢复 ORB 词袋字典引用。
    ORBVocabulary* mpORBVocabulary;

    /// 保护 Atlas 全局状态的互斥锁。
    std::mutex mMutexAtlas;


}; // class Atlas

} // namespace ORB_SLAM3

#endif // ATLAS_H
