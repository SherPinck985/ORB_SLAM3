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

#include "Atlas.h"
#include "Viewer.h"

#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

namespace ORB_SLAM3
{

/**
 * @brief 默认构造函数
 *
 * 这里只把当前地图指针置空，表示 Atlas 当前还没有任何可用地图。
 */
Atlas::Atlas(){
    mpCurrentMap = static_cast<Map*>(NULL);
}

/**
 * @brief 带初始关键帧 ID 的构造函数
 * @param initKFid 第一张地图的初始化关键帧 ID
 *
 * 构造完成后会立即创建一张新地图，因此该构造函数常用于系统初始化阶段。
 */
Atlas::Atlas(int initKFid): mnLastInitKFidMap(initKFid), mHasViewer(false)
{
    // 先显式清空当前地图指针，再调用统一的新地图创建流程。
    mpCurrentMap = static_cast<Map*>(NULL);
    CreateNewMap();
}

/**
 * @brief 析构函数，释放 Atlas 中仍然持有的地图对象
 *
 * Atlas 用 set 维护 Map*，这里逐个删除并从集合中移除，避免悬空指针。
 */
Atlas::~Atlas()
{
    for(std::set<Map*>::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;)
    {
        // 取出当前迭代器指向的地图对象。
        Map* pMi = *it;

        if(pMi)
        {
            // Atlas 拥有这些 Map 的生命周期，析构时逐个释放。
            delete pMi;
            pMi = static_cast<Map*>(NULL);

            // erase 会返回下一个有效迭代器，便于继续安全遍历。
            it = mspMaps.erase(it);
        }
        else
            ++it;

    }
}

/**
 * @brief 创建一张新地图并切换到该地图
 *
 * 主要流程：
 * 1. 对 Atlas 加锁，保证多线程下地图切换的一致性；
 * 2. 若当前已有地图，则先更新“下一张地图”的初始关键帧 ID；
 * 3. 将旧地图标记为已存储状态，不再作为当前活动地图；
 * 4. 构造新的 Map 对象并加入地图集合。
 */
void Atlas::CreateNewMap()
{
    // Atlas 的当前地图、地图集合和初始化 ID 都属于共享状态，需要互斥保护。
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Creation of new map with id: " << Map::nNextId << endl;
    if(mpCurrentMap){
        // 新地图的初始关键帧 ID 不能落后于当前地图内已使用的最大关键帧 ID。
        // 否则后续创建关键帧时可能与旧地图中的关键帧编号冲突。
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum

        // 旧地图不再是活动地图，只保留在 Atlas 中等待后续切换或保存。
        mpCurrentMap->SetStoredMap();
        cout << "Stored map with ID: " << mpCurrentMap->GetId() << endl;

        //if(mHasViewer)
        //    mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
    }
    cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << endl;

    // 以更新后的初始关键帧 ID 构造新地图。
    mpCurrentMap = new Map(mnLastInitKFidMap);
    // 明确标记该地图正在被系统使用。
    mpCurrentMap->SetCurrentMap();
    // Atlas 统一接管新地图的生命周期。
    mspMaps.insert(mpCurrentMap);
}

/**
 * @brief 切换当前地图
 * @param pMap 目标地图
 *
 * Atlas 允许同时持有多张地图，但任意时刻只有一张地图被视为“当前地图”。
 */
void Atlas::ChangeMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Change to map with id: " << pMap->GetId() << endl;
    if(mpCurrentMap){
        // 旧地图退回到“已存储但仍被 Atlas 管理”的状态。
        mpCurrentMap->SetStoredMap();
    }

    // 切换指针并把目标地图标记为当前活动地图。
    mpCurrentMap = pMap;
    mpCurrentMap->SetCurrentMap();
}

/**
 * @brief 获取最近一次记录的新地图初始关键帧 ID
 * @return 初始关键帧 ID
 */
unsigned long int Atlas::GetLastInitKFid()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mnLastInitKFidMap;
}

/**
 * @brief 设置 Viewer 指针
 * @param pViewer 外部可视化对象
 */
void Atlas::SetViewer(Viewer* pViewer)
{
    mpViewer = pViewer;
    mHasViewer = true;
}

/**
 * @brief 将关键帧加入其所属地图
 * @param pKF 待加入的关键帧
 *
 * 这里通过 pKF 自带的 Map 指针找到所属地图，而不是默认加入当前地图。
 * 这样可以兼容 Atlas 同时管理多张地图的情形。
 */
void Atlas::AddKeyFrame(KeyFrame* pKF)
{
    // 从关键帧反查其所属地图。
    Map* pMapKF = pKF->GetMap();
    // 真正的容器插入逻辑由 Map 类负责。
    pMapKF->AddKeyFrame(pKF);
}

/**
 * @brief 将地图点加入其所属地图
 * @param pMP 待加入的地图点
 */
void Atlas::AddMapPoint(MapPoint* pMP)
{
    // 从地图点自身记录的 Map 指针找到目标地图。
    Map* pMapMP = pMP->GetMap();
    pMapMP->AddMapPoint(pMP);
}

/**
 * @brief 注册相机对象，若已存在等价相机则直接复用
 * @param pCam 待加入的相机模型
 * @return Atlas 中最终使用的相机对象指针
 *
 * 相机去重的意义在于：
 * 1. 避免同一组内参被反复保存；
 * 2. 让读档后关键帧能通过统一相机对象恢复引用关系。
 */
GeometricCamera* Atlas::AddCamera(GeometricCamera* pCam)
{
    // 标记待加入相机是否已经在 Atlas 中存在。
    bool bAlreadyInMap = false;
    // 记录已存在相机在数组中的下标，便于后续直接返回。
    int index_cam = -1;

    // 遍历当前已登记的所有相机，尝试按“类型 + 参数”做精确匹配。
    for(size_t i=0; i < mvpCameras.size(); ++i)
    {
        GeometricCamera* pCam_i = mvpCameras[i];

        // 原始代码保留了简单的空指针调试输出，注释保留其意图，不改逻辑。
        if(!pCam) std::cout << "Not pCam" << std::endl;
        if(!pCam_i) std::cout << "Not pCam_i" << std::endl;

        // 相机模型类型不同，不可能是同一相机参数实例。
        if(pCam->GetType() != pCam_i->GetType())
            continue;

        if(pCam->GetType() == GeometricCamera::CAM_PINHOLE)
        {
            // 针孔模型下，调用 Pinhole::IsEqual 比较内参与畸变参数是否一致。
            if(((Pinhole*)pCam_i)->IsEqual(pCam))
            {
                bAlreadyInMap = true;
                index_cam = i;
            }
        }
        else if(pCam->GetType() == GeometricCamera::CAM_FISHEYE)
        {
            // 鱼眼模型下，调用 KannalaBrandt8::IsEqual 进行参数比较。
            if(((KannalaBrandt8*)pCam_i)->IsEqual(pCam))
            {
                bAlreadyInMap = true;
                index_cam = i;
            }
        }
    }

    if(bAlreadyInMap)
    {
        // 已存在等价相机时，直接复用旧对象，避免重复存储。
        return mvpCameras[index_cam];
    }
    else{
        // 否则将新相机登记到 Atlas 中。
        mvpCameras.push_back(pCam);
        return pCam;
    }
}

/**
 * @brief 获取 Atlas 中登记的全部相机
 * @return 相机数组
 */
std::vector<GeometricCamera*> Atlas::GetAllCameras()
{
    return mvpCameras;
}

/**
 * @brief 设置当前地图的参考地图点
 * @param vpMPs 参考地图点数组
 *
 * 参考地图点通常用于跟踪线程中做局部地图投影与匹配。
 */
void Atlas::SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

/**
 * @brief 通知当前地图发生一次大变化
 *
 * 例如回环闭合、全局优化等，这类操作通常会显著改变地图拓扑或位姿结果。
 */
void Atlas::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->InformNewBigChange();
}

/**
 * @brief 获取当前地图最近一次大变化的索引
 * @return 大变化编号
 */
int Atlas::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetLastBigChangeIdx();
}

/**
 * @brief 获取当前地图中的地图点数量
 * @return 地图点数
 */
long unsigned int Atlas::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->MapPointsInMap();
}

/**
 * @brief 获取当前地图中的关键帧数量
 * @return 关键帧数
 */
long unsigned Atlas::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->KeyFramesInMap();
}

/**
 * @brief 获取当前地图全部关键帧
 * @return 关键帧数组
 */
std::vector<KeyFrame*> Atlas::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllKeyFrames();
}

/**
 * @brief 获取当前地图全部地图点
 * @return 地图点数组
 */
std::vector<MapPoint*> Atlas::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMapPoints();
}

/**
 * @brief 获取当前地图的参考地图点
 * @return 参考地图点数组
 */
std::vector<MapPoint*> Atlas::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetReferenceMapPoints();
}

/**
 * @brief 获取 Atlas 中全部地图，并按地图 ID 排序后返回
 * @return 地图数组
 *
 * mspMaps 是 set<Map*>，其默认顺序按指针地址而非地图 ID。
 * 因此这里显式复制到 vector 再按 GetId() 排序，便于上层稳定遍历。
 */
vector<Map*> Atlas::GetAllMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };

    // 先把 set 中的元素拷贝到连续数组，方便排序和返回。
    vector<Map*> vMaps(mspMaps.begin(),mspMaps.end());
    // 按地图 ID 从小到大排序，而不是沿用 set 的指针顺序。
    sort(vMaps.begin(), vMaps.end(), compFunctor());
    return vMaps;
}

/**
 * @brief 统计 Atlas 当前持有的地图数量
 * @return 地图数
 */
int Atlas::CountMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mspMaps.size();
}

/**
 * @brief 清空当前地图中的内容
 *
 * 这里调用的是 Map::clear()，会清空当前地图中的关键帧、地图点和参考点等数据，
 * 但不销毁当前 Map 对象本身。
 */
void Atlas::clearMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->clear();
}

/**
 * @brief 清空 Atlas 的地图集合状态
 *
 * 当前实现只是清空集合并重置指针/计数，不在这里逐个 delete 地图。
 * 这是一个“状态重置”接口，而不是严格意义上的深度释放接口。
 */
void Atlas::clearAtlas()
{
    unique_lock<mutex> lock(mMutexAtlas);
    /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
    {
        (*it)->clear();
        delete *it;
    }*/
    mspMaps.clear();
    mpCurrentMap = static_cast<Map*>(NULL);
    mnLastInitKFidMap = 0;
}

/**
 * @brief 获取当前地图
 * @return 当前地图指针
 *
 * 若当前地图为空，则尝试创建新地图；
 * 若当前地图已被标记为坏地图，则循环等待其状态稳定后再返回。
 */
Map* Atlas::GetCurrentMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(!mpCurrentMap)
        CreateNewMap();

    // 避免把已经被标记为 bad 的地图直接返回给上层使用。
    while(mpCurrentMap->IsBad())
        usleep(3000);

    return mpCurrentMap;
}

/**
 * @brief 将地图标记为坏地图
 * @param pMap 待处理地图
 *
 * 处理步骤：
 * 1. 从 Atlas 主地图集合中移除；
 * 2. 给 Map 自身打上 bad 标记；
 * 3. 放入坏地图集合，等待后续统一清理。
 */
void Atlas::SetMapBad(Map* pMap)
{
    // 坏地图不再参与正常地图遍历。
    mspMaps.erase(pMap);
    pMap->SetBad();

    // 暂存到坏地图集合中，后续由 RemoveBadMaps 做容器级清理。
    mspBadMaps.insert(pMap);
}

/**
 * @brief 清理坏地图集合的记录
 *
 * 注意：当前代码没有在这里 delete 坏地图对象，只是把暂存集合清空。
 * 也就是说，这里清理的是“管理记录”，不是“对象内存”。
 */
void Atlas::RemoveBadMaps()
{
    /*for(Map* pMap : mspBadMaps)
    {
        delete pMap;
        pMap = static_cast<Map*>(NULL);
    }*/
    mspBadMaps.clear();
}

/**
 * @brief 判断当前地图是否为惯性地图
 * @return true 表示当前地图启用了惯性传感器信息
 */
bool Atlas::isInertial()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->IsInertial();
}

/**
 * @brief 将当前地图标记为惯性地图
 */
void Atlas::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetInertialSensor();
}

/**
 * @brief 将当前地图标记为 IMU 已初始化
 */
void Atlas::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetImuInitialized();
}

/**
 * @brief 查询当前地图的 IMU 初始化状态
 * @return true 表示 IMU 已初始化
 */
bool Atlas::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->isImuInitialized();
}

/**
 * @brief 序列化前预处理
 *
 * 该函数的职责不是直接写文件，而是把 Atlas 整理成“适合序列化”的状态：
 * 1. 更新下一张地图的初始关键帧 ID；
 * 2. 把 set 形式的地图集合复制到 vector 备份中；
 * 3. 为每张地图调用 Map::PreSave，提前清洗和备份内部引用；
 * 4. 去掉空地图和坏地图的暂存记录。
 */
void Atlas::PreSave()
{
    if(mpCurrentMap){
        // 为后续再次创建新地图预留不冲突的初始关键帧 ID。
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum
    }

    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };

    // 将 set 拷贝到 vector，满足序列化要求，同时方便按 ID 排序后稳定保存。
    std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
    sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

    // 先构造一个相机集合，后续每张地图在 PreSave 时可用它统一处理相机引用。
    std::set<GeometricCamera*> spCams(mvpCameras.begin(), mvpCameras.end());
    for(Map* pMi : mvpBackupMaps)
    {
        // 跳过空指针或已经失效的地图。
        if(!pMi || pMi->IsBad())
            continue;

        if(pMi->GetAllKeyFrames().size() == 0) {
            // 没有任何关键帧的地图不具备保存意义，先标坏并从主集合移除。
            SetMapBad(pMi);
            continue;
        }

        // 让每张地图把自己的关键帧、地图点和跨对象引用整理成可存档状态。
        pMi->PreSave(spCams);
    }

    // 清理掉本轮预保存过程中标记出的坏地图记录。
    RemoveBadMaps();
}

/**
 * @brief 反序列化后恢复 Atlas 内部状态
 *
 * 主要步骤：
 * 1. 先建立“相机 ID -> 相机指针”的查找表；
 * 2. 用备份数组重建 mspMaps；
 * 3. 调用每张地图的 PostLoad 恢复关键帧、地图点和数据库/词袋引用；
 * 4. 清空备份容器，表示恢复完成。
 */
void Atlas::PostLoad()
{
    // 构建相机 ID 到对象指针的映射，便于地图内部恢复相机引用。
    map<unsigned int,GeometricCamera*> mpCams;
    for(GeometricCamera* pCam : mvpCameras)
    {
        mpCams[pCam->GetId()] = pCam;
    }

    // 清空运行期地图集合，准备从备份容器完整重建。
    mspMaps.clear();

    // 统计量主要用于恢复过程中的验证或调试，当前实现未进一步使用。
    unsigned long int numKF = 0, numMP = 0;
    for(Map* pMi : mvpBackupMaps)
    {
        // 把地图重新纳入 Atlas 管理。
        mspMaps.insert(pMi);

        // 恢复地图内部对象之间的指针关系，以及词袋和数据库依赖。
        pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpCams);

        // 顺手累计恢复后的关键帧和地图点数量。
        numKF += pMi->GetAllKeyFrames().size();
        numMP += pMi->GetAllMapPoints().size();
    }

    // 备份数组只用于反序列化恢复，完成后即可清空。
    mvpBackupMaps.clear();
}

/**
 * @brief 设置关键帧数据库指针
 * @param pKFDB 关键帧数据库对象
 */
void Atlas::SetKeyFrameDababase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

/**
 * @brief 获取关键帧数据库指针
 * @return 关键帧数据库
 */
KeyFrameDatabase* Atlas::GetKeyFrameDatabase()
{
    return mpKeyFrameDB;
}

/**
 * @brief 设置 ORB 词袋字典指针
 * @param pORBVoc ORB 词袋字典
 */
void Atlas::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBVocabulary = pORBVoc;
}

/**
 * @brief 获取 ORB 词袋字典指针
 * @return ORB 词袋字典
 */
ORBVocabulary* Atlas::GetORBVocabulary()
{
    return mpORBVocabulary;
}

/**
 * @brief 统计 Atlas 中所有地图的关键帧总数
 * @return 关键帧总数
 */
long unsigned int Atlas::GetNumLivedKF()
{
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for(Map* pMap_i : mspMaps)
    {
        // 逐张地图累计关键帧数量。
        num += pMap_i->GetAllKeyFrames().size();
    }

    return num;
}

/**
 * @brief 统计 Atlas 中所有地图的地图点总数
 * @return 地图点总数
 */
long unsigned int Atlas::GetNumLivedMP() {
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for (Map* pMap_i : mspMaps) {
        // 逐张地图累计地图点数量。
        num += pMap_i->GetAllMapPoints().size();
    }

    return num;
}

/**
 * @brief 获取备份地图中全部关键帧的 ID 映射
 * @return key 为关键帧 ID，value 为关键帧指针
 *
 * 该接口遍历的是 mvpBackupMaps，因此它主要服务于保存/加载阶段，而不是运行期当前地图集合。
 */
map<long unsigned int, KeyFrame*> Atlas::GetAtlasKeyframes()
{
    map<long unsigned int, KeyFrame*> mpIdKFs;
    for(Map* pMap_i : mvpBackupMaps)
    {
        // 先取出当前备份地图中的全部关键帧。
        vector<KeyFrame*> vpKFs_Mi = pMap_i->GetAllKeyFrames();

        for(KeyFrame* pKF_j_Mi : vpKFs_Mi)
        {
            // 以关键帧全局 ID 作为键，建立快速查找表。
            mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
        }
    }

    return mpIdKFs;
}

} //namespace ORB_SLAM3
