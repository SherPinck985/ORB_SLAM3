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


#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Atlas.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <boost/algorithm/string.hpp>
#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM3
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class Map;


/**
 * @brief 回环检测与地图融合线程
 *
 * LoopClosing 是 ORB-SLAM3 后端中的“地点识别 + 回环校正 + 地图合并”线程，主要负责：
 * 1. 从 Tracking / LocalMapping 产生的关键帧中检测回到旧位置的情况；
 * 2. 在多地图场景下检测两张地图是否存在重叠区域并执行融合；
 * 3. 基于 ORB 描述子对应的 BoW 表示检索候选关键帧；
 * 4. 通过 ORB 匹配、投影匹配和 Sim3 几何验证确认候选；
 * 5. 进行 Essential Graph 优化和全局 BA。
 *
 * 需要特别说明：
 * LoopClosing 自身并不直接调用 ORBextractor 去“提取”特征。
 * 它使用的是关键帧里已经保存好的 ORB 描述子、BoW 向量以及地图点观测关系。
 * 因此它与 ORB 特征提取器的关系是“消费前端结果”，而不是“执行提取”。
 */
class LoopClosing
{
public:

    /// 一组共视一致的关键帧集合及其累计一致次数。
    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    /// 关键帧到 Sim3 位姿的映射，常用于回环校正和地图融合中的批量位姿修正。
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose;

public:
    /**
     * @brief 构造回环线程
     * @param pAtlas      多地图管理器
     * @param pDB         关键帧数据库
     * @param pVoc        ORB 词袋字典
     * @param bFixScale   是否固定 Sim3 中的尺度，双目/RGB-D 一般为 true
     * @param bActiveLC   是否启用地点识别与回环功能
     */
    LoopClosing(Atlas* pAtlas, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale, const bool bActiveLC);

    /**
     * @brief 绑定 Tracking 线程
     * @param pTracker Tracking 对象
     */
    void SetTracker(Tracking* pTracker);

    /**
     * @brief 绑定 LocalMapping 线程
     * @param pLocalMapper LocalMapping 对象
     */
    void SetLocalMapper(LocalMapping* pLocalMapper);

    /**
     * @brief 回环线程主循环
     *
     * 持续从关键帧队列中取出待检测关键帧，执行：
     * 1. 地点识别；
     * 2. 回环候选/合并候选几何验证；
     * 3. 回环校正或地图合并；
     * 4. 按需启动全局 BA。
     */
    void Run();

    /**
     * @brief 向回环线程压入一张新关键帧
     * @param pKF 新关键帧
     */
    void InsertKeyFrame(KeyFrame *pKF);

    /**
     * @brief 请求完全重置回环线程内部状态
     */
    void RequestReset();
    /**
     * @brief 请求只重置某张活动地图相关的回环状态
     * @param pMap 要重置的地图
     */
    void RequestResetActiveMap(Map* pMap);

    /**
     * @brief 在独立线程中执行全局 Bundle Adjustment
     * @param pActiveMap 执行优化的地图
     * @param nLoopKF    本次回环对应的关键帧编号
     */
    void RunGlobalBundleAdjustment(Map* pActiveMap, unsigned long nLoopKF);

    /**
     * @brief 查询 GBA 线程是否正在运行
     * @return true 表示运行中
     */
    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    /**
     * @brief 查询 GBA 线程是否已经结束
     * @return true 表示已结束
     */
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    /**
     * @brief 请求结束回环线程
     */
    void RequestFinish();

    /**
     * @brief 查询回环线程是否已经结束
     * @return true 表示已结束
     */
    bool isFinished();

    /// 可视化线程指针，当前版本里主要供系统整体协同使用。
    Viewer* mpViewer;

#ifdef REGISTER_TIMES

    vector<double> vdDataQuery_ms;
    vector<double> vdEstSim3_ms;
    vector<double> vdPRTotal_ms;

    vector<double> vdMergeMaps_ms;
    vector<double> vdWeldingBA_ms;
    vector<double> vdMergeOptEss_ms;
    vector<double> vdMergeTotal_ms;
    vector<int> vnMergeKFs;
    vector<int> vnMergeMPs;
    int nMerges;

    vector<double> vdLoopFusion_ms;
    vector<double> vdLoopOptEss_ms;
    vector<double> vdLoopTotal_ms;
    vector<int> vnLoopKFs;
    int nLoop;

    vector<double> vdGBA_ms;
    vector<double> vdUpdateMap_ms;
    vector<double> vdFGBATotal_ms;
    vector<int> vnGBAKFs;
    vector<int> vnGBAMPs;
    int nFGBA_exec;
    int nFGBA_abort;

#endif

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    /**
     * @brief 判断关键帧队列中是否有待处理关键帧
     * @return 非空返回 true
     */
    bool CheckNewKeyFrames();


    /**
     * @brief 新版地点识别入口
     * @return 检测到回环或地图融合候选返回 true
     *
     * 该函数会先尝试沿用上一次成功候选做时空连续验证，
     * 若失败再退回到基于 BoW 的候选检索。
     */
    bool NewDetectCommonRegions();
    /**
     * @brief 基于上一关键帧得到的候选进一步精化 Sim3
     * @param pCurrentKF        当前关键帧
     * @param pMatchedKF        候选匹配关键帧
     * @param gScw             输入输出 Sim3，表示当前帧到世界的相似变换
     * @param nNumProjMatches  输出：投影匹配数
     * @param vpMPs            输入输出：候选地图点集合
     * @param vpMatchedMPs     输出：当前帧中的匹配地图点
     * @return                 成功精化并通过验证返回 true
     */
    bool DetectAndReffineSim3FromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                        std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);
    /**
     * @brief 从 BoW 候选集合中寻找真正的公共区域
     * @param vpBowCand         BoW 候选关键帧集合
     * @param pMatchedKF        输出：最终匹配关键帧
     * @param pLastCurrentKF    输出：最近一次成功匹配的当前关键帧
     * @param g2oScw            输出：当前帧的 Sim3 位姿
     * @param nNumCoincidences  输出：连续命中次数
     * @param vpMPs             输出：候选地图点集合
     * @param vpMatchedMPs      输出：匹配到的地图点集合
     * @return                  成功找到稳定公共区域返回 true
     */
    bool DetectCommonRegionsFromBoW(std::vector<KeyFrame*> &vpBowCand, KeyFrame* &pMatchedKF, KeyFrame* &pLastCurrentKF, g2o::Sim3 &g2oScw,
                                     int &nNumCoincidences, std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);
    /**
     * @brief 使用上一关键帧的候选关系继续验证公共区域
     * @return 匹配数足够返回 true
     */
    bool DetectCommonRegionsFromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                            std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);
    /**
     * @brief 在给定 Sim3 下，通过投影搜索匹配地图点
     * @return 成功的投影匹配数量
     */
    int FindMatchesByProjection(KeyFrame* pCurrentKF, KeyFrame* pMatchedKFw, g2o::Sim3 &g2oScw,
                                set<MapPoint*> &spMatchedMPinOrigin, vector<MapPoint*> &vpMapPoints,
                                vector<MapPoint*> &vpMatchedMapPoints);


    /**
     * @brief 在一组 Sim3 修正位姿下执行地图点融合
     */
    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<MapPoint*> &vpMapPoints);
    /**
     * @brief 在一组关键帧原始位姿下执行地图点融合
     */
    void SearchAndFuse(const vector<KeyFrame*> &vConectedKFs, vector<MapPoint*> &vpMapPoints);

    /**
     * @brief 执行回环校正
     */
    void CorrectLoop();

    /**
     * @brief 视觉模式下的地图合并
     */
    void MergeLocal();
    /**
     * @brief 视觉惯性模式下的地图合并
     */
    void MergeLocal2();

    /**
     * @brief 调试函数：检查两组关键帧之间的公共观测
     */
    void CheckObservations(set<KeyFrame*> &spKFsMap1, set<KeyFrame*> &spKFsMap2);

    /**
     * @brief 如果收到 reset 请求，则在安全位置执行 reset
     */
    void ResetIfRequested();
    /// 是否请求完全重置。
    bool mbResetRequested;
    /// 是否请求只重置活动地图。
    bool mbResetActiveMapRequested;
    /// 需要被重置的活动地图。
    Map* mpMapToReset;
    std::mutex mMutexReset;

    /**
     * @brief 查询是否请求结束线程
     * @return true 表示应结束
     */
    bool CheckFinish();
    /**
     * @brief 标记线程已结束
     */
    void SetFinish();
    /// 是否请求结束线程。
    bool mbFinishRequested;
    /// 线程是否已经结束。
    bool mbFinished;
    std::mutex mMutexFinish;

    /// Atlas 多地图管理器。
    Atlas* mpAtlas;
    /// Tracking 线程指针。
    Tracking* mpTracker;

    /// 关键帧数据库，内部使用 BoW 倒排文件组织关键帧。
    KeyFrameDatabase* mpKeyFrameDB;
    /// ORB 词袋字典，用于关键帧相似性检索。
    ORBVocabulary* mpORBVocabulary;

    /// 局部建图线程指针。
    LocalMapping *mpLocalMapper;

    /// 待检测关键帧队列。
    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    /// 回环一致性阈值，表示需要连续几次命中才认为真正稳定。
    float mnCovisibilityConsistencyTh;

    /// 当前正在处理的关键帧。
    KeyFrame* mpCurrentKF;
    /// 最近一次成功识别公共区域时的“当前关键帧”。
    KeyFrame* mpLastCurrentKF;
    /// 一般回环流程中记录的匹配关键帧。
    KeyFrame* mpMatchedKF;
    /// 共视一致性分组。
    std::vector<ConsistentGroup> mvConsistentGroups;
    /// 满足一致性要求的候选关键帧。
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    /// 当前关键帧的共视邻居。
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    /// 当前关键帧匹配到的地图点。
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    /// 回环侧地图点集合。
    std::vector<MapPoint*> mvpLoopMapPoints;
    /// OpenCV 形式的 Sim3。
    cv::Mat mScw;
    /// 当前关键帧到世界的 Sim3。
    g2o::Sim3 mg2oScw;

    /// 最近一次处理关键帧所属地图。
    Map* mpLastMap;

    /// 是否已经检测到回环候选。
    bool mbLoopDetected;
    /// 当前回环候选已连续命中的次数。
    int mnLoopNumCoincidences;
    /// 当前回环候选连续未命中的次数。
    int mnLoopNumNotFound;
    /// 回环分支中最近一次成功的当前关键帧。
    KeyFrame* mpLoopLastCurrentKF;
    /// 回环分支中当前帧到候选世界的 Sim3。
    g2o::Sim3 mg2oLoopSlw;
    /// 回环校正时真正采用的 Sim3。
    g2o::Sim3 mg2oLoopScw;
    /// 回环匹配关键帧。
    KeyFrame* mpLoopMatchedKF;
    /// 回环侧地图点集合。
    std::vector<MapPoint*> mvpLoopMPs;
    /// 回环匹配到的地图点集合。
    std::vector<MapPoint*> mvpLoopMatchedMPs;
    /// 是否已经检测到地图合并候选。
    bool mbMergeDetected;
    /// 合并候选连续命中次数。
    int mnMergeNumCoincidences;
    /// 合并候选连续未命中次数。
    int mnMergeNumNotFound;
    /// 合并分支中最近一次成功的当前关键帧。
    KeyFrame* mpMergeLastCurrentKF;
    /// 合并分支中当前帧到候选世界的 Sim3。
    g2o::Sim3 mg2oMergeSlw;
    /// 合并目标地图在世界中的 Sim3。
    g2o::Sim3 mg2oMergeSmw;
    /// 合并校正时当前关键帧的 Sim3。
    g2o::Sim3 mg2oMergeScw;
    /// 合并匹配关键帧。
    KeyFrame* mpMergeMatchedKF;
    /// 合并侧地图点集合。
    std::vector<MapPoint*> mvpMergeMPs;
    /// 合并匹配到的地图点集合。
    std::vector<MapPoint*> mvpMergeMatchedMPs;
    /// 合并侧关键帧局部窗口。
    std::vector<KeyFrame*> mvpMergeConnectedKFs;

    /// 合并时估计得到的旧地图到新地图的 Sim3。
    g2o::Sim3 mSold_new;

    /// 最近一次成功回环的关键帧 id。
    long unsigned int mLastLoopKFid;

    /// GBA 线程是否运行中。
    bool mbRunningGBA;
    /// GBA 线程是否已完成。
    bool mbFinishedGBA;
    /// 是否请求停止 GBA。
    bool mbStopGBA;
    std::mutex mMutexGBA;
    /// GBA 线程对象。
    std::thread* mpThreadGBA;

    /// 是否固定尺度。双目 / RGB-D 一般固定，单目常允许尺度变化。
    bool mbFixScale;


    /// 全局 BA 版本号，每次中断并重启时会递增。
    bool mnFullBAIdx;

    /// 地点识别记录：当前关键帧时间戳。
    vector<double> vdPR_CurrentTime;
    /// 地点识别记录：匹配关键帧时间戳。
    vector<double> vdPR_MatchedTime;
    /// 地点识别类型，通常 0 表示回环，1 表示地图合并。
    vector<int> vnPR_TypeRecogn;

    /// 调试：子轨迹输出目录。
    string mstrFolderSubTraj;
    /// 当前已经执行过的回环/融合校正次数。
    int mnNumCorrection;
    /// 启动过的 GBA 修正次数。
    int mnCorrectionGBA;


    /// 是否启用回环线程中的地点识别与校正逻辑。
    bool mbActiveLC = true;

#ifdef REGISTER_LOOP
    string mstrFolderLoop;
#endif
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
