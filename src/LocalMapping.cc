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


#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Converter.h"
#include "GeometricTools.h"

#include<mutex>
#include<chrono>

namespace ORB_SLAM3
{

/**
 * @brief Local Mapping 构造函数
 * @param pSys        系统对象
 * @param pAtlas      地图集对象
 * @param bMonocular  是否为单目模式
 * @param bInertial   是否启用 IMU
 * @param _strSeqName 序列名称
 *
 * 这里只做成员初始化，不执行真正的建图逻辑。真正的工作会在 Run() 中的线程主循环里进行。
 */
LocalMapping::LocalMapping(System* pSys, Atlas *pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName):
    mpSystem(pSys), mbMonocular(bMonocular), mbInertial(bInertial), mbResetRequested(false), mbResetRequestedActiveMap(false), mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas), bInitializing(false),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true),
    mIdxInit(0), mScale(1.0), mInitSect(0), mbNotBA1(true), mbNotBA2(true), mIdxIteration(0), infoInertial(Eigen::MatrixXd::Zero(9,9))
{
    // 跟踪线程与当前关键帧的匹配内点数，初始化为 0。
    mnMatchesInliers = 0;

    // 标记 IMU 当前没有处于故障/初始化失败状态。
    mbBadImu = false;

    // 惯性初始化累计时间从 0 开始统计。
    mTinit = 0.f;

    // 统计变量初始化。
    mNumLM = 0;
    mNumKFCulling=0;

#ifdef REGISTER_TIMES
    nLBA_exec = 0;
    nLBA_abort = 0;
#endif

}

/**
 * @brief 设置闭环线程对象
 * @param pLoopCloser 闭环线程
 */
void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

/**
 * @brief 设置跟踪线程对象
 * @param pTracker 跟踪线程
 */
void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

/**
 * @brief Local Mapping 主线程循环
 *
 * 整个局部建图线程的节奏如下：
 * 1. 暂停接收新关键帧；
 * 2. 若队列非空，则处理最新关键帧；
 * 3. 剔除低质量新地图点；
 * 4. 与邻居关键帧三角化新地图点；
 * 5. 融合重复地图点；
 * 6. 执行局部 BA / 局部惯性 BA；
 * 7. 视需要做 IMU 初始化与尺度细化；
 * 8. 剔除冗余关键帧；
 * 9. 将当前关键帧送给闭环线程；
 * 10. 处理停止、重置、结束请求。
 */
void LocalMapping::Run()
{
    // 线程开始工作，先把“已结束”标志清掉。
    mbFinished = false;

    while(1)
    {
        // Local Mapping 正在处理关键帧时，暂时不希望 Tracking 继续塞入更多关键帧。
        SetAcceptKeyFrames(false);

        // 只要队列中存在新关键帧，并且 IMU 没有进入故障状态，就开始一轮完整的局部建图。
        if(CheckNewKeyFrames() && !mbBadImu)
        {
#ifdef REGISTER_TIMES
            double timeLBA_ms = 0;
            double timeKFCulling_ms = 0;

            std::chrono::steady_clock::time_point time_StartProcessKF = std::chrono::steady_clock::now();
#endif
            // 第一步：取出队列头关键帧，计算 BoW，建立观测关系，并加入地图。
            ProcessNewKeyFrame();
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndProcessKF = std::chrono::steady_clock::now();

            double timeProcessKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndProcessKF - time_StartProcessKF).count();
            vdKFInsert_ms.push_back(timeProcessKF);
#endif

            // 第二步：检查最近新建的地图点，尽早删除跟踪质量差或观测不足的点。
            MapPointCulling();
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMPCulling = std::chrono::steady_clock::now();

            double timeMPCulling = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMPCulling - time_EndProcessKF).count();
            vdMPCulling_ms.push_back(timeMPCulling);
#endif

            // 第三步：利用当前关键帧与共视邻居之间的匹配，通过三角化生成新地图点。
            CreateNewMapPoints();

            // 新地图点建完后，允许后续 BA 从“未中断”状态重新开始。
            mbAbortBA = false;

            if(!CheckNewKeyFrames())
            {
                // 如果处理期间没有新的关键帧插入，就继续做邻居搜索与重复点融合。
                SearchInNeighbors();
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMPCreation = std::chrono::steady_clock::now();

            double timeMPCreation = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMPCreation - time_EndMPCulling).count();
            vdMPCreation_ms.push_back(timeMPCreation);
#endif

            bool b_doneLBA = false;
            int num_FixedKF_BA = 0;
            int num_OptKF_BA = 0;
            int num_MPs_BA = 0;
            int num_edges_BA = 0;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // 地图中至少有 3 个关键帧时，局部 BA 才有意义。
                if(mpAtlas->KeyFramesInMap()>2)
                {

                    if(mbInertial && mpCurrentKeyFrame->GetMap()->isImuInitialized())
                    {
                        // 惯性模式下，先估计最近两段运动长度，用于判断 IMU 初始化后的运动是否足够。
                        float dist = (mpCurrentKeyFrame->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->GetCameraCenter()).norm() +
                                (mpCurrentKeyFrame->mPrevKF->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->mPrevKF->GetCameraCenter()).norm();

                        // 只有真实运动足够明显时，才累计用于 IMU 初始化判断的时间长度。
                        if(dist>0.05)
                            mTinit += mpCurrentKeyFrame->mTimeStamp - mpCurrentKeyFrame->mPrevKF->mTimeStamp;
                        if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2())
                        {
                            // 单目/双目惯性在初始化早期若运动太小，尺度和重力都不稳定，需要重置当前地图。
                            if((mTinit<10.f) && (dist<0.02))
                            {
                                cout << "Not enough motion for initializing. Reseting..." << endl;
                                unique_lock<mutex> lock(mMutexReset);
                                mbResetRequestedActiveMap = true;
                                mpMapToReset = mpCurrentKeyFrame->GetMap();
                                mbBadImu = true;
                            }
                        }

                        // 大量内点通常意味着当前跟踪稳定，可以扩大局部惯性 BA 的求解强度。
                        bool bLarge = ((mpTracker->GetMatchesInliers()>75)&&mbMonocular)||((mpTracker->GetMatchesInliers()>100)&&!mbMonocular);

                        // 已完成 IMU 初始化时，优先执行局部惯性 BA，同时联合优化位姿、速度、偏置和地图点。
                        Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA, bLarge, !mpCurrentKeyFrame->GetMap()->GetIniertialBA2());
                        b_doneLBA = true;
                    }
                    else
                    {
                        // 非惯性模式，或者 IMU 还没初始化时，退化为普通局部 BA。
                        Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA);
                        b_doneLBA = true;
                    }

                }
#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndLBA = std::chrono::steady_clock::now();

                if(b_doneLBA)
                {
                    timeLBA_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLBA - time_EndMPCreation).count();
                    vdLBA_ms.push_back(timeLBA_ms);

                    nLBA_exec += 1;
                    if(mbAbortBA)
                    {
                        nLBA_abort += 1;
                    }
                    vnLBA_edges.push_back(num_edges_BA);
                    vnLBA_KFopt.push_back(num_OptKF_BA);
                    vnLBA_KFfixed.push_back(num_FixedKF_BA);
                    vnLBA_MPs.push_back(num_MPs_BA);
                }

#endif

                // 第四步：如果当前地图尚未完成 IMU 初始化，则在这里触发惯性初始化。
                if(!mpCurrentKeyFrame->GetMap()->isImuInitialized() && mbInertial)
                {
                    // 单目惯性比双目惯性更依赖强先验，因此加速度偏置先验给得更强。
                    if (mbMonocular)
                        InitializeIMU(1e2, 1e10, true);
                    else
                        InitializeIMU(1e2, 1e5, true);
                }


                // 第五步：删除局部地图中冗余关键帧，避免图规模无节制膨胀。
                KeyFrameCulling();

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndKFCulling = std::chrono::steady_clock::now();

                timeKFCulling_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndKFCulling - time_EndLBA).count();
                vdKFCulling_ms.push_back(timeKFCulling_ms);
#endif

                if ((mTinit<50.0f) && mbInertial)
                {
                    // IMU 已初始化且跟踪状态正常时，分阶段触发更强的惯性 BA 和尺度细化。
                    if(mpCurrentKeyFrame->GetMap()->isImuInitialized() && mpTracker->mState==Tracking::OK) // Enter here everytime local-mapping is called
                    {
                        if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA1()){
                            if (mTinit>5.0f)
                            {
                                cout << "start VIBA 1" << endl;
                                mpCurrentKeyFrame->GetMap()->SetIniertialBA1();
                                if (mbMonocular)
                                    InitializeIMU(1.f, 1e5, true);
                                else
                                    InitializeIMU(1.f, 1e5, true);

                                cout << "end VIBA 1" << endl;
                            }
                        }
                        else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2()){
                            if (mTinit>15.0f){
                                cout << "start VIBA 2" << endl;
                                mpCurrentKeyFrame->GetMap()->SetIniertialBA2();
                                if (mbMonocular)
                                    InitializeIMU(0.f, 0.f, true);
                                else
                                    InitializeIMU(0.f, 0.f, true);

                                cout << "end VIBA 2" << endl;
                            }
                        }

                        // 单目惯性在初始化后还会周期性做尺度细化，使尺度和重力方向更稳定。
                        if (((mpAtlas->KeyFramesInMap())<=200) &&
                                ((mTinit>25.0f && mTinit<25.5f)||
                                (mTinit>35.0f && mTinit<35.5f)||
                                (mTinit>45.0f && mTinit<45.5f)||
                                (mTinit>55.0f && mTinit<55.5f)||
                                (mTinit>65.0f && mTinit<65.5f)||
                                (mTinit>75.0f && mTinit<75.5f))){
                            if (mbMonocular)
                                ScaleRefinement();
                        }
                    }
                }
            }

#ifdef REGISTER_TIMES
            vdLBASync_ms.push_back(timeKFCulling_ms);
            vdKFCullingSync_ms.push_back(timeKFCulling_ms);
#endif

            // 无论本轮是否执行了 BA，只要当前关键帧处理完成，都交给闭环线程做后续闭环检测。
            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndLocalMap = std::chrono::steady_clock::now();

            double timeLocalMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLocalMap - time_StartProcessKF).count();
            vdLMTotal_ms.push_back(timeLocalMap);
#endif
        }
        else if(Stop() && !mbBadImu)
        {
            // 没有关键建图任务且允许停止时，在安全区挂起线程。
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // 当前轮次处理完毕，重新允许 Tracking 插入关键帧。
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

    SetFinish();
}

/**
 * @brief 向 Local Mapping 队列中插入一个新关键帧
 * @param pKF 新关键帧
 */
void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);

    // 关键帧按时间顺序压入队尾，等待 Local Mapping 主线程处理。
    mlNewKeyFrames.push_back(pKF);

    // 插入新关键帧后，中断当前 BA，优先处理最新观测。
    mbAbortBA=true;
}


/**
 * @brief 检查关键帧队列是否为空
 * @return true 表示存在待处理关键帧
 */
bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

/**
 * @brief 处理一个新关键帧
 *
 * 该函数是 Local Mapping 真正“接管”关键帧的入口：
 * 1. 从队列里取出关键帧；
 * 2. 计算 BoW 表示；
 * 3. 把当前关键帧中已有的 MapPoint 观测关系补全；
 * 4. 更新共视连接；
 * 5. 将关键帧加入当前地图。
 *
 * 需要注意的是：
 * 当前关键帧中的 ORB 特征点、方向、描述子等信息，已经由 Tracking 线程内部调用
 * ORBextractor 提前提取完成。Local Mapping 在这里并不重新提特征，而是消费这些结果
 * 来补充地图结构。
 */
void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);

        // 取出队列头部关键帧，作为本轮局部建图的中心关键帧。
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // 计算当前关键帧的 BoW 表示，后续回环检测和重定位都会用到。
    mpCurrentKeyFrame->ComputeBoW();

    // 读取当前关键帧已经关联到的 MapPoint 集合。
    // 这些点很多是在 Tracking 线程中通过投影匹配、立体恢复或已有地图关联得到的。
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    // 若该 MapPoint 还未记录“当前关键帧观测”，则把第 i 个特征点观测加进去。
                    pMP->AddObservation(mpCurrentKeyFrame, i);

                    // 新增观测后，需要重新估计点的平均观测方向和可见深度范围。
                    pMP->UpdateNormalAndDepth();

                    // 重新选取更具代表性的 ORB 描述子，便于后续匹配更稳定。
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    // 这里通常对应“Tracking 线程刚刚用双目/深度直接恢复出来的新点”。
                    // 它已经在当前关键帧中登记过观测了，因此不重复添加，只放入最近新增列表，
                    // 让后续 MapPointCulling 对它做质量筛选。
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }

    // 根据共享 MapPoint 的数量更新当前关键帧在共视图中的边。
    mpCurrentKeyFrame->UpdateConnections();

    // 关键帧正式加入地图，后续局部 BA、闭环和地图管理都能访问到它。
    mpAtlas->AddKeyFrame(mpCurrentKeyFrame);
}

/**
 * @brief 连续处理队列中所有待处理关键帧
 */
void LocalMapping::EmptyQueue()
{
    while(CheckNewKeyFrames())
        ProcessNewKeyFrame();
}

/**
 * @brief 剔除最近新增但质量较差的地图点
 *
 * 新点在刚建立后的几帧里最不稳定，因此 ORB-SLAM3 会把它们放进
 * mlpRecentAddedMapPoints 中持续观察：
 * 1. 若点已经被标记为坏点，直接移除；
 * 2. 若被成功找到的比例过低，说明描述子或几何都不可靠，删除；
 * 3. 若经过若干关键帧后观测次数仍不足，删除；
 * 4. 若已经存活到足够“老”，则从“最近新增列表”中移出，不再特殊对待。
 */
void LocalMapping::MapPointCulling()
{
    // 遍历最近新增地图点列表，逐个决定保留、删除或移出观察列表。
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    // 单目初始化困难，对点的保留要求略低；双目/深度能够直接恢复尺度，要求更高。
    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    // 调试变量，记录当前列表中尚未被删除的点数。
    int borrar = mlpRecentAddedMapPoints.size();

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;

        // 情况 1：该点已经在别处被判坏，直接从“最近新增列表”移除。
        if(pMP->isBad())
            lit = mlpRecentAddedMapPoints.erase(lit);
        else if(pMP->GetFoundRatio()<0.25f)
        {
            // 情况 2：跟踪成功率过低，说明点不稳定，直接标坏并删除。
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            // 情况 3：经历了至少 2 个关键帧之后，观测次数仍不够，说明点缺乏稳定支持。
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            // 情况 4：新点已经挺过最脆弱的阶段，不再属于“recently added”。
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
        {
            // 其余情况暂时保留，等待后续关键帧继续验证。
            lit++;
            borrar--;
        }
    }
}


/**
 * @brief 利用当前关键帧与邻居关键帧三角化生成新的地图点
 *
 * 该函数是局部建图最关键的步骤之一。整体流程如下：
 * 1. 取当前关键帧的若干共视邻居；
 * 2. 对每个邻居做极线约束匹配，找到可用于三角化的特征点对；
 * 3. 根据双目或多视图几何恢复 3D 点；
 * 4. 检查点是否位于相机前方、重投影误差是否合理、尺度是否一致；
 * 5. 通过所有检查后创建 MapPoint，并挂接到两个关键帧上。
 *
 * 这里使用的特征点、金字塔层级、方向和 ORB 描述子，都来自 Tracking 阶段提前提取好的
 * ORB 特征。Local Mapping 自身不重新提特征，只基于这些结果做几何验证与建图。
 */
void LocalMapping::CreateNewMapPoints()
{
    // 根据共视图取一批最相关邻居。
    // 单目由于尺度更弱，需要更大的邻域来补充足够视差。
    int nn = 10;
    if(mbMonocular)
        nn=30;
    vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    if (mbInertial)
    {
        // 惯性模式下，额外沿时间链向前补一些相邻关键帧。
        // 原因是 IMU 场景中，时间上相邻的帧即使共视不强，也可能提供有价值的约束。
        KeyFrame* pKF = mpCurrentKeyFrame;
        int count=0;
        while((vpNeighKFs.size()<=nn)&&(pKF->mPrevKF)&&(count++<nn))
        {
            vector<KeyFrame*>::iterator it = std::find(vpNeighKFs.begin(), vpNeighKFs.end(), pKF->mPrevKF);
            if(it==vpNeighKFs.end())
                vpNeighKFs.push_back(pKF->mPrevKF);
            pKF = pKF->mPrevKF;
        }
    }

    float th = 0.6f;

    // ORBmatcher 先基于描述子和几何约束找到候选匹配。
    ORBmatcher matcher(th,false);

    // 取当前关键帧位姿和相机内参，后续会频繁使用。
    Sophus::SE3<float> sophTcw1 = mpCurrentKeyFrame->GetPose();
    Eigen::Matrix<float,3,4> eigTcw1 = sophTcw1.matrix3x4();
    Eigen::Matrix<float,3,3> Rcw1 = eigTcw1.block<3,3>(0,0);
    Eigen::Matrix<float,3,3> Rwc1 = Rcw1.transpose();
    Eigen::Vector3f tcw1 = sophTcw1.translation();
    Eigen::Vector3f Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;
    int countStereo = 0;
    int countStereoGoodProj = 0;
    int countStereoAttempt = 0;
    int totalStereoPts = 0;

    // 逐个邻居关键帧搜索匹配并尝试三角化。
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        // 如果处理过程中又来了新的关键帧，则优先返回，让主循环先处理更新的数据。
        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        GeometricCamera* pCamera1 = mpCurrentKeyFrame->mpCamera, *pCamera2 = pKF2->mpCamera;

        // 先检查两帧之间基线是否足够。
        // 基线太短时，三角化会因为视差过小而数值不稳定。
        Eigen::Vector3f Ow2 = pKF2->GetCameraCenter();
        Eigen::Vector3f vBaseline = Ow2-Ow1;
        const float baseline = vBaseline.norm();

        if(!mbMonocular)
        {
            // 双目/深度场景下，至少要求跨关键帧的基线不小于双目基线。
            if(baseline<pKF2->mb)
                continue;
        }
        else
        {
            // 单目无法直接知道真实尺度，因此用“基线 / 场景中值深度”的比值做归一化判断。
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
        }

        // 使用 ORB 描述子匹配，并额外施加极线约束，筛出可以参与三角化的匹配对。
        vector<pair<size_t,size_t> > vMatchedIndices;

        // 惯性模式下刚刚丢失又找回时，允许更粗的匹配策略帮助恢复局部地图。
        bool bCoarse = mbInertial && mpTracker->mState==Tracking::RECENTLY_LOST && mpCurrentKeyFrame->GetMap()->GetIniertialBA2();

        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,vMatchedIndices,false,bCoarse);

        // 取邻居关键帧位姿和内参，供三角化及重投影误差验证使用。
        Sophus::SE3<float> sophTcw2 = pKF2->GetPose();
        Eigen::Matrix<float,3,4> eigTcw2 = sophTcw2.matrix3x4();
        Eigen::Matrix<float,3,3> Rcw2 = eigTcw2.block<3,3>(0,0);
        Eigen::Matrix<float,3,3> Rwc2 = Rcw2.transpose();
        Eigen::Vector3f tcw2 = sophTcw2.translation();

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // 对每一对匹配特征尝试恢复 3D 点。
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = (mpCurrentKeyFrame -> NLeft == -1) ? mpCurrentKeyFrame->mvKeysUn[idx1]
                                                                         : (idx1 < mpCurrentKeyFrame -> NLeft) ? mpCurrentKeyFrame -> mvKeys[idx1]
                                                                                                               : mpCurrentKeyFrame -> mvKeysRight[idx1 - mpCurrentKeyFrame -> NLeft];
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = (!mpCurrentKeyFrame->mpCamera2 && kp1_ur>=0);
            const bool bRight1 = (mpCurrentKeyFrame -> NLeft == -1 || idx1 < mpCurrentKeyFrame -> NLeft) ? false
                                                                                                         : true;

            const cv::KeyPoint &kp2 = (pKF2 -> NLeft == -1) ? pKF2->mvKeysUn[idx2]
                                                            : (idx2 < pKF2 -> NLeft) ? pKF2 -> mvKeys[idx2]
                                                                                     : pKF2 -> mvKeysRight[idx2 - pKF2 -> NLeft];

            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = (!pKF2->mpCamera2 && kp2_ur>=0);
            const bool bRight2 = (pKF2 -> NLeft == -1 || idx2 < pKF2 -> NLeft) ? false
                                                                               : true;

            // 对双目鱼眼 / 双相机场景，需要根据匹配点落在左目还是右目，
            // 动态切换使用的相机模型和位姿。
            if(mpCurrentKeyFrame->mpCamera2 && pKF2->mpCamera2){
                if(bRight1 && bRight2){
                    sophTcw1 = mpCurrentKeyFrame->GetRightPose();
                    Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

                    sophTcw2 = pKF2->GetRightPose();
                    Ow2 = pKF2->GetRightCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera2;
                    pCamera2 = pKF2->mpCamera2;
                }
                else if(bRight1 && !bRight2){
                    sophTcw1 = mpCurrentKeyFrame->GetRightPose();
                    Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

                    sophTcw2 = pKF2->GetPose();
                    Ow2 = pKF2->GetCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera2;
                    pCamera2 = pKF2->mpCamera;
                }
                else if(!bRight1 && bRight2){
                    sophTcw1 = mpCurrentKeyFrame->GetPose();
                    Ow1 = mpCurrentKeyFrame->GetCameraCenter();

                    sophTcw2 = pKF2->GetRightPose();
                    Ow2 = pKF2->GetRightCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera;
                    pCamera2 = pKF2->mpCamera2;
                }
                else{
                    sophTcw1 = mpCurrentKeyFrame->GetPose();
                    Ow1 = mpCurrentKeyFrame->GetCameraCenter();

                    sophTcw2 = pKF2->GetPose();
                    Ow2 = pKF2->GetCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera;
                    pCamera2 = pKF2->mpCamera;
                }
                eigTcw1 = sophTcw1.matrix3x4();
                Rcw1 = eigTcw1.block<3,3>(0,0);
                Rwc1 = Rcw1.transpose();
                tcw1 = sophTcw1.translation();

                eigTcw2 = sophTcw2.matrix3x4();
                Rcw2 = eigTcw2.block<3,3>(0,0);
                Rwc2 = Rcw2.transpose();
                tcw2 = sophTcw2.translation();
            }

            // 将像素点反投影为归一化视线方向，检查两条观测射线之间的视差是否足够。
            // 视差过小时，多视图三角化会非常不稳定。
            Eigen::Vector3f xn1 = pCamera1->unprojectEig(kp1.pt);
            Eigen::Vector3f xn2 = pCamera2->unprojectEig(kp2.pt);

            Eigen::Vector3f ray1 = Rwc1 * xn1;
            Eigen::Vector3f ray2 = Rwc2 * xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(ray1.norm() * ray2.norm());

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            if (bStereo1 || bStereo2) totalStereoPts++;
            
            // 若两帧本身有立体深度，则它给出的“可接受最小视差”更可靠，取更严格者。
            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            Eigen::Vector3f x3D;

            bool goodProj = false;
            bool bPointStereo = false;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 ||
                                                                          (cosParallaxRays<0.9996 && mbInertial) || (cosParallaxRays<0.9998 && !mbInertial)))
            {
                // 视差足够时，优先使用两帧几何直接三角化。
                goodProj = GeometricTools::Triangulate(xn1, xn2, eigTcw1, eigTcw2, x3D);
                if(!goodProj)
                    continue;
            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                // 若当前关键帧自身带有更可靠的立体深度，则直接用当前关键帧单帧反投影恢复 3D 点。
                countStereoAttempt++;
                bPointStereo = true;
                goodProj = mpCurrentKeyFrame->UnprojectStereo(idx1, x3D);
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                // 若邻居关键帧的立体深度更可靠，则使用邻居关键帧反投影恢复 3D 点。
                countStereoAttempt++;
                bPointStereo = true;
                goodProj = pKF2->UnprojectStereo(idx2, x3D);
            }
            else
            {
                // 既没有稳定的立体深度，又没有足够视差，无法可靠建点。
                continue; //No stereo and very low parallax
            }

            if(goodProj && bPointStereo)
                countStereoGoodProj++;

            if(!goodProj)
                continue;

            // 检查恢复出来的 3D 点是否同时位于两个相机前方。
            // 若 z <= 0，则说明几何恢复不合理。
            float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
            if(z2<=0)
                continue;

            // 使用当前关键帧做第一次重投影误差检验。
            // 误差阈值随金字塔层级 sigma^2 缩放。
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3D)+tcw1(0);
            const float y1 = Rcw1.row(1).dot(x3D)+tcw1(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                // 单目观测时，只检查像素平面上的二维重投影误差。
                cv::Point2f uv1 = pCamera1->project(cv::Point3f(x1,y1,z1));
                float errX1 = uv1.x - kp1.pt.x;
                float errY1 = uv1.y - kp1.pt.y;

                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;

            }
            else
            {
                // 立体观测时，同时检查左目 u、v 误差和右目 u_r 误差。
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            // 用邻居关键帧做第二次重投影误差检验。
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3D)+tcw2(0);
            const float y2 = Rcw2.row(1).dot(x3D)+tcw2(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                // 单目邻居只检查二维重投影误差。
                cv::Point2f uv2 = pCamera2->project(cv::Point3f(x2,y2,z2));
                float errX2 = uv2.x - kp2.pt.x;
                float errY2 = uv2.y - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                // 立体邻居同时校验左右目一致性。
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            // 检查尺度一致性：
            // 近距离观察通常应该出现在更细尺度层，远距离观察则倾向更粗尺度层。
            // 如果两帧的距离比与两侧特征金字塔层级不匹配，则该三角化结果可疑。
            Eigen::Vector3f normal1 = x3D - Ow1;
            float dist1 = normal1.norm();

            Eigen::Vector3f normal2 = x3D - Ow2;
            float dist2 = normal2.norm();

            if(dist1==0 || dist2==0)
                continue;

            if(mbFarPoints && (dist1>=mThFarPoints||dist2>=mThFarPoints)) // MODIFICATION
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // 通过所有几何与尺度检查后，正式创建地图点。
            MapPoint* pMP = new MapPoint(x3D, mpCurrentKeyFrame, mpAtlas->GetCurrentMap());
            if (bPointStereo)
                countStereo++;
            
            // 将两个关键帧对该地图点的观测关系都登记起来。
            pMP->AddObservation(mpCurrentKeyFrame,idx1);
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            // 为新点计算代表性 ORB 描述子与法向/深度范围。
            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            // 加入地图，并放入“最近新增列表”等待后续质量筛除。
            mpAtlas->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);
        }
    }    
}

/**
 * @brief 在邻居关键帧之间搜索重复地图点并执行融合
 *
 * CreateNewMapPoints() 负责“扩张地图”，而本函数负责“收缩冗余”：
 * 1. 选出一批目标关键帧；
 * 2. 把当前关键帧的地图点投影到目标关键帧中，看能否与已有点融合；
 * 3. 再把目标关键帧中的地图点投影回当前关键帧，做反向融合；
 * 4. 融合完成后重新计算地图点描述子、法向和关键帧连接关系。
 */
void LocalMapping::SearchInNeighbors()
{
    // 先取一圈最强共视邻居。
    int nn = 10;
    if(mbMonocular)
        nn=30;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFrame*> vpTargetKFs;
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
    }

    // 再扩展一圈“邻居的邻居”，让融合范围稍微更大一些。
    // 这一层扩张可以帮助消除局部重复建图，但如果 BA 已请求中断，就及时停下。
    for(int i=0, imax=vpTargetKFs.size(); i<imax; i++)
    {
        const vector<KeyFrame*> vpSecondNeighKFs = vpTargetKFs[i]->GetBestCovisibilityKeyFrames(20);
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
            pKFi2->mnFuseTargetForKF=mpCurrentKeyFrame->mnId;
        }
        if (mbAbortBA)
            break;
    }

    // 惯性模式下，再沿时间链补一些相邻关键帧，增强局部时序一致性。
    if(mbInertial)
    {
        KeyFrame* pKFi = mpCurrentKeyFrame->mPrevKF;
        while(vpTargetKFs.size()<20 && pKFi)
        {
            if(pKFi->isBad() || pKFi->mnFuseTargetForKF==mpCurrentKeyFrame->mnId)
            {
                pKFi = pKFi->mPrevKF;
                continue;
            }
            vpTargetKFs.push_back(pKFi);
            pKFi->mnFuseTargetForKF=mpCurrentKeyFrame->mnId;
            pKFi = pKFi->mPrevKF;
        }
    }

    // 第一步融合：把“当前关键帧已有地图点”投影到目标关键帧中，尝试与目标关键帧已有点合并。
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches);
        if(pKFi->NLeft != -1) matcher.Fuse(pKFi,vpMapPointMatches,true);
    }


    if (mbAbortBA)
        return;

    // 第二步融合：收集目标关键帧中的地图点，再投影回当前关键帧，做反向融合。
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);
    if(mpCurrentKeyFrame->NLeft != -1) matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates,true);


    // 融合后，当前关键帧内关联到的地图点可能发生合并，需要重新计算描述子和法向信息。
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // 地图点观测关系变了，共视图也要相应更新。
    mpCurrentKeyFrame->UpdateConnections();
}

/**
 * @brief 请求停止 Local Mapping
 */
void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);

    // 停止前先中断 BA，避免长时间阻塞在线程内部。
    mbAbortBA = true;
}

/**
 * @brief 尝试进入停止状态
 * @return 若成功停止则返回 true
 */
bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        // 只有当外部已经请求停止，并且当前阶段允许停止时，才真正挂起线程。
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

/**
 * @brief 判断线程是否已经停止
 * @return true 表示线程已停止
 */
bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

/**
 * @brief 查询是否收到停止请求
 * @return true 表示已经收到停止请求
 */
bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

/**
 * @brief 从停止状态恢复线程运行
 */
void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;

    // 清掉停止标志，使主循环继续工作。
    mbStopped = false;
    mbStopRequested = false;

    // 队列里遗留的关键帧在“暂停-恢复”语义下被认为无效，直接删除。
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

/**
 * @brief 查询当前是否接受新关键帧
 * @return true 表示允许 Tracking 插入关键帧
 */
bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

/**
 * @brief 设置是否接受新关键帧
 * @param flag true 表示接受
 */
void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

/**
 * @brief 设置线程是否处于“不可停止”阶段
 * @param flag true 表示不可停止
 * @return 若设置成功返回 true
 */
bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    // 若线程已经处于停止状态，就不能再把它切换到“不可停止”。
    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

/**
 * @brief 中断当前 BA
 */
void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

/**
 * @brief 剔除局部地图中的冗余关键帧
 *
 * 基本思想：
 * 若一个关键帧看到的大多数地图点，都已经被其他足够多的关键帧以相同或更细尺度观测到，
 * 那么它携带的信息就是冗余的，可以删除。
 *
 * 惯性模式下除了共视冗余外，还需要维护时间链的连续性，因此删除前会额外处理：
 * 1. IMU 预积分的合并；
 * 2. 前后关键帧指针的重连；
 * 3. 只在时间间隔和运动条件合理时才允许删除。
 */
void LocalMapping::KeyFrameCulling()
{
    // 只在当前关键帧的局部共视邻域内做关键帧冗余判断。
    // 这里 Nd 也是惯性可优化时间窗口的大致长度。
    const int Nd = 21;
    mpCurrentKeyFrame->UpdateBestCovisibles();
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    // 非惯性和单目惯性通常采用更激进的冗余判定；
    // 双目惯性为了保留更稳的时序/尺度约束，阈值更宽松。
    float redundant_th;
    if(!mbInertial)
        redundant_th = 0.9;
    else if (mbMonocular)
        redundant_th = 0.9;
    else
        redundant_th = 0.5;

    const bool bInitImu = mpAtlas->isImuInitialized();
    int count=0;

    // 估计惯性可优化窗口的最老关键帧 id。
    // 该窗口之前的关键帧在删减时需要更谨慎。
    unsigned int last_ID;
    if (mbInertial)
    {
        int count = 0;
        KeyFrame* aux_KF = mpCurrentKeyFrame;
        while(count<Nd && aux_KF->mPrevKF)
        {
            aux_KF = aux_KF->mPrevKF;
            count++;
        }
        last_ID = aux_KF->mnId;
    }



    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        count++;
        KeyFrame* pKF = *vit;

        // 初始关键帧和坏关键帧不参与冗余判断。
        if((pKF->mnId==pKF->GetMap()->GetInitKFid()) || pKF->isBad())
            continue;
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        // 双目/深度模式只统计近点。
                        // 远点对平移约束弱，不能可靠支持冗余判定。
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        // 当前关键帧中该地图点的尺度层。
                        const int &scaleLevel = (pKF -> NLeft == -1) ? pKF->mvKeysUn[i].octave
                                                                     : (i < pKF -> NLeft) ? pKF -> mvKeys[i].octave
                                                                                          : pKF -> mvKeysRight[i].octave;

                        // 读取该地图点被哪些关键帧观测到，并统计“能替代当前关键帧”的观测数。
                        const map<KeyFrame*, tuple<int,int>> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFrame*, tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            tuple<int,int> indexes = mit->second;
                            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
                            int scaleLeveli = -1;
                            if(pKFi -> NLeft == -1)
                                scaleLeveli = pKFi->mvKeysUn[leftIndex].octave;
                            else {
                                if (leftIndex != -1) {
                                    scaleLeveli = pKFi->mvKeys[leftIndex].octave;
                                }
                                if (rightIndex != -1) {
                                    // 对双相机模型，左右目都可能观测到该点，取更细的一侧尺度层。
                                    int rightLevel = pKFi->mvKeysRight[rightIndex - pKFi->NLeft].octave;
                                    scaleLeveli = (scaleLeveli == -1 || scaleLeveli > rightLevel) ? rightLevel
                                                                                                  : scaleLeveli;
                                }
                            }

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                // 其他关键帧若以同尺度或更细尺度看到该点，说明它们足以替代当前关键帧。
                                nObs++;
                                if(nObs>thObs)
                                    break;
                            }
                        }
                        if(nObs>thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        // 若冗余观测占比足够高，则判定当前关键帧可以被删除。
        if(nRedundantObservations>redundant_th*nMPs)
        {
            if (mbInertial)
            {
                // 惯性场景下，不能删得太狠，否则会破坏局部时间链稳定性。
                if (mpAtlas->KeyFramesInMap()<=Nd)
                    continue;

                // 当前关键帧附近的最新两帧通常保留，避免刚插入就立刻删掉。
                if(pKF->mnId>(mpCurrentKeyFrame->mnId-2))
                    continue;

                if(pKF->mPrevKF && pKF->mNextKF)
                {
                    // t 表示删除当前关键帧后，前后关键帧之间跨越的时间长度。
                    const float t = pKF->mNextKF->mTimeStamp-pKF->mPrevKF->mTimeStamp;

                    if((bInitImu && (pKF->mnId<last_ID) && t<3.) || (t<0.5))
                    {
                        // 删除关键帧前，先把该关键帧到其前驱的 IMU 预积分，
                        // 合并到后继关键帧的预积分中，保持时序链连续。
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);

                        // 重新连接时间链。
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = NULL;
                        pKF->mPrevKF = NULL;
                        pKF->SetBadFlag();
                    }
                    else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2() && ((pKF->GetImuPosition()-pKF->mPrevKF->GetImuPosition()).norm()<0.02) && (t<3))
                    {
                        // 在惯性 BA2 之前，若运动特别小，也允许通过合并预积分删掉这类冗余关键帧。
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = NULL;
                        pKF->mPrevKF = NULL;
                        pKF->SetBadFlag();
                    }
                }
            }
            else
            {
                // 非惯性模式下，直接将其标记为坏关键帧即可。
                pKF->SetBadFlag();
            }
        }
        if((count > 20 && mbAbortBA) || count>100)
        {
            // 若 BA 被中断，就不要在关键帧剔除里停太久；
            // 同时也限制单次扫描的关键帧上限。
            break;
        }
    }
}

/**
 * @brief 请求重置 Local Mapping
 *
 * 调用方会一直等待，直到 Local Mapping 真正执行了 ResetIfRequested 并清除该标志。
 */
void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        cout << "LM: Map reset recieved" << endl;
        mbResetRequested = true;
    }
    cout << "LM: Map reset, waiting..." << endl;

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        // 等待 Local Mapping 线程在安全位置执行重置。
        usleep(3000);
    }
    cout << "LM: Map reset, Done!!!" << endl;
}

/**
 * @brief 请求重置当前活动地图
 * @param pMap 待重置的地图对象
 */
void LocalMapping::RequestResetActiveMap(Map* pMap)
{
    {
        unique_lock<mutex> lock(mMutexReset);
        cout << "LM: Active map reset recieved" << endl;
        mbResetRequestedActiveMap = true;
        mpMapToReset = pMap;
    }
    cout << "LM: Active map reset, waiting..." << endl;

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequestedActiveMap)
                break;
        }
        // 同样采用忙等待，直到 Local Mapping 主线程执行重置。
        usleep(3000);
    }
    cout << "LM: Active map reset, Done!!!" << endl;
}

/**
 * @brief 在安全时机执行重置
 *
 * 这里不会直接销毁地图，而是重置 Local Mapping 自己维护的运行态：
 * 关键帧队列、最近新增地图点列表，以及 IMU 初始化相关的状态量。
 */
void LocalMapping::ResetIfRequested()
{
    bool executed_reset = false;
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbResetRequested)
        {
            executed_reset = true;

            cout << "LM: Reseting Atlas in Local Mapping..." << endl;

            // 清空待处理关键帧和待筛选地图点，让局部建图从干净状态重新开始。
            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();
            mbResetRequested = false;
            mbResetRequestedActiveMap = false;

            // 惯性初始化相关状态全部复位。
            mTinit = 0.f;
            mbNotBA2 = true;
            mbNotBA1 = true;
            mbBadImu=false;

            mIdxInit=0;

            cout << "LM: End reseting Local Mapping..." << endl;
        }

        if(mbResetRequestedActiveMap) {
            executed_reset = true;
            cout << "LM: Reseting current map in Local Mapping..." << endl;
            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();

            // 活动地图重置时，同样重置惯性状态。
            mTinit = 0.f;
            mbNotBA2 = true;
            mbNotBA1 = true;
            mbBadImu=false;

            mbResetRequested = false;
            mbResetRequestedActiveMap = false;
            cout << "LM: End reseting Local Mapping..." << endl;
        }
    }
    if(executed_reset)
        cout << "LM: Reset free the mutex" << endl;

}

/**
 * @brief 请求结束 Local Mapping 线程
 */
void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

/**
 * @brief 查询是否收到结束请求
 * @return true 表示需要结束线程
 */
bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

/**
 * @brief 将线程状态置为结束
 */
void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);

    // 结束后也视为停止状态，方便外部统一查询。
    mbStopped = true;
}

/**
 * @brief 查询线程是否已经结束
 * @return true 表示线程已结束
 */
bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

/**
 * @brief 初始化 IMU、尺度和传感器偏置
 * @param priorG 重力先验权重
 * @param priorA 加速度偏置先验权重
 * @param bFIBA  是否在初始化后继续执行 Full Inertial BA
 *
 * 这是 ORB-SLAM3 单目/双目惯性初始化中的核心过程，主要包括：
 * 1. 收集时间上连续的一串关键帧；
 * 2. 估计初始重力方向与速度；
 * 3. 通过惯性优化联合求解旋转、尺度、陀螺仪偏置和加速度计偏置；
 * 4. 把解出来的尺度和姿态修正应用到整张地图；
 * 5. 必要时再做一次 Full Inertial BA，使初始化结果更稳定。
 */
void LocalMapping::InitializeIMU(float priorG, float priorA, bool bFIBA)
{
    // 若系统正处于重置过程中，就不要继续做 IMU 初始化。
    if (mbResetRequested)
        return;

    float minTime;
    int nMinKF;
    if (mbMonocular)
    {
        // 单目惯性初始化更难，需要更长时间跨度积累足够激励。
        minTime = 2.0;
        nMinKF = 10;
    }
    else
    {
        // 双目/深度由于已有尺度，初始化所需时间可以更短。
        minTime = 1.0;
        nMinKF = 10;
    }


    // 关键帧数量不够时，不进行初始化。
    if(mpAtlas->KeyFramesInMap()<nMinKF)
        return;

    // 沿时间链从当前关键帧向前回溯，收集按时间排序的一串关键帧。
    list<KeyFrame*> lpKF;
    KeyFrame* pKF = mpCurrentKeyFrame;
    while(pKF->mPrevKF)
    {
        lpKF.push_front(pKF);
        pKF = pKF->mPrevKF;
    }
    lpKF.push_front(pKF);
    vector<KeyFrame*> vpKF(lpKF.begin(),lpKF.end());

    if(vpKF.size()<nMinKF)
        return;

    // 时间跨度不足时，激励不够，重力和尺度估计通常不稳定。
    mFirstTs=vpKF.front()->mTimeStamp;
    if(mpCurrentKeyFrame->mTimeStamp-mFirstTs<minTime)
        return;

    // 进入初始化状态，供外部查询。
    bInitializing = true;

    while(CheckNewKeyFrames())
    {
        // 初始化期间若又插入了关键帧，把它们也纳入当前时间窗口，提高求解稳定性。
        ProcessNewKeyFrame();
        vpKF.push_back(mpCurrentKeyFrame);
        lpKF.push_back(mpCurrentKeyFrame);
    }

    const int N = vpKF.size();
    IMU::Bias b(0,0,0,0,0,0);

    // 若当前地图还没有完成 IMU 初始化，需要先估计一个粗略的重力方向和速度初值。
    if (!mpCurrentKeyFrame->GetMap()->isImuInitialized())
    {
        Eigen::Matrix3f Rwg;
        Eigen::Vector3f dirG;
        dirG.setZero();
        for(vector<KeyFrame*>::iterator itKF = vpKF.begin(); itKF!=vpKF.end(); itKF++)
        {
            if (!(*itKF)->mpImuPreintegrated)
                continue;
            if (!(*itKF)->mPrevKF)
                continue;

            // 利用预积分速度增量和相邻关键帧姿态，累计重力方向。
            dirG -= (*itKF)->mPrevKF->GetImuRotation() * (*itKF)->mpImuPreintegrated->GetUpdatedDeltaVelocity();

            // 用相邻关键帧位置差 / 时间差估计线速度，并写回关键帧。
            Eigen::Vector3f _vel = ((*itKF)->GetImuPosition() - (*itKF)->mPrevKF->GetImuPosition())/(*itKF)->mpImuPreintegrated->dT;
            (*itKF)->SetVelocity(_vel);
            (*itKF)->mPrevKF->SetVelocity(_vel);
        }

        // 把累计得到的重力方向单位化，并求旋转矩阵 Rwg，使世界系 z 轴对齐真实重力方向。
        dirG = dirG/dirG.norm();
        Eigen::Vector3f gI(0.0f, 0.0f, -1.0f);
        Eigen::Vector3f v = gI.cross(dirG);
        const float nv = v.norm();
        const float cosg = gI.dot(dirG);
        const float ang = acos(cosg);
        Eigen::Vector3f vzg = v*ang/nv;
        Rwg = Sophus::SO3f::exp(vzg).matrix();
        mRwg = Rwg.cast<double>();

        // 记录当前用于惯性初始化的累计时间长度。
        mTinit = mpCurrentKeyFrame->mTimeStamp-mFirstTs;
    }
    else
    {
        // 若地图已经完成 IMU 初始化，则直接使用当前估计的旋转和偏置作为初值。
        mRwg = Eigen::Matrix3d::Identity();
        mbg = mpCurrentKeyFrame->GetGyroBias().cast<double>();
        mba = mpCurrentKeyFrame->GetAccBias().cast<double>();
    }

    // 单目惯性需要显式估计尺度；双目一般也统一走这一套优化流程。
    mScale=1.0;

    // 记录当前初始化窗口覆盖的时间长度。
    mInitTime = mpTracker->mLastFrame.mTimeStamp-vpKF.front()->mTimeStamp;

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

    // 惯性优化：
    // 联合优化重力旋转 mRwg、尺度 mScale、陀螺仪偏置 mbg 和加速度偏置 mba。
    Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale, mbg, mba, mbMonocular, infoInertial, false, false, priorG, priorA);

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    if (mScale<1e-1)
    {
        // 求出的尺度太小通常意味着初始化失败或数据激励不足，直接放弃本轮初始化。
        cout << "scale too small" << endl;
        bInitializing=false;
        return;
    }

    // 到这里为止都只是“算参数”，还没有真正修改地图。
    // 从这里开始会把估计结果应用到地图和轨迹上，因此需要加地图更新锁。
    {
        unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
        if ((fabs(mScale - 1.f) > 0.00001) || !mbMonocular) {
            // 把估计得到的重力对齐旋转和尺度统一作用到整张地图。
            Sophus::SE3f Twg(mRwg.cast<float>().transpose(), Eigen::Vector3f::Zero());
            mpAtlas->GetCurrentMap()->ApplyScaledRotation(Twg, mScale, true);

            // 同时更新 Tracking 当前帧中保存的 IMU 状态，使前端与地图保持一致。
            mpTracker->UpdateFrameIMU(mScale, vpKF[0]->GetImuBias(), mpCurrentKeyFrame);
        }

        // 若地图此前尚未被标记为 IMU 已初始化，则把当前窗口内关键帧都标记为惯性关键帧。
        if (!mpAtlas->isImuInitialized())
            for (int i = 0; i < N; i++) {
                KeyFrame *pKF2 = vpKF[i];
                pKF2->bImu = true;
            }
    }

    // 再做一次帧状态同步，确保跟踪线程中当前帧状态一致。
    mpTracker->UpdateFrameIMU(1.0,vpKF[0]->GetImuBias(),mpCurrentKeyFrame);
    if (!mpAtlas->isImuInitialized())
    {
        // 地图级别的 IMU 初始化完成标志只在首次成功时设置一次。
        mpAtlas->SetImuInitialized();
        mpTracker->t0IMU = mpTracker->mCurrentFrame.mTimeStamp;
        mpCurrentKeyFrame->bImu = true;
    }

    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    if (bFIBA)
    {
        // 初始化后可选再做一次全局惯性 BA，把位姿、速度、偏置和地图点进一步联合优化。
        if (priorA!=0.f)
            Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false, mpCurrentKeyFrame->mnId, NULL, true, priorG, priorA);
        else
            Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false, mpCurrentKeyFrame->mnId, NULL, false);
    }

    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();

    Verbose::PrintMess("Global Bundle Adjustment finished\nUpdating map ...", Verbose::VERBOSITY_NORMAL);

    // 从这里开始，根据 BA 结果批量更新地图中的关键帧和地图点。
    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

    unsigned long GBAid = mpCurrentKeyFrame->mnId;

    // 把 BA 期间新进入队列的关键帧也处理掉，避免地图状态滞后。
    while(CheckNewKeyFrames())
    {
        ProcessNewKeyFrame();
        vpKF.push_back(mpCurrentKeyFrame);
        lpKF.push_back(mpCurrentKeyFrame);
    }

    // 从地图原点关键帧开始，沿生成树传播 BA 校正结果。
    list<KeyFrame*> lpKFtoCheck(mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.begin(),mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.end());

    while(!lpKFtoCheck.empty())
    {
        KeyFrame* pKF = lpKFtoCheck.front();
        const set<KeyFrame*> sChilds = pKF->GetChilds();
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
        {
            KeyFrame* pChild = *sit;
            if(!pChild || pChild->isBad())
                continue;

            if(pChild->mnBAGlobalForKF!=GBAid)
            {
                // 若子关键帧没有直接参与这次 GBA，则由父关键帧的校正结果递推得到它的新位姿。
                Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
                pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;

                // 速度也需要根据姿态改变量进行旋转修正。
                Sophus::SO3f Rcor = pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3();
                if(pChild->isVelocitySet()){
                    pChild->mVwbGBA = Rcor * pChild->GetVelocity();
                }
                else {
                    Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);
                }

                pChild->mBiasGBA = pChild->GetImuBias();
                pChild->mnBAGlobalForKF = GBAid;

            }
            lpKFtoCheck.push_back(pChild);
        }

        // 把优化前位姿保存起来，再写入优化后位姿。
        pKF->mTcwBefGBA = pKF->GetPose();
        pKF->SetPose(pKF->mTcwGBA);

        if(pKF->bImu)
        {
            // 惯性关键帧还要同步更新速度和偏置。
            pKF->mVwbBefGBA = pKF->GetVelocity();
            pKF->SetVelocity(pKF->mVwbGBA);
            pKF->SetNewBias(pKF->mBiasGBA);
        } else {
            cout << "KF " << pKF->mnId << " not set to inertial!! \n";
        }

        lpKFtoCheck.pop_front();
    }

    // 接着批量更新地图点位置。
    const vector<MapPoint*> vpMPs = mpAtlas->GetCurrentMap()->GetAllMapPoints();

    for(size_t i=0; i<vpMPs.size(); i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        if(pMP->mnBAGlobalForKF==GBAid)
        {
            // 若该地图点直接参与了全局 BA，就直接使用优化结果。
            pMP->SetWorldPos(pMP->mPosGBA);
        }
        else
        {
            // 否则根据参考关键帧优化前后的位姿变化，把地图点一并搬到新坐标系下。
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

            if(pRefKF->mnBAGlobalForKF!=GBAid)
                continue;

            // 先把旧世界坐标下的点映射到“旧相机坐标系”。
            Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

            // 再通过参考关键帧的新位姿反投回新的世界坐标系。
            pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
        }
    }

    Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);

    mnKFs=vpKF.size();
    mIdxInit++;

    // 初始化结束后，队列中残留的新关键帧不再保留，直接清理掉。
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
    {
        (*lit)->SetBadFlag();
        delete *lit;
    }
    mlNewKeyFrames.clear();

    // 初始化成功后，恢复 Tracking 正常工作状态。
    mpTracker->mState=Tracking::OK;
    bInitializing = false;

    // 通知地图：结构已经有显著更新。
    mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex();

    return;
}

/**
 * @brief IMU 已初始化后的尺度细化
 *
 * 单目惯性在运行一段时间后，尺度仍可能存在轻微漂移。
 * 该函数会重新收集一段时间窗口内的关键帧，仅优化旋转与尺度，
 * 然后把修正结果再次作用到地图上。
 */
void LocalMapping::ScaleRefinement()
{
    // 如果正在重置，就不做尺度细化。
    if (mbResetRequested)
        return;

    // 从当前关键帧沿时间链向前回溯，收集一串时序关键帧。
    list<KeyFrame*> lpKF;
    KeyFrame* pKF = mpCurrentKeyFrame;
    while(pKF->mPrevKF)
    {
        lpKF.push_front(pKF);
        pKF = pKF->mPrevKF;
    }
    lpKF.push_front(pKF);
    vector<KeyFrame*> vpKF(lpKF.begin(),lpKF.end());

    while(CheckNewKeyFrames())
    {
        // 同样把队列中新到达的关键帧纳入优化窗口。
        ProcessNewKeyFrame();
        vpKF.push_back(mpCurrentKeyFrame);
        lpKF.push_back(mpCurrentKeyFrame);
    }

    const int N = vpKF.size();

    // 尺度细化固定重力初值为单位旋转，重点只修正尺度和少量姿态漂移。
    mRwg = Eigen::Matrix3d::Identity();
    mScale=1.0;

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

    // 不带偏置项的惯性优化，目标更聚焦于尺度细化。
    Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    if (mScale<1e-1) // 1e-1
    {
        // 结果异常则直接丢弃。
        cout << "scale too small" << endl;
        bInitializing=false;
        return;
    }
    
    Sophus::SO3d so3wg(mRwg);

    // 从这里开始真正修改地图，需要加锁。
    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    if ((fabs(mScale-1.f)>0.002)||!mbMonocular)
    {
        // 将 refined 的尺度和旋转再次应用到地图。
        Sophus::SE3f Tgw(mRwg.cast<float>().transpose(),Eigen::Vector3f::Zero());
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(Tgw,mScale,true);
        mpTracker->UpdateFrameIMU(mScale,mpCurrentKeyFrame->GetImuBias(),mpCurrentKeyFrame);
    }
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    // 清理细化过程中累积到队列中的关键帧。
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
    {
        (*lit)->SetBadFlag();
        delete *lit;
    }
    mlNewKeyFrames.clear();

    double t_inertial_only = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t0).count();

    // 通知地图已经被重新缩放，后续前端/后端都需要看到这个版本变化。
    mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex();

    return;
}



/**
 * @brief 查询当前是否正在执行初始化流程
 * @return true 表示正在初始化
 */
bool LocalMapping::IsInitializing()
{
    return bInitializing;
}


/**
 * @brief 获取当前关键帧时间戳
 * @return 当前关键帧时间戳；若不存在则返回 0
 */
double LocalMapping::GetCurrKFTime()
{

    if (mpCurrentKeyFrame)
    {
        return mpCurrentKeyFrame->mTimeStamp;
    }
    else
        return 0.0;
}

/**
 * @brief 获取当前关键帧对象
 * @return 当前关键帧指针
 */
KeyFrame* LocalMapping::GetCurrKF()
{
    return mpCurrentKeyFrame;
}

} //namespace ORB_SLAM
