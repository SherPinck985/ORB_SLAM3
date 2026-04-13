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


#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Atlas.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "Settings.h"

#include <mutex>


namespace ORB_SLAM3
{

class System;
class Tracking;
class LoopClosing;
class Atlas;

class LocalMapping
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Local Mapping 线程构造函数
     * @param pSys        ORB-SLAM3 系统对象
     * @param pAtlas      地图集对象，用于管理当前活动地图及多地图切换
     * @param bMonocular  是否为单目模式
     * @param bInertial   是否启用惯性传感器
     * @param _strSeqName 当前数据序列名称，主要用于调试和统计输出
     *
     * Local Mapping 是 ORB-SLAM3 中的局部建图线程。它从 Tracking 线程接收新关键帧，
     * 并持续执行如下任务：
     * 1. 将关键帧插入地图和共视图；
     * 2. 剔除质量差的新地图点；
     * 3. 与邻居关键帧三角化生成新的 MapPoint；
     * 4. 融合重复地图点；
     * 5. 执行局部 BA / 局部惯性 BA；
     * 6. 剔除冗余关键帧；
     * 7. 在惯性模式下完成 IMU 初始化与尺度细化。
     */
    LocalMapping(System* pSys, Atlas* pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName=std::string());

    /**
     * @brief 设置闭环线程指针
     * @param pLoopCloser 闭环检测与地图融合线程
     */
    void SetLoopCloser(LoopClosing* pLoopCloser);

    /**
     * @brief 设置跟踪线程指针
     * @param pTracker Tracking 线程对象
     */
    void SetTracker(Tracking* pTracker);

    /**
     * @brief Local Mapping 线程主循环
     *
     * 该函数会不断检查是否有新关键帧到来，并在安全时机执行局部建图、
     * 局部优化、关键帧剔除和惯性初始化等操作。
     */
    void Run();

    /**
     * @brief 插入一帧新的关键帧到待处理队列
     * @param pKF 新关键帧
     *
     * Tracking 线程在判定当前帧应成为关键帧后，会调用该接口将关键帧交给
     * Local Mapping。插入新关键帧后会同时中断当前 BA，使建图线程优先处理新数据。
     */
    void InsertKeyFrame(KeyFrame* pKF);

    /**
     * @brief 清空关键帧队列并逐帧处理
     *
     * 该接口会持续调用 ProcessNewKeyFrame，直到队列中没有待处理关键帧。
     */
    void EmptyQueue();

    /**
     * @brief 请求暂停 Local Mapping 线程
     *
     * 设置停止请求标志，并中断当前 BA，使线程在安全位置进入暂停状态。
     */
    void RequestStop();

    /**
     * @brief 请求重置 Local Mapping 状态
     *
     * 该重置会清空待处理关键帧和最近新增地图点，并复位惯性初始化相关状态。
     */
    void RequestReset();

    /**
     * @brief 请求仅重置指定活动地图对应的 Local Mapping 状态
     * @param pMap 待重置的活动地图
     */
    void RequestResetActiveMap(Map* pMap);

    /**
     * @brief 尝试让 Local Mapping 停止运行
     * @return 若成功进入停止状态则返回 true，否则返回 false
     */
    bool Stop();

    /**
     * @brief 释放停止状态，使线程继续运行
     */
    void Release();

    /**
     * @brief 判断 Local Mapping 当前是否已经停止
     * @return true 表示线程已处于停止状态
     */
    bool isStopped();

    /**
     * @brief 查询是否收到停止请求
     * @return true 表示外部已经请求该线程停止
     */
    bool stopRequested();

    /**
     * @brief 查询当前是否接受新关键帧
     * @return true 表示 Tracking 可以继续向 Local Mapping 插入关键帧
     */
    bool AcceptKeyFrames();

    /**
     * @brief 设置是否接受新关键帧
     * @param flag true 表示接受，false 表示暂不接受
     */
    void SetAcceptKeyFrames(bool flag);

    /**
     * @brief 设置“不可停止”标志
     * @param flag true 表示当前阶段不允许停止
     * @return 若设置成功返回 true；若线程已停止且要求置 true，则返回 false
     */
    bool SetNotStop(bool flag);

    /**
     * @brief 中断当前正在进行的 BA
     *
     * 该标志主要由 Tracking 插入新关键帧时触发，使局部建图线程优先处理最新关键帧。
     */
    void InterruptBA();

    /**
     * @brief 请求结束 Local Mapping 线程
     */
    void RequestFinish();

    /**
     * @brief 查询 Local Mapping 线程是否已经结束
     * @return true 表示线程已经退出
     */
    bool isFinished();

    /**
     * @brief 返回当前待处理关键帧队列中的关键帧数量
     * @return 队列中关键帧个数
     */
    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    /**
     * @brief 查询当前是否处于 IMU 初始化流程中
     * @return true 表示正在初始化惯性参数
     */
    bool IsInitializing();

    /**
     * @brief 获取当前正在处理的关键帧时间戳
     * @return 当前关键帧时间戳；若当前为空则返回 0
     */
    double GetCurrKFTime();

    /**
     * @brief 获取当前正在处理的关键帧指针
     * @return 当前关键帧
     */
    KeyFrame* GetCurrKF();

    /// IMU 初始化与尺度优化过程使用的互斥锁。
    std::mutex mMutexImuInit;

    /// 惯性优化后得到的协方差矩阵。
    Eigen::MatrixXd mcovInertial;
    /// 世界坐标系到重力坐标系的旋转估计。
    Eigen::Matrix3d mRwg;
    /// 陀螺仪零偏估计。
    Eigen::Vector3d mbg;
    /// 加速度计零偏估计。
    Eigen::Vector3d mba;
    /// 地图尺度估计，单目惯性初始化中特别关键。
    double mScale;
    /// 单次惯性初始化的时间跨度。
    double mInitTime;
    /// 记录初始化或优化过程中的耗时统计。
    double mCostTime;

    /// 初始化阶段编号。
    unsigned int mInitSect;
    /// 初始化迭代次数计数。
    unsigned int mIdxInit;
    /// 初始化时使用的关键帧数量统计。
    unsigned int mnKFs;
    /// 初始化窗口的起始时间戳。
    double mFirstTs;
    /// 当前与 Tracking 线程共享的内点匹配数统计。
    int mnMatchesInliers;

    /// 调试用：初始化阶段对应的帧编号。
    int mInitFr;
    /// 调试用：当前迭代编号。
    int mIdxIteration;
    /// 调试用：当前序列名称。
    string strSequence;

    /// 调试/流程控制：是否尚未执行第一次惯性 BA。
    bool mbNotBA1;
    /// 调试/流程控制：是否尚未执行第二次惯性 BA。
    bool mbNotBA2;
    /// 标记 IMU 初始化是否失败，失败时会阻止继续建图。
    bool mbBadImu;

    /// 是否输出统计信息。
    bool mbWriteStats;

    /// 是否忽略距离过远的点云点。
    bool mbFarPoints;
    /// 远点阈值，大于该距离的点在建图时可被直接忽略。
    float mThFarPoints;

#ifdef REGISTER_TIMES
    vector<double> vdKFInsert_ms;
    vector<double> vdMPCulling_ms;
    vector<double> vdMPCreation_ms;
    vector<double> vdLBA_ms;
    vector<double> vdKFCulling_ms;
    vector<double> vdLMTotal_ms;


    vector<double> vdLBASync_ms;
    vector<double> vdKFCullingSync_ms;
    vector<int> vnLBA_edges;
    vector<int> vnLBA_KFopt;
    vector<int> vnLBA_KFfixed;
    vector<int> vnLBA_MPs;
    int nLBA_exec;
    int nLBA_abort;
#endif
protected:

    /**
     * @brief 检查是否存在待处理的新关键帧
     * @return true 表示关键帧队列非空
     */
    bool CheckNewKeyFrames();

    /**
     * @brief 处理队列头部的新关键帧
     *
     * 主要完成：
     * 1. 从队列中取出关键帧；
     * 2. 计算 BoW；
     * 3. 更新关键帧与地图点之间的观测关系；
     * 4. 更新共视图连接；
     * 5. 将关键帧正式加入地图。
     */
    void ProcessNewKeyFrame();

    /**
     * @brief 通过当前关键帧和邻居关键帧三角化创建新的地图点
     */
    void CreateNewMapPoints();

    /**
     * @brief 剔除最近新增但质量较差的地图点
     */
    void MapPointCulling();

    /**
     * @brief 在邻居关键帧间搜索重复地图点并进行融合
     */
    void SearchInNeighbors();

    /**
     * @brief 剔除局部地图中的冗余关键帧
     */
    void KeyFrameCulling();

    /// 上层系统对象。
    System *mpSystem;

    /// 是否为单目模式。
    bool mbMonocular;
    /// 是否启用 IMU。
    bool mbInertial;

    /**
     * @brief 如果收到了重置请求，则在安全时机执行重置
     */
    void ResetIfRequested();
    /// 是否请求了整张地图重置。
    bool mbResetRequested;
    /// 是否请求了活动地图重置。
    bool mbResetRequestedActiveMap;
    /// 待重置的地图对象。
    Map* mpMapToReset;
    /// 保护重置相关状态的互斥锁。
    std::mutex mMutexReset;

    /**
     * @brief 查询是否请求结束线程
     * @return true 表示收到结束请求
     */
    bool CheckFinish();

    /**
     * @brief 将线程状态设置为“已结束”
     */
    void SetFinish();
    /// 是否收到结束请求。
    bool mbFinishRequested;
    /// 线程是否已经真正结束。
    bool mbFinished;
    /// 保护结束状态的互斥锁。
    std::mutex mMutexFinish;

    /// 地图集对象。
    Atlas* mpAtlas;

    /// 闭环线程。
    LoopClosing* mpLoopCloser;
    /// 跟踪线程。
    Tracking* mpTracker;

    /// 待处理关键帧队列。
    std::list<KeyFrame*> mlNewKeyFrames;

    /// 当前正在处理的关键帧。
    KeyFrame* mpCurrentKeyFrame;

    /// 最近新建地图点列表，用于后续质量筛除。
    std::list<MapPoint*> mlpRecentAddedMapPoints;

    /// 保护关键帧队列的互斥锁。
    std::mutex mMutexNewKFs;

    /// 是否中断当前 BA。
    bool mbAbortBA;

    /// 线程是否已停止。
    bool mbStopped;
    /// 是否收到停止请求。
    bool mbStopRequested;
    /// 是否暂时禁止停止。
    bool mbNotStop;
    /// 保护停止状态的互斥锁。
    std::mutex mMutexStop;

    /// 当前是否允许 Tracking 插入关键帧。
    bool mbAcceptKeyFrames;
    /// 保护“是否接受关键帧”标志的互斥锁。
    std::mutex mMutexAccept;

    /**
     * @brief 初始化 IMU、尺度和偏置
     * @param priorG 重力方向先验权重
     * @param priorA 加速度偏置先验权重
     * @param bFirst 是否在首次惯性 BA 初始化场景下调用
     */
    void InitializeIMU(float priorG = 1e2, float priorA = 1e6, bool bFirst = false);

    /**
     * @brief 在 IMU 已初始化后进一步细化尺度
     */
    void ScaleRefinement();

    /// 是否正在执行初始化流程。
    bool bInitializing;

    /// 惯性优化信息矩阵。
    Eigen::MatrixXd infoInertial;
    /// Local Mapping 执行次数统计。
    int mNumLM;
    /// 关键帧剔除次数统计。
    int mNumKFCulling;

    /// 惯性初始化累计时间。
    float mTinit;

    /// 尺度细化计数。
    int countRefinement;

    /// 调试输出文件流。
    ofstream f_lm;

    };

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
