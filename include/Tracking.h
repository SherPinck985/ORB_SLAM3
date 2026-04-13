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


#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "MapDrawer.h"
#include "System.h"
#include "ImuTypes.h"
#include "Settings.h"

#include "GeometricCamera.h"

#include <mutex>
#include <unordered_set>

namespace ORB_SLAM3
{

class Viewer;
class FrameDrawer;
class Atlas;
class LocalMapping;
class LoopClosing;
class System;
class Settings;

/**
 * @brief 跟踪线程核心类
 *
 * Tracking 是 ORB-SLAM3 前端的控制中心，负责：
 * 1. 接收不同传感器输入并转换成 Frame；
 * 2. 组织 ORB 特征提取、匹配、位姿估计与重定位；
 * 3. 在合适时机创建关键帧，并与局部地图线程协同；
 * 4. 在惯性模式下完成 IMU 预积分与状态预测。
 *
 * 需要注意：
 * Tracking 本身并不直接实现 ORB 提取算法。它只负责创建并持有
 * `ORBextractor`，真正的提取发生在 `Frame` 构造期间，由
 * `Frame::ExtractORB()` 调用 `ORBextractor::operator()` 完成。
 */
class Tracking
{  

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * @brief 构造跟踪器并加载相机、ORB、IMU 等配置
     * @param pSys            系统总控对象
     * @param pVoc            ORB 词袋字典
     * @param pFrameDrawer    当前帧绘制器
     * @param pMapDrawer      地图绘制器
     * @param pAtlas          多地图管理器
     * @param pKFDB           关键帧数据库
     * @param strSettingPath  配置文件路径
     * @param sensor          传感器类型，取值来自 `System` 枚举
     * @param settings        已解析的配置对象；若为空则自行读取配置文件
     * @param _nameSeq        数据序列名称
     *
     * 构造过程会根据传感器模式创建对应的 ORB 提取器：
     * 1. 普通左目/单目提取器 `mpORBextractorLeft`；
     * 2. 双目右目提取器 `mpORBextractorRight`；
     * 3. 单目初始化阶段使用的高密度提取器 `mpIniORBextractor`。
     */
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Atlas* pAtlas,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, Settings* settings, const string &_nameSeq=std::string());

    /**
     * @brief 析构函数
     *
     * 当前实现中析构逻辑较轻，主要依赖系统整体生命周期统一释放资源。
     */
    ~Tracking();

    /**
     * @brief 解析相机相关参数
     * @param fSettings OpenCV 配置文件句柄
     * @return          参数合法返回 true，否则返回 false
     *
     * 该函数会读取：
     * 1. 相机模型类型；
     * 2. 内参和畸变参数；
     * 3. 图像缩放比例；
     * 4. 双目重叠区域与外参；
     * 5. 深度或双目相关参数。
     */
    bool ParseCamParamFile(cv::FileStorage &fSettings);
    /**
     * @brief 解析 ORB 提取器参数并创建提取器实例
     * @param fSettings OpenCV 配置文件句柄
     * @return          参数合法返回 true，否则返回 false
     *
     * 这里读取的参数会直接决定 ORB 特征提取行为，包括：
     * `nFeatures`、`scaleFactor`、`nLevels`、`iniThFAST`、`minThFAST`。
     */
    bool ParseORBParamFile(cv::FileStorage &fSettings);
    /**
     * @brief 解析 IMU 相关参数并初始化预积分器
     * @param fSettings OpenCV 配置文件句柄
     * @return          参数合法返回 true，否则返回 false
     */
    bool ParseIMUParamFile(cv::FileStorage &fSettings);

    /**
     * @brief 处理双目输入并执行完整跟踪
     * @param imRectLeft   左目图像
     * @param imRectRight  右目图像
     * @param timestamp    时间戳
     * @param filename     当前图像文件名
     * @return             当前帧估计位姿 `Tcw`
     *
     * 流程包括：
     * 1. 彩色图转灰度；
     * 2. 构造 `Frame`，在构造过程中完成左右 ORB 特征提取和双目匹配；
     * 3. 调用 `Track()` 完成位姿估计、局部地图跟踪和关键帧判定。
     */
    Sophus::SE3f GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp, string filename);
    /**
     * @brief 处理 RGB-D 输入并执行完整跟踪
     * @param imRGB      彩色或灰度图
     * @param imD        深度图
     * @param timestamp  时间戳
     * @param filename   当前图像文件名
     * @return           当前帧估计位姿 `Tcw`
     */
    Sophus::SE3f GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, string filename);
    /**
     * @brief 处理单目输入并执行完整跟踪
     * @param im         输入图像
     * @param timestamp  时间戳
     * @param filename   当前图像文件名
     * @return           当前帧估计位姿 `Tcw`
     *
     * 单目模式下在初始化阶段会优先使用 `mpIniORBextractor`，
     * 其特征点数量通常是正常提取器的 5 倍，以提高初始化成功率。
     */
    Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename);

    /**
     * @brief 将一条 IMU 测量压入缓冲队列
     * @param imuMeasurement 一次 IMU 采样
     */
    void GrabImuData(const IMU::Point &imuMeasurement);

    /**
     * @brief 设置局部建图线程指针
     * @param pLocalMapper 局部建图线程对象
     */
    void SetLocalMapper(LocalMapping* pLocalMapper);
    /**
     * @brief 设置闭环线程指针
     * @param pLoopClosing 闭环线程对象
     */
    void SetLoopClosing(LoopClosing* pLoopClosing);
    /**
     * @brief 设置可视化线程指针
     * @param pViewer Viewer 对象
     */
    void SetViewer(Viewer* pViewer);
    /**
     * @brief 开启或关闭单步执行模式
     * @param bSet true 表示开启，false 表示关闭
     */
    void SetStepByStep(bool bSet);
    /**
     * @brief 获取当前是否处于单步执行模式
     * @return true 表示开启
     */
    bool GetStepByStep();

    /**
     * @brief 重新加载相机标定参数
     * @param strSettingPath 新配置文件路径
     *
     * 这里只更新相机内参、畸变和基线等与投影相关的量。
     * 焦距变化不能太大，否则已有的尺度预测模型会明显失效。
     */
    void ChangeCalibration(const string &strSettingPath);

    /**
     * @brief 设置是否只做定位而不插入关键帧
     * @param flag true 表示只跟踪不建图
     *
     * 常用于关闭局部建图线程后的纯定位模式。
     */
    void InformOnlyTracking(const bool &flag);

    /**
     * @brief 在 IMU 初始化或尺度更新后统一修正历史帧状态
     * @param s                 尺度缩放因子
     * @param b                 最新 IMU 偏置
     * @param pCurrentKeyFrame  当前关键帧
     */
    void UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame);
    /**
     * @brief 获取最近插入的关键帧
     * @return 最近关键帧指针
     */
    KeyFrame* GetLastKeyFrame()
    {
        return mpLastKeyFrame;
    }

    /**
     * @brief 在 Atlas 中新建一张地图并重置前端状态
     *
     * 该函数通常在跟踪丢失后、但系统不希望整体清空时调用。
     */
    void CreateMapInAtlas();
    //std::mutex mMutexTracks;

    /**
     * @brief 切换到新的数据序列编号
     */
    void NewDataset();
    /**
     * @brief 获取当前数据序列编号
     * @return 序列编号
     */
    int GetNumberDataset();
    /**
     * @brief 获取当前帧的内点匹配数
     * @return 内点数
     */
    int GetMatchesInliers();

    /**
     * @brief 保存当前子轨迹
     * @param strNameFile_frames 帧轨迹输出文件名
     * @param strNameFile_kf     关键帧轨迹输出文件名
     * @param strFolder          输出目录
     */
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder="");
    /**
     * @brief 保存指定地图对应的子轨迹
     * @param strNameFile_frames 帧轨迹输出文件名
     * @param strNameFile_kf     关键帧轨迹输出文件名
     * @param pMap               指定地图
     */
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map* pMap);

    /**
     * @brief 获取图像缩放比例
     * @return 图像缩放因子
     */
    float GetImageScale();

#ifdef REGISTER_LOOP
    /**
     * @brief 请求暂停 Tracking 线程
     */
    void RequestStop();
    /**
     * @brief 判断 Tracking 是否已停止
     * @return true 表示已停止
     */
    bool isStopped();
    /**
     * @brief 释放暂停状态，使 Tracking 线程继续运行
     */
    void Release();
    /**
     * @brief 查询是否收到停止请求
     * @return true 表示收到请求
     */
    bool stopRequested();
#endif

public:

    /**
     * @brief 跟踪线程状态机
     */
    enum eTrackingState{
        /// 系统还没准备好，通常出现在初始化最早期。
        SYSTEM_NOT_READY=-1,
        /// 还没有读入任何图像。
        NO_IMAGES_YET=0,
        /// 已有图像，但地图尚未初始化。
        NOT_INITIALIZED=1,
        /// 跟踪正常。
        OK=2,
        /// 刚刚丢失，仍尝试短时间恢复。
        RECENTLY_LOST=3,
        /// 跟踪彻底丢失。
        LOST=4,
        /// 基于 KLT 的特殊状态，当前版本 Tracking 主线很少使用。
        OK_KLT=5
    };

    /// 当前跟踪状态。
    eTrackingState mState;
    /// 上一帧处理完成后的状态，用于 UI 和日志展示。
    eTrackingState mLastProcessedState;

    /// 输入传感器类型。
    int mSensor;

    /// 当前正在处理的帧。
    Frame mCurrentFrame;
    /// 上一帧。
    Frame mLastFrame;

    /// 当前输入的灰度图像。
    cv::Mat mImGray;

    /// 单目初始化阶段中上一帧与当前帧的匹配索引。
    std::vector<int> mvIniLastMatches;
    /// 单目初始化阶段的匹配关系。
    std::vector<int> mvIniMatches;
    /// 单目初始化时上一帧关键点的二维坐标。
    std::vector<cv::Point2f> mvbPrevMatched;
    /// 单目初始化三角化得到的 3D 点。
    std::vector<cv::Point3f> mvIniP3D;
    /// 单目初始化参考帧。
    Frame mInitialFrame;

    /**
     * @brief 用于在程序结束后恢复完整相机轨迹的缓存
     *
     * 核心思想是：
     * 每一帧不直接长期保存绝对位姿，而是保存它相对参考关键帧的位姿，
     * 最后再结合关键帧轨迹恢复全局轨迹。
     */
    list<Sophus::SE3f> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    /// 已经成功完成局部地图跟踪的帧数统计。
    int mTrackedFr;
    /// 单步模式下，由外部触发的“继续执行一步”标志。
    bool mbStep;

    /// 为 true 时表示关闭局部建图，仅执行定位。
    bool mbOnlyTracking;

    /**
     * @brief 全局重置系统状态
     * @param bLocMap 是否只跳过局部建图线程的 reset
     */
    void Reset(bool bLocMap = false);
    /**
     * @brief 仅重置当前活动地图
     * @param bLocMap 是否只跳过局部建图线程的 reset
     */
    void ResetActiveMap(bool bLocMap = false);

    float mMeanTrack;
    bool mbInitWith3KFs;
    /// 第一帧图像时间戳。
    double t0;
    /// 第一张被插入地图的关键帧时间戳。
    double t0vis;
    /// IMU 初始化完成的时间戳。
    double t0IMU;
    /// 是否使用快速 IMU 初始化策略。
    bool mFastInit = false;


    /**
     * @brief 获取当前局部地图点集合
     * @return 局部地图点数组
     */
    vector<MapPoint*> GetLocalMapMPS();

    /// 是否将统计信息写入文件。
    bool mbWriteStats;

#ifdef REGISTER_TIMES
    /**
     * @brief 将局部建图耗时统计写入文件
     */
    void LocalMapStats2File();
    /**
     * @brief 将跟踪耗时统计写入文件
     */
    void TrackStats2File();
    /**
     * @brief 打印整体耗时统计
     */
    void PrintTimeStats();

    vector<double> vdRectStereo_ms;
    vector<double> vdResizeImage_ms;
    vector<double> vdORBExtract_ms;
    vector<double> vdStereoMatch_ms;
    vector<double> vdIMUInteg_ms;
    vector<double> vdPosePred_ms;
    vector<double> vdLMTrack_ms;
    vector<double> vdNewKF_ms;
    vector<double> vdTrackTotal_ms;
#endif

protected:

    /**
     * @brief 跟踪线程主流程
     *
     * 该函数和输入传感器解耦，内部统一处理：
     * 1. 初始化；
     * 2. 位姿预测；
     * 3. 参考关键帧或运动模型跟踪；
     * 4. 局部地图跟踪；
     * 5. 关键帧插入与状态维护。
     */
    void Track();

    /**
     * @brief 双目/RGB-D 初始化
     *
     * 依赖深度信息直接创建首个关键帧和初始地图点。
     */
    void StereoInitialization();

    /**
     * @brief 单目初始化
     *
     * 通过两帧间特征匹配、两视图重建和三角化完成初始地图构建。
     */
    void MonocularInitialization();
    //void CreateNewMapPoints();
    /**
     * @brief 根据单目初始化结果真正创建初始地图
     */
    void CreateInitialMapMonocular();

    /**
     * @brief 检查上一帧中的地图点是否已被替换
     */
    void CheckReplacedInLastFrame();
    /**
     * @brief 使用参考关键帧进行跟踪
     * @return 跟踪成功返回 true
     */
    bool TrackReferenceKeyFrame();
    /**
     * @brief 更新上一帧位姿并在纯定位模式下生成临时 VO 点
     */
    void UpdateLastFrame();
    /**
     * @brief 使用运动模型进行跟踪
     * @return 跟踪成功返回 true
     */
    bool TrackWithMotionModel();
    /**
     * @brief 使用 IMU 预积分预测当前状态
     * @return 预测成功返回 true
     */
    bool PredictStateIMU();

    /**
     * @brief 当普通跟踪失败时执行重定位
     * @return 重定位成功返回 true
     */
    bool Relocalization();

    /**
     * @brief 更新局部地图，包括局部关键帧和局部地图点
     */
    void UpdateLocalMap();
    /**
     * @brief 根据局部关键帧集合更新局部地图点集合
     */
    void UpdateLocalPoints();
    /**
     * @brief 根据当前帧已关联的地图点投票选出局部关键帧
     */
    void UpdateLocalKeyFrames();

    /**
     * @brief 跟踪局部地图
     * @return 跟踪成功返回 true
     */
    bool TrackLocalMap();
    /**
     * @brief 将局部地图点重新投影到当前帧并搜索匹配
     */
    void SearchLocalPoints();

    /**
     * @brief 判断当前帧是否需要提升为关键帧
     * @return 需要插入关键帧返回 true
     */
    bool NeedNewKeyFrame();
    /**
     * @brief 创建并插入新的关键帧
     */
    void CreateNewKeyFrame();

    /**
     * @brief 对上一帧到当前帧之间的 IMU 数据做预积分
     */
    void PreintegrateIMU();

    /**
     * @brief 在 IMU 重置时刷新偏置与帧速度
     */
    void ResetFrameIMU();

    /// 标记局部地图是否在当前帧前发生过更新。
    bool mbMapUpdated;

    /// 从上一关键帧累计到当前时刻的 IMU 预积分对象。
    IMU::Preintegrated *mpImuPreintegratedFromLastKF;

    /// 帧间 IMU 测量队列。
    std::list<IMU::Point> mlQueueImuData;

    /// 当前帧实际参与预积分的 IMU 测量序列。
    std::vector<IMU::Point> mvImuFromLastFrame;
    std::mutex mMutexImuQueue;

    /// IMU 标定参数与噪声模型。
    IMU::Calib *mpImuCalib;

    /// 最近一次关键帧处估计得到的 IMU 偏置。
    IMU::Bias mLastBias;

    /**
     * @brief 纯定位模式下是否退化为 VO 状态
     *
     * 为 true 表示当前主要依靠临时视觉里程计点在跟踪，
     * 而不是依赖已有地图点，因此系统会持续尝试重定位回地图。
     */
    bool mbVO;

    /// 局部建图线程。
    LocalMapping* mpLocalMapper;
    /// 闭环线程。
    LoopClosing* mpLoopClosing;

    /**
     * @brief ORB 特征提取器
     *
     * - `mpORBextractorLeft`：左目或单目正常跟踪使用；
     * - `mpORBextractorRight`：右目使用，仅双目模式存在；
     * - `mpIniORBextractor`：单目初始化阶段使用，特征点更密集。
     */
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    /// ORB 词袋字典，用于 BoW 向量构建和重定位。
    ORBVocabulary* mpORBVocabulary;
    /// 关键帧数据库，用于回环与重定位候选检索。
    KeyFrameDatabase* mpKeyFrameDB;

    /// 单目初始化是否已经具备可用参考帧。
    bool mbReadyToInitializate;
    /// 是否已经设置过初始化相关状态。
    bool mbSetInit;

    /// 当前参考关键帧。
    KeyFrame* mpReferenceKF;
    /// 当前局部关键帧集合。
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    /// 当前局部地图点集合。
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    /// 系统主控对象。
    System* mpSystem;
    
    /// 可视化线程。
    Viewer* mpViewer;
    /// 当前帧绘制器。
    FrameDrawer* mpFrameDrawer;
    /// 地图绘制器。
    MapDrawer* mpMapDrawer;
    /// 是否启用单步调试模式。
    bool bStepByStep;

    /// Atlas 多地图管理器。
    Atlas* mpAtlas;

    /// OpenCV 格式相机内参矩阵。
    cv::Mat mK;
    /// Eigen 格式相机内参矩阵。
    Eigen::Matrix3f mK_;
    /// 畸变系数。
    cv::Mat mDistCoef;
    /// 基线乘焦距 `bf`。
    float mbf;
    /// 图像缩放系数。
    float mImageScale;

    /// IMU 频率。
    float mImuFreq;
    /// IMU 采样周期，当前实现中按毫秒量级近似。
    double mImuPer;
    /// 跟踪丢失时是否允许继续插入关键帧。
    bool mInsertKFsLost;

    /// 关键帧插入最小间隔帧数。
    int mMinFrames;
    /// 关键帧插入最大间隔帧数，通常与 fps 对应。
    int mMaxFrames;

    /// IMU 初始化后第一帧的编号。
    int mnFirstImuFrameId;
    /// 重定位后需要重置 IMU 的帧数窗口。
    int mnFramesToResetIMU;

    /**
     * @brief 近点/远点深度阈值
     *
     * 双目或 RGB-D 中，深度小于该阈值的点被视为近点，可靠性更高，
     * 可以仅凭单帧观测就插入为地图点；远点则往往需要更多视角验证。
     */
    float mThDepth;

    /// RGB-D 深度图缩放因子，用于把原始深度值转换为米。
    float mDepthMapFactor;

    /// 当前帧经优化后保留下来的内点匹配数量。
    int mnMatchesInliers;

    /// 最近关键帧。
    KeyFrame* mpLastKeyFrame;
    /// 最近关键帧对应的帧编号。
    unsigned int mnLastKeyFrameId;
    /// 最近一次重定位成功时的帧编号。
    unsigned int mnLastRelocFrameId;
    /// 跟踪丢失时刻的时间戳。
    double mTimeStampLost;
    /// RECENTLY_LOST 状态允许维持的最长时间。
    double time_recently_lost;

    /// 当前地图首帧编号。
    unsigned int mnFirstFrameId;
    /// 最近一次地图初始化起始帧编号。
    unsigned int mnInitialFrameId;
    /// 最近一次重新建图时的初始化前一帧编号。
    unsigned int mnLastInitFrameId;

    /// 本轮是否刚刚新建过地图。
    bool mbCreatedMap;

    /// 是否已有可用恒速运动模型。
    bool mbVelocity{false};
    /// 当前保存的恒速位姿增量模型。
    Sophus::SE3f mVelocity;

    /// 彩色图通道顺序，true 表示 RGB，false 表示 BGR。
    bool mbRGB;

    /// 纯 VO 模式下临时创建、待后续释放的地图点。
    list<MapPoint*> mlpTemporalPoints;

    //int nMapChangeIndex;

    /// 当前数据集编号。
    int mnNumDataset;

    ofstream f_track_stats;

    ofstream f_track_times;
    double mTime_PreIntIMU;
    double mTime_PosePred;
    double mTime_LocalMapTrack;
    double mTime_NewKF_Dec;

    /// 主相机与副相机模型。
    GeometricCamera* mpCamera, *mpCamera2;

    /// 单目初始化期间起始帧与最近帧编号。
    int initID, lastID;

    /// 右相机到左相机的外参变换。
    Sophus::SE3f mTlr;

    /**
     * @brief 从 `Settings` 对象直接加载参数
     * @param settings 已经完成解析的配置对象
     */
    void newParameterLoader(Settings* settings);

#ifdef REGISTER_LOOP
    /**
     * @brief 真正执行停止逻辑
     * @return 若本次调用进入停止状态则返回 true
     */
    bool Stop();

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;
#endif

public:
    /// 当前右目原始图像，仅双目模式有意义。
    cv::Mat mImRight;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
