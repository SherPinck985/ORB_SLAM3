# 1. ORB-SLAM3 总体调用链

你当前双目惯性模式的主调用链可以概括成：

```text
rosbag play
    ↓
/camera/left/image_raw
/camera/right/image_raw
/imu
    ↓
ros_stereo_inertial.cc
    ↓
ImageGrabber::SyncWithImu()
    ↓
ORB_SLAM3::System::TrackStereo()
    ↓
Tracking::GrabImuData()
Tracking::GrabImageStereo()
    ↓
Frame 构造
    ├── ORBextractor 提取左右目 ORB
    ├── Frame::ComputeStereoMatches() 做双目匹配
    └── 得到深度 mvDepth
    ↓
Tracking::Track()
    ├── IMU 预积分
    ├── 初始化 / 正常跟踪 / 重定位
    ├── ORBmatcher 匹配地图点
    ├── Optimizer 位姿优化
    ├── TrackLocalMap()
    └── NeedNewKeyFrame()
            ↓
        CreateNewKeyFrame()
            ↓
        LocalMapping::InsertKeyFrame()
            ↓
        LocalMapping 线程
            ├── ProcessNewKeyFrame()
            ├── MapPointCulling()
            ├── CreateNewMapPoints()
            ├── Local BA / Local Inertial BA
            ├── IMU 初始化
            └── LoopClosing::InsertKeyFrame()
                    ↓
                LoopClosing 线程
                    ├── BoW 回环检测
                    ├── Sim3 / 4DoF 几何验证
                    ├── CorrectLoop()
                    ├── MergeLocal()
                    └── Global BA
```

---

# 2. `System.cc`：ORB-SLAM3 的总控中心

你在 `ros_stereo_inertial.cc` 中创建了：

```cpp
ORB_SLAM3::System SLAM(
    argv[1],
    argv[2],
    ORB_SLAM3::System::IMU_STEREO,
    true
);
```

也就是：

```text
词袋文件 ORBvoc.txt
配置文件 EuRoC.yaml
传感器模式 IMU_STEREO
启用 Viewer
```

---

## 2.1 `System::System()` 做了什么？

它是整个工程的初始化总入口，主要创建这些模块：

```cpp
mpVocabulary = new ORBVocabulary();
mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
mpAtlas = new Atlas(0);

mpFrameDrawer = new FrameDrawer(mpAtlas);
mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_);

mpTracker = new Tracking(...);
mpLocalMapper = new LocalMapping(...);
mpLoopCloser = new LoopClosing(...);
mpViewer = new Viewer(...);
```

线程关系：

```text
Tracking
    运行在调用 TrackStereo() 的线程里，也就是你的 SyncWithImu 线程

LocalMapping
    单独线程 mptLocalMapping

LoopClosing
    单独线程 mptLoopClosing

Viewer
    单独线程 mptViewer
```

也就是说，ORB-SLAM3 不是单线程结构，而是典型的多线程 SLAM：

```text
前端 Tracking 线程
后端 LocalMapping 线程
回环 LoopClosing 线程
显示 Viewer 线程
```

---

## 2.2 System 内部互相设置指针

`System.cc` 里有这几句：

```cpp
mpTracker->SetLocalMapper(mpLocalMapper);
mpTracker->SetLoopClosing(mpLoopCloser);

mpLocalMapper->SetTracker(mpTracker);
mpLocalMapper->SetLoopCloser(mpLoopCloser);

mpLoopCloser->SetTracker(mpTracker);
mpLoopCloser->SetLocalMapper(mpLocalMapper);
```

这非常关键，说明三大线程之间是互相通信的：

```text
Tracking 需要通知 LocalMapping 插入关键帧
Tracking 需要知道 LoopClosing 是否影响地图

LocalMapping 需要通知 LoopClosing 检查新关键帧
LocalMapping 需要知道 Tracking 的状态

LoopClosing 需要暂停 LocalMapping
LoopClosing 需要通知 Tracking 地图发生改变
```

---

# 3. `System::TrackStereo()`：从 ROS 进入 SLAM 的第一站

`ros_stereo_inertial.cc` 调用的是：

```cpp
System::TrackStereo(imLeft, imRight, timestamp, vImuMeas)
```

它做的事情很清楚：

## 3.1 检查传感器模式

```cpp
if(mSensor != STEREO && mSensor != IMU_STEREO)
{
    error
}
```

你的模式是：

```cpp
IMU_STEREO
```

所以合法。

---

## 3.2 可选图像矫正 / resize

```cpp
if(settings_ && settings_->needToRectify())
{
    cv::remap(...)
}
else if(settings_ && settings_->needToResize())
{
    cv::resize(...)
}
else
{
    clone
}
```

注意，这里和 `ros_stereo_inertial.cc` 里面的 `do_rectify` 有潜在重复关系：

```text
ros_stereo_inertial.cc 里可以做一次 remap
System::TrackStereo() 里也可能根据 Settings 再做一次 remap
```

所以复现时要特别检查：

```text
不要出现“不矫正”或“双重矫正”。
```

---

## 3.3 把 IMU 数据交给 Tracking

```cpp
if (mSensor == System::IMU_STEREO)
    for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
        mpTracker->GrabImuData(vImuMeas[i_imu]);
```

这里的 `GrabImuData()` 会把 IMU 数据放进 Tracking 自己的队列：

```cpp
mlQueueImuData.push_back(imuMeasurement);
```

---

## 3.4 把双目图像交给 Tracking

```cpp
Sophus::SE3f Tcw =
    mpTracker->GrabImageStereo(imLeftToFeed, imRightToFeed, timestamp, filename);
```

从这里进入前端跟踪主体。

---

# 4. `Tracking.cc`：前端跟踪主状态机

`Tracking` 是 ORB-SLAM3 的核心前端。你这套双目惯性系统里，它负责：

```text
读取相机/IMU/ORB 参数
构造当前 Frame
ORB 特征提取
双目匹配得到深度
IMU 预积分
视觉/惯性联合位姿估计
局部地图跟踪
判断是否插入关键帧
丢失后的重定位或新建地图
```

---

## 4.1 `Tracking::GrabImageStereo()`

`System::TrackStereo()` 会调用它。

核心流程是：

```cpp
mImGray = imRectLeft;
cv::Mat imGrayRight = imRectRight;
mImRight = imRectRight;
```

然后构造当前帧：

```cpp
mCurrentFrame = Frame(
    mImGray,
    imGrayRight,
    timestamp,
    mpORBextractorLeft,
    mpORBextractorRight,
    mpORBVocabulary,
    mK,
    mDistCoef,
    mbf,
    mThDepth,
    mpCamera,
    &mLastFrame,
    *mpImuCalib
);
```

对于你的 `IMU_STEREO` 模式，Frame 构造时会拿到：

```text
左图
右图
时间戳
左 ORB 提取器
右 ORB 提取器
词袋
相机内参
畸变参数
bf
深度阈值
相机模型
上一帧
IMU 标定
```

然后调用：

```cpp
Track();
```

---

## 4.2 `Tracking::Track()` 是前端状态机

它有几个主要状态：

```text
NO_IMAGES_YET
NOT_INITIALIZED
OK
RECENTLY_LOST
LOST
```

大致逻辑是：

```text
如果还没初始化：
    双目/深度模式走 StereoInitialization()

如果已经初始化：
    先做 IMU 预积分
    再用运动模型、IMU 或参考关键帧预测当前位姿
    匹配地图点
    优化当前帧位姿
    跟踪局部地图
    判断是否插入关键帧

如果跟踪失败：
    短时失败 → RECENTLY_LOST
    长时间失败 → LOST
    小地图直接 reset
    大地图则在 Atlas 中创建新地图
```

---

# 5. `Frame.cc`：当前帧的数据容器，也是特征提取入口

在双目惯性模式中，`Tracking::GrabImageStereo()` 会创建一个 `Frame`。

`Frame` 构造函数中最重要的事情是：

## 5.1 左右目并行提取 ORB

```cpp
thread threadLeft(&Frame::ExtractORB, this, 0, imLeft, 0, 0);
thread threadRight(&Frame::ExtractORB, this, 1, imRight, 0, 0);
threadLeft.join();
threadRight.join();
```

所以左右目特征提取是并行的。

---

## 5.2 `Frame::ExtractORB()`

```cpp
if(flag == 0)
    monoLeft = (*mpORBextractorLeft)(im, ..., mvKeys, mDescriptors, ...);
else
    monoRight = (*mpORBextractorRight)(im, ..., mvKeysRight, mDescriptorsRight, ...);
```

实际提取算法在：

```text
ORBextractor.cc
```

---

## 5.3 `Frame::ComputeStereoMatches()`

双目 ORB-SLAM3 和单目最大的区别之一就是这里。

它会根据左右图 ORB 描述子和极线约束做匹配，然后计算：

```cpp
mvuRight[i]
mvDepth[i]
```

其中：

```text
mvuRight[i]：左目第 i 个特征在右目中的横坐标
mvDepth[i]：由视差恢复出来的深度
```

双目深度基本关系是：

```text
depth = bf / disparity
```

这一步使得双目初始化比单目简单很多，因为很多点可以直接三角化成 3D。

---

# 6. `ORBextractor.cc`：ORB 特征提取器

`Frame::ExtractORB()` 最终调用的是：

```cpp
ORBextractor::operator()
```

主要流程是：

```text
输入灰度图
    ↓
ComputePyramid()
构建图像金字塔
    ↓
ComputeKeyPointsOctTree()
每层 FAST 角点检测 + 四叉树均匀化
    ↓
计算角点方向
    ↓
computeOrbDescriptor()
计算 ORB 二进制描述子
    ↓
输出 keypoints + descriptors
```

所以 ORBextractor 只关心：

```text
图像 → ORB 特征点 + ORB 描述子
```

它不关心地图、不关心位姿、不关心 IMU。

---

# 7. `ImuTypes.cc`：IMU 预积分的数据结构和数学实现

你 ROS 节点中收集到的 IMU 数据会变成：

```cpp
ORB_SLAM3::IMU::Point(acc, gyr, t)
```

然后在 Tracking 中进入：

```cpp
Tracking::PreintegrateIMU()
```

里面会创建：

```cpp
IMU::Preintegrated* pImuPreintegratedFromLastFrame
```

并反复调用：

```cpp
IntegrateNewMeasurement(acc, angVel, dt)
```

---

## 7.1 预积分维护什么？

`Preintegrated` 主要维护：

```text
dR：两帧之间的旋转增量
dV：两帧之间的速度增量
dP：两帧之间的位置增量
dT：累计时间
JRg/JVg/JVa/JPg/JPa：对 IMU bias 的雅可比
C / Info：协方差与信息矩阵
```

这部分是视觉惯性 SLAM 的核心，后续优化器会用这些预积分约束来联合估计：

```text
位姿
速度
陀螺仪 bias
加速度计 bias
重力方向
尺度
```

双目惯性有真实尺度，所以比单目惯性更稳定。

---

# 8. `Tracking::PreintegrateIMU()`：真正把 IMU 接入前端

`System::TrackStereo()` 只是把 IMU 数据塞进 Tracking 队列。

真正做预积分的是：

```cpp
Tracking::PreintegrateIMU()
```

它会从：

```cpp
mlQueueImuData
```

中取出时间上属于：

```text
上一帧时间戳 → 当前帧时间戳
```

之间的 IMU 数据，然后做中值积分。

同时维护两个预积分对象：

```text
1. 从上一普通帧到当前帧的预积分
2. 从上一关键帧到当前帧的预积分
```

原因是：

```text
普通跟踪时，需要上一帧 → 当前帧的预测
局部 BA / 关键帧约束时，需要上一关键帧 → 当前帧/当前关键帧的约束
```

---

# 9. `Tracking::PredictStateIMU()`：用 IMU 预测当前帧状态

当 IMU 已初始化后，Tracking 会优先用 IMU 来预测当前帧的：

```text
Rwb
twb
Vwb
```

也就是：

```text
IMU 在世界坐标系下的旋转
IMU 在世界坐标系下的位置
IMU 在世界坐标系下的速度
```

如果地图刚刚被 LocalMapping / LoopClosing 更新过，就从最近关键帧传播。

如果地图没更新，就从上一普通帧传播。

---

# 10. `ORBmatcher.cc`：所有匹配基本都在这里

ORBmatcher 是视觉前端和后端都频繁调用的工具类。

在你的双目惯性链路中，它主要用于：

```text
TrackReferenceKeyFrame()
    └── SearchByBoW()

TrackWithMotionModel()
    └── SearchByProjection()

TrackLocalMap()
    └── SearchByProjection()

LocalMapping::CreateNewMapPoints()
    └── SearchForTriangulation()

LocalMapping::SearchInNeighbors()
    └── Fuse()

LoopClosing
    ├── SearchByBoW()
    ├── SearchBySim3()
    └── Fuse()
```

它解决的核心问题是：

```text
当前帧特征点应该匹配哪个地图点？
两个关键帧之间哪些特征对应？
两个地图中哪些点是重复点？
回环候选和当前关键帧之间是否有足够几何一致匹配？
```

---

# 11. `Optimizer.cc`：所有优化都在这里

ORB-SLAM3 的优化非常集中，基本都在 `Optimizer.cc` 里。

常见调用关系如下：

```text
Tracking::TrackReferenceKeyFrame()
    └── Optimizer::PoseOptimization()

Tracking::TrackWithMotionModel()
    └── Optimizer::PoseOptimization()

Tracking::TrackLocalMap()
    ├── PoseOptimization()
    ├── PoseInertialOptimizationLastFrame()
    └── PoseInertialOptimizationLastKeyFrame()

LocalMapping::Run()
    ├── LocalBundleAdjustment()
    └── LocalInertialBA()

LocalMapping::InitializeIMU()
    └── InertialOptimization()

LoopClosing::CorrectLoop()
    ├── OptimizeEssentialGraph()
    └── GlobalBundleAdjustemnt()

LoopClosing::MergeLocal2()
    └── MergeInertialBA()
```

可以简单理解为：

```text
Tracking 中的优化：
    主要优化当前帧位姿

LocalMapping 中的优化：
    优化局部关键帧 + 局部地图点 + IMU 状态

LoopClosing 中的优化：
    优化整张图的位姿图 / Essential Graph / 全局 BA
```

---

# 12. `LocalMapping.cc`：局部建图线程

Tracking 如果认为当前帧值得成为关键帧，会调用：

```cpp
CreateNewKeyFrame()
```

里面会执行：

```cpp
mpLocalMapper->InsertKeyFrame(pKF);
```

于是关键帧被放进 LocalMapping 队列。

---

## 12.1 `LocalMapping::Run()` 主流程

LocalMapping 线程循环运行，核心流程是：

```text
CheckNewKeyFrames()
    ↓
ProcessNewKeyFrame()
    ↓
MapPointCulling()
    ↓
CreateNewMapPoints()
    ↓
SearchInNeighbors()
    ↓
Local BA / Local Inertial BA
    ↓
InitializeIMU()
    ↓
KeyFrameCulling()
    ↓
LoopClosing::InsertKeyFrame()
```

---

## 12.2 `ProcessNewKeyFrame()`

它会：

```text
取出一个新关键帧
计算 BoW
补全 MapPoint 与 KeyFrame 的观测关系
更新共视连接
把 KeyFrame 加入 Atlas / Map
```

关键代码逻辑是：

```cpp
mpCurrentKeyFrame->ComputeBoW();
pMP->AddObservation(mpCurrentKeyFrame, i);
pMP->UpdateNormalAndDepth();
pMP->ComputeDistinctiveDescriptors();
mpCurrentKeyFrame->UpdateConnections();
mpAtlas->AddKeyFrame(mpCurrentKeyFrame);
```

也就是说，Tracking 创建关键帧只是开始，LocalMapping 才真正把这个关键帧纳入地图结构。

---

## 12.3 `CreateNewMapPoints()`

这个函数通过当前关键帧和相邻关键帧之间的匹配来三角化新点。

双目模式中，Tracking 已经可以通过当前帧深度直接创建一部分新点。

LocalMapping 则进一步利用多关键帧几何关系扩展地图。

---

## 12.4 IMU 初始化也在 LocalMapping 中完成

你跑的是 `IMU_STEREO`，所以 LocalMapping 中会在合适时机调用：

```cpp
InitializeIMU(...)
```

IMU 初始化主要估计：

```text
重力方向
速度
陀螺仪 bias
加速度计 bias
尺度相关量
```

双目本身有尺度，因此相比单目惯性更容易稳定。

---

# 13. `LoopClosing.cc`：回环检测、地图合并和全局一致性

LocalMapping 处理完关键帧后，会调用：

```cpp
mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
```

然后 LoopClosing 线程会检测：

```text
这个新关键帧是否回到了以前经过的地方？
这个新关键帧是否可以和另一张地图合并？
```

---

## 13.1 `LoopClosing::Run()`

主流程是：

```text
CheckNewKeyFrames()
    ↓
NewDetectCommonRegions()
    ↓
如果发现地图合并：
    MergeLocal() / MergeLocal2()

如果发现回环：
    CorrectLoop()
```

在惯性模式下有额外检查：

```text
尺度修正不能离谱
重力方向不能被错误改变
IMU 稳定后通常只允许 yaw 方向修正
```

这也是 ORB-SLAM3 相比 ORB-SLAM2 的一个重要变化。

---

# 14. `Atlas.cc` 和 `Map.cc`：多地图管理

ORB-SLAM3 的核心设计之一是 Atlas。

ORB-SLAM2 基本是一张地图，而 ORB-SLAM3 可以维护多张地图。

---

## 14.1 `Atlas`

`Atlas` 管理多个 `Map`：

```text
Atlas
    ├── 当前活动地图 mpCurrentMap
    ├── 所有正常地图 mspMaps
    └── 坏地图 mspBadMaps
```

如果 Tracking 丢失，并且当前地图已经比较大，它可能不会直接 reset，而是：

```cpp
CreateMapInAtlas();
```

然后 Atlas 创建新地图：

```cpp
Atlas::CreateNewMap()
```

所以 ORB-SLAM3 可以：

```text
丢失后新建地图
之后通过 LoopClosing 做地图合并
```

---

## 14.2 `Map`

`Map` 管理的是具体一张地图内部的：

```text
KeyFrame 集合
MapPoint 集合
IMU 初始化状态
地图更新编号
第一关键帧
最大关键帧 ID
```

例如：

```cpp
Map::AddKeyFrame()
Map::AddMapPoint()
Map::EraseMapPoint()
Map::EraseKeyFrame()
Map::SetImuInitialized()
```

---

# 15. `KeyFrame.cc`：关键帧节点

`KeyFrame` 是图优化中的核心节点。

它从 `Frame` 转换而来：

```cpp
KeyFrame* pKF = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);
```

它保存：

```text
ORB 特征点
ORB 描述子
BoW 向量
相机位姿
IMU 位姿、速度、bias
关联 MapPoint
共视图连接
生成树关系
回环边
地图指针
```

---

## 15.1 `KeyFrame::ComputeBoW()`

用于：

```text
重定位
回环检测
地点识别
BoW 匹配
```

---

## 15.2 `KeyFrame::UpdateConnections()`

它根据共享 MapPoint 的数量建立共视图连接。

简单说：

```text
两个关键帧看到的共同地图点越多，它们之间的边权越大。
```

共视图后续会被用于：

```text
局部地图跟踪
局部 BA
关键帧剔除
回环检测
地图合并
```

---

# 16. `MapPoint.cc`：地图点

`MapPoint` 是三维地图点。

它保存：

```text
世界坐标
观测到它的关键帧
代表性描述子
平均观测方向
可观测距离范围
被看到次数
成功匹配次数
是否为坏点
是否被其他点替换
```

核心函数：

```cpp
AddObservation()
EraseObservation()
ComputeDistinctiveDescriptors()
UpdateNormalAndDepth()
Replace()
SetBadFlag()
```

LocalMapping 和 LoopClosing 经常会融合重复点，调用：

```cpp
MapPoint::Replace()
```

也就是：

```text
把一个重复地图点的观测关系转移给另一个地图点。
```

---

# 17. `KeyFrameDatabase.cc`：关键帧倒排索引

它是基于 ORB 词袋的数据库。

结构类似：

```text
word id → 拥有这个 word 的 KeyFrame 列表
```

核心用途：

```text
回环候选检测
地图合并候选检测
重定位候选检测
```

主要函数：

```cpp
DetectLoopCandidates()
DetectNBestCandidates()
DetectRelocalizationCandidates()
```

LoopClosing 和 Tracking 都会用它。

---

# 18. `FrameDrawer.cc`：显示当前跟踪图像

它和算法主体关系不大，主要给 Viewer 用。

Tracking 每处理完一帧会调用：

```cpp
mpFrameDrawer->Update(this);
```

它会复制当前帧图像、特征点、地图点匹配状态，然后 Viewer 可以画出：

```text
绿色点
红色外点
跟踪状态
地图点数量
关键帧数量
```

---

# 19. `TwoViewReconstruction.cc`：主要服务单目初始化

这个文件在你当前双目惯性流程里不是主线。

它主要用于单目初始化：

```text
两帧匹配
同时估计 Homography 和 Fundamental
选择更合理模型
恢复 R/t
三角化初始地图点
```

你的模式是 `IMU_STEREO`，初始化走的是：

```cpp
Tracking::StereoInitialization()
```

双目有深度，所以不需要依赖 `TwoViewReconstruction` 做初始两视图重建。

但是这个文件仍然是 ORB-SLAM3 工程的一部分，主要服务：

```text
MONOCULAR
IMU_MONOCULAR
```

---

# 20. 你当前工程运行时的数据结构流动

一帧数据从 ROS 到地图，大致是：

```text
sensor_msgs::Image
sensor_msgs::Imu
    ↓
cv::Mat 左图 / 右图
vector<IMU::Point>
    ↓
System::TrackStereo()
    ↓
Tracking::GrabImageStereo()
    ↓
Frame
    ├── mvKeys
    ├── mvKeysRight
    ├── mDescriptors
    ├── mDescriptorsRight
    ├── mvDepth
    ├── mvuRight
    └── mpImuPreintegrated
    ↓
Tracking::Track()
    ↓
如果需要关键帧：
    KeyFrame
        ├── BoW
        ├── mvpMapPoints
        ├── pose
        ├── velocity
        └── IMU bias
    ↓
MapPoint
    ├── world position
    ├── observations
    ├── descriptor
    └── normal/depth range
    ↓
Map
    ├── KeyFrames
    └── MapPoints
    ↓
Atlas
    └── 管理一张或多张 Map
```

---

# 21. 复现时最需要注意的几个点

## 21.1 话题必须完全匹配

你的程序订阅的是绝对话题：

```cpp
/imu
/camera/left/image_raw
/camera/right/image_raw
```

所以 rosbag 必须发布到这些话题。

可以用：

```bash
rostopic list
```

确认是否存在：

```text
/camera/left/image_raw
/camera/right/image_raw
/imu
```

---

## 21.2 `do_rectify` 要和数据源匹配

你现在 `stereo_inertial.launch` 默认：

```xml
<arg name="do_rectify" default="false" />
```

如果输入图像已经是校正后的，`false` 是合理的。

如果输入是 EuRoC 原始 raw 图像，通常需要检查是否要设置：

```bash
do_rectify:=true
```

但是也要注意：`System::TrackStereo()` 内部也可能根据新版 `Settings` 再做矫正。所以你要避免两种错误：

```text
没有矫正：左右目不满足极线约束，双目匹配差
重复矫正：图像被 remap 两次，特征和标定都不对
```

---

## 21.3 rosbag 默认暂停

你的 `rosbag.launch` 默认：

```xml
pause=true
```

所以启动后没有数据进入 SLAM 是正常的，需要在 rosbag 终端按空格。

---

## 21.4 你的 ROS 入口没有在退出时调用 `SLAM.Shutdown()`

`ros_stereo_inertial.cc` 最后是：

```cpp
ros::spin();
return 0;
```

官方示例通常会在结束时调用：

```cpp
SLAM.Shutdown();
```

有时还会保存轨迹：

```cpp
SLAM.SaveTrajectoryEuRoC(...);
SLAM.SaveKeyFrameTrajectoryEuRoC(...);
```

你这个版本如果直接 Ctrl+C，LocalMapping、LoopClosing、Viewer 线程可能没有被优雅关闭，也不会自动保存轨迹。复现实验时建议你后面补上关闭和保存逻辑。

---

## 21.5 `SyncWithImu()` 空转时可能占 CPU

你的同步线程只有在成功处理一帧后才：

```cpp
sleep_for(1ms)
```

如果队列为空，它会一直 `while(1)` 空转。

这不一定导致错误，但会浪费 CPU。可以考虑在循环末尾统一加一个短 sleep。

---

# 22. 总结一句话

你这套启动流程可以理解为：

```text
rosbag.launch 负责提供双目图像和 IMU 数据；
stereo_inertial.launch 启动 Stereo_Inertial 节点；
ros_stereo_inertial.cc 做 ROS 数据同步，并调用 System::TrackStereo；
System.cc 创建并组织 ORB-SLAM3 的 Tracking、LocalMapping、LoopClosing、Viewer；
Tracking.cc 完成前端跟踪、IMU 预积分、初始化、位姿估计和关键帧生成；
LocalMapping.cc 消费关键帧，扩展和优化局部地图，并初始化 IMU；
LoopClosing.cc 做回环检测、地图合并和全局一致性优化；
Atlas/Map/KeyFrame/MapPoint/KeyFrameDatabase 是地图和图优化的数据骨架；
ORBextractor/ORBmatcher/Optimizer/ImuTypes 是特征、匹配、优化和惯性预积分的算法工具层。
```

最核心的运行主线就是：

```text
rosbag 数据
 → ros_stereo_inertial.cc 同步
 → System::TrackStereo()
 → Tracking::GrabImageStereo()
 → Frame 特征提取和双目深度
 → Tracking::Track()
 → LocalMapping 建图优化
 → LoopClosing 回环修正
 → Atlas/Map 维护全局地图
```
