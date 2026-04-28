非常推荐你用 RViz 给 ORB-SLAM3 加一套**外部观察窗口**。我建议不要一开始就可视化全部数据，而是按层次来做：

```text id="w8wf5x"
第一层：轨迹、当前位姿、地图点
第二层：关键帧、局部地图、共视图
第三层：特征跟踪、IMU 状态、回环/地图合并
第四层：调试统计量、状态机、优化窗口
```

---

# 1. 最推荐优先可视化的数据

## 1.1 当前相机位姿 `/orb_slam3/camera_pose`

这是最应该首先发布的。

推荐消息类型：

```text id="l1yb74"
geometry_msgs/PoseStamped
```

RViz 显示方式：

```text id="x87fss"
Pose
Axes
TF
```

它能帮你判断：

```text id="zpf1lq"
相机有没有动
位姿是否连续
是否突然跳变
是否发散
是否频繁 reset
```

ORB-SLAM3 中可以从这里得到：

```cpp id="h1kib4"
Sophus::SE3f Tcw = mpSLAM->TrackStereo(...);
Sophus::SE3f Twc = Tcw.inverse();
```

注意：

```text id="asayk3"
Tcw：世界坐标到相机坐标
Twc：相机坐标到世界坐标
```

发布到 RViz 时一般应该发布 `Twc`。

---

## 1.2 相机轨迹 `/orb_slam3/camera_path`

这是第二重要的数据。

推荐消息类型：

```text id="6ry2j0"
nav_msgs/Path
```

RViz 显示方式：

```text id="d8h4r2"
Path
```

它能帮你判断：

```text id="61uosq"
整体轨迹形状是否合理
有没有漂移
有没有尺度错误
有没有明显跳变
回环后轨迹是否被修正
```

建议每次 `TrackStereo()` 返回一个有效位姿，就把当前 pose 追加到 `Path` 中。

推荐 topic：

```text id="29tq41"
/orb_slam3/camera_path
```

---

## 1.3 关键帧轨迹 `/orb_slam3/keyframe_path`

当前帧轨迹是连续的，关键帧轨迹是稀疏的。

推荐消息类型：

```text id="t2ezv4"
nav_msgs/Path
```

RViz 显示方式：

```text id="0tnzpe"
Path
```

它能帮你判断：

```text id="gg66qi"
关键帧是否正常插入
LocalMapping 是否正常运行
是否长时间没有新关键帧
是否关键帧过密或过稀
```

从 ORB-SLAM3 里可以取：

```text id="sy2t6i"
Atlas → Map → KeyFrames
```

也就是：

```cpp id="77ekxu"
mpAtlas->GetAllKeyFrames()
```

或者当前地图：

```cpp id="a2b5lu"
mpAtlas->GetCurrentMap()->GetAllKeyFrames()
```

---

## 1.4 地图点 `/orb_slam3/map_points`

这是 SLAM 最核心的可视化对象之一。

推荐消息类型：

```text id="9kpsqz"
sensor_msgs/PointCloud2
```

RViz 显示方式：

```text id="8vbaq1"
PointCloud2
```

它能帮你判断：

```text id="rkj3g3"
地图点是否生成
地图点是否发散
地图尺度是否正常
点云是否集中在合理空间
是否出现大量离群点
```

从 ORB-SLAM3 里可以取：

```cpp id="ft9jov"
std::vector<MapPoint*> vpMPs = mpAtlas->GetAllMapPoints();
```

每个地图点可以取世界坐标：

```cpp id="4hwygl"
Eigen::Vector3f pos = pMP->GetWorldPos();
```

建议过滤掉坏点：

```cpp id="e4q3m1"
if(pMP && !pMP->isBad())
```

推荐 topic：

```text id="4en3oo"
/orb_slam3/map_points
```

---

# 2. 第二优先级：关键帧和局部地图

## 2.1 关键帧相机模型 `/orb_slam3/keyframes`

不要只看关键帧路径，最好把每个关键帧画成一个小相机锥体。

推荐消息类型：

```text id="9ltixn"
visualization_msgs/MarkerArray
```

RViz 显示方式：

```text id="orl48h"
MarkerArray
```

每个 KeyFrame 显示成一个小 frustum：

```text id="i0arzu"
四棱锥 / 相机视锥
```

它能帮你判断：

```text id="r7lchv"
关键帧朝向是否正确
相机姿态是否翻转
关键帧分布是否合理
是否出现异常跳变关键帧
```

推荐 topic：

```text id="pgksk3"
/orb_slam3/keyframes
```

---

## 2.2 当前局部地图点 `/orb_slam3/local_map_points`

全局地图点多了以后，RViz 会比较乱。

所以建议单独发布当前 Tracking 正在使用的局部地图点。

推荐消息类型：

```text id="xubumr"
sensor_msgs/PointCloud2
```

RViz 显示方式：

```text id="jq9tq6"
PointCloud2
```

它能帮你判断：

```text id="3u841g"
Tracking 当前依赖哪些点
局部地图是否覆盖当前视野
局部点是否过少
TrackLocalMap 是否正常
```

推荐 topic：

```text id="jo4z5m"
/orb_slam3/local_map_points
```

可以从 Tracking 里的局部地图结构取，例如：

```cpp id="j7v39j"
mvpLocalMapPoints
mvpLocalKeyFrames
```

---

## 2.3 当前局部关键帧 `/orb_slam3/local_keyframes`

推荐消息类型：

```text id="m3p0u9"
visualization_msgs/MarkerArray
```

它能帮你判断：

```text id="5q4qmh"
当前帧周围有哪些关键帧参与局部跟踪
局部窗口是否正常
LocalMapping 是否给 Tracking 提供了足够局部约束
```

推荐 topic：

```text id="gr67jm"
/orb_slam3/local_keyframes
```

---

# 3. 第三优先级：共视图、生成树、回环边

这些数据不是每次都必须看，但非常适合调后端。

## 3.1 共视图 `/orb_slam3/covisibility_graph`

推荐消息类型：

```text id="jywtu2"
visualization_msgs/MarkerArray
```

或者：

```text id="ltqs2e"
visualization_msgs/Marker
type = LINE_LIST
```

RViz 显示方式：

```text id="qrlywk"
MarkerArray / Marker
```

共视图连接的是：

```text id="u57e26"
看到相同 MapPoint 的 KeyFrame
```

它能帮你判断：

```text id="pvaw7k"
关键帧之间连接是否正常
局部 BA 的图结构是否合理
地图是否断裂
关键帧是否孤立
```

推荐 topic：

```text id="tcmy03"
/orb_slam3/covisibility_graph
```

---

## 3.2 生成树 `/orb_slam3/spanning_tree`

ORB-SLAM3 内部关键帧之间有生成树关系。

推荐消息类型：

```text id="600y0p"
visualization_msgs/Marker
type = LINE_LIST
```

它能帮你判断：

```text id="o36s26"
关键帧父子关系是否正常
地图结构是否断裂
地图合并后结构是否异常
```

推荐 topic：

```text id="7x95vk"
/orb_slam3/spanning_tree
```

---

## 3.3 回环边 `/orb_slam3/loop_edges`

如果你想观察 LoopClosing，这个非常重要。

推荐消息类型：

```text id="x1yzk1"
visualization_msgs/Marker
type = LINE_LIST
```

它能帮你判断：

```text id="by146c"
是否检测到回环
回环连接是否合理
是否误回环
回环后轨迹是否被优化
```

推荐 topic：

```text id="ebk03h"
/orb_slam3/loop_edges
```

---

# 4. 第四优先级：图像层面的 debug

RViz 不仅可以看 3D，也可以显示图像。

## 4.1 当前跟踪图像 `/orb_slam3/tracking_image`

推荐消息类型：

```text id="f8wd4k"
sensor_msgs/Image
```

RViz 显示方式：

```text id="tsx1fe"
Image
```

图像上建议画：

```text id="s7jxdc"
绿色点：成功跟踪的地图点
红色点：外点
蓝色点：普通 ORB 特征
黄色线：双目匹配或帧间匹配
```

它能帮你判断：

```text id="g3l9zl"
ORB 特征是否正常提取
跟踪点是否足够
外点是否很多
图像是否矫正正确
匹配是否集中在某些区域
是否因为图像模糊导致跟踪失败
```

推荐 topic：

```text id="dev23b"
/orb_slam3/tracking_image
```

这个数据可以从：

```text id="8zrd4d"
FrameDrawer
Tracking
```

中取。

实际上 ORB-SLAM3 的 Pangolin 窗口已经有类似图像，但发布到 RViz 后你可以和轨迹、点云、TF 放在一个界面里观察。

---

## 4.2 左右目双目匹配图 `/orb_slam3/stereo_matches`

推荐消息类型：

```text id="hrickg"
sensor_msgs/Image
```

建议画成：

```text id="6rp1wn"
左图 | 右图 拼接
匹配点之间连线
```

它能帮你判断：

```text id="7ooldf"
左右图是否同步
左右图是否正确矫正
双目匹配是否满足水平极线
深度估计是否可靠
```

如果你发现匹配线大量倾斜，说明大概率是：

```text id="xdxq4f"
双目没有正确 rectify
相机参数错误
左右图话题反了
时间同步有问题
```

推荐 topic：

```text id="cnz80y"
/orb_slam3/stereo_matches
```

---

# 5. 第五优先级：IMU 相关可视化

你跑的是 `IMU_STEREO`，所以 IMU debug 很重要。

## 5.1 IMU 位姿 `/orb_slam3/imu_pose`

ORB-SLAM3 里相机和 IMU 之间有外参：

```text id="8qbr8f"
Tbc / Tcb
```

如果你只发布相机位姿，有时候看不出 IMU 外参问题。

推荐也发布 IMU 位姿：

```text id="9uqu4x"
/orb_slam3/imu_pose
```

消息类型：

```text id="83z761"
geometry_msgs/PoseStamped
```

或者 TF：

```text id="8wwne1"
map → imu
map → camera
```

它能帮你判断：

```text id="81ko8o"
相机-IMU 外参是否正确
IMU body 方向是否合理
视觉轨迹和惯性状态是否一致
```

---

## 5.2 速度箭头 `/orb_slam3/velocity`

推荐消息类型：

```text id="bxw6dh"
visualization_msgs/Marker
type = ARROW
```

它能帮你判断：

```text id="irpjq8"
速度方向是否和运动方向一致
IMU 初始化后速度是否稳定
是否出现速度突然爆炸
```

从 KeyFrame 或 Frame 中可以取：

```text id="r71m0v"
mVw
```

也就是世界坐标下速度。

---

## 5.3 重力方向 `/orb_slam3/gravity`

推荐消息类型：

```text id="x66uvg"
visualization_msgs/Marker
type = ARROW
```

它能帮你判断：

```text id="vrh06w"
IMU 初始化后的重力方向是否正确
地图有没有倾斜
视觉惯性优化是否发散
```

如果重力方向明显不合理，通常要检查：

```text id="2xrr0e"
IMU 噪声参数
IMU bias
相机-IMU 外参
时间同步
加速度单位
```

---

## 5.4 IMU 初始化状态 `/orb_slam3/imu_status`

推荐消息类型：

```text id="zqqfkz"
std_msgs/String
std_msgs/Bool
diagnostic_msgs/DiagnosticArray
```

也可以用 RViz 的 Text Marker 显示：

```text id="fjk9uz"
IMU initialized: true / false
bias acc: ...
bias gyro: ...
```

它能帮你判断：

```text id="og55f3"
IMU 是否完成初始化
初始化用了多久
bias 是否离谱
是否反复 reset
```

推荐 topic：

```text id="x6wpg1"
/orb_slam3/imu_status
```

---

# 6. 第六优先级：Tracking 状态和统计量

这个虽然不是传统 RViz 数据，但非常推荐显示。

## 6.1 Tracking 状态 `/orb_slam3/tracking_status`

推荐消息类型：

```text id="ifz9dx"
std_msgs/String
```

或者：

```text id="d75k4k"
visualization_msgs/Marker
type = TEXT_VIEW_FACING
```

内容可以显示：

```text id="k3br7n"
State: OK
Frame ID: 523
Inliers: 148
Features: 1062
KeyFrames: 36
MapPoints: 2145
IMU Init: true
```

它能帮你判断：

```text id="408y7m"
系统当前是初始化、正常跟踪、短暂丢失还是完全丢失
是否频繁 LOST
内点数量是否突然下降
关键帧和地图点是否增长
```

推荐 topic：

```text id="v4u95k"
/orb_slam3/tracking_status
```

建议把状态机转成字符串：

```text id="v975gq"
NO_IMAGES_YET
NOT_INITIALIZED
OK
RECENTLY_LOST
LOST
```

---

## 6.2 每帧统计量 `/orb_slam3/debug_stats`

推荐消息类型可以简单用：

```text id="2ys42s"
std_msgs/String
```

更规范可以自定义 msg，例如：

```text id="hikpdu"
ORBStatus.msg
```

包含：

```text id="9qwqzt"
Header header
int32 frame_id
int32 tracking_state
int32 num_features
int32 num_tracked_mappoints
int32 num_inliers
int32 num_keyframes
int32 num_mappoints
int32 num_imu_measurements
bool imu_initialized
float32 tracking_time_ms
```

这些数据在 RViz 中不一定最好看，但可以和 `rqt_plot`、`rqt_console` 一起用。

---

# 7. 我建议你最终形成这些 topic

下面是我推荐你自己扩展 ORB-SLAM3 ROS wrapper 后发布的 topic。

## 必做

```text id="88q0v5"
/orb_slam3/camera_pose          geometry_msgs/PoseStamped
/orb_slam3/camera_path          nav_msgs/Path
/orb_slam3/map_points           sensor_msgs/PointCloud2
/orb_slam3/keyframes            visualization_msgs/MarkerArray
/orb_slam3/tracking_image       sensor_msgs/Image
/orb_slam3/tracking_status      std_msgs/String 或 Marker Text
```

这几个就足够你定位 70% 的问题。

---

## 强烈推荐

```text id="eiyjpj"
/orb_slam3/keyframe_path        nav_msgs/Path
/orb_slam3/local_map_points     sensor_msgs/PointCloud2
/orb_slam3/local_keyframes      visualization_msgs/MarkerArray
/orb_slam3/imu_pose             geometry_msgs/PoseStamped
/orb_slam3/velocity             visualization_msgs/Marker
/orb_slam3/gravity              visualization_msgs/Marker
/orb_slam3/imu_status           std_msgs/String
```

这些适合调双目惯性。

---

## 后端调试时再加

```text id="3oj2jd"
/orb_slam3/covisibility_graph   visualization_msgs/MarkerArray
/orb_slam3/spanning_tree        visualization_msgs/MarkerArray
/orb_slam3/loop_edges           visualization_msgs/MarkerArray
/orb_slam3/stereo_matches       sensor_msgs/Image
/orb_slam3/debug_stats          自定义 msg 或 std_msgs/String
```

这些适合调 LocalMapping、LoopClosing、地图结构。

---

# 8. RViz 中建议的显示布局

建议你在 RViz 中固定坐标系设置为：

```text id="9nzozl"
Fixed Frame: map
```

然后添加：

```text id="cjqshd"
TF
Path: /orb_slam3/camera_path
Pose: /orb_slam3/camera_pose
PointCloud2: /orb_slam3/map_points
MarkerArray: /orb_slam3/keyframes
MarkerArray: /orb_slam3/local_keyframes
PointCloud2: /orb_slam3/local_map_points
Image: /orb_slam3/tracking_image
Marker: /orb_slam3/velocity
Marker: /orb_slam3/gravity
Marker: /orb_slam3/tracking_status
```

---

# 9. 这些可视化分别能定位什么问题？

## 如果没有轨迹

重点检查：

```text id="t7l3pe"
System::TrackStereo() 是否被调用
Tracking 是否初始化
Tcw 是否为空或无效
时间戳是否正常
```

---

## 如果有轨迹但没有地图点

重点检查：

```text id="q0m5c2"
Frame 是否有深度
ComputeStereoMatches 是否正常
Tracking 是否创建 MapPoint
LocalMapping 是否工作
```

---

## 如果地图点很多但轨迹乱跳

重点检查：

```text id="899ij7"
双目参数
IMU 参数
相机-IMU 外参
图像是否重复矫正
Tcw/Twc 是否发布反了
坐标系转换是否错了
```

---

## 如果关键帧很少

重点检查：

```text id="eiib1p"
NeedNewKeyFrame()
LocalMapping 是否阻塞
Tracking 是否认为当前帧质量不足
```

---

## 如果关键帧很多

重点检查：

```text id="r43wtn"
跟踪不稳定
局部地图匹配差
运动模型预测差
IMU 初始化失败
```

---

## 如果局部地图点很少

重点检查：

```text id="373h04"
UpdateLocalMap()
SearchLocalPoints()
共视图连接是否正常
MapPoint 是否大量被剔除
```

---

## 如果 IMU 速度或重力方向异常

重点检查：

```text id="qy8iys"
IMU 单位
IMU 噪声参数
IMU bias
时间同步
Tbc/Tcb 外参
```

---

# 10. 最推荐你的实现顺序

你可以按这个顺序一步一步加，不要一次性全加。

## 第一步：发布当前位姿和轨迹

```text id="bbkixl"
/orb_slam3/camera_pose
/orb_slam3/camera_path
```

先确认：

```text id="pxsn6n"
Tcw/Twc 没弄反
RViz 里轨迹能动
坐标系方向合理
```

---

## 第二步：发布地图点

```text id="cop3lz"
/orb_slam3/map_points
```

确认：

```text id="xmo6k1"
地图点能增长
地图没有爆炸
点云和轨迹在同一个坐标系下
```

---

## 第三步：发布关键帧

```text id="dkjvjp"
/orb_slam3/keyframes
/orb_slam3/keyframe_path
```

确认：

```text id="2uc2fl"
关键帧插入正常
相机朝向正常
LocalMapping 正常工作
```

---

## 第四步：发布跟踪图像

```text id="mfhwn5"
/orb_slam3/tracking_image
```

确认：

```text id="z7spr8"
特征点和跟踪点正常
内点数量够
图像没有左右颠倒或畸变错误
```

---

## 第五步：发布 IMU 状态

```text id="yzz72s"
/orb_slam3/imu_pose
/orb_slam3/velocity
/orb_slam3/gravity
/orb_slam3/imu_status
```

确认：

```text id="b3uzne"
IMU 初始化成功
速度方向合理
重力方向合理
```

---

# 11. 一个推荐的最终 topic 架构

```text id="u42hfk"
ROS 输入层
────────────────────────────
/camera/left/image_raw
/camera/right/image_raw
/imu

ORB-SLAM3 可视化输出层
────────────────────────────
/orb_slam3/camera_pose
/orb_slam3/camera_path
/orb_slam3/keyframe_path
/orb_slam3/map_points
/orb_slam3/local_map_points
/orb_slam3/keyframes
/orb_slam3/local_keyframes
/orb_slam3/tracking_image
/orb_slam3/stereo_matches
/orb_slam3/imu_pose
/orb_slam3/velocity
/orb_slam3/gravity
/orb_slam3/tracking_status

ORB-SLAM3 后端调试层
────────────────────────────
/orb_slam3/covisibility_graph
/orb_slam3/spanning_tree
/orb_slam3/loop_edges
/orb_slam3/debug_stats

TF 层
────────────────────────────
map → camera
map → imu
camera → left_camera
camera → right_camera
imu → camera
```

---

# 12. 最核心建议

你最先做这 5 个就够了：

```text id="6cy4er"
/orb_slam3/camera_pose
/orb_slam3/camera_path
/orb_slam3/map_points
/orb_slam3/keyframes
/orb_slam3/tracking_status
```

这 5 个能回答最关键的问题：

```text id="l9mgxd"
SLAM 有没有在跑？
相机位姿有没有出来？
轨迹是否合理？
地图点是否正常生成？
关键帧是否正常插入？
Tracking 当前是否丢失？
```

然后再加：

```text id="793uvj"
/orb_slam3/tracking_image
/orb_slam3/local_map_points
/orb_slam3/imu_status
/orb_slam3/velocity
/orb_slam3/gravity
```

这时你就能比较系统地观察 ORB-SLAM3 的前端、后端、地图和 IMU 状态了。
