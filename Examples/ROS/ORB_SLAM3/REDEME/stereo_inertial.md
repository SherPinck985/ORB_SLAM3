# 1. 你启动的两个 launch 文件分别做什么？

## 1.1 `rosbag.launch`：负责播放数据，并重映射话题

这个文件只做一件事：播放 EuRoC 的 `.bag` 数据，并把 EuRoC 原始话题改成你的 ORB-SLAM3 ROS 节点订阅的话题。

原始 EuRoC bag 里通常是：

```text
/cam0/image_raw
/cam1/image_raw
/imu0
```

你的 `rosbag.launch` 把它们重映射成：

```text
/camera/left/image_raw
/camera/right/image_raw
/imu
```

对应代码逻辑是：

```xml
$(arg cam0_src):=$(arg left_dst)
$(arg cam1_src):=$(arg right_dst)
$(arg imu_src):=$(arg imu_dst)
```

也就是说，`rosbag.launch` 不是启动 SLAM 的，它只是给 SLAM 节点喂数据。

默认参数里：

```xml
<arg name="pause" default="true" />
```

所以默认启动后 rosbag 是暂停状态，你需要在 rosbag 终端按空格继续播放。

---

## 1.2 `stereo_inertial.launch`：负责启动 ORB-SLAM3 双目惯性节点

这个文件启动的是：

```xml
<node pkg="ORB_SLAM3"
      type="$(arg node_type)"
      name="stereo_inertial"
      output="screen"
      args="$(arg voc_file) $(arg settings_file) $(arg do_rectify)" />
```

默认等价于：

```bash
rosrun ORB_SLAM3 Stereo_Inertial \
  /home/sherpinck_holomz/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /home/sherpinck_holomz/ORB_SLAM3/Examples/Stereo-Inertial/EuRoC.yaml \
  false
```

其中：

```text
Stereo_Inertial
```

对应的源码入口就是你上传的：

```text
ros_stereo_inertial.cc
```

所以整体启动关系是：

```text
rosbag.launch
    └── rosbag play
        ├── 发布 /camera/left/image_raw
        ├── 发布 /camera/right/image_raw
        └── 发布 /imu

stereo_inertial.launch
    └── 启动 Stereo_Inertial 节点
        └── ros_stereo_inertial.cc
            └── 创建 ORB_SLAM3::System
```

---

# 2. `ros_stereo_inertial.cc` 是 ROS 和 ORB-SLAM3 的桥梁

这个文件不是 ORB-SLAM3 算法主体，它主要负责：

```text
ROS 图像/IMU 接收
        ↓
左右目图像同步
        ↓
IMU 数据按图像时间戳截取
        ↓
可选图像矫正 / 增强
        ↓
调用 ORB_SLAM3::System::TrackStereo()
```

它里面有两个核心类：

```cpp
class ImuGrabber
class ImageGrabber
```

---

## 2.1 `ImuGrabber`：只负责缓存 IMU

核心函数：

```cpp
void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
```

它收到 `/imu` 后只是：

```cpp
imuBuf.push(imu_msg);
```

也就是说，IMU 回调线程不做预积分、不做优化，只是把消息放入队列。

为什么 IMU 不像图像一样丢弃旧数据？

因为视觉惯性系统需要保留两帧图像之间的完整 IMU 序列，否则预积分会断，状态传播会出问题。

---

## 2.2 `ImageGrabber`：接收左右图，并做视觉-惯性同步

它订阅：

```cpp
/camera/left/image_raw
/camera/right/image_raw
```

两个图像回调：

```cpp
GrabImageLeft()
GrabImageRight()
```

都采用一个策略：

```cpp
如果队列里已经有旧图，则 pop 掉，只保留最新一帧。
```

这是为了降低延迟。图像处理如果跟不上，它宁愿丢图，也不希望 SLAM 处理严重滞后的图像。

---

## 2.3 最核心函数：`ImageGrabber::SyncWithImu()`

这个函数是单独开的线程：

```cpp
std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);
```

它不断做下面几件事：

### 第一步：检查左右图和 IMU 是否都有数据

```cpp
if (!imgLeftBuf.empty() && !imgRightBuf.empty() && !mpImuGb->imuBuf.empty())
```

---

### 第二步：左右图像时间同步

允许的最大左右目时间差：

```cpp
const double maxTimeDiff = 0.01;
```

也就是 10ms。

如果右图太早，就丢右图；如果左图太早，就丢左图。

最终要求：

```cpp
abs(tImLeft - tImRight) <= 0.01
```

否则这一轮跳过。

---

### 第三步：等待当前图像时刻之前的 IMU 数据到齐

```cpp
if(tImLeft > mpImuGb->imuBuf.back()->header.stamp.toSec())
    continue;
```

这句很重要。

意思是：

```text
如果当前左图时间戳比 IMU 队列最后一条还晚，
说明当前图像时刻之前的 IMU 数据还没全部到达，
所以暂时不能送入 SLAM。
```

---

### 第四步：取出当前图像时间之前的所有 IMU

```cpp
while(!imuBuf.empty() && imuBuf.front()->header.stamp.toSec() <= tImLeft)
{
    acc = linear_acceleration
    gyr = angular_velocity
    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
    imuBuf.pop();
}
```

于是这一帧送入 ORB-SLAM3 的数据是：

```text
左图 imLeft
右图 imRight
图像时间 tImLeft
从上一帧到当前帧附近的 IMU 序列 vImuMeas
```

---

### 第五步：调用 ORB-SLAM3 主入口

```cpp
mpSLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
```

到这里，ROS 部分结束，正式进入 ORB-SLAM3 工程内部。

---
