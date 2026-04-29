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

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"
#include"../include/RvizVisualizer.h"

using namespace std;

/**
 * @brief IMU 数据接收类
 *
 * 该类只负责在 ROS IMU 回调中缓存传感器消息。
 * 之所以单独封装成一个类，是为了把“接收数据”和“处理数据”分开：
 * - 回调函数只做轻量级入队，避免阻塞 ROS 通信线程
 * - 真正消费 IMU 数据的工作由图像同步线程完成
 */
class ImuGrabber
{
public:
    ImuGrabber(){};

    /**
     * @brief IMU 回调函数，将接收到的 IMU 消息压入缓冲队列
     * @param imu_msg ROS 中的 IMU 消息智能指针
     */
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    // IMU 数据缓冲区。队首是最早到达、尚未被消费的 IMU 数据。
    queue<sensor_msgs::ImuConstPtr> imuBuf;

    // 保护 imuBuf 的互斥锁，防止回调线程和同步线程并发访问产生竞争。
    std::mutex mBufMutex;
};

/**
 * @brief 图像接收与双目-IMU 同步类
 *
 * 职责包括：
 * 1. 接收左右目图像并缓存
 * 2. 将 ROS 图像消息转换为 OpenCV 图像
 * 3. 将左右图像按时间戳对齐
 * 4. 从 IMU 队列中提取当前图像时刻之前的惯性数据
 * 5. 做可选的图像增强与极线校正
 * 6. 调用 ORB-SLAM3 的 TrackStereo 接口
 */
class ImageGrabber
{
public:
    /**
     * @brief 构造函数
     * @param pSLAM   ORB-SLAM3 系统对象指针
     * @param pImuGb  IMU 数据接收对象指针
     * @param bRect   是否对双目图像执行去畸变/极线校正
     * @param bClahe  是否执行 CLAHE 直方图均衡增强
     */
    ImageGrabber(ORB_SLAM3::System* pSLAM,
                 ImuGrabber *pImuGb,
                 ORB_SLAM3::RvizVisualizer *pRviz,
                 const bool bRect,
                 const bool bClahe)
        : mpSLAM(pSLAM),
          mpImuGb(pImuGb),
          mpRviz(pRviz),
          do_rectify(bRect),
          mbClahe(bClahe){}

    /**
     * @brief 左目图像回调函数
     * @param msg ROS 左目图像消息
     */
    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);

    /**
     * @brief 右目图像回调函数
     * @param msg ROS 右目图像消息
     */
    void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);

    /**
     * @brief 将 ROS 图像消息转换为 OpenCV 灰度图
     * @param img_msg ROS 图像消息
     * @return 转换后的 OpenCV 图像
     */
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);

    /**
     * @brief 同步左右图像与 IMU 数据，并调用 ORB-SLAM3 前端跟踪
     */
    void SyncWithImu();

    // 左右图像缓冲区。这里采用“最多只保留最新的一帧”的策略，以控制延迟。
    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;

    // 左右图像缓冲区各自的互斥锁。
    std::mutex mBufMutexLeft,mBufMutexRight;
   
    // ORB-SLAM3 系统对象，用于调用 TrackStereo。
    ORB_SLAM3::System* mpSLAM;

    // IMU 数据接收器，用于访问 imuBuf。
    ImuGrabber *mpImuGb;

    // RViz 可视化器。它只负责发布位姿、轨迹和地图点，不参与 SLAM 算法。
    ORB_SLAM3::RvizVisualizer *mpRviz;

    // 是否执行双目图像矫正。
    const bool do_rectify;

    // remap 所需映射表：
    // M1* / M2* 由 initUndistortRectifyMap 预先计算得到。
    cv::Mat M1l,M2l,M1r,M2r;

    // 是否执行 CLAHE 图像增强。
    const bool mbClahe;

    // CLAHE 对象：
    // clipLimit = 3.0，tileGridSize = 8x8。
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};


/**
 * @brief ROS 双目惯性示例程序主函数
 *
 * 程序整体流程：
 * 1. 初始化 ROS 节点
 * 2. 检查命令行参数
 * 3. 创建 ORB-SLAM3 系统对象
 * 4. 根据配置文件准备双目矫正映射表
 * 5. 订阅左右图像与 IMU 话题
 * 6. 启动同步线程，持续调用 TrackStereo
 * 7. 进入 ROS 事件循环
 *
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 0 表示正常退出，非 0 表示异常退出
 */
int main(int argc, char **argv)
{
  // 初始化 ROS 节点，节点名为 Stereo_Inertial。
  ros::init(argc, argv, "Stereo_Inertial");

  // 使用当前节点的私有命名空间创建 NodeHandle。
  ros::NodeHandle n("~");

  // 设置日志级别为 Info，便于在终端中观察运行状态。
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  // 是否启用图像均衡化，默认关闭。
  bool bEqual = false;

  // 参数格式：
  // rosrun ORB_SLAM3 Stereo_Inertial path_to_vocabulary path_to_settings do_rectify [do_equalize]
  // 因此 argc 只能是 4 或 5。
  if(argc < 4 || argc > 5)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo_Inertial path_to_vocabulary path_to_settings do_rectify [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }

  // 第 3 个参数表示是否执行双目校正，通常为字符串 "true" 或 "false"。
  std::string sbRect(argv[3]);
  if(argc==5)
  {
    // 第 4 个可选参数表示是否执行 CLAHE 图像增强。
    std::string sbEqual(argv[4]);
    if(sbEqual == "true")
      bEqual = true;
  }

  // 创建 ORB-SLAM3 系统对象：
  // argv[1] 为词袋文件路径
  // argv[2] 为相机/IMU 配置文件路径
  // IMU_STEREO 表示使用双目 + IMU 的工作模式
  // 最后一个 true 表示启用可视化相关线程
  ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO,true);

  // 创建通用 RViz 可视化器。Fixed Frame 统一使用 world。
  ORB_SLAM3::RvizVisualizer rviz(&SLAM, n, ORB_SLAM3::System::GetWorldFrameId(), 5);

  // 创建 IMU 接收器和图像接收/同步器。
  ImuGrabber imugb;
  ImageGrabber igb(&SLAM,&imugb,&rviz,sbRect == "true",bEqual);
  
    // 当 do_rectify 为 true 时，需要从配置文件中读取双目标定参数，
    // 并预先构建去畸变与极线校正映射表。
    if(igb.do_rectify)
    {      
        // 打开配置文件（通常是 EuRoC.yaml 一类的传感器配置文件）。
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            SLAM.Shutdown();
            return -1;
        }

        // K：相机内参矩阵
        // P：投影矩阵
        // R：校正旋转矩阵
        // D：畸变系数
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        // 若标定参数缺失，则无法进行图像矫正，程序直接退出。
        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            SLAM.Shutdown();
            return -1;
        }

        // 根据标定参数预计算重映射表。
        // 之后每一帧图像只需要调用 remap 即可完成去畸变和极线对齐。
        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

  // 订阅 IMU、左目图像、右目图像话题。
  // 这里的队列长度由 subscribe 的第二个参数指定。
  ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
  ros::Subscriber sub_img_left = n.subscribe("/camera/left/image_raw", 100, &ImageGrabber::GrabImageLeft,&igb);
  ros::Subscriber sub_img_right = n.subscribe("/camera/right/image_raw", 100, &ImageGrabber::GrabImageRight,&igb);

  // 启动独立同步线程。
  // 该线程持续从左右图像缓存和 IMU 缓存中取数据，完成时间同步并调用 TrackStereo。
  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

  // 进入 ROS 事件循环，等待回调触发。
  ros::spin();

  // ros::spin() 返回后，通常表示 ROS 已经开始关闭（例如 Ctrl+C）。
  // 先等待同步线程退出，避免它在 SLAM.Shutdown() 期间继续调用 TrackStereo。
  if(sync_thread.joinable())
    sync_thread.join();

  // 优雅关闭 ORB-SLAM3 内部线程，包括 LocalMapping、LoopClosing 和 Viewer。
  SLAM.Shutdown();

  // 保存默认 EuRoC 格式轨迹，便于复现实验和后续评估。
  SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
  SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

  return 0;
}


/**
 * @brief 左目图像回调函数
 *
 * 该函数只负责把最新接收到的左目图像放入缓冲队列。
 * 如果队列中已有旧图像，则丢弃旧图，只保留最新一帧，
 * 这样可以避免图像处理滞后造成的累积延迟。
 *
 * @param img_msg ROS 左目图像消息
 */
void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
  // 加锁，防止与同步线程同时访问左图缓冲区。
  mBufMutexLeft.lock();

  // 若缓冲中已有旧图，则直接丢弃，只保留最新图像。
  if (!imgLeftBuf.empty())
    imgLeftBuf.pop();

  // 压入最新收到的左图消息。
  imgLeftBuf.push(img_msg);

  // 解锁。
  mBufMutexLeft.unlock();
}

/**
 * @brief 右目图像回调函数
 *
 * 与左目图像回调相同，始终只保留最新的右图，降低系统整体延迟。
 *
 * @param img_msg ROS 右目图像消息
 */
void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
  // 对右图缓冲区加锁。
  mBufMutexRight.lock();

  // 丢弃旧图，只保留最新图像。
  if (!imgRightBuf.empty())
    imgRightBuf.pop();

  // 压入当前最新右图。
  imgRightBuf.push(img_msg);

  // 解锁。
  mBufMutexRight.unlock();
}

/**
 * @brief 将 ROS 图像消息转换为 OpenCV 灰度图
 *
 * ORB-SLAM3 前端这里期望拿到单通道灰度图，因此调用 toCvShare 时
 * 直接请求 MONO8 格式。
 *
 * @param img_msg ROS 图像消息
 * @return 转换后的 cv::Mat 图像
 */
cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // 使用 cv_bridge 在 ROS 图像消息和 OpenCV 图像之间转换。
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    // 将输入图像转换/共享为 MONO8 格式，即 8 位单通道灰度图。
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    // 若格式转换失败，则打印错误。
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  // type()==0 对应 CV_8UC1，即标准灰度图。
  if(cv_ptr->image.type()==0)
  {
    // clone() 返回深拷贝，避免底层 ROS 消息内存生命周期带来影响。
    return cv_ptr->image.clone();
  }
  else
  {
    // 若图像类型不是预期值，则输出提示，但仍返回图像副本。
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

/**
 * @brief 同步左右图像与 IMU 数据，并将同步结果送入 ORB-SLAM3
 *
 * 这是整个文件的核心函数，主要工作流程为：
 * 1. 等待左右图像和 IMU 缓冲都非空
 * 2. 按时间戳对齐左右图像
 * 3. 确保当前图像对应时刻之前的 IMU 数据已经到齐
 * 4. 取出该时刻之前的所有 IMU 测量
 * 5. 对图像做可选增强与矫正
 * 6. 调用 TrackStereo 进入 ORB-SLAM3 前端
 */
void ImageGrabber::SyncWithImu()
{
  // 左右图像允许的最大时间差，单位为秒。
  // 这里设置为 0.01，即 10ms。
  const double maxTimeDiff = 0.01;

  // 持续运行同步线程，不断消费缓冲区中的数据。
  while(ros::ok())
  {
    // 保存当前一对同步后的左右图像。
    cv::Mat imLeft, imRight;

    // 分别记录左右图像时间戳。
    double tImLeft = 0, tImRight = 0;

    // 只有在左右图像队列和 IMU 队列都非空时，才尝试进行同步。
    if (!imgLeftBuf.empty()&&!imgRightBuf.empty()&&!mpImuGb->imuBuf.empty())
    {
      // 读取当前待处理的左右图像时间戳。
      tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      tImRight = imgRightBuf.front()->header.stamp.toSec();

      // 若右图时间明显落后于左图，则持续丢弃过早的右图，
      // 直到左右图时间接近，或者右图队列只剩下一张。
      this->mBufMutexRight.lock();
      while((tImLeft-tImRight)>maxTimeDiff && imgRightBuf.size()>1)
      {
        imgRightBuf.pop();
        tImRight = imgRightBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexRight.unlock();

      // 若左图时间明显落后于右图，则持续丢弃过早的左图。
      this->mBufMutexLeft.lock();
      while((tImRight-tImLeft)>maxTimeDiff && imgLeftBuf.size()>1)
      {
        imgLeftBuf.pop();
        tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexLeft.unlock();

      // 如果经过以上处理后，左右图时间差仍然过大，
      // 说明当前无法组成一对有效双目帧，本轮跳过。
      if((tImLeft-tImRight)>maxTimeDiff || (tImRight-tImLeft)>maxTimeDiff)
      {
        continue;
      }

      // 若当前图像时间晚于 imuBuf 中最后一条 IMU 的时间，
      // 说明“该图像时刻之前”的 IMU 数据还没完全到达，需要继续等待。
      if(tImLeft>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;

      // 取出同步成功的左图并从队列中弹出。
      this->mBufMutexLeft.lock();
      imLeft = GetImage(imgLeftBuf.front());
      imgLeftBuf.pop();
      this->mBufMutexLeft.unlock();

      // 取出同步成功的右图并从队列中弹出。
      this->mBufMutexRight.lock();
      imRight = GetImage(imgRightBuf.front());
      imgRightBuf.pop();
      this->mBufMutexRight.unlock();

      // 用于保存送入当前图像帧的 IMU 测量序列。
      vector<ORB_SLAM3::IMU::Point> vImuMeas;

      // 访问 IMU 缓冲区前先加锁。
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // 准备从缓冲区中提取“当前图像时刻之前”的所有 IMU 测量。
        vImuMeas.clear();

        // 将时间戳小于等于当前图像时间的 IMU 数据全部取出。
        // 这些数据将用于当前帧与上一帧之间的惯性预积分。
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImLeft)
        {
          // 读取当前 IMU 数据的时间戳。
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();

          // 读取三轴线加速度。
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);

          // 读取三轴角速度。
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);

          // 封装成 ORB-SLAM3 定义的 IMU::Point 结构，并压入当前帧的 IMU 序列。
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));

          // 当前这条 IMU 数据已经被消费，从全局缓冲区弹出。
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();

      // 若启用了 CLAHE，则在送入前端之前增强图像局部对比度。
      // 这在低对比度、曝光不足或纹理较弱场景中可能有帮助。
      if(mbClahe)
      {
        mClahe->apply(imLeft,imLeft);
        mClahe->apply(imRight,imRight);
      }

      // 若启用了双目矫正，则使用预计算的映射表对图像进行 remap。
      // 目的是消除镜头畸变并使左右图满足极线对齐，便于双目匹配。
      if(do_rectify)
      {
        cv::remap(imLeft,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imRight,imRight,M1r,M2r,cv::INTER_LINEAR);
      }

      // 调用 ORB-SLAM3 的双目惯性跟踪入口。
      // 参数分别为：
      // - 左图
      // - 右图
      // - 当前图像时间戳
      // - 当前图像对应的 IMU 测量序列
      Sophus::SE3f Tcw = mpSLAM->TrackStereo(imLeft,imRight,tImLeft,vImuMeas);

      // 将 ORB-SLAM3 内部状态交给通用 RViz 可视化器发布。
      // 注意：RViz 可视化逻辑已从本入口程序中分离出去。
      if(mpRviz)
        mpRviz->PublishAll(Tcw, tImLeft);

      // 适当休眠 1ms，避免 while(1) 在高频空转时过度占用 CPU。
      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}

/**
 * @brief IMU 回调函数，将 ROS IMU 消息压入 IMU 缓冲队列
 *
 * 与图像回调不同，这里不会主动丢弃旧 IMU 数据。
 * 原因是视觉惯性系统需要保留图像之间完整的 IMU 序列，
 * 以便进行预积分和状态传播。
 *
 * @param imu_msg ROS IMU 消息
 */
void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  // 对 IMU 缓冲区加锁。
  mBufMutex.lock();

  // 将新到达的 IMU 数据按时间顺序压入队列尾部。
  imuBuf.push(imu_msg);

  // 解锁。
  mBufMutex.unlock();
  return;
}
