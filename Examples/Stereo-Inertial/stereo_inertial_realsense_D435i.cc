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

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <condition_variable>

#include <opencv2/core/core.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"

#include <System.h>

using namespace std;

// Ctrl+C 退出标志。RealSense 回调和主循环会根据该标志结束当前会话。
bool b_continue_session;

void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;

}

// 旧接口声明，当前文件中没有实际调用。
void interpolateData(const std::vector<double> &vBase_times,
                     std::vector<rs2_vector> &vInterp_data, std::vector<double> &vInterp_times,
                     const rs2_vector &prev_data, const double &prev_time);

// 将低频 accel 数据补齐到 gyro 时间戳上。
// D435i 的 gyro 频率通常高于 accel，ORB-SLAM3 期望每个 IMU::Point 同时有加速度和角速度。
rs2_vector interpolateMeasure(const double target_time,
                              const rs2_vector current_data, const double current_time,
                              const rs2_vector prev_data, const double prev_time);

static rs2_option get_sensor_option(const rs2::sensor& sensor)
{
    // Sensors usually have several options to control their properties
    //  such as Exposure, Brightness etc.

    std::cout << "Sensor supports the following options:\n" << std::endl;

    // The following loop shows how to iterate over all available options
    // Starting from 0 until RS2_OPTION_COUNT (exclusive)
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
    {
        rs2_option option_type = static_cast<rs2_option>(i);
        //SDK enum types can be streamed to get a string that represents them
        std::cout << "  " << i << ": " << option_type;

        // To control an option, use the following api:

        // First, verify that the sensor actually supports this option
        if (sensor.supports(option_type))
        {
            std::cout << std::endl;

            // Get a human readable description of the option
            const char* description = sensor.get_option_description(option_type);
            std::cout << "       Description   : " << description << std::endl;

            // Get the current value of the option
            float current_value = sensor.get_option(option_type);
            std::cout << "       Current Value : " << current_value << std::endl;

            //To change the value of an option, please follow the change_sensor_option() function
        }
        else
        {
            std::cout << " is not supported" << std::endl;
        }
    }

    uint32_t selected_sensor_option = 0;
    return static_cast<rs2_option>(selected_sensor_option);
}

int main(int argc, char **argv) {

    // 参数：
    // argv[1] 词袋文件路径，例如 Vocabulary/ORBvoc.txt
    // argv[2] D435i 对应的 ORB-SLAM3 配置文件
    // argv[3] 可选轨迹文件名，会传给 System 构造函数中的 sequence/file_name 字段。
    if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: ./stereo_inertial_realsense_D435i path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name;

    if (argc == 4) {
        file_name = string(argv[argc - 1]);
    }

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    double offset = 0; // ms

    // 枚举并选择第一个 RealSense 设备。本例默认机器上只接入一台 D435i。
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device selected_device;
    if (devices.size() == 0)
    {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        return 0;
    }
    else
        selected_device = devices[0];

    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
    int index = 0;
    // We can now iterate the sensors and print their names
    for (rs2::sensor sensor : sensors)
        if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
            ++index;
            if (index == 1) {
                // 深度/红外模组：打开自动曝光，限制曝光时间，并关闭红外投射器。
                // 关闭 emitter 可以减少主动红外纹理对双目特征和外部环境的干扰。
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
                sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT,5000);
                sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0); // switch off emitter
            }
            // std::cout << "  " << index << " : " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
            get_sensor_option(sensor);
            if (index == 2){
                // RGB camera (not used here...)
                // 本例只使用双红外相机，RGB 相机曝光设置不会进入 SLAM 前端。
                sensor.set_option(RS2_OPTION_EXPOSURE,100.f);
            }

            if (index == 3){
                // IMU 硬件运动校正关闭，保持原始惯性测量交给 ORB-SLAM3 预积分处理。
                sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION,0);
            }

        }

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // D435i 的左右红外图作为双目输入，分辨率 640x480，帧率 30Hz。
    // 注意：这里没有读取 RGB 或深度流，ORB-SLAM3 使用的是 stereo-inertial 模式。
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
    // 启用加速度计和陀螺仪原始数据流，频率由设备默认 profile 决定。
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    // IMU callback
    // RealSense pipeline 使用异步回调：图像帧和 IMU 帧会从 librealsense 内部线程进入 imu_callback。
    // 主线程通过 mutex + condition_variable 等待新图像，并一次性取走图像时刻之前的 IMU 缓存。
    std::mutex imu_mutex;
    std::condition_variable cond_image_rec;

    // 原始 accel/gyro 缓存。gyro 频率更高，因此后续会把 accel 对齐到 gyro 时间戳。
    vector<double> v_accel_timestamp;
    vector<rs2_vector> v_accel_data;
    vector<double> v_gyro_timestamp;
    vector<rs2_vector> v_gyro_data;

    // accel 的上一帧和当前帧，用于在两个加速度采样之间为 gyro 时间戳补齐加速度。
    double prev_accel_timestamp = 0;
    rs2_vector prev_accel_data;
    double current_accel_timestamp = 0;
    rs2_vector current_accel_data;
    // 与 gyro 时间戳一一对应的加速度序列。
    vector<double> v_accel_timestamp_sync;
    vector<rs2_vector> v_accel_data_sync;

    // 图像回调写入 imCV / imRightCV，主线程 clone 后送入 ORB-SLAM3。
    cv::Mat imCV, imRightCV;
    int width_img, height_img;
    double timestamp_image = -1.0;
    bool image_ready = false;
    int count_im_buffer = 0; // count dropped frames

    auto imu_callback = [&](const rs2::frame& frame)
    {
        std::unique_lock<std::mutex> lock(imu_mutex);

        if(rs2::frameset fs = frame.as<rs2::frameset>())
        {
            // frameset 表示一组同步到来的图像帧。这里取左右红外图作为双目图像。
            count_im_buffer++;

            double new_timestamp_image = fs.get_timestamp()*1e-3;
            if(abs(timestamp_image-new_timestamp_image)<0.001){
                // librealsense 偶尔可能推送重复时间戳的 frameset，直接丢弃避免重复跟踪同一帧。
                // cout << "Two frames with the same timeStamp!!!\n";
                count_im_buffer--;
                return;
            }

            rs2::video_frame ir_frameL = fs.get_infrared_frame(1);
            rs2::video_frame ir_frameR = fs.get_infrared_frame(2);

            imCV = cv::Mat(cv::Size(width_img, height_img), CV_8U, (void*)(ir_frameL.get_data()), cv::Mat::AUTO_STEP);
            imRightCV = cv::Mat(cv::Size(width_img, height_img), CV_8U, (void*)(ir_frameR.get_data()), cv::Mat::AUTO_STEP);

            timestamp_image = fs.get_timestamp()*1e-3;
            image_ready = true;

            while(v_gyro_timestamp.size() > v_accel_timestamp_sync.size())
            {
                // 如果图像先于新的 accel 到达，则对尚未配对的 gyro 暂时使用当前 accel。
                // 主循环取数据前还会再尝试补齐一次。
                int index = v_accel_timestamp_sync.size();
                double target_time = v_gyro_timestamp[index];

                v_accel_data_sync.push_back(current_accel_data);
                v_accel_timestamp_sync.push_back(target_time);
            }

            lock.unlock();
            cond_image_rec.notify_all();
        }
        else if (rs2::motion_frame m_frame = frame.as<rs2::motion_frame>())
        {
            if (m_frame.get_profile().stream_name() == "Gyro")
            {
                // It runs at 200Hz
                // gyro 高频到达，先按原始顺序缓存；单位和坐标系由 RealSense SDK 给出。
                v_gyro_data.push_back(m_frame.get_motion_data());
                v_gyro_timestamp.push_back((m_frame.get_timestamp()+offset)*1e-3);
                //rs2_vector gyro_sample = m_frame.get_motion_data();
                //std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
            }
            else if (m_frame.get_profile().stream_name() == "Accel")
            {
                // It runs at 60Hz
                // accel 低频到达，保留上一帧和当前帧，用于为已经到达的 gyro 时间戳生成对应加速度。
                prev_accel_timestamp = current_accel_timestamp;
                prev_accel_data = current_accel_data;

                current_accel_data = m_frame.get_motion_data();
                current_accel_timestamp = (m_frame.get_timestamp()+offset)*1e-3;

                while(v_gyro_timestamp.size() > v_accel_timestamp_sync.size())
                {
                    int index = v_accel_timestamp_sync.size();
                    double target_time = v_gyro_timestamp[index];

                    // 为第 index 个 gyro 样本生成同时间戳的 accel 样本，保证两组 vector 后续可以按下标配对。
                    rs2_vector interp_data = interpolateMeasure(target_time, current_accel_data, current_accel_timestamp,
                                                                prev_accel_data, prev_accel_timestamp);

                    v_accel_data_sync.push_back(interp_data);
                    v_accel_timestamp_sync.push_back(target_time);

                }
                // std::cout << "Accel:" << current_accel_data.x << ", " << current_accel_data.y << ", " << current_accel_data.z << std::endl;
            }
        }
    };

    rs2::pipeline_profile pipe_profile = pipe.start(cfg, imu_callback);

    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    // 从实际启动后的 pipeline profile 读取左右红外和 IMU stream profile。
    // 这些 profile 用于打印内外参，便于核对 YAML 配置是否与设备一致。
    rs2::stream_profile cam_left = pipe_profile.get_stream(RS2_STREAM_INFRARED, 1);
    rs2::stream_profile cam_right = pipe_profile.get_stream(RS2_STREAM_INFRARED, 2);


    rs2::stream_profile imu_stream = pipe_profile.get_stream(RS2_STREAM_GYRO);
    // RealSense SDK 中 get_extrinsics_to 返回从当前 stream 到目标 stream 的外参。
    // 这里打印左红外到 gyro 的 Tbc 信息，仅用于检查，不会自动写入 ORB-SLAM3 配置。
    float* Rbc = cam_left.get_extrinsics_to(imu_stream).rotation;
    float* tbc = cam_left.get_extrinsics_to(imu_stream).translation;
    std::cout << "Tbc (left) = " << std::endl;
    for(int i = 0; i<3; i++){
        for(int j = 0; j<3; j++)
            std::cout << Rbc[i*3 + j] << ", ";
        std::cout << tbc[i] << "\n";
    }

    // 打印右红外到左红外的相对外参，便于核对双目基线和左右相机配置。
    float* Rlr = cam_right.get_extrinsics_to(cam_left).rotation;
    float* tlr = cam_right.get_extrinsics_to(cam_left).translation;
    std::cout << "Tlr  = " << std::endl;
    for(int i = 0; i<3; i++){
        for(int j = 0; j<3; j++)
            std::cout << Rlr[i*3 + j] << ", ";
        std::cout << tlr[i] << "\n";
    }



    // 读取并打印左红外内参。真正参与 ORB-SLAM3 的参数仍来自 argv[2] 指定的 YAML。
    rs2_intrinsics intrinsics_left = cam_left.as<rs2::video_stream_profile>().get_intrinsics();
    width_img = intrinsics_left.width;
    height_img = intrinsics_left.height;
    cout << "Left camera: \n";
    std::cout << " fx = " << intrinsics_left.fx << std::endl;
    std::cout << " fy = " << intrinsics_left.fy << std::endl;
    std::cout << " cx = " << intrinsics_left.ppx << std::endl;
    std::cout << " cy = " << intrinsics_left.ppy << std::endl;
    std::cout << " height = " << intrinsics_left.height << std::endl;
    std::cout << " width = " << intrinsics_left.width << std::endl;
    std::cout << " Coeff = " << intrinsics_left.coeffs[0] << ", " << intrinsics_left.coeffs[1] << ", " <<
        intrinsics_left.coeffs[2] << ", " << intrinsics_left.coeffs[3] << ", " << intrinsics_left.coeffs[4] << ", " << std::endl;
    std::cout << " Model = " << intrinsics_left.model << std::endl;

    // 读取并打印右红外内参。width_img/height_img 最终用于构造 OpenCV Mat 视图。
    rs2_intrinsics intrinsics_right = cam_right.as<rs2::video_stream_profile>().get_intrinsics();
    width_img = intrinsics_right.width;
    height_img = intrinsics_right.height;
    cout << "Right camera: \n";
    std::cout << " fx = " << intrinsics_right.fx << std::endl;
    std::cout << " fy = " << intrinsics_right.fy << std::endl;
    std::cout << " cx = " << intrinsics_right.ppx << std::endl;
    std::cout << " cy = " << intrinsics_right.ppy << std::endl;
    std::cout << " height = " << intrinsics_right.height << std::endl;
    std::cout << " width = " << intrinsics_right.width << std::endl;
    std::cout << " Coeff = " << intrinsics_right.coeffs[0] << ", " << intrinsics_right.coeffs[1] << ", " <<
        intrinsics_right.coeffs[2] << ", " << intrinsics_right.coeffs[3] << ", " << intrinsics_right.coeffs[4] << ", " << std::endl;
    std::cout << " Model = " << intrinsics_right.model << std::endl;


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // System 内部会根据 argv[2] 的 YAML 读取相机、IMU、ORB 和 viewer 配置。
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, true, 0, file_name);
    float imageScale = SLAM.GetImageScale();

    double timestamp;
    cv::Mat im, imRight;

    // Clear IMU vectors
    v_gyro_data.clear();
    v_gyro_timestamp.clear();
    v_accel_data_sync.clear();
    v_accel_timestamp_sync.clear();

    double t_resize = 0.f;
    double t_track = 0.f;

    // 主循环只负责等待一帧新双目图像，然后取走这帧之前缓存的 IMU 序列并调用 TrackStereo。
    // RealSense 数据采集在回调线程中进行，SLAM 跟踪在当前主线程中进行。
    while (!SLAM.isShutDown())
    {
        std::vector<rs2_vector> vGyro;
        std::vector<double> vGyro_times;
        std::vector<rs2_vector> vAccel;
        std::vector<double> vAccel_times;

        {
            std::unique_lock<std::mutex> lk(imu_mutex);
            if(!image_ready)
                // 等待回调线程收到新的 frameset。被唤醒后仍持锁，避免复制数据时回调继续修改缓存。
                cond_image_rec.wait(lk);

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point time_Start_Process = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point time_Start_Process = std::chrono::monotonic_clock::now();
#endif

            if(count_im_buffer>1)
                cout << count_im_buffer -1 << " dropped frs\n";
            count_im_buffer = 0;

            while(v_gyro_timestamp.size() > v_accel_timestamp_sync.size())
            {
                int index = v_accel_timestamp_sync.size();
                double target_time = v_gyro_timestamp[index];

                // 在主线程取数据前做最后一次 accel 补齐，防止图像到达时仍有 gyro 没有对应 accel。
                rs2_vector interp_data = interpolateMeasure(target_time, current_accel_data, current_accel_timestamp, prev_accel_data, prev_accel_timestamp);

                v_accel_data_sync.push_back(interp_data);
                // v_accel_data_sync.push_back(current_accel_data); // 0 interpolation
                v_accel_timestamp_sync.push_back(target_time);
            }

            // Copy the IMU data
            // 锁内复制图像和 IMU 缓存，随后清空共享缓存；锁外再做耗时的 resize 和 SLAM 跟踪。
            vGyro = v_gyro_data;
            vGyro_times = v_gyro_timestamp;
            vAccel = v_accel_data_sync;
            vAccel_times = v_accel_timestamp_sync;
            timestamp = timestamp_image;
            im = imCV.clone();
            imRight = imRightCV.clone();

            // Clear IMU vectors
            v_gyro_data.clear();
            v_gyro_timestamp.clear();
            v_accel_data_sync.clear();
            v_accel_timestamp_sync.clear();

            image_ready = false;
        }


        for(int i=0; i<vGyro.size(); ++i)
        {
            // ORB-SLAM3 的 IMU::Point 顺序为：加速度 xyz、角速度 xyz、时间戳。
            // 这里按相同下标把 gyro 与同步后的 accel 合成一个 IMU 测量点。
            ORB_SLAM3::IMU::Point lastPoint(vAccel[i].x, vAccel[i].y, vAccel[i].z,
                                  vGyro[i].x, vGyro[i].y, vGyro[i].z,
                                  vGyro_times[i]);
            vImuMeas.push_back(lastPoint);
        }

        if(imageScale != 1.f)
        {
            // 若 YAML 中配置了图像缩放，Tracking 内部期望输入图像与缩放后的内参一致。
            // 因此这里在调用 TrackStereo 前主动 resize 左右图。
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_Start_Track = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t_Start_Track = std::chrono::monotonic_clock::now();
    #endif
#endif
        // Stereo images are already rectified.
        // D435i 左右红外流由设备输出为已校正的双目图像，本例不再执行 OpenCV remap。
        // TrackStereo 返回当前帧 Tcw，但本例只依赖 System 内部状态和保存轨迹接口。
        SLAM.TrackStereo(im, imRight, timestamp, vImuMeas);
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_End_Track = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t_End_Track = std::chrono::monotonic_clock::now();
    #endif
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Track - t_Start_Track).count();
        SLAM.InsertTrackTime(t_track);
#endif



        // Clear the previous IMU measurements to load the new ones
        vImuMeas.clear();
    }
    cout << "System shutdown!\n";
}

rs2_vector interpolateMeasure(const double target_time,
                              const rs2_vector current_data, const double current_time,
                              const rs2_vector prev_data, const double prev_time)
{

    // If there are not previous information, the current data is propagated
    // 第一条 accel 到来前没有上一帧可插值，只能直接使用当前 accel。
    if(prev_time == 0)
    {
        return current_data;
    }

    rs2_vector increment;
    rs2_vector value_interp;

    if(target_time > current_time) {
        // 目标时间晚于最新 accel，只能外推为当前 accel。
        value_interp = current_data;
    }
    else if(target_time > prev_time)
    {
        // 目标时间落在上一帧和当前帧 accel 之间，先计算线性插值结果。
        increment.x = current_data.x - prev_data.x;
        increment.y = current_data.y - prev_data.y;
        increment.z = current_data.z - prev_data.z;

        double factor = (target_time - prev_time) / (current_time - prev_time);

        value_interp.x = prev_data.x + increment.x * factor;
        value_interp.y = prev_data.y + increment.y * factor;
        value_interp.z = prev_data.z + increment.z * factor;

        // zero interpolation
        // 注意：原始代码最终覆盖为 current_data，实际采用零阶保持而不是线性插值。
        value_interp = current_data;
    }
    else {
        // 目标时间早于上一帧 accel，使用上一帧 accel。
        value_interp = prev_data;
    }

    return value_interp;
}
