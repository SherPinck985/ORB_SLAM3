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


#ifndef IMUTYPES_H
#define IMUTYPES_H

#include <vector>
#include <utility>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <mutex>

#include "SerializationUtils.h"

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

namespace ORB_SLAM3
{

namespace IMU
{

/**
 * @brief 重力加速度常量
 *
 * 在视觉惯性优化和状态传播时，系统默认使用该常量作为重力模长。
 * 单位为 m/s^2。
 */
const float GRAVITY_VALUE=9.81;

/**
 * @brief 单次 IMU 采样
 *
 * 封装一帧 IMU 观测中的加速度、角速度和时间戳。
 * 这是预积分模块最基础的输入数据单元。
 */
class Point
{
public:
    /**
     * @brief       由标量构造一条 IMU 测量
     * @param acc_x x 轴加速度
     * @param acc_y y 轴加速度
     * @param acc_z z 轴加速度
     * @param ang_vel_x x 轴角速度
     * @param ang_vel_y y 轴角速度
     * @param ang_vel_z z 轴角速度
     * @param timestamp 该条测量对应的时间戳
     */
    Point(const float &acc_x, const float &acc_y, const float &acc_z,
             const float &ang_vel_x, const float &ang_vel_y, const float &ang_vel_z,
             const double &timestamp): a(acc_x,acc_y,acc_z), w(ang_vel_x,ang_vel_y,ang_vel_z), t(timestamp){}
    /**
     * @brief       由 OpenCV 三维点构造一条 IMU 测量
     * @param Acc   三轴加速度
     * @param Gyro  三轴角速度
     * @param timestamp 该条测量对应的时间戳
     */
    Point(const cv::Point3f Acc, const cv::Point3f Gyro, const double &timestamp):
        a(Acc.x,Acc.y,Acc.z), w(Gyro.x,Gyro.y,Gyro.z), t(timestamp){}
public:
    /// 三轴加速度测量值
    Eigen::Vector3f a;
    /// 三轴角速度测量值
    Eigen::Vector3f w;
    /// 时间戳
    double t;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief IMU 偏置
 *
 * 该结构同时保存加速度计偏置和陀螺仪偏置。
 * 在预积分中，所有原始测量都会先减去对应偏置后再参与积分。
 */
class Bias
{
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        // 序列化顺序必须与反序列化顺序一致，依次保存加速度偏置和陀螺仪偏置。
        ar & bax;
        ar & bay;
        ar & baz;

        ar & bwx;
        ar & bwy;
        ar & bwz;
    }

public:
    /**
     * @brief 默认构造函数
     *
     * 将所有偏置初始化为 0，表示默认无偏置。
     */
    Bias():bax(0),bay(0),baz(0),bwx(0),bwy(0),bwz(0){}
    /**
     * @brief       用给定的六维偏置量构造 Bias
     * @param b_acc_x x 轴加速度偏置
     * @param b_acc_y y 轴加速度偏置
     * @param b_acc_z z 轴加速度偏置
     * @param b_ang_vel_x x 轴角速度偏置
     * @param b_ang_vel_y y 轴角速度偏置
     * @param b_ang_vel_z z 轴角速度偏置
     */
    Bias(const float &b_acc_x, const float &b_acc_y, const float &b_acc_z,
            const float &b_ang_vel_x, const float &b_ang_vel_y, const float &b_ang_vel_z):
            bax(b_acc_x), bay(b_acc_y), baz(b_acc_z), bwx(b_ang_vel_x), bwy(b_ang_vel_y), bwz(b_ang_vel_z){}
    /**
     * @brief   从另一个 Bias 对象复制六个偏置量
     * @param b 输入偏置对象
     */
    void CopyFrom(Bias &b);
    /**
     * @brief       以可读形式输出偏置
     * @param out   输出流
     * @param b     待输出的偏置
     * @return      输出流本身，便于链式输出
     */
    friend std::ostream& operator<< (std::ostream &out, const Bias &b);

public:
    /// 加速度计偏置 x/y/z
    float bax, bay, baz;
    /// 陀螺仪偏置 x/y/z
    float bwx, bwy, bwz;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief IMU 标定参数
 *
 * 该类保存相机与 IMU 的外参，以及连续时间噪声和随机游走噪声模型。
 * 这些参数会被预积分过程用于构建测量噪声协方差。
 */
class Calib
{
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        // 保存 IMU 与相机之间的双向位姿变换。
        serializeSophusSE3(ar,mTcb,version);
        serializeSophusSE3(ar,mTbc,version);

        // 仅保存对角项，因为这里假设噪声协方差矩阵是对角阵。
        ar & boost::serialization::make_array(Cov.diagonal().data(), Cov.diagonal().size());
        ar & boost::serialization::make_array(CovWalk.diagonal().data(), CovWalk.diagonal().size());

        // 保存当前标定参数是否已经正确初始化。
        ar & mbIsSet;
    }

public:

    /**
     * @brief       用外参和噪声参数构造标定对象
     * @param Tbc   从相机坐标系到 IMU 坐标系的位姿变换
     * @param ng    陀螺仪白噪声标准差
     * @param na    加速度计白噪声标准差
     * @param ngw   陀螺仪随机游走标准差
     * @param naw   加速度计随机游走标准差
     */
    Calib(const Sophus::SE3<float> &Tbc, const float &ng, const float &na, const float &ngw, const float &naw)
    {
        Set(Tbc,ng,na,ngw,naw);
    }

    /**
     * @brief   拷贝构造函数
     * @param calib 输入标定对象
     */
    Calib(const Calib &calib);
    /**
     * @brief 默认构造函数
     *
     * 默认状态下不包含有效标定，mbIsSet 为 false。
     */
    Calib(){mbIsSet = false;}

    //void Set(const cv::Mat &cvTbc, const float &ng, const float &na, const float &ngw, const float &naw);
    /**
     * @brief       设置 IMU 标定参数
     * @param sophTbc 从相机坐标系到 IMU 坐标系的位姿变换
     * @param ng      陀螺仪白噪声标准差
     * @param na      加速度计白噪声标准差
     * @param ngw     陀螺仪随机游走标准差
     * @param naw     加速度计随机游走标准差
     *
     * 该函数会同时计算逆变换 Tcb，并把输入的标准差平方成协方差对角项。
     */
    void Set(const Sophus::SE3<float> &sophTbc, const float &ng, const float &na, const float &ngw, const float &naw);

public:
    // Sophus/Eigen implementation
    /// 从 IMU 坐标系到相机坐标系的位姿变换
    Sophus::SE3<float> mTcb;
    /// 从相机坐标系到 IMU 坐标系的位姿变换
    Sophus::SE3<float> mTbc;
    /// Cov 为测量白噪声协方差，CovWalk 为随机游走协方差；顺序均为陀螺仪三维 + 加速度计三维
    Eigen::DiagonalMatrix<float,6> Cov, CovWalk;
    /// 标记当前对象是否已经完成初始化
    bool mbIsSet;
};

/**
 * @brief 单个时间间隔内的旋转积分结果
 *
 * 给定一段去偏后的角速度和积分时间，构造该时间段的旋转增量，
 * 并同时计算右雅可比，供后续偏置修正和协方差传播使用。
 */
class IntegratedRotation
{
public:
    /**
     * @brief 默认构造函数
     */
    IntegratedRotation(){}
    /**
     * @brief       对单次陀螺仪测量做旋转积分
     * @param angVel 当前时刻的角速度测量
     * @param imuBias 参与去偏的 IMU 偏置
     * @param time   当前测量对应的积分时间
     */
    IntegratedRotation(const Eigen::Vector3f &angVel, const Bias &imuBias, const float &time);

public:
    /// 本次积分对应的时间长度
    float deltaT; //integration time
    /// 积分得到的旋转增量矩阵
    Eigen::Matrix3f deltaR;
    /// SO(3) 上的右雅可比，用于线性化误差传播
    Eigen::Matrix3f rightJ; // right jacobian
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief IMU 预积分容器
 *
 * 该类负责缓存两帧之间的多条 IMU 测量，并在给定参考偏置下
 * 递推得到旋转、速度、位置增量以及它们对偏置的雅可比和协方差。
 */
class Preintegrated
{
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        // 保存总积分时长和误差协方差。
        ar & dT;
        ar & boost::serialization::make_array(C.data(), C.size());
        ar & boost::serialization::make_array(Info.data(), Info.size());
        ar & boost::serialization::make_array(Nga.diagonal().data(), Nga.diagonal().size());
        ar & boost::serialization::make_array(NgaWalk.diagonal().data(), NgaWalk.diagonal().size());
        // 保存参考偏置和预积分状态量。
        ar & b;
        ar & boost::serialization::make_array(dR.data(), dR.size());
        ar & boost::serialization::make_array(dV.data(), dV.size());
        ar & boost::serialization::make_array(dP.data(), dP.size());
        ar & boost::serialization::make_array(JRg.data(), JRg.size());
        ar & boost::serialization::make_array(JVg.data(), JVg.size());
        ar & boost::serialization::make_array(JVa.data(), JVa.size());
        ar & boost::serialization::make_array(JPg.data(), JPg.size());
        ar & boost::serialization::make_array(JPa.data(), JPa.size());
        ar & boost::serialization::make_array(avgA.data(), avgA.size());
        ar & boost::serialization::make_array(avgW.data(), avgW.size());

        // 保存更新后的偏置、偏置增量，以及原始 IMU 测量缓存。
        ar & bu;
        ar & boost::serialization::make_array(db.data(), db.size());
        ar & mvMeasurements;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * @brief       构造一个预积分对象
     * @param b_    初始参考偏置
     * @param calib IMU 标定与噪声参数
     */
    Preintegrated(const Bias &b_, const Calib &calib);
    /**
     * @brief       拷贝构造函数
     * @param pImuPre 待拷贝的预积分对象指针
     */
    Preintegrated(Preintegrated* pImuPre);
    /**
     * @brief 默认构造函数
     */
    Preintegrated() {}
    /**
     * @brief 析构函数
     */
    ~Preintegrated() {}
    /**
     * @brief       从另一个预积分对象复制所有状态
     * @param pImuPre 输入的预积分对象指针
     */
    void CopyFrom(Preintegrated* pImuPre);
    /**
     * @brief       用指定偏置重置预积分状态
     * @param b_    重置后作为参考的偏置
     *
     * 该函数会把旋转、速度、位置增量清零/置单位阵，并清空测量缓存。
     */
    void Initialize(const Bias &b_);
    /**
     * @brief       融合一条新的 IMU 测量
     * @param acceleration 当前时刻加速度计读数
     * @param angVel 当前时刻陀螺仪读数
     * @param dt     当前测量对应的时间间隔
     *
     * 该函数是 IMU 预积分的核心递推入口，会同步更新增量、雅可比、协方差和均值。
     */
    void IntegrateNewMeasurement(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &angVel, const float &dt);
    /**
     * @brief 依据当前更新偏置重新积分全部缓存测量
     *
     * 当偏置估计发生明显变化时，通过重放缓存测量得到新的基准预积分结果。
     */
    void Reintegrate();
    /**
     * @brief       将另一段预积分之前的测量拼接到当前对象前面
     * @param pPrev 需要合并的上一段预积分
     */
    void MergePrevious(Preintegrated* pPrev);
    /**
     * @brief       设置新的偏置估计
     * @param bu_   更新后的偏置
     *
     * 该函数并不立即重新积分，而是只记录偏置差，供一阶修正使用。
     */
    void SetNewBias(const Bias &bu_);
    /**
     * @brief       获取输入偏置相对于参考偏置的差值
     * @param b_    需要比较的偏置
     * @return      六维偏置差，按加速度偏置和角速度偏置组织
     */
    IMU::Bias GetDeltaBias(const Bias &b_);

    /**
     * @brief       在给定偏置下查询修正后的旋转增量
     * @param b_    目标偏置
     * @return      经过一阶偏置修正后的旋转增量
     */
    Eigen::Matrix3f GetDeltaRotation(const Bias &b_);
    /**
     * @brief       在给定偏置下查询修正后的速度增量
     * @param b_    目标偏置
     * @return      经过一阶偏置修正后的速度增量
     */
    Eigen::Vector3f GetDeltaVelocity(const Bias &b_);
    /**
     * @brief       在给定偏置下查询修正后的位置增量
     * @param b_    目标偏置
     * @return      经过一阶偏置修正后的位置增量
     */
    Eigen::Vector3f GetDeltaPosition(const Bias &b_);

    /**
     * @brief   获取当前更新偏置 bu 对应的旋转增量
     * @return  使用 db 做一阶修正后的旋转增量
     */
    Eigen::Matrix3f GetUpdatedDeltaRotation();
    /**
     * @brief   获取当前更新偏置 bu 对应的速度增量
     * @return  使用 db 做一阶修正后的速度增量
     */
    Eigen::Vector3f GetUpdatedDeltaVelocity();
    /**
     * @brief   获取当前更新偏置 bu 对应的位置增量
     * @return  使用 db 做一阶修正后的位置增量
     */
    Eigen::Vector3f GetUpdatedDeltaPosition();

    /**
     * @brief   获取以原始参考偏置 b 计算得到的旋转增量
     * @return  原始旋转预积分结果
     */
    Eigen::Matrix3f GetOriginalDeltaRotation();
    /**
     * @brief   获取以原始参考偏置 b 计算得到的速度增量
     * @return  原始速度预积分结果
     */
    Eigen::Vector3f GetOriginalDeltaVelocity();
    /**
     * @brief   获取以原始参考偏置 b 计算得到的位置增量
     * @return  原始位置预积分结果
     */
    Eigen::Vector3f GetOriginalDeltaPosition();

    /**
     * @brief   获取当前记录的六维偏置差
     * @return  六维向量，前 3 维为陀螺仪偏置差，后 3 维为加速度偏置差
     */
    Eigen::Matrix<float,6,1> GetDeltaBias();

    /**
     * @brief   获取原始参考偏置
     * @return  预积分建立时使用的偏置
     */
    Bias GetOriginalBias();
    /**
     * @brief   获取更新后的偏置
     * @return  当前用于修正的偏置
     */
    Bias GetUpdatedBias();

    /**
     * @brief 打印缓存的 IMU 测量时间间隔
     *
     * 这是一个调试辅助函数，用于快速查看缓存中的积分片段个数和时长。
     */
    void printMeasurements() const {
        std::cout << "pint meas:\n";
        // 逐条输出每个积分片段的时间长度 t。
        for(int i=0; i<mvMeasurements.size(); i++){
            std::cout << "meas " << mvMeasurements[i].t << std::endl;
        }
        std::cout << "end pint meas:\n";
    }

public:
    /// 总积分时长
    float dT;
    /// 15 维误差状态协方差矩阵
    Eigen::Matrix<float,15,15> C;
    /// 信息矩阵，部分流程中会在外部基于协方差进一步计算
    Eigen::Matrix<float,15,15> Info;
    /// 连续时间测量噪声协方差与随机游走噪声协方差
    Eigen::DiagonalMatrix<float,6> Nga, NgaWalk;

    // Values for the original bias (when integration was computed)
    /// 建立当前预积分时采用的参考偏置
    Bias b;
    /// 旋转、速度、位置预积分量
    Eigen::Matrix3f dR;
    Eigen::Vector3f dV, dP;
    /// 预积分量对偏置的一阶雅可比
    Eigen::Matrix3f JRg, JVg, JVa, JPg, JPa;
    /// 积分窗口内平均加速度与平均角速度
    Eigen::Vector3f avgA, avgW;


private:
    // Updated bias
    /// 当前用于一阶修正的更新偏置
    Bias bu;
    // Dif between original and updated bias
    // This is used to compute the updated values of the preintegration
    /// 六维偏置差，前 3 维为陀螺仪偏置差，后 3 维为加速度偏置差
    Eigen::Matrix<float,6,1> db;

    /**
     * @brief 可积分的单条 IMU 测量缓存结构
     *
     * 与 Point 不同，这里缓存的是预积分真正使用的输入单元：
     * 三轴加速度、三轴角速度和对应的时间间隔 dt。
     */
    struct integrable
    {
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            // 直接按原始数值顺序保存，便于后续重新积分。
            ar & boost::serialization::make_array(a.data(), a.size());
            ar & boost::serialization::make_array(w.data(), w.size());
            ar & t;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /**
         * @brief 默认构造函数
         */
        integrable(){}
        /**
         * @brief       构造一条可重放的 IMU 测量
         * @param a_    三轴加速度
         * @param w_    三轴角速度
         * @param t_    当前采样对应的时间间隔 dt
         */
        integrable(const Eigen::Vector3f &a_, const Eigen::Vector3f &w_ , const float &t_):a(a_),w(w_),t(t_){}
        /// 当前测量的加速度和角速度
        Eigen::Vector3f a, w;
        /// 该测量对应的积分时间间隔
        float t;
    };

    /// 用于重积分与合并操作的原始 IMU 测量缓存
    std::vector<integrable> mvMeasurements;

    /// 保护预积分状态的互斥锁，防止多线程读写冲突
    std::mutex mMutex;
};

/**
 * @brief       计算 SO(3) 的右雅可比
 * @param x     旋转向量 x 分量
 * @param y     旋转向量 y 分量
 * @param z     旋转向量 z 分量
 * @return      右雅可比矩阵
 */
Eigen::Matrix3f RightJacobianSO3(const float &x, const float &y, const float &z);
/**
 * @brief       计算 SO(3) 的右雅可比
 * @param v     三维旋转向量
 * @return      右雅可比矩阵
 */
Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v);

/**
 * @brief       计算 SO(3) 右雅可比的逆矩阵
 * @param x     旋转向量 x 分量
 * @param y     旋转向量 y 分量
 * @param z     旋转向量 z 分量
 * @return      右雅可比逆矩阵
 */
Eigen::Matrix3f InverseRightJacobianSO3(const float &x, const float &y, const float &z);
/**
 * @brief       计算 SO(3) 右雅可比的逆矩阵
 * @param v     三维旋转向量
 * @return      右雅可比逆矩阵
 */
Eigen::Matrix3f InverseRightJacobianSO3(const Eigen::Vector3f &v);

/**
 * @brief       将接近旋转矩阵的 3x3 矩阵重新投影到 SO(3)
 * @param R     输入矩阵
 * @return      经过正交化后的旋转矩阵
 *
 * 数值积分和线性化过程中，矩阵可能因为浮点误差偏离正交约束。
 * 该函数通过 SVD 投影恢复合法旋转矩阵。
 */
Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f &R);

}

} //namespace ORB_SLAM2

#endif // IMUTYPES_H
