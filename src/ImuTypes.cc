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

#include "ImuTypes.h"
#include "Converter.h"

#include "GeometricTools.h"

#include<iostream>

namespace ORB_SLAM3
{

namespace IMU
{

// eps 用于判断“小角度”情形，避免闭式公式中的分母接近 0。
const float eps = 1e-4;

/**
 * @brief       将数值上接近旋转矩阵的矩阵重新归一化到 SO(3)
 * @param R     输入的 3x3 矩阵
 * @return      正交化后的旋转矩阵
 *
 * 预积分会不断做旋转矩阵相乘，浮点误差可能让结果偏离正交约束。
 * 这里通过 SVD 把它投影回最近的合法旋转矩阵。
 */
Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f &R){
    // 对输入矩阵做奇异值分解：R = U * S * V^T。
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // 对旋转矩阵来说，最接近它的正交矩阵可由 U * V^T 得到。
    return svd.matrixU() * svd.matrixV().transpose();
}

/**
 * @brief       计算 SO(3) 的右雅可比
 * @param x     旋转向量 x 分量
 * @param y     旋转向量 y 分量
 * @param z     旋转向量 z 分量
 * @return      右雅可比矩阵
 *
 * 右雅可比用于把李代数上的小扰动正确映射到 SO(3) 群元素更新中。
 */
Eigen::Matrix3f RightJacobianSO3(const float &x, const float &y, const float &z)
{
    // I 是闭式解中的单位阵项。
    Eigen::Matrix3f I;
    I.setIdentity();

    // d2 和 d 分别是旋转向量的模长平方与模长。
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);

    // 把三个标量分量组装成旋转向量。
    Eigen::Vector3f v;
    v << x, y, z;

    // hat(v) 把向量写成反对称矩阵形式，便于套用 Rodrigues 公式。
    Eigen::Matrix3f W = Sophus::SO3f::hat(v);

    // 在极小旋转附近，右雅可比可近似看作单位阵，数值更稳定。
    if(d<eps) {
        return I;
    }
    else {
        // SO(3) 右雅可比闭式：
        // Jr = I - (1-cos d)/d^2 * W + (d-sin d)/d^3 * W^2
        return I - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}

/**
 * @brief       计算 SO(3) 的右雅可比
 * @param v     三维旋转向量
 * @return      右雅可比矩阵
 */
Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v)
{
    // 直接复用标量重载，避免重复实现。
    return RightJacobianSO3(v(0),v(1),v(2));
}

/**
 * @brief       计算 SO(3) 右雅可比的逆矩阵
 * @param x     旋转向量 x 分量
 * @param y     旋转向量 y 分量
 * @param z     旋转向量 z 分量
 * @return      右雅可比逆矩阵
 */
Eigen::Matrix3f InverseRightJacobianSO3(const float &x, const float &y, const float &z)
{
    // 单位阵对应零旋转处的一阶近似。
    Eigen::Matrix3f I;
    I.setIdentity();

    // 旋转向量模长平方与模长。
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);

    // 构造旋转向量及其 hat 矩阵。
    Eigen::Vector3f v;
    v << x, y, z;
    Eigen::Matrix3f W = Sophus::SO3f::hat(v);

    // 小角度时直接返回单位阵近似。
    if(d<eps) {
        return I;
    }
    else {
        // SO(3) 逆右雅可比闭式公式。
        return I + W/2 + W*W*(1.0f/d2 - (1.0f+cos(d))/(2.0f*d*sin(d)));
    }
}

/**
 * @brief       计算 SO(3) 右雅可比的逆矩阵
 * @param v     三维旋转向量
 * @return      右雅可比逆矩阵
 */
Eigen::Matrix3f InverseRightJacobianSO3(const Eigen::Vector3f &v)
{
    return InverseRightJacobianSO3(v(0),v(1),v(2));
}

/**
 * @brief       对单次陀螺仪测量做旋转积分
 * @param angVel 原始角速度测量
 * @param imuBias 用于去偏的参考偏置
 * @param time   当前时间间隔
 *
 * 该构造函数会同时给出：
 * 1. 当前时间段的旋转增量 deltaR；
 * 2. 对应的右雅可比 rightJ。
 */
IntegratedRotation::IntegratedRotation(const Eigen::Vector3f &angVel, const Bias &imuBias, const float &time) {
    // 先减去陀螺仪偏置，再乘以 dt，得到该时间段的旋转向量增量。
    const float x = (angVel(0)-imuBias.bwx)*time;
    const float y = (angVel(1)-imuBias.bwy)*time;
    const float z = (angVel(2)-imuBias.bwz)*time;

    // 旋转向量模长平方与模长。
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);

    // 构造李代数向量及其 hat 矩阵表示。
    Eigen::Vector3f v;
    v << x, y, z;
    Eigen::Matrix3f W = Sophus::SO3f::hat(v);

    // 小角度时使用一阶近似，既简单又能保证数值稳定。
    if(d<eps)
    {
        deltaR = Eigen::Matrix3f::Identity() + W;
        rightJ = Eigen::Matrix3f::Identity();
    }
    else
    {
        // Rodrigues 公式给出该时间段的旋转增量矩阵。
        deltaR = Eigen::Matrix3f::Identity() + W*sin(d)/d + W*W*(1.0f-cos(d))/d2;
        // 同时计算右雅可比，供误差传播和偏置修正使用。
        rightJ = Eigen::Matrix3f::Identity() - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}

/**
 * @brief       构造预积分对象
 * @param b_    初始参考偏置
 * @param calib IMU 标定与噪声参数
 */
Preintegrated::Preintegrated(const Bias &b_, const Calib &calib)
{
    // 从标定中取出测量白噪声与偏置随机游走噪声。
    Nga = calib.Cov;
    NgaWalk = calib.CovWalk;

    // 按给定偏置把内部状态重置到“未积分”的起点。
    Initialize(b_);
}

/**
 * @brief       预积分对象拷贝构造函数
 * @param pImuPre 待拷贝的预积分对象指针
 *
 * 通过初始化列表一次性复制所有状态，避免重复赋值。
 */
Preintegrated::Preintegrated(Preintegrated* pImuPre): dT(pImuPre->dT),C(pImuPre->C), Info(pImuPre->Info),
     Nga(pImuPre->Nga), NgaWalk(pImuPre->NgaWalk), b(pImuPre->b), dR(pImuPre->dR), dV(pImuPre->dV),
    dP(pImuPre->dP), JRg(pImuPre->JRg), JVg(pImuPre->JVg), JVa(pImuPre->JVa), JPg(pImuPre->JPg), JPa(pImuPre->JPa),
    avgA(pImuPre->avgA), avgW(pImuPre->avgW), bu(pImuPre->bu), db(pImuPre->db), mvMeasurements(pImuPre->mvMeasurements)
{

}

/**
 * @brief       用另一个预积分对象覆盖当前对象
 * @param pImuPre 输入预积分对象指针
 */
void Preintegrated::CopyFrom(Preintegrated* pImuPre)
{
    // 复制时间与统计矩阵。
    dT = pImuPre->dT;
    C = pImuPre->C;
    Info = pImuPre->Info;
    Nga = pImuPre->Nga;
    NgaWalk = pImuPre->NgaWalk;

    // 复制参考偏置与主状态量。
    b.CopyFrom(pImuPre->b);
    dR = pImuPre->dR;
    dV = pImuPre->dV;
    dP = pImuPre->dP;

    // 复制对偏置的一阶雅可比。
    JRg = pImuPre->JRg;
    JVg = pImuPre->JVg;
    JVa = pImuPre->JVa;
    JPg = pImuPre->JPg;
    JPa = pImuPre->JPa;

    // 复制时间平均量、更新偏置和偏置差。
    avgA = pImuPre->avgA;
    avgW = pImuPre->avgW;
    bu.CopyFrom(pImuPre->bu);
    db = pImuPre->db;

    // 复制原始测量缓存，便于后续重积分。
    mvMeasurements = pImuPre->mvMeasurements;
}


/**
 * @brief       初始化预积分状态
 * @param b_    作为参考的偏置
 *
 * 该函数会把内部状态恢复到“尚未积分任何测量”的初始状态。
 */
void Preintegrated::Initialize(const Bias &b_)
{
    // 初始旋转增量是单位阵。
    dR.setIdentity();

    // 初始速度和位置增量为 0。
    dV.setZero();
    dP.setZero();

    // 与偏置相关的雅可比从 0 开始累积。
    JRg.setZero();
    JVg.setZero();
    JVa.setZero();
    JPg.setZero();
    JPa.setZero();

    // 误差协方差、信息矩阵和偏置差清零。
    C.setZero();
    Info.setZero();
    db.setZero();

    // 原始参考偏置与更新偏置初始时都设为同一个值。
    b=b_;
    bu=b_;

    // 平均量清零。
    avgA.setZero();
    avgW.setZero();

    // 总积分时长归零。
    dT=0.0f;

    // 清空历史测量缓存。
    mvMeasurements.clear();
}

/**
 * @brief 使用当前更新偏置重新积分全部缓存测量
 */
void Preintegrated::Reintegrate()
{
    // 加锁，确保重积分过程中内部状态不被并发访问。
    std::unique_lock<std::mutex> lock(mMutex);

    // 先保存一份测量缓存，因为 Initialize() 会清空原容器。
    const std::vector<integrable> aux = mvMeasurements;

    // 以当前更新偏置 bu 为新的参考偏置重新开始积分。
    Initialize(bu);

    // 按原始顺序重放所有 IMU 测量。
    for(size_t i=0;i<aux.size();i++)
        IntegrateNewMeasurement(aux[i].a,aux[i].w,aux[i].t);
}

/**
 * @brief       递推融合一条新的 IMU 测量
 * @param acceleration 当前时刻加速度计读数
 * @param angVel 当前时刻陀螺仪读数
 * @param dt     当前测量对应的时间间隔
 *
 * 这是预积分的核心函数，会同时更新：
 * 1. 旋转/速度/位置增量；
 * 2. 对偏置的一阶雅可比；
 * 3. 误差协方差；
 * 4. 原始测量缓存。
 */
void Preintegrated::IntegrateNewMeasurement(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &angVel, const float &dt)
{
    // 先把原始测量保存下来，后续若偏置更新可直接重放重积分。
    mvMeasurements.push_back(integrable(acceleration,angVel,dt));

    // 位置依赖前一时刻速度和旋转，所以先更新位置；
    // 速度依赖前一时刻旋转，所以第二个更新；
    // 旋转放在最后更新。

    // A/B 是离散误差状态传播矩阵：
    // x_{k+1} = A * x_k + B * n_k
    // 这里只构造前 9 维（旋转、速度、位置）的传播部分。
    Eigen::Matrix<float,9,9> A;
    A.setIdentity();
    Eigen::Matrix<float,9,6> B;
    B.setZero();

    // 去除参考偏置后的无偏加速度与无偏角速度。
    Eigen::Vector3f acc, accW;
    acc << acceleration(0)-b.bax, acceleration(1)-b.bay, acceleration(2)-b.baz;
    accW << angVel(0)-b.bwx, angVel(1)-b.bwy, angVel(2)-b.bwz;

    // 用时间加权方式维护积分窗口内的平均加速度和平均角速度。
    avgA = (dT*avgA + dR*acc*dt)/(dT+dt);
    avgW = (dT*avgW + accW*dt)/(dT+dt);

    // 使用“尚未并入本次角增量”的 dR 更新位置和速度。
    // 这与离散积分的物理顺序保持一致。
    dP = dP + dV*dt + 0.5f*dR*acc*dt*dt;
    dV = dV + dR*acc*dt;

    // Wacc = hat(acc)，用于描述姿态误差如何耦合到速度和位置误差。
    Eigen::Matrix<float,3,3> Wacc = Sophus::SO3f::hat(acc);

    // 构造 A 中与速度、位置相关的传播项。
    A.block<3,3>(3,0) = -dR*dt*Wacc;
    A.block<3,3>(6,0) = -0.5f*dR*dt*dt*Wacc;
    // 位置误差还直接积累上一时刻的速度误差。
    A.block<3,3>(6,3) = Eigen::DiagonalMatrix<float,3>(dt, dt, dt);

    // 构造 B 中与加速度白噪声相关的注入项。
    B.block<3,3>(3,3) = dR*dt;
    B.block<3,3>(6,3) = 0.5f*dR*dt*dt;

    // 更新位置和速度对偏置的雅可比。
    // JPa/JPg 对应位置对加速度偏置/陀螺仪偏置的敏感度；
    // JVa/JVg 对应速度对加速度偏置/陀螺仪偏置的敏感度。
    JPa = JPa + JVa*dt -0.5f*dR*dt*dt;
    JPg = JPg + JVg*dt -0.5f*dR*dt*dt*Wacc*JRg;
    JVa = JVa - dR*dt;
    JVg = JVg - dR*dt*Wacc*JRg;

    // 把当前时间段的角速度测量积分成一个小旋转。
    IntegratedRotation dRi(angVel,b,dt);

    // 将小旋转右乘到累计旋转上，再投影回 SO(3)。
    dR = NormalizeRotation(dR*dRi.deltaR);

    // 补齐 A/B 中与旋转误差和陀螺仪噪声相关的块。
    A.block<3,3>(0,0) = dRi.deltaR.transpose();
    B.block<3,3>(0,0) = dRi.rightJ*dt;

    // 协方差离散递推：
    // C_{k+1} = A * C_k * A^T + B * Nga * B^T
    C.block<9,9>(0,0) = A * C.block<9,9>(0,0) * A.transpose() + B*Nga*B.transpose();

    // 后 6 维对应偏置随机游走，按时间步独立累加噪声。
    C.block<6,6>(9,9) += NgaWalk;

    // 更新旋转对陀螺仪偏置的雅可比。
    JRg = dRi.deltaR.transpose()*JRg - dRi.rightJ*dt;

    // 最后累加总积分时长。
    dT += dt;
}

/**
 * @brief       将上一段预积分拼接到当前对象前面
 * @param pPrev 待合并的上一段预积分
 */
void Preintegrated::MergePrevious(Preintegrated* pPrev)
{
    // 若传入自身则无需处理。
    if (pPrev==this)
        return;

    // 同时锁住两个对象，避免合并过程中缓存被并发修改。
    std::unique_lock<std::mutex> lock1(mMutex);
    std::unique_lock<std::mutex> lock2(pPrev->mMutex);

    // 以当前对象的更新偏置作为合并后的统一参考偏置。
    Bias bav;
    bav.bwx = bu.bwx;
    bav.bwy = bu.bwy;
    bav.bwz = bu.bwz;
    bav.bax = bu.bax;
    bav.bay = bu.bay;
    bav.baz = bu.baz;

    // 分别保存“上一段”和“当前段”的原始测量缓存。
    const std::vector<integrable > aux1 = pPrev->mvMeasurements;
    const std::vector<integrable> aux2 = mvMeasurements;

    // 重置状态，然后按时间顺序依次重放两段测量。
    Initialize(bav);
    for(size_t i=0;i<aux1.size();i++)
        IntegrateNewMeasurement(aux1[i].a,aux1[i].w,aux1[i].t);
    for(size_t i=0;i<aux2.size();i++)
        IntegrateNewMeasurement(aux2[i].a,aux2[i].w,aux2[i].t);

}

/**
 * @brief       设置新的偏置估计
 * @param bu_   更新后的偏置
 *
 * 这里只记录偏置差 db，不立即重积分。
 * 后续可通过雅可比对预积分结果做一阶修正。
 */
void Preintegrated::SetNewBias(const Bias &bu_)
{
    std::unique_lock<std::mutex> lock(mMutex);

    // 保存更新偏置。
    bu = bu_;

    // db 的前 3 维是陀螺仪偏置差，后 3 维是加速度偏置差。
    db(0) = bu_.bwx-b.bwx;
    db(1) = bu_.bwy-b.bwy;
    db(2) = bu_.bwz-b.bwz;
    db(3) = bu_.bax-b.bax;
    db(4) = bu_.bay-b.bay;
    db(5) = bu_.baz-b.baz;
}

/**
 * @brief       获取输入偏置相对于参考偏置的差值
 * @param b_    目标偏置
 * @return      Bias 形式的偏置差
 */
IMU::Bias Preintegrated::GetDeltaBias(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);

    // 返回“目标偏置 - 原始参考偏置”。
    return IMU::Bias(b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz,b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz);
}


/**
 * @brief       查询给定偏置下的一阶修正旋转增量
 * @param b_    目标偏置
 * @return      修正后的旋转增量
 */
Eigen::Matrix3f Preintegrated::GetDeltaRotation(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);

    // 旋转只与陀螺仪偏置修正有关。
    Eigen::Vector3f dbg;
    dbg << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz;

    // 对原始旋转增量右乘一个由偏置差诱导的小旋转修正。
    return NormalizeRotation(dR * Sophus::SO3f::exp(JRg * dbg).matrix());
}

/**
 * @brief       查询给定偏置下的一阶修正速度增量
 * @param b_    目标偏置
 * @return      修正后的速度增量
 */
Eigen::Vector3f Preintegrated::GetDeltaVelocity(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);

    // 速度同时受陀螺仪偏置和加速度偏置影响。
    Eigen::Vector3f dbg, dba;
    dbg << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz;
    dba << b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz;

    // 采用一阶线性修正：dV' = dV + JVg*dbg + JVa*dba。
    return dV + JVg * dbg + JVa * dba;
}

/**
 * @brief       查询给定偏置下的一阶修正位置增量
 * @param b_    目标偏置
 * @return      修正后的位置增量
 */
Eigen::Vector3f Preintegrated::GetDeltaPosition(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);

    // 位置同样同时依赖两类偏置。
    Eigen::Vector3f dbg, dba;
    dbg << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz;
    dba << b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz;

    // 采用一阶线性修正：dP' = dP + JPg*dbg + JPa*dba。
    return dP + JPg * dbg + JPa * dba;
}

/**
 * @brief   获取当前更新偏置 bu 对应的旋转增量
 * @return  使用 db 做一阶修正后的旋转增量
 */
Eigen::Matrix3f Preintegrated::GetUpdatedDeltaRotation()
{
    std::unique_lock<std::mutex> lock(mMutex);

    // db.head(3) 即当前记录的陀螺仪偏置差。
    return NormalizeRotation(dR * Sophus::SO3f::exp(JRg*db.head(3)).matrix());
}

/**
 * @brief   获取当前更新偏置 bu 对应的速度增量
 * @return  使用 db 做一阶修正后的速度增量
 */
Eigen::Vector3f Preintegrated::GetUpdatedDeltaVelocity()
{
    std::unique_lock<std::mutex> lock(mMutex);

    // 前 3 维修正陀螺仪偏置，后 3 维修正加速度偏置。
    return dV + JVg * db.head(3) + JVa * db.tail(3);
}

/**
 * @brief   获取当前更新偏置 bu 对应的位置增量
 * @return  使用 db 做一阶修正后的位置增量
 */
Eigen::Vector3f Preintegrated::GetUpdatedDeltaPosition()
{
    std::unique_lock<std::mutex> lock(mMutex);

    // 位置修正形式与速度修正类似，只是使用位置雅可比。
    return dP + JPg*db.head(3) + JPa*db.tail(3);
}

/**
 * @brief   获取原始旋转预积分结果
 * @return  以参考偏置 b 计算得到的 dR
 */
Eigen::Matrix3f Preintegrated::GetOriginalDeltaRotation() {
    std::unique_lock<std::mutex> lock(mMutex);
    return dR;
}

/**
 * @brief   获取原始速度预积分结果
 * @return  以参考偏置 b 计算得到的 dV
 */
Eigen::Vector3f Preintegrated::GetOriginalDeltaVelocity() {
    std::unique_lock<std::mutex> lock(mMutex);
    return dV;
}

/**
 * @brief   获取原始位置预积分结果
 * @return  以参考偏置 b 计算得到的 dP
 */
Eigen::Vector3f Preintegrated::GetOriginalDeltaPosition()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dP;
}

/**
 * @brief   获取原始参考偏置
 * @return  预积分建立时使用的偏置 b
 */
Bias Preintegrated::GetOriginalBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return b;
}

/**
 * @brief   获取更新偏置
 * @return  当前记录的更新偏置 bu
 */
Bias Preintegrated::GetUpdatedBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return bu;
}

/**
 * @brief   获取当前记录的六维偏置差
 * @return  六维偏置差向量
 */
Eigen::Matrix<float,6,1> Preintegrated::GetDeltaBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return db;
}

/**
 * @brief       从另一个 Bias 对象复制偏置值
 * @param b     输入偏置对象
 */
void Bias::CopyFrom(Bias &b)
{
    // 六个偏置量逐项复制，保持语义一一对应。
    bax = b.bax;
    bay = b.bay;
    baz = b.baz;
    bwx = b.bwx;
    bwy = b.bwy;
    bwz = b.bwz;
}

/**
 * @brief       将 Bias 以文本形式输出到流中
 * @param out   输出流
 * @param b     待输出的偏置
 * @return      输出流本身
 *
 * 输出顺序为：
 * bwx, bwy, bwz, bax, bay, baz
 */
std::ostream& operator<< (std::ostream &out, const Bias &b)
{
    // 正数前补空格，便于日志中正负号对齐观察。
    if(b.bwx>0)
        out << " ";
    out << b.bwx << ",";
    if(b.bwy>0)
        out << " ";
    out << b.bwy << ",";
    if(b.bwz>0)
        out << " ";
    out << b.bwz << ",";
    if(b.bax>0)
        out << " ";
    out << b.bax << ",";
    if(b.bay>0)
        out << " ";
    out << b.bay << ",";
    if(b.baz>0)
        out << " ";
    out << b.baz;

    return out;
}

/**
 * @brief       设置 IMU 标定参数
 * @param sophTbc 从相机坐标系到 IMU 坐标系的位姿变换
 * @param ng      陀螺仪白噪声标准差
 * @param na      加速度计白噪声标准差
 * @param ngw     陀螺仪随机游走标准差
 * @param naw     加速度计随机游走标准差
 */
void Calib::Set(const Sophus::SE3<float> &sophTbc, const float &ng, const float &na, const float &ngw, const float &naw) {
    // 标记当前标定对象已经被正确设置。
    mbIsSet = true;

    // 预积分中实际使用的是方差，因此先对标准差平方。
    const float ng2 = ng*ng;
    const float na2 = na*na;
    const float ngw2 = ngw*ngw;
    const float naw2 = naw*naw;

    // 保存 Tbc，并同步计算其逆变换 Tcb。
    mTbc = sophTbc;
    mTcb = mTbc.inverse();

    // 构造测量白噪声和偏置随机游走噪声的对角协方差矩阵。
    Cov.diagonal() << ng2, ng2, ng2, na2, na2, na2;
    CovWalk.diagonal() << ngw2, ngw2, ngw2, naw2, naw2, naw2;
}

/**
 * @brief       IMU 标定参数拷贝构造函数
 * @param calib 输入标定对象
 */
Calib::Calib(const Calib &calib)
{
    // 复制初始化标记。
    mbIsSet = calib.mbIsSet;

    // 复制外参和噪声参数。
    mTbc = calib.mTbc;
    mTcb = calib.mTcb;
    Cov = calib.Cov;
    CovWalk = calib.CovWalk;
}

} //namespace IMU

} //namespace ORB_SLAM3
