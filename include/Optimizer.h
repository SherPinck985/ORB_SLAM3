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


#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include <math.h>

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

namespace ORB_SLAM3
{

class LoopClosing;

/**
 * @brief ORB-SLAM3 中统一封装各类图优化入口的工具类
 *
 * 该类本身不负责 ORB 特征提取。ORB 特征的提取、金字塔构建、方向计算和描述子生成
 * 主要发生在前端的 Frame / KeyFrame 构造与 ORBextractor 中；Optimizer 接收前端已经
 * 建立好的关键点观测、MapPoint 关联、金字塔层尺度信息以及 IMU 预积分结果，并基于
 * 这些数据构建 g2o 图，完成位姿优化、局部/全局 BA、Sim3 位姿图优化以及惯性优化。
 */
class Optimizer
{
public:

    /**
     * @brief       对给定关键帧集合和地图点集合执行标准 BA
     * @param vpKF  参与优化的关键帧集合
     * @param vpMP  参与优化的地图点集合
     * @param nIterations 高斯牛顿/LM 迭代次数
     * @param pbStopFlag  外部停止标志，若非空则优化过程中可被提前终止
     * @param nLoopKF     当前 BA 所属的回环关键帧 id，0 表示普通 BA
     * @param bRobust     是否对重投影边启用 Huber 鲁棒核
     * @return      无
     */
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
    /**
     * @brief       对整张地图执行全局 BA
     * @param pMap  待优化的地图
     * @param nIterations 迭代次数
     * @param pbStopFlag  外部停止标志
     * @param nLoopKF     当前回环关键帧 id，0 表示普通全局 BA
     * @param bRobust     是否启用鲁棒核
     * @return      无
     */
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    /**
     * @brief       执行包含视觉和 IMU 变量的全量惯性 BA
     * @param pMap  待优化地图
     * @param its   迭代次数
     * @param bFixLocal 是否固定最近的局部窗口，仅优化更早状态
     * @param nLoopKF   回环优化编号，0 表示直接回写到地图
     * @param pbStopFlag 外部停止标志
     * @param bInit     是否处于惯性初始化阶段，此时使用全局共享 bias 顶点
     * @param priorG    陀螺仪 bias 先验信息强度
     * @param priorA    加速度计 bias 先验信息强度
     * @param vSingVal  可选输出，保存某些初始化/调试阶段使用的奇异值信息
     * @param bHess     可选输出，标记 Hessian 条件是否满足要求
     * @return      无
     */
    void static FullInertialBA(Map *pMap, int its, const bool bFixLocal=false, const unsigned long nLoopKF=0, bool *pbStopFlag=NULL, bool bInit=false, float priorG = 1e2, float priorA=1e6, Eigen::VectorXd *vSingVal = NULL, bool *bHess=NULL);

    /**
     * @brief       围绕当前关键帧执行局部 BA
     * @param pKF   当前关键帧，也是局部窗口中心
     * @param pbStopFlag 外部停止标志
     * @param pMap  当前地图
     * @param num_fixedKF 输出，固定关键帧数量
     * @param num_OptKF   输出，可优化关键帧数量
     * @param num_MPs     输出，参与优化的地图点数量
     * @param num_edges   输出，图中加入的观测边数量
     * @return      无
     */
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges);

    /**
     * @brief       仅优化当前帧位姿，不调整地图点和关键帧
     * @param pFrame 当前帧
     * @return      优化后内点匹配数量
     */
    int static PoseOptimization(Frame* pFrame);
    /**
     * @brief       利用上一关键帧的 IMU 约束与视觉观测，优化当前帧的惯性状态
     * @param pFrame    当前帧
     * @param bRecInit  是否处于重新初始化阶段，决定外点恢复策略
     * @return      优化后保留的有效匹配数量
     */
    int static PoseInertialOptimizationLastKeyFrame(Frame* pFrame, bool bRecInit = false);
    /**
     * @brief       利用上一帧的先验和 IMU 约束，优化当前帧的惯性状态
     * @param pFrame    当前帧
     * @param bRecInit  是否处于重新初始化阶段
     * @return      优化后保留的有效匹配数量
     */
    int static PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit = false);

    /**
     * @brief       回环闭合后，对整张地图执行 Essential Graph / Sim3 位姿图优化
     * @param pMap  当前地图
     * @param pLoopKF 回环匹配到的关键帧
     * @param pCurKF  当前触发回环的关键帧
     * @param NonCorrectedSim3 回环校正前的关键帧 Sim3
     * @param CorrectedSim3    回环校正后的关键帧 Sim3
     * @param LoopConnections  本次新增回环边
     * @param bFixScale 若为 true 则固定尺度，只做 6DoF；否则优化完整 Sim3
     * @return      无
     */
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);
    /**
     * @brief       地图合并后，对参与合并的关键帧子图执行 Essential Graph 优化
     * @param pCurKF 当前关键帧
     * @param vpFixedKFs          已在融合地图中固定的关键帧
     * @param vpFixedCorrectedKFs 已纠正并固定的旧地图关键帧
     * @param vpNonFixedKFs       仍可优化的关键帧
     * @param vpNonCorrectedMPs   需要跟随位姿更新的地图点
     * @return      无
     */
    void static OptimizeEssentialGraph(KeyFrame* pCurKF, vector<KeyFrame*> &vpFixedKFs, vector<KeyFrame*> &vpFixedCorrectedKFs,
                                       vector<KeyFrame*> &vpNonFixedKFs, vector<MapPoint*> &vpNonCorrectedMPs);

    /**
     * @brief       针对惯性系统回环，执行仅 4DoF 的位姿图优化
     * @param pMap  当前地图
     * @param pLoopKF 回环关键帧
     * @param pCurKF  当前关键帧
     * @param NonCorrectedSim3 校正前姿态
     * @param CorrectedSim3    校正后姿态
     * @param LoopConnections  新建立的回环连接
     * @return      无
     */
    void static OptimizeEssentialGraph4DoF(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections);


    /**
     * @brief       在两帧关键帧之间优化 Sim3 / SE3 关系
     * @param pKF1  第一帧关键帧
     * @param pKF2  第二帧关键帧
     * @param vpMatches1 从 pKF1 到 pKF2 的匹配地图点，函数内部会剔除外点
     * @param g2oS12 输入输出参数，既作为初值，也返回优化后的 Sim3
     * @param th2   卡方阈值
     * @param bFixScale 是否固定尺度；双目/RGB-D 常为 true，单目常为 false
     * @param mAcumHessian 累积 Hessian 输出接口
     * @param bAllPoints   是否连 pKF2 中不存在直接观测的点也参与优化
     * @return      最终保留下来的内点匹配数量
     */
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale,
                            Eigen::Matrix<double,7,7> &mAcumHessian, const bool bAllPoints=false);

    /**
     * @brief       围绕当前关键帧执行局部惯性 BA
     * @param pKF   当前关键帧
     * @param pbStopFlag 外部停止标志
     * @param pMap  当前地图
     * @param num_fixedKF 输出，固定关键帧数量
     * @param num_OptKF   输出，可优化关键帧数量
     * @param num_MPs     输出，参与优化的地图点数量
     * @param num_edges   输出，边数量
     * @param bLarge      是否扩大时间窗口
     * @param bRecInit    是否处于惯性重初始化阶段
     * @return      无
     */
    void static LocalInertialBA(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges, bool bLarge = false, bool bRecInit = false);
    /**
     * @brief       两张地图融合后，在合并区域执行惯性 BA
     * @param pCurrKF   当前地图中的关键帧
     * @param pMergeKF  被并入地图中的关键帧
     * @param pbStopFlag 外部停止标志
     * @param pMap      当前地图
     * @param corrPoses 输出，保存优化后的关键帧校正位姿
     * @return      无
     */
    void static MergeInertialBA(KeyFrame* pCurrKF, KeyFrame* pMergeKF, bool *pbStopFlag, Map *pMap, LoopClosing::KeyFrameAndPose &corrPoses);

    /**
     * @brief       地图焊接区域的局部 BA，用于地图合并后的过渡优化
     * @param pMainKF   主关键帧
     * @param vpAdjustKF 参与优化的关键帧
     * @param vpFixedKF  固定不动的关键帧
     * @param pbStopFlag 外部停止标志
     * @return      无
     */
    void static LocalBundleAdjustment(KeyFrame* pMainKF,vector<KeyFrame*> vpAdjustKF, vector<KeyFrame*> vpFixedKF, bool *pbStopFlag);

    /**
     * @brief       对 Hessian 的指定块做边缘化，并通过舒尔补消去对应变量
     * @param H     输入 Hessian
     * @param start 待边缘化块的起始下标
     * @param end   待边缘化块的结束下标
     * @return      边缘化后的 Hessian，已被消去的块会被置零
     */
    static Eigen::MatrixXd Marginalize(const Eigen::MatrixXd &H, const int &start, const int &end);

    /**
     * @brief       联合优化重力方向、尺度、速度和 bias 的惯性初始化/重估计过程
     * @param pMap  当前地图
     * @param Rwg   输入输出参数，世界到重力坐标系的旋转
     * @param scale 输入输出参数，地图尺度
     * @param bg    输入输出参数，陀螺仪 bias
     * @param ba    输入输出参数，加速度计 bias
     * @param bMono 是否为单目模式
     * @param covInertial 惯性协方差输出接口
     * @param bFixedVel   是否固定速度
     * @param bGauss      是否使用高斯牛顿近似流程
     * @param priorG      陀螺仪 bias 先验强度
     * @param priorA      加速度计 bias 先验强度
     * @return      无
     */
    void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale, Eigen::Vector3d &bg, Eigen::Vector3d &ba, bool bMono, Eigen::MatrixXd  &covInertial, bool bFixedVel=false, bool bGauss=false, float priorG = 1e2, float priorA = 1e6);
    /**
     * @brief       在尺度和重力固定的前提下，仅优化全局 bias 与各关键帧速度
     * @param pMap  当前地图
     * @param bg    输入输出参数，陀螺仪 bias
     * @param ba    输入输出参数，加速度计 bias
     * @param priorG 陀螺仪 bias 先验强度
     * @param priorA 加速度计 bias 先验强度
     * @return      无
     */
    void static InertialOptimization(Map *pMap, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG = 1e2, float priorA = 1e6);
    /**
     * @brief       在位姿、速度和 bias 固定时，只优化重力方向和尺度
     * @param pMap  当前地图
     * @param Rwg   输入输出参数，世界到重力坐标系的旋转
     * @param scale 输入输出参数，地图尺度
     * @return      无
     */
    void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

} //namespace ORB_SLAM3

#endif // OPTIMIZER_H
