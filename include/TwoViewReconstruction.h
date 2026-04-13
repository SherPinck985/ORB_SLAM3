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

#ifndef TwoViewReconstruction_H
#define TwoViewReconstruction_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <unordered_set>

#include <sophus/se3.hpp>

namespace ORB_SLAM3
{

    /**
     * @brief 两视图重建器
     *
     * 该类用于 ORB-SLAM3 初始化阶段的“两帧几何恢复”：
     * 1. 输入两帧 ORB 关键点及其匹配关系；
     * 2. 通过 RANSAC 并行估计单应矩阵 H 和基础矩阵 F；
     * 3. 根据评分选择更合适的几何模型；
     * 4. 从模型中恢复相对位姿 T21；
     * 5. 三角化得到初始稀疏地图点。
     *
     * 其中输入关键点通常来自 ORBextractor：
     * 1. 先构建尺度金字塔；
     * 2. 用 FAST 提取角点；
     * 3. 用四叉树做空间均匀化；
     * 4. 用灰度质心法计算方向；
     * 5. 生成旋转不变的 ORB 描述子并完成匹配。
     *
     * 本类并不负责提取 ORB 特征本身，而是消费已经匹配好的特征点，
     * 利用多视图几何恢复两帧之间的相对运动与场景结构。
     */
    class TwoViewReconstruction
    {
        /// 一对匹配关系，first 对应参考帧特征点下标，second 对应当前帧特征点下标。
        typedef std::pair<int,int> Match;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief 构造两视图重建器
         * @param k          相机内参矩阵 K
         * @param sigma      图像观测噪声标准差，用于构造重投影误差阈值
         * @param iterations RANSAC 最大迭代次数
         *
         * 说明：
         * 1. 该类默认把第一帧当作参考坐标系；
         * 2. 构造时只保存参数，不执行重建；
         * 3. sigma 的平方会被缓存，后续频繁用于误差检验。
         */
        TwoViewReconstruction(const Eigen::Matrix3f& k, float sigma = 1.0, int iterations = 200);

        /**
         * @brief 从两帧匹配中恢复相对位姿和稀疏三维点
         * @param vKeys1           第一帧关键点集合，作为参考帧
         * @param vKeys2           第二帧关键点集合，作为当前帧
         * @param vMatches12       匹配表，下标 i 表示第一帧第 i 个点；值为第二帧对应下标，-1 表示未匹配
         * @param T21              输出位姿，表示从参考帧 1 到当前帧 2 的变换
         * @param vP3D             输出三维点，索引与第一帧关键点对齐
         * @param vbTriangulated   输出标记，表示对应第一帧关键点是否成功三角化
         * @return                 重建成功返回 true，否则返回 false
         *
         * 主流程：
         * 1. 整理有效匹配对；
         * 2. 为 RANSAC 预生成最小采样集；
         * 3. 并行估计 H 和 F；
         * 4. 用评分比值决定优先采用平面模型还是极几何模型；
         * 5. 恢复位姿并筛选可用三角化点。
         */
        bool Reconstruct(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
                          Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated);

    private:

        /**
         * @brief 使用 RANSAC 估计两帧之间的单应矩阵 H21
         * @param vbMatchesInliers 输出内点标记，和 mvMatches12 对齐
         * @param score            输出评分，值越大表示模型解释数据越好
         * @param H21              输出从帧 1 到帧 2 的单应矩阵
         *
         * 单应模型更适合场景近似共面、纯旋转或低视差情况。
         */
        void FindHomography(std::vector<bool> &vbMatchesInliers, float &score, Eigen::Matrix3f &H21);

        /**
         * @brief 使用 RANSAC 估计两帧之间的基础矩阵 F21
         * @param vbInliers 输出内点标记，和 mvMatches12 对齐
         * @param score     输出评分，值越大表示模型解释数据越好
         * @param F21       输出从帧 1 到帧 2 的基础矩阵
         *
         * 基础矩阵模型更适合一般三维场景下存在平移基线的情况。
         */
        void FindFundamental(std::vector<bool> &vbInliers, float &score, Eigen::Matrix3f &F21);

        /**
         * @brief 由归一化点对计算单应矩阵 H21
         * @param vP1 第一帧归一化点
         * @param vP2 第二帧归一化点
         * @return    估计得到的归一化单应矩阵
         *
         * 使用 DLT（Direct Linear Transform）线性求解。
         */
        Eigen::Matrix3f ComputeH21(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);

        /**
         * @brief 由归一化点对计算基础矩阵 F21
         * @param vP1 第一帧归一化点
         * @param vP2 第二帧归一化点
         * @return    满足秩为 2 约束的基础矩阵
         *
         * 使用八点法求初值，再通过 SVD 强制秩约束。
         */
        Eigen::Matrix3f ComputeF21(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);

        /**
         * @brief 评估单应矩阵的好坏并筛选内点
         * @param H21               从帧 1 到帧 2 的单应矩阵
         * @param H12               H21 的逆矩阵，用于反向投影
         * @param vbMatchesInliers  输出内点标记
         * @param sigma             观测噪声标准差
         * @return                  评分，综合了内点数量与重投影误差大小
         *
         * 会同时检查：
         * 1. x1 经过 H21 投影到图像 2 的误差；
         * 2. x2 经过 H12 投影到图像 1 的误差。
         */
        float CheckHomography(const Eigen::Matrix3f &H21, const Eigen::Matrix3f &H12, std::vector<bool> &vbMatchesInliers, float sigma);

        /**
         * @brief 评估基础矩阵的好坏并筛选内点
         * @param F21               从帧 1 到帧 2 的基础矩阵
         * @param vbMatchesInliers  输出内点标记
         * @param sigma             观测噪声标准差
         * @return                  评分，综合了内点数量与极线距离误差
         *
         * 会检查两个方向上的点到极线距离是否足够小。
         */
        float CheckFundamental(const Eigen::Matrix3f &F21, std::vector<bool> &vbMatchesInliers, float sigma);

        /**
         * @brief 从基础矩阵恢复相对位姿并三角化空间点
         * @param vbMatchesInliers  基础矩阵对应的内点标记
         * @param F21               基础矩阵
         * @param K                 相机内参矩阵
         * @param T21               输出位姿
         * @param vP3D              输出三维点
         * @param vbTriangulated    输出三角化成功标记
         * @param minParallax       最小视差角阈值
         * @param minTriangulated   最小可接受三角化点数量
         * @return                  恢复成功返回 true
         *
         * 该函数会把 F 转成 E，并枚举 4 组 [R|t] 假设进行验证。
         */
        bool ReconstructF(std::vector<bool> &vbMatchesInliers, Eigen::Matrix3f &F21, Eigen::Matrix3f &K,
                          Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

        /**
         * @brief 从单应矩阵恢复相对位姿并三角化空间点
         * @param vbMatchesInliers  单应矩阵对应的内点标记
         * @param H21               单应矩阵
         * @param K                 相机内参矩阵
         * @param T21               输出位姿
         * @param vP3D              输出三维点
         * @param vbTriangulated    输出三角化成功标记
         * @param minParallax       最小视差角阈值
         * @param minTriangulated   最小可接受三角化点数量
         * @return                  恢复成功返回 true
         *
         * 按 Faugeras 的分解方法恢复 8 组运动假设并做几何验证。
         */
        bool ReconstructH(std::vector<bool> &vbMatchesInliers, Eigen::Matrix3f &H21, Eigen::Matrix3f &K,
                          Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D,std:: vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

        /**
         * @brief 对关键点坐标做归一化，提高数值稳定性
         * @param vKeys               输入关键点
         * @param vNormalizedPoints   输出归一化后的二维点
         * @param T                   输出归一化变换矩阵
         *
         * 处理后点集大致满足：
         * 1. 均值接近原点；
         * 2. x / y 方向平均绝对偏差约为 1。
         */
        void Normalize(const std::vector<cv::KeyPoint> &vKeys, std::vector<cv::Point2f> &vNormalizedPoints, Eigen::Matrix3f &T);


        /**
         * @brief 检查给定位姿假设是否成立
         * @param R                  候选旋转
         * @param t                  候选平移方向
         * @param vKeys1             第一帧关键点
         * @param vKeys2             第二帧关键点
         * @param vMatches12         匹配关系
         * @param vbMatchesInliers   模型内点标记
         * @param K                  相机内参矩阵
         * @param vP3D               输出三维点，索引与第一帧关键点对齐
         * @param th2                重投影误差平方阈值
         * @param vbGood             输出标记，表示该点是否通过全部几何检验
         * @param parallax           输出代表性视差角
         * @return                   通过检验的三角化点数量
         *
         * 检查内容包括：
         * 1. 三角化结果是否有限；
         * 2. 点是否位于两相机前方；
         * 3. 两帧重投影误差是否足够小；
         * 4. 视差是否能支持稳定三角化。
         */
        int CheckRT(const Eigen::Matrix3f &R, const Eigen::Vector3f &t, const std::vector<cv::KeyPoint> &vKeys1, const std::vector<cv::KeyPoint> &vKeys2,
                    const std::vector<Match> &vMatches12, std::vector<bool> &vbMatchesInliers,
                    const Eigen::Matrix3f &K, std::vector<cv::Point3f> &vP3D, float th2, std::vector<bool> &vbGood, float &parallax);

        /**
         * @brief 分解本质矩阵，恢复两个旋转候选和一个平移方向候选
         * @param E   本质矩阵
         * @param R1  输出旋转候选 1
         * @param R2  输出旋转候选 2
         * @param t   输出单位平移方向
         *
         * 结合 t 的正负号，可构成 4 组 [R|t] 假设。
         */
        void DecomposeE(const Eigen::Matrix3f &E, Eigen::Matrix3f &R1, Eigen::Matrix3f &R2, Eigen::Vector3f &t);


        /// 参考帧（第 1 帧）的关键点集合。
        std::vector<cv::KeyPoint> mvKeys1;

        /// 当前帧（第 2 帧）的关键点集合。
        std::vector<cv::KeyPoint> mvKeys2;

        /// 当前有效匹配对，只保存 vMatches12 中值不为 -1 的匹配。
        std::vector<Match> mvMatches12;
        /// 参考帧每个关键点是否拥有有效匹配的标记。
        std::vector<bool> mvbMatched1;

        /// 相机内参矩阵 K。
        Eigen::Matrix3f mK;

        /// 图像观测噪声参数：mSigma 为标准差，mSigma2 为方差。
        float mSigma, mSigma2;

        /// RANSAC 最大迭代次数。
        int mMaxIterations;

        /// 每次 RANSAC 迭代预先采样好的 8 点最小集。
        std::vector<std::vector<size_t> > mvSets;

    };

} //namespace ORB_SLAM

#endif // TwoViewReconstruction_H
