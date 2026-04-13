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


#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include"sophus/sim3.hpp"

#include"MapPoint.h"
#include"KeyFrame.h"
#include"Frame.h"


namespace ORB_SLAM3
{

    /**
     * @brief ORB 特征匹配器
     *
     * 该类负责 ORB-SLAM3 中与“特征匹配”相关的核心工作。它并不负责提取 ORB，
     * 而是负责在不同数据结构之间建立对应关系，例如：
     * 1. Frame 与 MapPoint 的投影匹配；
     * 2. KeyFrame 与 Frame 的词袋（BoW）匹配；
     * 3. 两个 KeyFrame 之间的匹配；
     * 4. 初始化阶段的两帧匹配；
     * 5. 闭环 / 融合阶段基于 Sim3 的匹配与融合。
     *
     * ORBmatcher 的匹配依据主要包括：
     * 1. ORB 描述子的汉明距离；
     * 2. 图像金字塔尺度一致性；
     * 3. 几何约束，如投影窗口、极线约束、重投影误差等；
     * 4. 方向直方图一致性过滤。
     */
    class ORBmatcher
    {
    public:

        /**
         * @brief 构造 ORB 匹配器
         * @param nnratio   最近邻比值测试阈值
         * @param checkOri  是否启用方向一致性检查
         */
        ORBmatcher(float nnratio=0.6, bool checkOri=true);

        /**
         * @brief 计算两个 ORB 描述子之间的汉明距离
         * @param a 第一个描述子
         * @param b 第二个描述子
         * @return  两个 256 bit ORB 描述子的不同 bit 数
         */
        static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

        /**
         * @brief 在当前帧中搜索投影后的地图点匹配
         * @param F            当前帧
         * @param vpMapPoints  待投影的地图点集合
         * @param th           搜索窗口缩放系数
         * @param bFarPoints   是否启用远点过滤
         * @param thFarPoints  远点深度阈值
         * @return             匹配成功的数量
         *
         * 用途：局部地图跟踪阶段。
         */
        int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3, const bool bFarPoints = false, const float thFarPoints = 50.0f);

        /**
         * @brief 将上一帧中的地图点投影到当前帧并进行匹配
         * @param CurrentFrame 当前帧
         * @param LastFrame    上一帧
         * @param th           搜索窗口缩放系数
         * @param bMono        是否单目模式
         * @return             匹配成功的数量
         *
         * 用途：跟踪线程中基于上一帧的跟踪。
         */
        int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);

        /**
         * @brief 将关键帧中的地图点投影到当前帧并进行匹配
         * @param CurrentFrame   当前帧
         * @param pKF            参考关键帧
         * @param sAlreadyFound  已经匹配到的地图点集合
         * @param th             搜索窗口缩放系数
         * @param ORBdist        描述子距离阈值
         * @return               匹配成功的数量
         *
         * 用途：重定位阶段。
         */
        int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);

        /**
         * @brief 使用 Sim3 变换将地图点投影到关键帧中并搜索匹配
         * @param pKF          目标关键帧
         * @param Scw          当前 Sim3 变换
         * @param vpPoints     待匹配地图点
         * @param vpMatched    输出匹配结果
         * @param th           搜索窗口缩放系数
         * @param ratioHamming 汉明距离阈值倍率
         * @return             匹配成功的数量
         *
         * 用途：闭环检测阶段。
         */
        int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th, float ratioHamming=1.0);

        /**
         * @brief 使用 Sim3 变换投影地图点并同时记录匹配来源关键帧
         * @param pKF           目标关键帧
         * @param Scw           当前 Sim3 变换
         * @param vpPoints      待匹配地图点
         * @param vpPointsKFs   每个地图点对应的来源关键帧
         * @param vpMatched     输出匹配到的地图点
         * @param vpMatchedKF   输出匹配地图点对应的来源关键帧
         * @param th            搜索窗口缩放系数
         * @param ratioHamming  汉明距离阈值倍率
         * @return              匹配成功的数量
         *
         * 用途：闭环与地图融合中的地点识别。
         */
        int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, const std::vector<KeyFrame*> &vpPointsKFs, std::vector<MapPoint*> &vpMatched, std::vector<KeyFrame*> &vpMatchedKF, int th, float ratioHamming=1.0);

        /**
         * @brief 使用 BoW 在关键帧与普通帧之间搜索匹配
         * @param pKF                关键帧
         * @param F                  当前帧
         * @param vpMapPointMatches  输出当前帧中匹配到的地图点
         * @return                   匹配成功的数量
         *
         * 只在相同词袋结点内暴力匹配，减少不必要的描述子比较。
         */
        int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);

        /**
         * @brief 使用 BoW 在两个关键帧之间搜索匹配
         * @param pKF1        关键帧 1
         * @param pKF2        关键帧 2
         * @param vpMatches12 输出从关键帧 1 到关键帧 2 的地图点匹配结果
         * @return            匹配成功的数量
         */
        int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);

        /**
         * @brief 单目初始化阶段的两帧匹配
         * @param F1             第一帧
         * @param F2             第二帧
         * @param vbPrevMatched  第一帧特征在第二帧中的预测位置
         * @param vnMatches12    输出匹配关系
         * @param windowSize     搜索窗口大小
         * @return               匹配成功的数量
         */
        int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);

        /**
         * @brief 为新地图点三角化搜索匹配
         * @param pKF1          关键帧 1
         * @param pKF2          关键帧 2
         * @param vMatchedPairs 输出匹配到的特征对
         * @param bOnlyStereo   是否只使用具有立体观测的特征
         * @param bCoarse       是否跳过严格极线约束，采用粗匹配
         * @return              匹配成功的数量
         */
        int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2,
                                   std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo, const bool bCoarse = false);

        /**
         * @brief 在两个关键帧之间基于 Sim3 关系搜索匹配
         * @param pKF1        关键帧 1
         * @param pKF2        关键帧 2
         * @param vpMatches12 输入输出匹配结果
         * @param S12         从关键帧 2 到关键帧 1 的 Sim3 变换
         * @param th          搜索窗口缩放系数
         * @return            匹配成功的数量
         */
        int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const Sophus::Sim3f &S12, const float th);

        /**
         * @brief 将地图点投影到关键帧中，搜索重复地图点并尝试融合
         * @param pKF         目标关键帧
         * @param vpMapPoints 待融合地图点集合
         * @param th          搜索窗口缩放系数
         * @param bRight      是否投影到右目
         * @return            融合成功的数量
         */
        int Fuse(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float th=3.0, const bool bRight = false);

        /**
         * @brief 使用给定 Sim3 将地图点投影到关键帧并尝试融合
         * @param pKF            目标关键帧
         * @param Scw            Sim3 变换
         * @param vpPoints       待融合地图点集合
         * @param th             搜索窗口缩放系数
         * @param vpReplacePoint 输出需要被替换的地图点
         * @return               融合成功的数量
         */
        int Fuse(KeyFrame* pKF, Sophus::Sim3f &Scw, const std::vector<MapPoint*> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);

    public:

        /// 松阈值，通常用于允许较粗匹配或投影匹配。
        static const int TH_LOW;
        /// 严阈值上界，通常用于投影匹配中的描述子过滤。
        static const int TH_HIGH;
        /// 方向一致性直方图的 bin 数。
        static const int HISTO_LENGTH;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        /**
         * @brief 根据视角余弦值返回搜索半径
         * @param viewCos 地图点法向与观测方向夹角的余弦值
         * @return        搜索半径倍率
         */
        float RadiusByViewingCos(const float &viewCos);

        /**
         * @brief 从方向直方图中找出元素数量最多的三个 bin
         * @param histo 方向直方图
         * @param L     直方图长度
         * @param ind1  第一大 bin 下标
         * @param ind2  第二大 bin 下标
         * @param ind3  第三大 bin 下标
         */
        void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

        /// 最近邻比值测试参数。
        float mfNNratio;
        /// 是否启用方向一致性检查。
        bool mbCheckOrientation;
    };

}// namespace ORB_SLAM

#endif // ORBMATCHER_H
