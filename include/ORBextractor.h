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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>


namespace ORB_SLAM3
{

/**
 * @brief 四叉树分配过程中的结点
 *
 * ORBextractor 会先在每个网格内检测 FAST 角点，然后把候选角点交给四叉树做空间均匀化。
 * 该类描述四叉树中的一个矩形区域，记录：
 * 1. 该区域的边界；
 * 2. 当前区域内包含的候选关键点；
 * 3. 在链表中的位置，便于后续继续分裂或删除；
 * 4. 是否已经无法继续分裂。
 */
class ExtractorNode
{
public:
    /**
     * @brief 默认构造函数
     *
     * 初始时结点还没有边界和关键点，只把“不可再分裂”标志置为 false。
     */
    ExtractorNode():bNoMore(false){}

    /**
     * @brief 将当前结点划分为四个子结点
     * @param n1 左上子结点
     * @param n2 右上子结点
     * @param n3 左下子结点
     * @param n4 右下子结点
     *
     * 该函数只负责根据当前结点的矩形范围进行划分，并把当前结点中的关键点
     * 按照坐标分配到对应子结点中。
     */
    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    /// 当前结点内包含的候选关键点集合。
    std::vector<cv::KeyPoint> vKeys;
    /// 当前矩形区域的四个顶点：左上、右上、左下、右下。
    cv::Point2i UL, UR, BL, BR;
    /// 当前结点在链表中的迭代器，用于在分裂后快速删除旧结点。
    std::list<ExtractorNode>::iterator lit;
    /// 为 true 表示当前结点只有 0/1 个关键点，或者已不值得继续细分。
    bool bNoMore;
};

/**
 * @brief ORB 特征提取器
 *
 * 该类实现了 ORB-SLAM3 中使用的 ORB 特征提取流程，主要分为四步：
 * 1. 构建尺度金字塔；
 * 2. 在每层图像上使用 FAST 检测候选角点；
 * 3. 使用四叉树让关键点在图像空间内分布更均匀；
 * 4. 计算关键点方向并生成旋转不变的 ORB 描述子。
 */
class ORBextractor
{
public:
    
    /// 关键点评分方式。当前实现主要沿用 FAST 自带响应值。
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    /**
     * @brief 构造 ORB 特征提取器
     * @param nfeatures    期望提取的总特征点数量
     * @param scaleFactor  金字塔相邻层之间的尺度因子
     * @param nlevels      金字塔层数
     * @param iniThFAST    FAST 初始阈值
     * @param minThFAST    FAST 降级阈值，当初始阈值提取不到点时使用
     */
    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    /**
     * @brief 在输入图像上提取 ORB 关键点和描述子
     * @param _image        输入灰度图像
     * @param _mask         输入掩码，当前实现中未使用
     * @param _keypoints    输出关键点
     * @param _descriptors  输出描述子，每个关键点对应 32 字节
     * @param vLappingArea  双目重叠区域的 x 范围，用于调整输出关键点顺序
     * @return              返回非重叠区域关键点的数量；若输入为空则返回 -1
     *
     * 说明：
     * 1. 关键点会先按金字塔层提取，再统一缩放回原图坐标系；
     * 2. 处于双目重叠区域内的点会被放到输出数组尾部，以加速后续双目匹配。
     */
    int operator()( cv::InputArray _image, cv::InputArray _mask,
                    std::vector<cv::KeyPoint>& _keypoints,
                    cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

    /// 返回金字塔层数。
    int inline GetLevels(){
        return nlevels;}

    /// 返回相邻尺度层之间的缩放因子。
    float inline GetScaleFactor(){
        return scaleFactor;}

    /// 返回每一层相对于第 0 层的尺度因子。
    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    /// 返回每一层尺度因子的倒数。
    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    /// 返回每一层尺度对应的方差 sigma^2。
    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    /// 返回每一层尺度方差的倒数。
    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    /// 图像金字塔。每一层图像都额外带有边界填充，便于计算方向和描述子。
    std::vector<cv::Mat> mvImagePyramid;

protected:

    /**
     * @brief 构建图像金字塔
     * @param image 输入原始灰度图像
     */
    void ComputePyramid(cv::Mat image);

    /**
     * @brief 在每一层金字塔图像上提取关键点，并用四叉树做均匀化
     * @param allKeypoints 输出每层的关键点集合
     */
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    

    /**
     * @brief 使用四叉树在指定区域内均匀分配候选关键点
     * @param vToDistributeKeys 待分配的候选关键点
     * @param minX              区域左边界
     * @param maxX              区域右边界
     * @param minY              区域上边界
     * @param maxY              区域下边界
     * @param nFeatures         该区域最终希望保留的关键点数
     * @param level             当前金字塔层级
     * @return                  空间分布更均匀后的关键点集合
     */
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    /**
     * @brief 旧版关键点提取策略
     * @param allKeypoints 输出每层的关键点集合
     *
     * 该函数保留了早期基于固定网格重新分配配额的实现，目前主流程默认使用
     * ComputeKeyPointsOctTree。
     */
    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);

    /// ORB 描述子使用的 BRIEF 采样模式，共 512 个采样点。
    std::vector<cv::Point> pattern;

    /// 总目标特征点数量。
    int nfeatures;
    /// 金字塔相邻层的尺度因子。
    double scaleFactor;
    /// 金字塔层数。
    int nlevels;
    /// FAST 首选阈值。
    int iniThFAST;
    /// FAST 回退阈值。
    int minThFAST;

    /// 每层希望分配到的目标特征点数量。
    std::vector<int> mnFeaturesPerLevel;

    /// 方向计算时，圆形 patch 在每一行对应的最大 u 范围。
    std::vector<int> umax;

    /// 每层相对于第 0 层的尺度。
    std::vector<float> mvScaleFactor;
    /// 每层尺度的倒数。
    std::vector<float> mvInvScaleFactor;    
    /// 每层尺度对应的 sigma^2。
    std::vector<float> mvLevelSigma2;
    /// 每层 sigma^2 的倒数。
    std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif
