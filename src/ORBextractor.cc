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

/**
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*/


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>

#include "ORBextractor.h"


using namespace cv;
using namespace std;

namespace ORB_SLAM3
{

    // ORB 默认使用 31x31 的 patch 来计算方向和描述子。
    const int PATCH_SIZE = 31;
    // patch 半径，对应像素范围 [-15, 15]。
    const int HALF_PATCH_SIZE = 15;
    // 为了在边缘附近安全访问 patch，需要为图像补出的边界宽度。
    const int EDGE_THRESHOLD = 19;


    /**
     * @brief       计算图像中特征点的方向，这里以返回角度作为方向
     * @param image 要进行操作的某层金字塔层图像
     * @param pt    特征点坐标
     * @param u_max 图像块的每一行的坐标边界
     * @return      角度，范围为[0,360）
     *
     * 该实现使用图像灰度质心法（Intensity Centroid, IC）估计方向：
     * 1. 在关键点周围取一个圆形 patch；
     * 2. 计算 patch 灰度分布关于 x/y 轴的一阶矩；
     * 3. 由向量 (m_10, m_01) 的方向得到关键点主方向。
     */
    static float IC_Angle(const Mat& image, Point2f pt,  const vector<int> & u_max)
    {
        // m_10 和 m_01 是灰度质心法使用的一阶矩：
        // m_10 = sum(u * I(u,v))
        // m_01 = sum(v * I(u,v))
        // 它们共同决定灰度质心相对于 patch 中心的偏移方向。
        int m_01 = 0, m_10 = 0;

        // center 指向关键点中心像素。后续访问邻域时，直接通过指针偏移完成，
        // 可以避免频繁的 at<>() 边界检查，效率更高。
        const uchar* center = &image.at<uchar> (cvRound(pt.y), cvRound(pt.x));

        // 中心行 v = 0 时，m_01 中不含 v 项，因此只需要累计 m_10。
        // 因为此时上下对称，不需要成对采样。
        for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
            m_10 += u * center[u];

        // image.step1() 表示图像一行跨越多少个 uchar 元素。
        // 后面访问第 +/-v 行时，使用 v*step 做指针跳转。
        int step = (int)image.step1();

        // 从 patch 中心向上/向下成对扫描每一行。
        // 之所以只循环正半轴的 v，是因为对称的两行会被一起处理。
        for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
        {
            // 累计当前上下两行对 m_01 的贡献。
            // 对于固定的 v，有：
            // m_01 += v * (I(u, +v) - I(u, -v))
            int v_sum = 0;

            // u_max[v] 给出了圆形 patch 在第 v 行允许访问的最大水平半径。
            // 这样只会统计落在圆形区域内的像素，而不是完整方块。
            int d = u_max[v];
            for (int u = -d; u <= d; ++u)
            {
                // 取当前列 u 在上下两条对称扫描线上的像素值。
                int val_plus = center[u + v*step], val_minus = center[u - v*step];

                // 上下像素差值用于累计 m_01；
                // 若上方更亮，则质心更偏向正 y，反之更偏向负 y。
                v_sum += (val_plus - val_minus);

                // 左右位置权重 u 与上下两行像素和相乘，用来累计 m_10。
                m_10 += u * (val_plus + val_minus);
            }

            // 当前扫描半径 v 对纵向矩的最终贡献。
            m_01 += v * v_sum;
        }

        // 将矩向量转换为角度，fastAtan2 返回单位为度，范围 [0, 360)。
        return fastAtan2((float)m_01, (float)m_10);
    }


    // OpenCV 中关键点 angle 以角度保存，这里预先存一份“角度转弧度”系数。
    const float factorPI = (float)(CV_PI/180.f);

    /**
     * @brief       计算单个关键点的 ORB 描述子
     * @param kpt   关键点，内部会使用其位置和方向
     * @param img   当前尺度层的图像
     * @param pattern BRIEF 采样模式
     * @param desc  输出描述子缓存，长度为 32 字节
     *
     * ORB 描述子本质上是带方向补偿的 BRIEF：
     * 1. 取关键点方向 angle；
     * 2. 用 angle 旋转预定义采样点对；
     * 3. 比较每对采样点灰度大小，得到 0/1；
     * 4. 每 8 次比较打包成 1 个字节，共 32 字节 = 256 bit。
     */
    static void computeOrbDescriptor(const KeyPoint& kpt,
                                     const Mat& img, const Point* pattern,
                                     uchar* desc)
    {
        // OpenCV 的关键点角度是“度”，三角函数需要“弧度”。
        float angle = (float)kpt.angle*factorPI;
        float a = (float)cos(angle), b = (float)sin(angle);

        // center 指向关键点中心像素。描述子中的所有采样都相对于该中心展开。
        const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));

        // img.step 是按字节计的一行跨度。由于图像类型是 uchar，所以可以直接使用。
        const int step = (int)img.step;

        // 将 pattern 中的局部坐标按关键点方向旋转后，映射到图像中的真实采样位置。
        // 这里等价于将采样模板绕关键点中心旋转，从而让描述子具备旋转不变性。
#define GET_VALUE(idx) \
        center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + \
               cvRound(pattern[idx].x*a - pattern[idx].y*b)]


        // ORB 描述子固定为 32 个字节。
        // 每轮使用 16 个采样点（组成 8 对比较），得到 1 个字节。
        for (int i = 0; i < 32; ++i, pattern += 16)
        {
            int t0, t1, val;

            // 每次比较都会得到一个二值位：
            // 若第一个采样点灰度 < 第二个采样点灰度，则该位记为 1，否则为 0。
            t0 = GET_VALUE(0); t1 = GET_VALUE(1);
            val = t0 < t1;
            t0 = GET_VALUE(2); t1 = GET_VALUE(3);
            val |= (t0 < t1) << 1;
            t0 = GET_VALUE(4); t1 = GET_VALUE(5);
            val |= (t0 < t1) << 2;
            t0 = GET_VALUE(6); t1 = GET_VALUE(7);
            val |= (t0 < t1) << 3;
            t0 = GET_VALUE(8); t1 = GET_VALUE(9);
            val |= (t0 < t1) << 4;
            t0 = GET_VALUE(10); t1 = GET_VALUE(11);
            val |= (t0 < t1) << 5;
            t0 = GET_VALUE(12); t1 = GET_VALUE(13);
            val |= (t0 < t1) << 6;
            t0 = GET_VALUE(14); t1 = GET_VALUE(15);
            val |= (t0 < t1) << 7;

            desc[i] = (uchar)val;
        }

#undef GET_VALUE
    }


    // 31x31 patch 上预学习得到的 BRIEF 采样点模式。
    // 数组中每 4 个整数表示一对采样点：(x1, y1, x2, y2)。
    // 总共 256 对，用于生成 256 bit 描述子。
    static int bit_pattern_31_[256*4] =
            {
                    8,-3, 9,5/*mean (0), correlation (0)*/,
                    4,2, 7,-12/*mean (1.12461e-05), correlation (0.0437584)*/,
                    -11,9, -8,2/*mean (3.37382e-05), correlation (0.0617409)*/,
                    7,-12, 12,-13/*mean (5.62303e-05), correlation (0.0636977)*/,
                    2,-13, 2,12/*mean (0.000134953), correlation (0.085099)*/,
                    1,-7, 1,6/*mean (0.000528565), correlation (0.0857175)*/,
                    -2,-10, -2,-4/*mean (0.0188821), correlation (0.0985774)*/,
                    -13,-13, -11,-8/*mean (0.0363135), correlation (0.0899616)*/,
                    -13,-3, -12,-9/*mean (0.121806), correlation (0.099849)*/,
                    10,4, 11,9/*mean (0.122065), correlation (0.093285)*/,
                    -13,-8, -8,-9/*mean (0.162787), correlation (0.0942748)*/,
                    -11,7, -9,12/*mean (0.21561), correlation (0.0974438)*/,
                    7,7, 12,6/*mean (0.160583), correlation (0.130064)*/,
                    -4,-5, -3,0/*mean (0.228171), correlation (0.132998)*/,
                    -13,2, -12,-3/*mean (0.00997526), correlation (0.145926)*/,
                    -9,0, -7,5/*mean (0.198234), correlation (0.143636)*/,
                    12,-6, 12,-1/*mean (0.0676226), correlation (0.16689)*/,
                    -3,6, -2,12/*mean (0.166847), correlation (0.171682)*/,
                    -6,-13, -4,-8/*mean (0.101215), correlation (0.179716)*/,
                    11,-13, 12,-8/*mean (0.200641), correlation (0.192279)*/,
                    4,7, 5,1/*mean (0.205106), correlation (0.186848)*/,
                    5,-3, 10,-3/*mean (0.234908), correlation (0.192319)*/,
                    3,-7, 6,12/*mean (0.0709964), correlation (0.210872)*/,
                    -8,-7, -6,-2/*mean (0.0939834), correlation (0.212589)*/,
                    -2,11, -1,-10/*mean (0.127778), correlation (0.20866)*/,
                    -13,12, -8,10/*mean (0.14783), correlation (0.206356)*/,
                    -7,3, -5,-3/*mean (0.182141), correlation (0.198942)*/,
                    -4,2, -3,7/*mean (0.188237), correlation (0.21384)*/,
                    -10,-12, -6,11/*mean (0.14865), correlation (0.23571)*/,
                    5,-12, 6,-7/*mean (0.222312), correlation (0.23324)*/,
                    5,-6, 7,-1/*mean (0.229082), correlation (0.23389)*/,
                    1,0, 4,-5/*mean (0.241577), correlation (0.215286)*/,
                    9,11, 11,-13/*mean (0.00338507), correlation (0.251373)*/,
                    4,7, 4,12/*mean (0.131005), correlation (0.257622)*/,
                    2,-1, 4,4/*mean (0.152755), correlation (0.255205)*/,
                    -4,-12, -2,7/*mean (0.182771), correlation (0.244867)*/,
                    -8,-5, -7,-10/*mean (0.186898), correlation (0.23901)*/,
                    4,11, 9,12/*mean (0.226226), correlation (0.258255)*/,
                    0,-8, 1,-13/*mean (0.0897886), correlation (0.274827)*/,
                    -13,-2, -8,2/*mean (0.148774), correlation (0.28065)*/,
                    -3,-2, -2,3/*mean (0.153048), correlation (0.283063)*/,
                    -6,9, -4,-9/*mean (0.169523), correlation (0.278248)*/,
                    8,12, 10,7/*mean (0.225337), correlation (0.282851)*/,
                    0,9, 1,3/*mean (0.226687), correlation (0.278734)*/,
                    7,-5, 11,-10/*mean (0.00693882), correlation (0.305161)*/,
                    -13,-6, -11,0/*mean (0.0227283), correlation (0.300181)*/,
                    10,7, 12,1/*mean (0.125517), correlation (0.31089)*/,
                    -6,-3, -6,12/*mean (0.131748), correlation (0.312779)*/,
                    10,-9, 12,-4/*mean (0.144827), correlation (0.292797)*/,
                    -13,8, -8,-12/*mean (0.149202), correlation (0.308918)*/,
                    -13,0, -8,-4/*mean (0.160909), correlation (0.310013)*/,
                    3,3, 7,8/*mean (0.177755), correlation (0.309394)*/,
                    5,7, 10,-7/*mean (0.212337), correlation (0.310315)*/,
                    -1,7, 1,-12/*mean (0.214429), correlation (0.311933)*/,
                    3,-10, 5,6/*mean (0.235807), correlation (0.313104)*/,
                    2,-4, 3,-10/*mean (0.00494827), correlation (0.344948)*/,
                    -13,0, -13,5/*mean (0.0549145), correlation (0.344675)*/,
                    -13,-7, -12,12/*mean (0.103385), correlation (0.342715)*/,
                    -13,3, -11,8/*mean (0.134222), correlation (0.322922)*/,
                    -7,12, -4,7/*mean (0.153284), correlation (0.337061)*/,
                    6,-10, 12,8/*mean (0.154881), correlation (0.329257)*/,
                    -9,-1, -7,-6/*mean (0.200967), correlation (0.33312)*/,
                    -2,-5, 0,12/*mean (0.201518), correlation (0.340635)*/,
                    -12,5, -7,5/*mean (0.207805), correlation (0.335631)*/,
                    3,-10, 8,-13/*mean (0.224438), correlation (0.34504)*/,
                    -7,-7, -4,5/*mean (0.239361), correlation (0.338053)*/,
                    -3,-2, -1,-7/*mean (0.240744), correlation (0.344322)*/,
                    2,9, 5,-11/*mean (0.242949), correlation (0.34145)*/,
                    -11,-13, -5,-13/*mean (0.244028), correlation (0.336861)*/,
                    -1,6, 0,-1/*mean (0.247571), correlation (0.343684)*/,
                    5,-3, 5,2/*mean (0.000697256), correlation (0.357265)*/,
                    -4,-13, -4,12/*mean (0.00213675), correlation (0.373827)*/,
                    -9,-6, -9,6/*mean (0.0126856), correlation (0.373938)*/,
                    -12,-10, -8,-4/*mean (0.0152497), correlation (0.364237)*/,
                    10,2, 12,-3/*mean (0.0299933), correlation (0.345292)*/,
                    7,12, 12,12/*mean (0.0307242), correlation (0.366299)*/,
                    -7,-13, -6,5/*mean (0.0534975), correlation (0.368357)*/,
                    -4,9, -3,4/*mean (0.099865), correlation (0.372276)*/,
                    7,-1, 12,2/*mean (0.117083), correlation (0.364529)*/,
                    -7,6, -5,1/*mean (0.126125), correlation (0.369606)*/,
                    -13,11, -12,5/*mean (0.130364), correlation (0.358502)*/,
                    -3,7, -2,-6/*mean (0.131691), correlation (0.375531)*/,
                    7,-8, 12,-7/*mean (0.160166), correlation (0.379508)*/,
                    -13,-7, -11,-12/*mean (0.167848), correlation (0.353343)*/,
                    1,-3, 12,12/*mean (0.183378), correlation (0.371916)*/,
                    2,-6, 3,0/*mean (0.228711), correlation (0.371761)*/,
                    -4,3, -2,-13/*mean (0.247211), correlation (0.364063)*/,
                    -1,-13, 1,9/*mean (0.249325), correlation (0.378139)*/,
                    7,1, 8,-6/*mean (0.000652272), correlation (0.411682)*/,
                    1,-1, 3,12/*mean (0.00248538), correlation (0.392988)*/,
                    9,1, 12,6/*mean (0.0206815), correlation (0.386106)*/,
                    -1,-9, -1,3/*mean (0.0364485), correlation (0.410752)*/,
                    -13,-13, -10,5/*mean (0.0376068), correlation (0.398374)*/,
                    7,7, 10,12/*mean (0.0424202), correlation (0.405663)*/,
                    12,-5, 12,9/*mean (0.0942645), correlation (0.410422)*/,
                    6,3, 7,11/*mean (0.1074), correlation (0.413224)*/,
                    5,-13, 6,10/*mean (0.109256), correlation (0.408646)*/,
                    2,-12, 2,3/*mean (0.131691), correlation (0.416076)*/,
                    3,8, 4,-6/*mean (0.165081), correlation (0.417569)*/,
                    2,6, 12,-13/*mean (0.171874), correlation (0.408471)*/,
                    9,-12, 10,3/*mean (0.175146), correlation (0.41296)*/,
                    -8,4, -7,9/*mean (0.183682), correlation (0.402956)*/,
                    -11,12, -4,-6/*mean (0.184672), correlation (0.416125)*/,
                    1,12, 2,-8/*mean (0.191487), correlation (0.386696)*/,
                    6,-9, 7,-4/*mean (0.192668), correlation (0.394771)*/,
                    2,3, 3,-2/*mean (0.200157), correlation (0.408303)*/,
                    6,3, 11,0/*mean (0.204588), correlation (0.411762)*/,
                    3,-3, 8,-8/*mean (0.205904), correlation (0.416294)*/,
                    7,8, 9,3/*mean (0.213237), correlation (0.409306)*/,
                    -11,-5, -6,-4/*mean (0.243444), correlation (0.395069)*/,
                    -10,11, -5,10/*mean (0.247672), correlation (0.413392)*/,
                    -5,-8, -3,12/*mean (0.24774), correlation (0.411416)*/,
                    -10,5, -9,0/*mean (0.00213675), correlation (0.454003)*/,
                    8,-1, 12,-6/*mean (0.0293635), correlation (0.455368)*/,
                    4,-6, 6,-11/*mean (0.0404971), correlation (0.457393)*/,
                    -10,12, -8,7/*mean (0.0481107), correlation (0.448364)*/,
                    4,-2, 6,7/*mean (0.050641), correlation (0.455019)*/,
                    -2,0, -2,12/*mean (0.0525978), correlation (0.44338)*/,
                    -5,-8, -5,2/*mean (0.0629667), correlation (0.457096)*/,
                    7,-6, 10,12/*mean (0.0653846), correlation (0.445623)*/,
                    -9,-13, -8,-8/*mean (0.0858749), correlation (0.449789)*/,
                    -5,-13, -5,-2/*mean (0.122402), correlation (0.450201)*/,
                    8,-8, 9,-13/*mean (0.125416), correlation (0.453224)*/,
                    -9,-11, -9,0/*mean (0.130128), correlation (0.458724)*/,
                    1,-8, 1,-2/*mean (0.132467), correlation (0.440133)*/,
                    7,-4, 9,1/*mean (0.132692), correlation (0.454)*/,
                    -2,1, -1,-4/*mean (0.135695), correlation (0.455739)*/,
                    11,-6, 12,-11/*mean (0.142904), correlation (0.446114)*/,
                    -12,-9, -6,4/*mean (0.146165), correlation (0.451473)*/,
                    3,7, 7,12/*mean (0.147627), correlation (0.456643)*/,
                    5,5, 10,8/*mean (0.152901), correlation (0.455036)*/,
                    0,-4, 2,8/*mean (0.167083), correlation (0.459315)*/,
                    -9,12, -5,-13/*mean (0.173234), correlation (0.454706)*/,
                    0,7, 2,12/*mean (0.18312), correlation (0.433855)*/,
                    -1,2, 1,7/*mean (0.185504), correlation (0.443838)*/,
                    5,11, 7,-9/*mean (0.185706), correlation (0.451123)*/,
                    3,5, 6,-8/*mean (0.188968), correlation (0.455808)*/,
                    -13,-4, -8,9/*mean (0.191667), correlation (0.459128)*/,
                    -5,9, -3,-3/*mean (0.193196), correlation (0.458364)*/,
                    -4,-7, -3,-12/*mean (0.196536), correlation (0.455782)*/,
                    6,5, 8,0/*mean (0.1972), correlation (0.450481)*/,
                    -7,6, -6,12/*mean (0.199438), correlation (0.458156)*/,
                    -13,6, -5,-2/*mean (0.211224), correlation (0.449548)*/,
                    1,-10, 3,10/*mean (0.211718), correlation (0.440606)*/,
                    4,1, 8,-4/*mean (0.213034), correlation (0.443177)*/,
                    -2,-2, 2,-13/*mean (0.234334), correlation (0.455304)*/,
                    2,-12, 12,12/*mean (0.235684), correlation (0.443436)*/,
                    -2,-13, 0,-6/*mean (0.237674), correlation (0.452525)*/,
                    4,1, 9,3/*mean (0.23962), correlation (0.444824)*/,
                    -6,-10, -3,-5/*mean (0.248459), correlation (0.439621)*/,
                    -3,-13, -1,1/*mean (0.249505), correlation (0.456666)*/,
                    7,5, 12,-11/*mean (0.00119208), correlation (0.495466)*/,
                    4,-2, 5,-7/*mean (0.00372245), correlation (0.484214)*/,
                    -13,9, -9,-5/*mean (0.00741116), correlation (0.499854)*/,
                    7,1, 8,6/*mean (0.0208952), correlation (0.499773)*/,
                    7,-8, 7,6/*mean (0.0220085), correlation (0.501609)*/,
                    -7,-4, -7,1/*mean (0.0233806), correlation (0.496568)*/,
                    -8,11, -7,-8/*mean (0.0236505), correlation (0.489719)*/,
                    -13,6, -12,-8/*mean (0.0268781), correlation (0.503487)*/,
                    2,4, 3,9/*mean (0.0323324), correlation (0.501938)*/,
                    10,-5, 12,3/*mean (0.0399235), correlation (0.494029)*/,
                    -6,-5, -6,7/*mean (0.0420153), correlation (0.486579)*/,
                    8,-3, 9,-8/*mean (0.0548021), correlation (0.484237)*/,
                    2,-12, 2,8/*mean (0.0616622), correlation (0.496642)*/,
                    -11,-2, -10,3/*mean (0.0627755), correlation (0.498563)*/,
                    -12,-13, -7,-9/*mean (0.0829622), correlation (0.495491)*/,
                    -11,0, -10,-5/*mean (0.0843342), correlation (0.487146)*/,
                    5,-3, 11,8/*mean (0.0929937), correlation (0.502315)*/,
                    -2,-13, -1,12/*mean (0.113327), correlation (0.48941)*/,
                    -1,-8, 0,9/*mean (0.132119), correlation (0.467268)*/,
                    -13,-11, -12,-5/*mean (0.136269), correlation (0.498771)*/,
                    -10,-2, -10,11/*mean (0.142173), correlation (0.498714)*/,
                    -3,9, -2,-13/*mean (0.144141), correlation (0.491973)*/,
                    2,-3, 3,2/*mean (0.14892), correlation (0.500782)*/,
                    -9,-13, -4,0/*mean (0.150371), correlation (0.498211)*/,
                    -4,6, -3,-10/*mean (0.152159), correlation (0.495547)*/,
                    -4,12, -2,-7/*mean (0.156152), correlation (0.496925)*/,
                    -6,-11, -4,9/*mean (0.15749), correlation (0.499222)*/,
                    6,-3, 6,11/*mean (0.159211), correlation (0.503821)*/,
                    -13,11, -5,5/*mean (0.162427), correlation (0.501907)*/,
                    11,11, 12,6/*mean (0.16652), correlation (0.497632)*/,
                    7,-5, 12,-2/*mean (0.169141), correlation (0.484474)*/,
                    -1,12, 0,7/*mean (0.169456), correlation (0.495339)*/,
                    -4,-8, -3,-2/*mean (0.171457), correlation (0.487251)*/,
                    -7,1, -6,7/*mean (0.175), correlation (0.500024)*/,
                    -13,-12, -8,-13/*mean (0.175866), correlation (0.497523)*/,
                    -7,-2, -6,-8/*mean (0.178273), correlation (0.501854)*/,
                    -8,5, -6,-9/*mean (0.181107), correlation (0.494888)*/,
                    -5,-1, -4,5/*mean (0.190227), correlation (0.482557)*/,
                    -13,7, -8,10/*mean (0.196739), correlation (0.496503)*/,
                    1,5, 5,-13/*mean (0.19973), correlation (0.499759)*/,
                    1,0, 10,-13/*mean (0.204465), correlation (0.49873)*/,
                    9,12, 10,-1/*mean (0.209334), correlation (0.49063)*/,
                    5,-8, 10,-9/*mean (0.211134), correlation (0.503011)*/,
                    -1,11, 1,-13/*mean (0.212), correlation (0.499414)*/,
                    -9,-3, -6,2/*mean (0.212168), correlation (0.480739)*/,
                    -1,-10, 1,12/*mean (0.212731), correlation (0.502523)*/,
                    -13,1, -8,-10/*mean (0.21327), correlation (0.489786)*/,
                    8,-11, 10,-6/*mean (0.214159), correlation (0.488246)*/,
                    2,-13, 3,-6/*mean (0.216993), correlation (0.50287)*/,
                    7,-13, 12,-9/*mean (0.223639), correlation (0.470502)*/,
                    -10,-10, -5,-7/*mean (0.224089), correlation (0.500852)*/,
                    -10,-8, -8,-13/*mean (0.228666), correlation (0.502629)*/,
                    4,-6, 8,5/*mean (0.22906), correlation (0.498305)*/,
                    3,12, 8,-13/*mean (0.233378), correlation (0.503825)*/,
                    -4,2, -3,-3/*mean (0.234323), correlation (0.476692)*/,
                    5,-13, 10,-12/*mean (0.236392), correlation (0.475462)*/,
                    4,-13, 5,-1/*mean (0.236842), correlation (0.504132)*/,
                    -9,9, -4,3/*mean (0.236977), correlation (0.497739)*/,
                    0,3, 3,-9/*mean (0.24314), correlation (0.499398)*/,
                    -12,1, -6,1/*mean (0.243297), correlation (0.489447)*/,
                    3,2, 4,-8/*mean (0.00155196), correlation (0.553496)*/,
                    -10,-10, -10,9/*mean (0.00239541), correlation (0.54297)*/,
                    8,-13, 12,12/*mean (0.0034413), correlation (0.544361)*/,
                    -8,-12, -6,-5/*mean (0.003565), correlation (0.551225)*/,
                    2,2, 3,7/*mean (0.00835583), correlation (0.55285)*/,
                    10,6, 11,-8/*mean (0.00885065), correlation (0.540913)*/,
                    6,8, 8,-12/*mean (0.0101552), correlation (0.551085)*/,
                    -7,10, -6,5/*mean (0.0102227), correlation (0.533635)*/,
                    -3,-9, -3,9/*mean (0.0110211), correlation (0.543121)*/,
                    -1,-13, -1,5/*mean (0.0113473), correlation (0.550173)*/,
                    -3,-7, -3,4/*mean (0.0140913), correlation (0.554774)*/,
                    -8,-2, -8,3/*mean (0.017049), correlation (0.55461)*/,
                    4,2, 12,12/*mean (0.01778), correlation (0.546921)*/,
                    2,-5, 3,11/*mean (0.0224022), correlation (0.549667)*/,
                    6,-9, 11,-13/*mean (0.029161), correlation (0.546295)*/,
                    3,-1, 7,12/*mean (0.0303081), correlation (0.548599)*/,
                    11,-1, 12,4/*mean (0.0355151), correlation (0.523943)*/,
                    -3,0, -3,6/*mean (0.0417904), correlation (0.543395)*/,
                    4,-11, 4,12/*mean (0.0487292), correlation (0.542818)*/,
                    2,-4, 2,1/*mean (0.0575124), correlation (0.554888)*/,
                    -10,-6, -8,1/*mean (0.0594242), correlation (0.544026)*/,
                    -13,7, -11,1/*mean (0.0597391), correlation (0.550524)*/,
                    -13,12, -11,-13/*mean (0.0608974), correlation (0.55383)*/,
                    6,0, 11,-13/*mean (0.065126), correlation (0.552006)*/,
                    0,-1, 1,4/*mean (0.074224), correlation (0.546372)*/,
                    -13,3, -9,-2/*mean (0.0808592), correlation (0.554875)*/,
                    -9,8, -6,-3/*mean (0.0883378), correlation (0.551178)*/,
                    -13,-6, -8,-2/*mean (0.0901035), correlation (0.548446)*/,
                    5,-9, 8,10/*mean (0.0949843), correlation (0.554694)*/,
                    2,7, 3,-9/*mean (0.0994152), correlation (0.550979)*/,
                    -1,-6, -1,-1/*mean (0.10045), correlation (0.552714)*/,
                    9,5, 11,-2/*mean (0.100686), correlation (0.552594)*/,
                    11,-3, 12,-8/*mean (0.101091), correlation (0.532394)*/,
                    3,0, 3,5/*mean (0.101147), correlation (0.525576)*/,
                    -1,4, 0,10/*mean (0.105263), correlation (0.531498)*/,
                    3,-6, 4,5/*mean (0.110785), correlation (0.540491)*/,
                    -13,0, -10,5/*mean (0.112798), correlation (0.536582)*/,
                    5,8, 12,11/*mean (0.114181), correlation (0.555793)*/,
                    8,9, 9,-6/*mean (0.117431), correlation (0.553763)*/,
                    7,-4, 8,-12/*mean (0.118522), correlation (0.553452)*/,
                    -10,4, -10,9/*mean (0.12094), correlation (0.554785)*/,
                    7,3, 12,4/*mean (0.122582), correlation (0.555825)*/,
                    9,-7, 10,-2/*mean (0.124978), correlation (0.549846)*/,
                    7,0, 12,-2/*mean (0.127002), correlation (0.537452)*/,
                    -1,-6, 0,-11/*mean (0.127148), correlation (0.547401)*/
            };

    /**
     * @brief           构造 ORB 特征提取器并预计算所有静态数据
     * @param _nfeatures    期望提取的总特征点数
     * @param _scaleFactor  金字塔尺度因子
     * @param _nlevels      金字塔层数
     * @param _iniThFAST    FAST 初始阈值
     * @param _minThFAST    FAST 最小阈值
     *
     * 构造函数会预先完成三类准备工作：
     * 1. 计算每层的尺度及其方差；
     * 2. 把总特征点数按层分配；
     * 3. 准备 BRIEF 采样模板和方向估计所需的圆形边界表。
     */
    ORBextractor::ORBextractor(int _nfeatures, float _scaleFactor, int _nlevels,
                               int _iniThFAST, int _minThFAST):
            nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels),
            iniThFAST(_iniThFAST), minThFAST(_minThFAST)
    {
        // 预计算每一层相对于原图的尺度，以及该尺度对应的 sigma^2。
        // 这些量在特征金字塔、尺度恢复和匹配加权时都会使用。
        mvScaleFactor.resize(nlevels);
        mvLevelSigma2.resize(nlevels);
        mvScaleFactor[0]=1.0f;
        mvLevelSigma2[0]=1.0f;
        for(int i=1; i<nlevels; i++)
        {
            mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;
            mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
        }

        // 同时保存倒数，避免后续重复做除法。
        mvInvScaleFactor.resize(nlevels);
        mvInvLevelSigma2.resize(nlevels);
        for(int i=0; i<nlevels; i++)
        {
            mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
            mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
        }

        // 为图像金字塔预留每一层的存储空间。
        mvImagePyramid.resize(nlevels);

        // 按几何级数为每一层分配期望保留的特征点数。
        // 低层（高分辨率）一般分配更多，高层（低分辨率）逐渐减少。
        mnFeaturesPerLevel.resize(nlevels);
        float factor = 1.0f / scaleFactor;
        float nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels));

        int sumFeatures = 0;
        for( int level = 0; level < nlevels-1; level++ )
        {
            mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
            sumFeatures += mnFeaturesPerLevel[level];
            nDesiredFeaturesPerScale *= factor;
        }
        mnFeaturesPerLevel[nlevels-1] = std::max(nfeatures - sumFeatures, 0);

        // 读取固定的 BRIEF 采样模板。
        const int npoints = 512;
        const Point* pattern0 = (const Point*)bit_pattern_31_;
        std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

        // 为方向估计预计算圆形 patch 每一行允许访问的最大水平半径。
        // 例如在 v=0 时半径最大，越接近上下边缘半径越小。
        umax.resize(HALF_PATCH_SIZE + 1);

        int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
        int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
        const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;

        // 根据圆方程 u^2 + v^2 = r^2 计算每一行的最大 u。
        for (v = 0; v <= vmax; ++v)
            umax[v] = cvRound(sqrt(hp2 - v * v));

        // 为了保证 patch 关于中心对称，对后半部分进行镜像修正。
        for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
        {
            while (umax[v0] == umax[v0 + 1])
                ++v0;
            umax[v] = v0;
            ++v0;
        }
    }

    /**
     * @brief           为一组关键点计算方向
     * @param image      当前尺度层图像
     * @param keypoints  待计算方向的关键点集合
     * @param umax       圆形 patch 行边界表
     */
    static void computeOrientation(const Mat& image, vector<KeyPoint>& keypoints, const vector<int>& umax)
    {
        // 对当前层所有关键点逐个调用 IC_Angle。
        for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                     keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
        {
            keypoint->angle = IC_Angle(image, keypoint->pt, umax);
        }
    }

    /**
     * @brief       将当前结点划分为四个子结点并分配关键点
     * @param n1    左上子结点
     * @param n2    右上子结点
     * @param n3    左下子结点
     * @param n4    右下子结点
     *
     * 四叉树均匀化的核心操作之一。当前结点会按几何中心切成四块，
     * 然后把该结点中的关键点按坐标送入对应子结点中。
     */
    void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4)
    {
        // 计算当前矩形在 x / y 方向上的一半尺寸。
        // 使用 ceil 可以避免奇数宽高时出现像素遗漏。
        const int halfX = ceil(static_cast<float>(UR.x-UL.x)/2);
        const int halfY = ceil(static_cast<float>(BR.y-UL.y)/2);

        // 依次定义四个子结点的矩形边界。
        n1.UL = UL;
        n1.UR = cv::Point2i(UL.x+halfX,UL.y);
        n1.BL = cv::Point2i(UL.x,UL.y+halfY);
        n1.BR = cv::Point2i(UL.x+halfX,UL.y+halfY);
        n1.vKeys.reserve(vKeys.size());

        n2.UL = n1.UR;
        n2.UR = UR;
        n2.BL = n1.BR;
        n2.BR = cv::Point2i(UR.x,UL.y+halfY);
        n2.vKeys.reserve(vKeys.size());

        n3.UL = n1.BL;
        n3.UR = n1.BR;
        n3.BL = BL;
        n3.BR = cv::Point2i(n1.BR.x,BL.y);
        n3.vKeys.reserve(vKeys.size());

        n4.UL = n3.UR;
        n4.UR = n2.BR;
        n4.BL = n3.BR;
        n4.BR = BR;
        n4.vKeys.reserve(vKeys.size());

        // 遍历当前结点内的所有关键点，按照坐标落入的象限分发到子结点。
        for(size_t i=0;i<vKeys.size();i++)
        {
            const cv::KeyPoint &kp = vKeys[i];
            if(kp.pt.x<n1.UR.x)
            {
                if(kp.pt.y<n1.BR.y)
                    n1.vKeys.push_back(kp);
                else
                    n3.vKeys.push_back(kp);
            }
            else if(kp.pt.y<n1.BR.y)
                n2.vKeys.push_back(kp);
            else
                n4.vKeys.push_back(kp);
        }

        // 如果一个子结点只剩 1 个关键点，就不再继续细分。
        // 因为继续分裂既没有意义，也无法提升空间分布。
        if(n1.vKeys.size()==1)
            n1.bNoMore = true;
        if(n2.vKeys.size()==1)
            n2.bNoMore = true;
        if(n3.vKeys.size()==1)
            n3.bNoMore = true;
        if(n4.vKeys.size()==1)
            n4.bNoMore = true;

    }

    /**
     * @brief      比较两个待分裂结点的优先级
     * @param e1   结点 1，first 为关键点数，second 为结点指针
     * @param e2   结点 2，first 为关键点数，second 为结点指针
     * @return     若 e1 排在 e2 前面则返回 true
     *
     * 排序规则：
     * 1. 优先按结点内关键点数量从小到大排序；
     * 2. 若数量相同，再按左上角 x 坐标排序。
     *
     * 后续代码会从排序结果尾部开始遍历，因此实际会优先扩展“关键点更多”的结点。
     */
    static bool compareNodes(pair<int,ExtractorNode*>& e1, pair<int,ExtractorNode*>& e2){
        if(e1.first < e2.first){
            return true;
        }
        else if(e1.first > e2.first){
            return false;
        }
        else{
            if(e1.second->UL.x < e2.second->UL.x){
                return true;
            }
            else{
                return false;
            }
        }
    }

    /**
     * @brief                   使用四叉树在给定区域内均匀分配关键点
     * @param vToDistributeKeys 待分配的候选关键点
     * @param minX              区域左边界
     * @param maxX              区域右边界
     * @param minY              区域上边界
     * @param maxY              区域下边界
     * @param N                 希望最终保留的关键点数
     * @param level             当前金字塔层级，当前实现中未直接使用
     * @return                  经过四叉树均匀化后的关键点集合
     *
     * 该函数的目标不是“找到响应值最大的 N 个点”，而是“找到空间分布更均匀的 N 个点”。
     * 具体做法是：
     * 1. 先按照图像宽高比，把图像粗分成若干初始结点；
     * 2. 不断分裂包含多个关键点的结点；
     * 3. 当结点数接近目标数 N 时停止；
     * 4. 每个叶子结点中只保留响应值最大的那个点。
     */
    vector<cv::KeyPoint> ORBextractor::DistributeOctTree(const vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                                         const int &maxX, const int &minY, const int &maxY, const int &N, const int &level)
    {
        // 根据区域宽高比决定初始横向结点个数。
        // 这样做的目的是让初始划分尽量接近“方形”，避免极端长条结点。
        const int nIni = round(static_cast<float>(maxX-minX)/(maxY-minY));

        // 每个初始结点在 x 方向上的宽度。
        const float hX = static_cast<float>(maxX-minX)/nIni;

        // 使用链表保存当前所有待处理结点。
        // 链表在插入/删除时稳定，不会像 vector 一样导致指针失效。
        list<ExtractorNode> lNodes;

        // 保存初始结点指针，便于把候选点快速分配到对应结点。
        vector<ExtractorNode*> vpIniNodes;
        vpIniNodes.resize(nIni);

        for(int i=0; i<nIni; i++)
        {
            ExtractorNode ni;

            // 注意：传入的候选点坐标此时是相对于搜索区域左上角的局部坐标，
            // 因此这里的初始结点边界从 0 开始定义，而不是从 minX / minY 开始。
            ni.UL = cv::Point2i(hX*static_cast<float>(i),0);
            ni.UR = cv::Point2i(hX*static_cast<float>(i+1),0);
            ni.BL = cv::Point2i(ni.UL.x,maxY-minY);
            ni.BR = cv::Point2i(ni.UR.x,maxY-minY);
            ni.vKeys.reserve(vToDistributeKeys.size());

            lNodes.push_back(ni);
            vpIniNodes[i] = &lNodes.back();
        }

        // 先把所有候选关键点投放到初始结点中。
        for(size_t i=0;i<vToDistributeKeys.size();i++)
        {
            const cv::KeyPoint &kp = vToDistributeKeys[i];
            vpIniNodes[kp.pt.x/hX]->vKeys.push_back(kp);
        }

        list<ExtractorNode>::iterator lit = lNodes.begin();

        // 清理初始结点：
        // 1. 空结点直接删除；
        // 2. 只有 1 个点的结点标记为不可继续分裂。
        while(lit!=lNodes.end())
        {
            if(lit->vKeys.size()==1)
            {
                lit->bNoMore=true;
                lit++;
            }
            else if(lit->vKeys.empty())
                lit = lNodes.erase(lit);
            else
                lit++;
        }

        bool bFinish = false;

        // 仅用于表示分裂轮次，当前实现没有进一步使用，便于调试观察流程。
        int iteration = 0;

        // 保存“可继续扩展的结点”的信息：
        // pair.first  = 结点中的关键点数量
        // pair.second = 对应结点指针
        // 当结点数即将超过目标数量 N 时，会优先扩展这些结点中“点更多”的那些。
        vector<pair<int,ExtractorNode*> > vSizeAndPointerToNode;
        vSizeAndPointerToNode.reserve(lNodes.size()*4);

        // 不断迭代分裂，直到结点数量足够，或者再分裂也不会产生更多有效结点。
        while(!bFinish)
        {
            iteration++;

            // 记录本轮开始前的结点数，用于判断本轮是否还有进展。
            int prevSize = lNodes.size();

            lit = lNodes.begin();

            // 统计本轮理论上还有多少个结点可继续扩展。
            int nToExpand = 0;

            vSizeAndPointerToNode.clear();

            while(lit!=lNodes.end())
            {
                if(lit->bNoMore)
                {
                    // 该结点只有 1 个点或已不可继续细分，直接跳过。
                    lit++;
                    continue;
                }
                else
                {
                    // 当前结点中还有多个候选点，继续向下分裂成四个子结点。
                    ExtractorNode n1,n2,n3,n4;
                    lit->DivideNode(n1,n2,n3,n4);

                    // 只有包含关键点的子结点才保留。
                    // 对于包含多个点的子结点，记录下来，后面可能还要继续扩展。
                    if(n1.vKeys.size()>0)
                    {
                        lNodes.push_front(n1);
                        if(n1.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n2.vKeys.size()>0)
                    {
                        lNodes.push_front(n2);
                        if(n2.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n3.vKeys.size()>0)
                    {
                        lNodes.push_front(n3);
                        if(n3.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n4.vKeys.size()>0)
                    {
                        lNodes.push_front(n4);
                        if(n4.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    // 当前父结点已经被子结点替代，因此从链表中删除。
                    lit=lNodes.erase(lit);
                    continue;
                }
            }

            // 两种情况可以终止：
            // 1. 当前结点数已经达到或超过目标特征点数；
            // 2. 本轮分裂前后结点数没变化，说明已经无法继续有效细分。
            if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
            {
                bFinish = true;
            }
            else if(((int)lNodes.size()+nToExpand*3)>N)
            {
                // 若把所有可扩展结点都继续完整分裂，结点数会明显超过目标 N。
                // 此时进入“受控扩展”模式：
                // 只优先分裂那些包含关键点更多的结点，尽量逼近 N。

                while(!bFinish)
                {

                    prevSize = lNodes.size();

                    // 拷贝当前的候选扩展结点列表，并清空原列表，准备记录下一轮。
                    vector<pair<int,ExtractorNode*> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                    vSizeAndPointerToNode.clear();

                    // 从小到大排序，但后面会逆序遍历，因此最先处理的是“大结点”。
                    sort(vPrevSizeAndPointerToNode.begin(),vPrevSizeAndPointerToNode.end(),compareNodes);
                    for(int j=vPrevSizeAndPointerToNode.size()-1;j>=0;j--)
                    {
                        ExtractorNode n1,n2,n3,n4;
                        vPrevSizeAndPointerToNode[j].second->DivideNode(n1,n2,n3,n4);

                        // 只保留非空子结点，并继续记录仍可扩展的结点。
                        if(n1.vKeys.size()>0)
                        {
                            lNodes.push_front(n1);
                            if(n1.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if(n2.vKeys.size()>0)
                        {
                            lNodes.push_front(n2);
                            if(n2.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if(n3.vKeys.size()>0)
                        {
                            lNodes.push_front(n3);
                            if(n3.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if(n4.vKeys.size()>0)
                        {
                            lNodes.push_front(n4);
                            if(n4.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }

                        // 删除已被分裂的旧父结点。
                        lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                        // 结点数达到目标后即可立即退出。
                        if((int)lNodes.size()>=N)
                            break;
                    }

                    // 如果已经达到目标，或者本轮没有新增有效结点，则停止。
                    if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
                        bFinish = true;

                }
            }
        }

        // 到这里，每个链表结点都可视为最终叶子结点。
        // 为了在“均匀分布”和“角点质量”之间平衡，
        // 每个叶子结点中只保留响应值最大的那个关键点。
        vector<cv::KeyPoint> vResultKeys;
        vResultKeys.reserve(nfeatures);
        for(list<ExtractorNode>::iterator lit=lNodes.begin(); lit!=lNodes.end(); lit++)
        {
            vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;
            cv::KeyPoint* pKP = &vNodeKeys[0];
            float maxResponse = pKP->response;

            // 在当前叶子结点内部做一次“局部最优”选择。
            for(size_t k=1;k<vNodeKeys.size();k++)
            {
                if(vNodeKeys[k].response>maxResponse)
                {
                    pKP = &vNodeKeys[k];
                    maxResponse = vNodeKeys[k].response;
                }
            }

            vResultKeys.push_back(*pKP);
        }

        return vResultKeys;
    }

    /**
     * @brief             在图像金字塔每一层上提取关键点，并利用四叉树实现均匀分布
     * @param allKeypoints 输出每一层的关键点集合
     *
     * 该函数是关键点检测阶段的主干流程：
     * 1. 在每层图像上按固定网格划分搜索区域；
     * 2. 每个网格内先用较高阈值运行 FAST，若失败再降低阈值；
     * 3. 将全部候选点交给四叉树做均匀化；
     * 4. 补回边界偏移、层号和尺度；
     * 5. 最后统一计算方向。
     */
    void ORBextractor::ComputeKeyPointsOctTree(vector<vector<KeyPoint> >& allKeypoints)
    {
        // allKeypoints[level] 存放金字塔第 level 层的最终关键点。
        allKeypoints.resize(nlevels);

        // 每个初始网格希望接近的边长。不是最终 patch 大小，而是 FAST 的搜索网格尺寸。
        const float W = 35;

        for (int level = 0; level < nlevels; ++level)
        {
            // 由于方向和描述子都需要访问关键点周围的 patch，
            // 因此边缘附近的一圈区域不能作为检测中心。
            // 这里减 3 / 加 3 是为了配合后面网格搜索窗口的额外扩展。
            const int minBorderX = EDGE_THRESHOLD-3;
            const int minBorderY = minBorderX;
            const int maxBorderX = mvImagePyramid[level].cols-EDGE_THRESHOLD+3;
            const int maxBorderY = mvImagePyramid[level].rows-EDGE_THRESHOLD+3;

            // 先收集当前层所有 FAST 候选点，再做四叉树筛选。
            vector<cv::KeyPoint> vToDistributeKeys;
            vToDistributeKeys.reserve(nfeatures*10);

            const float width = (maxBorderX-minBorderX);
            const float height = (maxBorderY-minBorderY);

            // 按近似 35x35 的尺度将可检测区域划成网格。
            const int nCols = width/W;
            const int nRows = height/W;
            const int wCell = ceil(width/nCols);
            const int hCell = ceil(height/nRows);

            for(int i=0; i<nRows; i++)
            {
                const float iniY =minBorderY+i*hCell;
                float maxY = iniY+hCell+6;

                // 超出底边则直接跳过；否则把最后一行网格裁到合法范围内。
                if(iniY>=maxBorderY-3)
                    continue;
                if(maxY>maxBorderY)
                    maxY = maxBorderY;

                for(int j=0; j<nCols; j++)
                {
                    const float iniX =minBorderX+j*wCell;
                    float maxX = iniX+wCell+6;

                    // 同理，控制右侧边界合法。
                    if(iniX>=maxBorderX-6)
                        continue;
                    if(maxX>maxBorderX)
                        maxX = maxBorderX;

                    vector<cv::KeyPoint> vKeysCell;

                    // 先以较高阈值提取 FAST 角点。
                    // nonmaxSuppression=true，表示保留局部极大响应点。
                    FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                         vKeysCell,iniThFAST,true);

                    /*if(bRight && j <= 13){
                        FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                             vKeysCell,10,true);
                    }
                    else if(!bRight && j >= 16){
                        FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                             vKeysCell,10,true);
                    }
                    else{
                        FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                             vKeysCell,iniThFAST,true);
                    }*/


                    if(vKeysCell.empty())
                    {
                        // 如果当前网格没有检测到角点，则降低阈值再试一次，
                        // 以避免纹理较弱区域完全没有特征。
                        FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                             vKeysCell,minThFAST,true);
                        /*if(bRight && j <= 13){
                            FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                                 vKeysCell,5,true);
                        }
                        else if(!bRight && j >= 16){
                            FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                                 vKeysCell,5,true);
                        }
                        else{
                            FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                                 vKeysCell,minThFAST,true);
                        }*/
                    }

                    if(!vKeysCell.empty())
                    {
                        for(vector<cv::KeyPoint>::iterator vit=vKeysCell.begin(); vit!=vKeysCell.end();vit++)
                        {
                            // FAST 在 cell 子图上返回的是局部坐标，
                            // 这里先映射回“当前可检测区域”的坐标系。
                            (*vit).pt.x+=j*wCell;
                            (*vit).pt.y+=i*hCell;
                            vToDistributeKeys.push_back(*vit);
                        }
                    }

                }
            }

            vector<KeyPoint> & keypoints = allKeypoints[level];
            keypoints.reserve(nfeatures);

            // 候选点通常远多于期望点数，使用四叉树让保留下来的点更均匀地覆盖整幅图像。
            keypoints = DistributeOctTree(vToDistributeKeys, minBorderX, maxBorderX,
                                          minBorderY, maxBorderY,mnFeaturesPerLevel[level], level);

            const int scaledPatchSize = PATCH_SIZE*mvScaleFactor[level];

            // 把关键点坐标从“可检测区域局部坐标系”恢复到当前金字塔图像坐标系，
            // 同时记录所属层号和对应 patch 尺度。
            const int nkps = keypoints.size();
            for(int i=0; i<nkps ; i++)
            {
                keypoints[i].pt.x+=minBorderX;
                keypoints[i].pt.y+=minBorderY;
                keypoints[i].octave=level;
                keypoints[i].size = scaledPatchSize;
            }
        }

        // 关键点位置确定后，逐层计算方向。
        for (int level = 0; level < nlevels; ++level)
            computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
    }

    /**
     * @brief             使用旧版网格配额策略提取关键点
     * @param allKeypoints 输出每一层的关键点集合
     *
     * 与四叉树版本不同，这个旧实现先把图像划分为规则网格，
     * 再给每个网格分配一个大致相同的特征点预算，并把富余预算重新分发。
     * 该方法保留在代码中主要用于参考，默认主流程不再调用它。
     */
    void ORBextractor::ComputeKeyPointsOld(std::vector<std::vector<KeyPoint> > &allKeypoints)
    {
        allKeypoints.resize(nlevels);

        // 用原图宽高比估计每层网格的行列数，使网格形状尽量和图像比例一致。
        float imageRatio = (float)mvImagePyramid[0].cols/mvImagePyramid[0].rows;

        for (int level = 0; level < nlevels; ++level)
        {
            // 当前层希望保留的关键点总数。
            const int nDesiredFeatures = mnFeaturesPerLevel[level];

            // 旧实现会先根据经验公式估计网格行列数。
            const int levelCols = sqrt((float)nDesiredFeatures/(5*imageRatio));
            const int levelRows = imageRatio*levelCols;

            // 同样需要避开边缘区域，因为后续还要计算 patch。
            const int minBorderX = EDGE_THRESHOLD;
            const int minBorderY = minBorderX;
            const int maxBorderX = mvImagePyramid[level].cols-EDGE_THRESHOLD;
            const int maxBorderY = mvImagePyramid[level].rows-EDGE_THRESHOLD;

            const int W = maxBorderX - minBorderX;
            const int H = maxBorderY - minBorderY;
            const int cellW = ceil((float)W/levelCols);
            const int cellH = ceil((float)H/levelRows);

            // 统计总网格数，并估计每个网格理想应保留多少个特征点。
            const int nCells = levelRows*levelCols;
            const int nfeaturesCell = ceil((float)nDesiredFeatures/nCells);

            // 保存每个网格内检测出的 FAST 候选关键点。
            vector<vector<vector<KeyPoint> > > cellKeyPoints(levelRows, vector<vector<KeyPoint> >(levelCols));

            // nToRetain: 每个网格最终应保留多少个点
            // nTotal:    每个网格最初检测出了多少点
            // bNoMore:   该网格是否已经没有额外冗余，无法继续接收新的配额
            vector<vector<int> > nToRetain(levelRows,vector<int>(levelCols,0));
            vector<vector<int> > nTotal(levelRows,vector<int>(levelCols,0));
            vector<vector<bool> > bNoMore(levelRows,vector<bool>(levelCols,false));
            vector<int> iniXCol(levelCols);
            vector<int> iniYRow(levelRows);
            int nNoMore = 0;
            int nToDistribute = 0;


            // 在 y 方向上，网格窗口比理论 cell 略大 6 像素，用于覆盖边界过渡区域。
            float hY = cellH + 6;

            for(int i=0; i<levelRows; i++)
            {
                const float iniY = minBorderY + i*cellH - 3;
                iniYRow[i] = iniY;

                // 最后一行网格需要裁剪到图像范围内。
                if(i == levelRows-1)
                {
                    hY = maxBorderY+3-iniY;
                    if(hY<=0)
                        continue;
                }

                float hX = cellW + 6;

                for(int j=0; j<levelCols; j++)
                {
                    float iniX;

                    if(i==0)
                    {
                        // 第一行顺便把每列网格的起始 x 位置记录下来，后面复用。
                        iniX = minBorderX + j*cellW - 3;
                        iniXCol[j] = iniX;
                    }
                    else
                    {
                        iniX = iniXCol[j];
                    }


                    // 最后一列网格同样需要进行边界裁剪。
                    if(j == levelCols-1)
                    {
                        hX = maxBorderX+3-iniX;
                        if(hX<=0)
                            continue;
                    }


                    // 取出当前网格对应的图像块。
                    Mat cellImage = mvImagePyramid[level].rowRange(iniY,iniY+hY).colRange(iniX,iniX+hX);

                    cellKeyPoints[i][j].reserve(nfeaturesCell*5);

                    // 先用较高阈值提取 FAST。
                    FAST(cellImage,cellKeyPoints[i][j],iniThFAST,true);

                    if(cellKeyPoints[i][j].size()<=3)
                    {
                        // 点太少时，清空并降低阈值重新提取。
                        cellKeyPoints[i][j].clear();

                        FAST(cellImage,cellKeyPoints[i][j],minThFAST,true);
                    }


                    // 记录该网格的候选点总数。
                    const int nKeys = cellKeyPoints[i][j].size();
                    nTotal[i][j] = nKeys;

                    if(nKeys>nfeaturesCell)
                    {
                        // 候选点过多时，先按统一配额保留，后面再按评分裁剪。
                        nToRetain[i][j] = nfeaturesCell;
                        bNoMore[i][j] = false;
                    }
                    else
                    {
                        // 候选点不足时，能保留多少就保留多少，
                        // 剩余预算后面再分给其它网格。
                        nToRetain[i][j] = nKeys;
                        nToDistribute += nfeaturesCell-nKeys;
                        bNoMore[i][j] = true;
                        nNoMore++;
                    }

                }
            }


            // 把空闲预算重新分配给那些“还有更多候选点可供保留”的网格。

            while(nToDistribute>0 && nNoMore<nCells)
            {
                int nNewFeaturesCell = nfeaturesCell + ceil((float)nToDistribute/(nCells-nNoMore));
                nToDistribute = 0;

                for(int i=0; i<levelRows; i++)
                {
                    for(int j=0; j<levelCols; j++)
                    {
                        if(!bNoMore[i][j])
                        {
                            if(nTotal[i][j]>nNewFeaturesCell)
                            {
                                // 该网格候选点足够多，可以接受新的更大配额。
                                nToRetain[i][j] = nNewFeaturesCell;
                                bNoMore[i][j] = false;
                            }
                            else
                            {
                                // 该网格已经装不下更多预算，剩余预算继续往外分配。
                                nToRetain[i][j] = nTotal[i][j];
                                nToDistribute += nNewFeaturesCell-nTotal[i][j];
                                bNoMore[i][j] = true;
                                nNoMore++;
                            }
                        }
                    }
                }
            }

            vector<KeyPoint> & keypoints = allKeypoints[level];
            keypoints.reserve(nDesiredFeatures*2);

            const int scaledPatchSize = PATCH_SIZE*mvScaleFactor[level];

            // 现在真正按评分保留每个网格中的最佳点，并恢复到整层图像坐标系。
            for(int i=0; i<levelRows; i++)
            {
                for(int j=0; j<levelCols; j++)
                {
                    vector<KeyPoint> &keysCell = cellKeyPoints[i][j];

                    // 按响应值保留该网格中最好的 nToRetain[i][j] 个点。
                    KeyPointsFilter::retainBest(keysCell,nToRetain[i][j]);
                    if((int)keysCell.size()>nToRetain[i][j])
                        keysCell.resize(nToRetain[i][j]);


                    for(size_t k=0, kend=keysCell.size(); k<kend; k++)
                    {
                        // 把局部坐标映射回整层图像坐标，并填充层号/尺度信息。
                        keysCell[k].pt.x+=iniXCol[j];
                        keysCell[k].pt.y+=iniYRow[i];
                        keysCell[k].octave=level;
                        keysCell[k].size = scaledPatchSize;
                        keypoints.push_back(keysCell[k]);
                    }
                }
            }

            if((int)keypoints.size()>nDesiredFeatures)
            {
                // 如果总数仍然超过当前层预算，则在整层范围内再做一次全局截断。
                KeyPointsFilter::retainBest(keypoints,nDesiredFeatures);
                keypoints.resize(nDesiredFeatures);
            }
        }

        // 最终再统一计算方向。
        for (int level = 0; level < nlevels; ++level)
            computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
    }

    /**
     * @brief             为一组关键点计算 ORB 描述子
     * @param image        当前尺度层图像
     * @param keypoints    待计算描述子的关键点集合
     * @param descriptors  输出描述子矩阵
     * @param pattern      预定义 BRIEF 采样模式
     */
    static void computeDescriptors(const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors,
                                   const vector<Point>& pattern)
    {
        // ORB 描述子固定为 32 字节，因此矩阵列数恒为 32。
        descriptors = Mat::zeros((int)keypoints.size(), 32, CV_8UC1);

        // 对每个关键点逐个生成描述子。
        for (size_t i = 0; i < keypoints.size(); i++)
            computeOrbDescriptor(keypoints[i], image, &pattern[0], descriptors.ptr((int)i));
    }

    /**
     * @brief             ORB 提取主入口
     * @param _image       输入灰度图像
     * @param _mask        输入掩码，当前实现未使用
     * @param _keypoints   输出关键点
     * @param _descriptors 输出描述子
     * @param vLappingArea 双目重叠区域的 x 范围
     * @return             非重叠区域关键点数量；输入为空时返回 -1
     *
     * 主流程如下：
     * 1. 构建图像金字塔；
     * 2. 在每层图像中提取关键点；
     * 3. 统计总关键点数并分配描述子矩阵；
     * 4. 每层图像做高斯模糊后计算 ORB 描述子；
     * 5. 将各层关键点坐标缩放回原图；
     * 6. 按是否落入双目重叠区域重新排列输出。
     */
    int ORBextractor::operator()( InputArray _image, InputArray _mask, vector<KeyPoint>& _keypoints,
                                  OutputArray _descriptors, std::vector<int> &vLappingArea)
    {
        //cout << "[ORBextractor]: Max Features: " << nfeatures << endl;

        // 空图像无法提取特征，直接返回错误码。
        if(_image.empty())
            return -1;

        Mat image = _image.getMat();

        // ORB 描述子建立在灰度图像上，这里强制要求单通道 8 位输入。
        assert(image.type() == CV_8UC1 );

        // 第一步：构建多尺度图像金字塔。
        ComputePyramid(image);

        // 第二步：在每一层上提取关键点。
        vector < vector<KeyPoint> > allKeypoints;
        ComputeKeyPointsOctTree(allKeypoints);
        //ComputeKeyPointsOld(allKeypoints);

        Mat descriptors;

        // 统计所有尺度层上的关键点总数。
        int nkeypoints = 0;
        for (int level = 0; level < nlevels; ++level)
            nkeypoints += (int)allKeypoints[level].size();

        // 没有关键点时释放输出描述子；否则一次性分配完整矩阵。
        if( nkeypoints == 0 )
            _descriptors.release();
        else
        {
            _descriptors.create(nkeypoints, 32, CV_8U);
            descriptors = _descriptors.getMat();
        }

        //_keypoints.clear();
        //_keypoints.reserve(nkeypoints);

        // 预先分配完整输出空间，后续按索引直接写入。
        _keypoints = vector<cv::KeyPoint>(nkeypoints);

        // offset 原本用于顺序拼接每层描述子，这里保留变量；
        // 目前实际输出顺序由 monoIndex / stereoIndex 控制。
        int offset = 0;

        // ORB-SLAM3 针对双目鱼眼做了一个小优化：
        // 非重叠区域关键点从前往后放，重叠区域关键点从后往前放，
        // 这样后续匹配时可以更快地区分单目区域和双目公共区域。
        int monoIndex = 0, stereoIndex = nkeypoints-1;
        for (int level = 0; level < nlevels; ++level)
        {
            vector<KeyPoint>& keypoints = allKeypoints[level];
            int nkeypointsLevel = (int)keypoints.size();

            // 某层没有关键点时直接跳过。
            if(nkeypointsLevel==0)
                continue;

            // 描述子对噪声较敏感，先对当前层图像做一次高斯平滑。
            Mat workingMat = mvImagePyramid[level].clone();
            GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);

            // 计算当前层所有关键点的 ORB 描述子。
            //Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
            Mat desc = cv::Mat(nkeypointsLevel, 32, CV_8U);
            computeDescriptors(workingMat, keypoints, desc, pattern);

            offset += nkeypointsLevel;


            float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
            int i = 0;
            for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                         keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint){

                // 关键点最初位于各自的金字塔层坐标系中。
                // 这里统一映射回原图坐标系，方便后续跟踪和匹配直接使用。
                if (level != 0){
                    keypoint->pt *= scale;
                }

                // 如果关键点位于双目重叠区域，则放到输出数组尾部；
                // 否则放到前部。
                if(keypoint->pt.x >= vLappingArea[0] && keypoint->pt.x <= vLappingArea[1]){
                    _keypoints.at(stereoIndex) = (*keypoint);
                    desc.row(i).copyTo(descriptors.row(stereoIndex));
                    stereoIndex--;
                }
                else{
                    _keypoints.at(monoIndex) = (*keypoint);
                    desc.row(i).copyTo(descriptors.row(monoIndex));
                    monoIndex++;
                }
                i++;
            }
        }
        //cout << "[ORBextractor]: extracted " << _keypoints.size() << " KeyPoints" << endl;

        // 返回前半部分（非重叠区域）关键点数量。
        return monoIndex;
    }

    /**
     * @brief       构建图像金字塔
     * @param image 输入原始灰度图像
     *
     * 每一层图像都会额外补一圈边界，原因有两个：
     * 1. 关键点方向和描述子都要访问周围 patch；
     * 2. 这样可避免在边界附近频繁做越界判断。
     */
    void ORBextractor::ComputePyramid(cv::Mat image)
    {
        for (int level = 0; level < nlevels; ++level)
        {
            // 当前层相对于原图的缩小比例。
            float scale = mvInvScaleFactor[level];

            // 当前层真实图像尺寸，以及加上边缘扩展后的完整尺寸。
            Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
            Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
            Mat temp(wholeSize, image.type()), masktemp;

            // mvImagePyramid[level] 只是 temp 中间那块“有效图像区域”的视图。
            // temp 外围保留的边界区域用于方向和描述子计算。
            mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

            // 第 0 层直接来自原图；其余层由上一层下采样得到。
            if( level != 0 )
            {
                resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);

                // 对非首层图像补边时使用 BORDER_ISOLATED，避免 border 计算受 ROI 外部影响。
                copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                               BORDER_REFLECT_101+BORDER_ISOLATED);
            }
            else
            {
                // 首层直接对原图补边。
                copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                               BORDER_REFLECT_101);
            }
        }

    }

} //namespace ORB_SLAM
