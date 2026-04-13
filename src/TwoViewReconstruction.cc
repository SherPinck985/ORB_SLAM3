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

#include "TwoViewReconstruction.h"

#include "Converter.h"
#include "GeometricTools.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

#include<thread>


using namespace std;
namespace ORB_SLAM3
{
    /**
     * @brief 构造两视图重建器并缓存初始化参数
     * @param k          相机内参矩阵 K
     * @param sigma      像素级观测噪声标准差
     * @param iterations RANSAC 最大迭代次数
     *
     * 这里只做最基本的参数保存：
     * 1. mK 在后续由 F 推 E、重投影、三角化时反复使用；
     * 2. mSigma2 提前缓存，避免在误差检验阶段重复平方；
     * 3. mMaxIterations 控制单应矩阵和基础矩阵的 RANSAC 轮数。
     */
    TwoViewReconstruction::TwoViewReconstruction(const Eigen::Matrix3f& k, float sigma, int iterations)
    {
        mK = k;

        mSigma = sigma;
        mSigma2 = sigma*sigma;
        mMaxIterations = iterations;
    }

    /**
     * @brief 从两帧 ORB 匹配关系中恢复相对位姿和初始三维点
     * @param vKeys1          参考帧关键点
     * @param vKeys2          当前帧关键点
     * @param vMatches12      匹配表，索引来自参考帧，值指向当前帧，-1 表示无匹配
     * @param T21             输出位姿，表示从帧 1 到帧 2 的变换
     * @param vP3D            输出三维点，索引与参考帧关键点对齐
     * @param vbTriangulated  输出标记，表示对应参考帧关键点是否成功三角化
     * @return                若成功恢复出可信的两视图几何关系则返回 true
     *
     * 该函数是两视图初始化主入口。
     * 输入关键点一般由 ORBextractor 提供，而 ORBextractor 的工作链条是：
     * 1. 构建金字塔，保证多尺度鲁棒性；
     * 2. 用 FAST 在每层找角点；
     * 3. 用四叉树让角点分布更均匀；
     * 4. 用灰度质心法计算方向；
     * 5. 用旋转后的 BRIEF 生成 ORB 描述子并完成匹配。
     *
     * 到这里，特征已经提好且完成匹配，本函数只关心几何恢复：
     * 1. 整理有效匹配；
     * 2. 预生成 RANSAC 的 8 点采样集；
     * 3. 并行估计 H 和 F；
     * 4. 根据评分比值选择更合理的模型；
     * 5. 从模型中恢复位姿并三角化空间点。
     */
    bool TwoViewReconstruction::Reconstruct(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const vector<int> &vMatches12,
                                             Sophus::SE3f &T21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
    {
        // 先清空并缓存当前两帧的关键点，后续所有函数都通过成员变量访问。
        mvKeys1.clear();
        mvKeys2.clear();

        mvKeys1 = vKeys1;
        mvKeys2 = vKeys2;

        // 把输入的“稀疏匹配表”转换为紧凑的有效匹配对数组。
        // 约定：
        // 1. Frame 1 是参考帧；
        // 2. Frame 2 是当前帧；
        // 3. mvMatches12 中只保存真正匹配成功的点对。
        mvMatches12.clear();
        mvMatches12.reserve(mvKeys2.size());
        mvbMatched1.resize(mvKeys1.size());
        for(size_t i=0, iend=vMatches12.size();i<iend; i++)
        {
            if(vMatches12[i]>=0)
            {
                mvMatches12.push_back(make_pair(i,vMatches12[i]));
                mvbMatched1[i]=true;
            }
            else
                mvbMatched1[i]=false;
        }

        const int N = mvMatches12.size();

        // vAllIndices 记录所有有效匹配对在 mvMatches12 中的下标。
        // 后续 RANSAC 每轮都从这里无放回采样 8 个匹配。
        vector<size_t> vAllIndices;
        vAllIndices.reserve(N);
        vector<size_t> vAvailableIndices;

        for(int i=0; i<N; i++)
        {
            vAllIndices.push_back(i);
        }

        // 预先为每轮 RANSAC 生成一个 8 点最小样本集合。
        // 这里提前采样有两个好处：
        // 1. 单应矩阵和基础矩阵分支共享同一批采样，比较更公平；
        // 2. 后面两个线程可以直接复用这些样本，避免并发随机数问题。
        mvSets = vector< vector<size_t> >(mMaxIterations,vector<size_t>(8,0));

        // 固定随机种子，使初始化过程可复现。
        DUtils::Random::SeedRandOnce(0);

        for(int it=0; it<mMaxIterations; it++)
        {
            vAvailableIndices = vAllIndices;

            // 从当前可用下标中无放回抽取 8 个匹配，构成八点法最小集。
            for(size_t j=0; j<8; j++)
            {
                int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
                int idx = vAvailableIndices[randi];

                mvSets[it][j] = idx;

                // 通过“与末尾交换再弹出”的写法实现 O(1) 删除。
                vAvailableIndices[randi] = vAvailableIndices.back();
                vAvailableIndices.pop_back();
            }
        }

        // 并行估计两种几何模型：
        // 1. H 更适合共面场景、纯旋转或低视差；
        // 2. F 更适合一般三维场景、存在明显平移基线的情况。
        vector<bool> vbMatchesInliersH, vbMatchesInliersF;
        float SH, SF;
        Eigen::Matrix3f H, F;

        thread threadH(&TwoViewReconstruction::FindHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
        thread threadF(&TwoViewReconstruction::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));

        // 等待两个模型估计都结束，再做模型选择。
        threadH.join();
        threadF.join();

        // 两个模型都完全解释不了数据时，初始化失败。
        if(SH+SF == 0.f) return false;

        // RH 越大，说明“单应矩阵模型”相对于“基础矩阵模型”更有解释力。
        float RH = SH/(SH+SF);

        // 初始化要求至少有一个较小但非零的视差角，避免纯旋转或退化情况。
        float minParallax = 1.0;

        // 根据评分占比选择重建模型。
        // ORB-SLAM 系列经验上会在 0.40~0.50 一带切换，这里采用 0.50。
        if(RH>0.50) // if(RH>0.40)
        {
            return ReconstructH(vbMatchesInliersH,H, mK,T21,vP3D,vbTriangulated,minParallax,50);
        }
        else //if(pF_HF>0.6)
        {
            return ReconstructF(vbMatchesInliersF,F,mK,T21,vP3D,vbTriangulated,minParallax,50);
        }
    }

    /**
     * @brief 使用 RANSAC 估计单应矩阵并记录最佳模型
     * @param vbMatchesInliers 输出最佳模型对应的内点集合
     * @param score            输出最佳评分
     * @param H21              输出最佳单应矩阵
     *
     * 核心步骤：
     * 1. 对两帧关键点做归一化；
     * 2. 对每轮预采样的 8 对点执行 DLT；
     * 3. 把归一化空间求得的 H 反变换回像素坐标系；
     * 4. 用双向重投影误差给该 H 打分；
     * 5. 保留评分最高的结果。
     */
    void TwoViewReconstruction::FindHomography(vector<bool> &vbMatchesInliers, float &score, Eigen::Matrix3f &H21)
    {
        // 有效匹配对数量，也是后续 inlier 标记的长度。
        const int N = mvMatches12.size();

        // 对两帧关键点做独立归一化，降低 DLT 求解时的数值病态。
        // 注意这里归一化的是整帧关键点数组，后面用匹配索引取出对应点。
        vector<cv::Point2f> vPn1, vPn2;
        Eigen::Matrix3f T1, T2;
        Normalize(mvKeys1,vPn1, T1);
        Normalize(mvKeys2,vPn2, T2);

        // DLT 得到的是归一化坐标系中的 Hn，最终需要左乘/右乘把它还原回像素坐标系。
        Eigen::Matrix3f T2inv = T2.inverse();

        // 当前找到的最佳结果。
        score = 0.0;
        vbMatchesInliers = vector<bool>(N,false);

        // 单次 RANSAC 迭代的临时变量。
        vector<cv::Point2f> vPn1i(8);
        vector<cv::Point2f> vPn2i(8);
        Eigen::Matrix3f H21i, H12i;
        vector<bool> vbCurrentInliers(N,false);
        float currentScore;

        // 遍历全部 RANSAC 采样，寻找评分最高的单应矩阵。
        for(int it=0; it<mMaxIterations; it++)
        {
            // 取出当前轮最小采样集中的 8 个匹配点对。
            for(size_t j=0; j<8; j++)
            {
                int idx = mvSets[it][j];

                vPn1i[j] = vPn1[mvMatches12[idx].first];
                vPn2i[j] = vPn2[mvMatches12[idx].second];
            }

            // 先在归一化坐标系中估计 Hn，再反归一化到原图像坐标系。
            Eigen::Matrix3f Hn = ComputeH21(vPn1i,vPn2i);
            H21i = T2inv * Hn * T1;

            // 检查双向误差时同时需要 H21 和 H12。
            H12i = H21i.inverse();

            currentScore = CheckHomography(H21i, H12i, vbCurrentInliers, mSigma);

            // 保留评分最高的那组结果。
            if(currentScore>score)
            {
                H21 = H21i;
                vbMatchesInliers = vbCurrentInliers;
                score = currentScore;
            }
        }
    }


    /**
     * @brief 使用 RANSAC 估计基础矩阵并记录最佳模型
     * @param vbMatchesInliers 输出最佳模型对应的内点集合
     * @param score            输出最佳评分
     * @param F21              输出最佳基础矩阵
     *
     * 整体流程与 FindHomography 类似，不同点在于：
     * 1. 使用八点法求解 F；
     * 2. 再通过 SVD 强制 F 满足秩 2 约束；
     * 3. 用点到极线距离评估几何一致性。
     */
    void TwoViewReconstruction::FindFundamental(vector<bool> &vbMatchesInliers, float &score, Eigen::Matrix3f &F21)
    {
        // 这里仅用于初始化输出容器；真实尺寸会在 CheckFundamental 中按匹配数重置。
        const int N = vbMatchesInliers.size();

        // 同样先做归一化，提高八点法线性求解的稳定性。
        vector<cv::Point2f> vPn1, vPn2;
        Eigen::Matrix3f T1, T2;
        Normalize(mvKeys1,vPn1, T1);
        Normalize(mvKeys2,vPn2, T2);

        // F 的反归一化公式是 F = T2^T * Fn * T1。
        Eigen::Matrix3f T2t = T2.transpose();

        // 当前最优结果。
        score = 0.0;
        vbMatchesInliers = vector<bool>(N,false);

        // 单轮迭代的临时变量。
        vector<cv::Point2f> vPn1i(8);
        vector<cv::Point2f> vPn2i(8);
        Eigen::Matrix3f F21i;
        vector<bool> vbCurrentInliers(N,false);
        float currentScore;

        // 遍历所有 RANSAC 采样集，寻找评分最高的 F。
        for(int it=0; it<mMaxIterations; it++)
        {
            // 取出当前轮的 8 组匹配。
            for(int j=0; j<8; j++)
            {
                int idx = mvSets[it][j];

                vPn1i[j] = vPn1[mvMatches12[idx].first];
                vPn2i[j] = vPn2[mvMatches12[idx].second];
            }

            // 在归一化坐标系中用八点法估计 Fn。
            Eigen::Matrix3f Fn = ComputeF21(vPn1i,vPn2i);

            // 反归一化回像素坐标系下的基础矩阵。
            F21i = T2t * Fn * T1;

            currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);

            // 记录评分最优的模型。
            if(currentScore>score)
            {
                F21 = F21i;
                vbMatchesInliers = vbCurrentInliers;
                score = currentScore;
            }
        }
    }

    /**
     * @brief 使用 DLT 线性估计单应矩阵
     * @param vP1 第一帧归一化点集
     * @param vP2 第二帧归一化点集
     * @return    归一化坐标系下的单应矩阵 H21
     *
     * 对于每一对对应点，会从 x2 ~ H21*x1 推出两条独立线性约束，
     * 因此 N 对点构成大小为 2N x 9 的线性系统 A*h = 0。
     */
    Eigen::Matrix3f TwoViewReconstruction::ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
    {
        const int N = vP1.size();

        // 每个对应点提供两行约束，未知量是 3x3 单应矩阵展开后的 9 个元素。
        Eigen::MatrixXf A(2*N, 9);

        for(int i=0; i<N; i++)
        {
            const float u1 = vP1[i].x;
            const float v1 = vP1[i].y;
            const float u2 = vP2[i].x;
            const float v2 = vP2[i].y;

            // 下面两行对应标准 DLT 形式。
            // 写成矩阵后，目标是在最小二乘意义下求解使 A*h=0 的非零向量 h。
            A(2*i,0) = 0.0;
            A(2*i,1) = 0.0;
            A(2*i,2) = 0.0;
            A(2*i,3) = -u1;
            A(2*i,4) = -v1;
            A(2*i,5) = -1;
            A(2*i,6) = v2*u1;
            A(2*i,7) = v2*v1;
            A(2*i,8) = v2;

            A(2*i+1,0) = u1;
            A(2*i+1,1) = v1;
            A(2*i+1,2) = 1;
            A(2*i+1,3) = 0.0;
            A(2*i+1,4) = 0.0;
            A(2*i+1,5) = 0.0;
            A(2*i+1,6) = -u2*u1;
            A(2*i+1,7) = -u2*v1;
            A(2*i+1,8) = -u2;

        }

        // A 的最小奇异值对应的右奇异向量，就是最优解 h。
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullV);

        // 以行主序方式把长度为 9 的向量重解释为 3x3 矩阵。
        Eigen::Matrix<float,3,3,Eigen::RowMajor> H(svd.matrixV().col(8).data());

        return H;
    }

    /**
     * @brief 使用归一化八点法估计基础矩阵
     * @param vP1 第一帧归一化点集
     * @param vP2 第二帧归一化点集
     * @return    满足秩为 2 的基础矩阵
     *
     * 处理步骤：
     * 1. 由点对构造线性系统 A*f = 0；
     * 2. 用 SVD 得到基础矩阵初值 Fpre；
     * 3. 再对 Fpre 做一次 SVD，把最小奇异值压成 0；
     * 4. 得到满足 rank(F)=2 的最终基础矩阵。
     */
    Eigen::Matrix3f TwoViewReconstruction::ComputeF21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2)
    {
        const int N = vP1.size();

        // 每一行对应一个匹配点对带来的极几何约束 x2^T * F * x1 = 0。
        Eigen::MatrixXf A(N, 9);

        for(int i=0; i<N; i++)
        {
            const float u1 = vP1[i].x;
            const float v1 = vP1[i].y;
            const float u2 = vP2[i].x;
            const float v2 = vP2[i].y;

            A(i,0) = u2*u1;
            A(i,1) = u2*v1;
            A(i,2) = u2;
            A(i,3) = v2*u1;
            A(i,4) = v2*v1;
            A(i,5) = v2;
            A(i,6) = u1;
            A(i,7) = v1;
            A(i,8) = 1;
        }

        // 与单应矩阵类似，先取最小奇异值对应的右奇异向量作为初始解。
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Matrix<float,3,3,Eigen::RowMajor> Fpre(svd.matrixV().col(8).data());

        // 线性求解得到的 Fpre 一般不是严格秩 2，因此需要再做一次投影。
        Eigen::JacobiSVD<Eigen::Matrix3f> svd2(Fpre, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Vector3f w = svd2.singularValues();

        // 基础矩阵理论上秩为 2，把最小奇异值置零即可。
        w(2) = 0;

        return svd2.matrixU() * Eigen::DiagonalMatrix<float,3>(w) * svd2.matrixV().transpose();
    }

    /**
     * @brief 通过双向重投影误差评估单应矩阵
     * @param H21              从帧 1 到帧 2 的单应矩阵
     * @param H12              从帧 2 到帧 1 的单应矩阵
     * @param vbMatchesInliers 输出内点标记
     * @param sigma            像素观测噪声标准差
     * @return                 模型评分
     *
     * 每个匹配会做两次检查：
     * 1. x2 逆投影到图像 1 后，与真实 x1 的误差；
     * 2. x1 正投影到图像 2 后，与真实 x2 的误差。
     *
     * 两次都满足阈值时才视为内点。评分使用“阈值减误差”的形式，
     * 因而内点越多、误差越小，模型分数越高。
     */
    float TwoViewReconstruction::CheckHomography(const Eigen::Matrix3f &H21, const Eigen::Matrix3f &H12, vector<bool> &vbMatchesInliers, float sigma)
    {
        const int N = mvMatches12.size();

        // 先把矩阵元素展开到标量，减少循环中的索引开销。
        const float h11 = H21(0,0);
        const float h12 = H21(0,1);
        const float h13 = H21(0,2);
        const float h21 = H21(1,0);
        const float h22 = H21(1,1);
        const float h23 = H21(1,2);
        const float h31 = H21(2,0);
        const float h32 = H21(2,1);
        const float h33 = H21(2,2);

        const float h11inv = H12(0,0);
        const float h12inv = H12(0,1);
        const float h13inv = H12(0,2);
        const float h21inv = H12(1,0);
        const float h22inv = H12(1,1);
        const float h23inv = H12(1,2);
        const float h31inv = H12(2,0);
        const float h32inv = H12(2,1);
        const float h33inv = H12(2,2);

        vbMatchesInliers.resize(N);

        float score = 0;

        // 5.991 是卡方分布在自由度 2、置信度 95% 下的阈值，
        // 适合二维重投影误差检验。
        const float th = 5.991;

        const float invSigmaSquare = 1.0/(sigma*sigma);

        for(int i=0; i<N; i++)
        {
            bool bIn = true;

            const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
            const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

            const float u1 = kp1.pt.x;
            const float v1 = kp1.pt.y;
            const float u2 = kp2.pt.x;
            const float v2 = kp2.pt.y;

            // 第一步：把图像 2 中的点通过 H12 投影回图像 1。
            // 这里使用齐次坐标，最后要除以第三维完成归一化。
            const float w2in1inv = 1.0/(h31inv*u2+h32inv*v2+h33inv);
            const float u2in1 = (h11inv*u2+h12inv*v2+h13inv)*w2in1inv;
            const float v2in1 = (h21inv*u2+h22inv*v2+h23inv)*w2in1inv;

            const float squareDist1 = (u1-u2in1)*(u1-u2in1)+(v1-v2in1)*(v1-v2in1);

            const float chiSquare1 = squareDist1*invSigmaSquare;

            // 若反向投影误差超阈值，则该匹配不可能是单应模型内点。
            if(chiSquare1>th)
                bIn = false;
            else
                score += th - chiSquare1;

            // 第二步：把图像 1 中的点通过 H21 投影到图像 2。
            const float w1in2inv = 1.0/(h31*u1+h32*v1+h33);
            const float u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
            const float v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;

            const float squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);

            const float chiSquare2 = squareDist2*invSigmaSquare;

            // 正向投影也必须同时满足误差约束。
            if(chiSquare2>th)
                bIn = false;
            else
                score += th - chiSquare2;

            // 只有双向一致时才记作内点。
            if(bIn)
                vbMatchesInliers[i]=true;
            else
                vbMatchesInliers[i]=false;
        }

        return score;
    }

    /**
     * @brief 通过对称极线距离评估基础矩阵
     * @param F21              从帧 1 到帧 2 的基础矩阵
     * @param vbMatchesInliers 输出内点标记
     * @param sigma            像素观测噪声标准差
     * @return                 模型评分
     *
     * 对于每个匹配：
     * 1. 用 x1 生成图像 2 中的极线 l2 = F*x1，并计算 x2 到 l2 的距离；
     * 2. 用 x2 生成图像 1 中的极线 l1 = F^T*x2，并计算 x1 到 l1 的距离；
     * 3. 两个方向都过阈值，才认为该匹配符合基础矩阵模型。
     */
    float TwoViewReconstruction::CheckFundamental(const Eigen::Matrix3f &F21, vector<bool> &vbMatchesInliers, float sigma)
    {
        const int N = mvMatches12.size();

        // 展开矩阵元素，避免循环内频繁访问 Eigen 索引。
        const float f11 = F21(0,0);
        const float f12 = F21(0,1);
        const float f13 = F21(0,2);
        const float f21 = F21(1,0);
        const float f22 = F21(1,1);
        const float f23 = F21(1,2);
        const float f31 = F21(2,0);
        const float f32 = F21(2,1);
        const float f33 = F21(2,2);

        vbMatchesInliers.resize(N);

        float score = 0;

        // 3.841 是卡方分布在自由度 1、置信度 95% 下的阈值，
        // 因为这里误差被投影为“点到极线”的一维距离。
        const float th = 3.841;

        // 评分时仍采用与单应矩阵相同的 2 自由度阈值，保持分值尺度可比。
        const float thScore = 5.991;

        const float invSigmaSquare = 1.0/(sigma*sigma);

        for(int i=0; i<N; i++)
        {
            bool bIn = true;

            const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
            const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

            const float u1 = kp1.pt.x;
            const float v1 = kp1.pt.y;
            const float u2 = kp2.pt.x;
            const float v2 = kp2.pt.y;

            // 由图像 1 中的点生成图像 2 中的极线 l2。
            const float a2 = f11*u1+f12*v1+f13;
            const float b2 = f21*u1+f22*v1+f23;
            const float c2 = f31*u1+f32*v1+f33;

            // 点到直线距离公式：d^2 = (ax+by+c)^2 / (a^2+b^2)。
            const float num2 = a2*u2+b2*v2+c2;

            const float squareDist1 = num2*num2/(a2*a2+b2*b2);

            const float chiSquare1 = squareDist1*invSigmaSquare;

            // 若 x2 偏离由 x1 诱导出的极线太远，则该匹配不符合当前 F。
            if(chiSquare1>th)
                bIn = false;
            else
                score += thScore - chiSquare1;

            // 再反过来，由图像 2 中的点生成图像 1 中的极线 l1。
            const float a1 = f11*u2+f21*v2+f31;
            const float b1 = f12*u2+f22*v2+f32;
            const float c1 = f13*u2+f23*v2+f33;

            const float num1 = a1*u1+b1*v1+c1;

            const float squareDist2 = num1*num1/(a1*a1+b1*b1);

            const float chiSquare2 = squareDist2*invSigmaSquare;

            // 双向都过阈值，才是稳定的极几何内点。
            if(chiSquare2>th)
                bIn = false;
            else
                score += thScore - chiSquare2;

            if(bIn)
                vbMatchesInliers[i]=true;
            else
                vbMatchesInliers[i]=false;
        }

        return score;
    }

    /**
     * @brief 从基础矩阵模型恢复位姿并验证 4 组运动假设
     * @param vbMatchesInliers  基础矩阵对应的内点标记
     * @param F21               基础矩阵
     * @param K                 相机内参矩阵
     * @param T21               输出位姿
     * @param vP3D              输出三维点
     * @param vbTriangulated    输出三角化成功标记
     * @param minParallax       最小允许视差角
     * @param minTriangulated   最少成功三角化点数
     * @return                  若有唯一且可信的运动解则返回 true
     *
     * 处理逻辑：
     * 1. 由 F 和 K 计算本质矩阵 E；
     * 2. 分解 E 得到 2 个旋转候选和 1 个平移方向；
     * 3. 结合平移正负号形成 4 组 [R|t] 假设；
     * 4. 对每组假设执行三角化与重投影检查；
     * 5. 选择三角化点最多且视差足够大的唯一优胜解。
     */
    bool TwoViewReconstruction::ReconstructF(vector<bool> &vbMatchesInliers, Eigen::Matrix3f &F21, Eigen::Matrix3f &K,
                                             Sophus::SE3f &T21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
    {
        // 统计基础矩阵模型下的内点数量。
        int N=0;
        for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
            if(vbMatchesInliers[i])
                N++;

        // 已知内参时，E = K^T * F * K。
        Eigen::Matrix3f E21 = K.transpose() * F21 * K;

        Eigen::Matrix3f R1, R2;
        Eigen::Vector3f t;

        // E 的标准分解给出两个旋转候选和一个平移方向。
        DecomposeE(E21,R1,R2,t);

        // 平移只确定到尺度和符号，因此需要同时考虑 +t 和 -t。
        Eigen::Vector3f t1 = t;
        Eigen::Vector3f t2 = -t;

        // 分别验证 4 组 [R|t] 假设。
        vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
        vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
        float parallax1,parallax2, parallax3, parallax4;

        int nGood1 = CheckRT(R1,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D1, 4.0*mSigma2, vbTriangulated1, parallax1);
        int nGood2 = CheckRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2, 4.0*mSigma2, vbTriangulated2, parallax2);
        int nGood3 = CheckRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3, 4.0*mSigma2, vbTriangulated3, parallax3);
        int nGood4 = CheckRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4, 4.0*mSigma2, vbTriangulated4, parallax4);

        // 找到三角化成功点数最多的那组解。
        int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

        // 至少要满足：
        // 1. 不少于基础矩阵内点的 90%；
        // 2. 不少于调用者要求的最小数量。
        int nMinGood = max(static_cast<int>(0.9*N),minTriangulated);

        // 如果有多组解都接近最优，说明解不够唯一，通常属于退化或不稳定情况。
        int nsimilar = 0;
        if(nGood1>0.7*maxGood)
            nsimilar++;
        if(nGood2>0.7*maxGood)
            nsimilar++;
        if(nGood3>0.7*maxGood)
            nsimilar++;
        if(nGood4>0.7*maxGood)
            nsimilar++;

        // 缺少明显优胜解，或者总体三角化点数太少，直接拒绝初始化。
        if(maxGood<nMinGood || nsimilar>1)
        {
            return false;
        }

        // 最佳解还必须具备足够视差，否则即便三角化成功，深度也会非常不稳定。
        if(maxGood==nGood1)
        {
            if(parallax1>minParallax)
            {
                vP3D = vP3D1;
                vbTriangulated = vbTriangulated1;

                T21 = Sophus::SE3f(R1, t1);
                return true;
            }
        }else if(maxGood==nGood2)
        {
            if(parallax2>minParallax)
            {
                vP3D = vP3D2;
                vbTriangulated = vbTriangulated2;

                T21 = Sophus::SE3f(R2, t1);
                return true;
            }
        }else if(maxGood==nGood3)
        {
            if(parallax3>minParallax)
            {
                vP3D = vP3D3;
                vbTriangulated = vbTriangulated3;

                T21 = Sophus::SE3f(R1, t2);
                return true;
            }
        }else if(maxGood==nGood4)
        {
            if(parallax4>minParallax)
            {
                vP3D = vP3D4;
                vbTriangulated = vbTriangulated4;

                T21 = Sophus::SE3f(R2, t2);
                return true;
            }
        }

        return false;
    }

    /**
     * @brief 从单应矩阵模型恢复位姿并验证 8 组运动假设
     * @param vbMatchesInliers  单应矩阵对应的内点标记
     * @param H21               单应矩阵
     * @param K                 相机内参矩阵
     * @param T21               输出位姿
     * @param vP3D              输出三维点
     * @param vbTriangulated    输出三角化成功标记
     * @param minParallax       最小允许视差角
     * @param minTriangulated   最少成功三角化点数
     * @return                  若存在足够明确的单应矩阵运动解则返回 true
     *
     * 单应矩阵分解比本质矩阵更复杂，因为平面诱导几何会产生更多等价解。
     * 这里采用 Faugeras 等人的经典方法，把 H 分解成 8 组候选运动，
     * 然后再用真实三角化和重投影结果做筛选。
     */
    bool TwoViewReconstruction::ReconstructH(vector<bool> &vbMatchesInliers, Eigen::Matrix3f &H21, Eigen::Matrix3f &K,
                                             Sophus::SE3f &T21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
    {
        // 统计单应矩阵模型下的内点数量。
        int N=0;
        for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
            if(vbMatchesInliers[i])
                N++;

        // 先把像素坐标系中的 H 转成归一化相机坐标中的 A，方便后续分解。
        // 参考：
        // Faugeras et al., "Motion and structure from motion in a piecewise planar environment", 1988.
        Eigen::Matrix3f invK = K.inverse();
        Eigen::Matrix3f A = invK * H21 * K;

        // 对 A 做 SVD，后续分解公式全部依赖其奇异值和左右奇异向量。
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3f U = svd.matrixU();
        Eigen::Matrix3f V = svd.matrixV();
        Eigen::Matrix3f Vt = V.transpose();
        Eigen::Vector3f w = svd.singularValues();

        // s 用来校正旋转矩阵的手性，使最终结果尽量保持 det(R)=+1。
        float s = U.determinant() * Vt.determinant();

        float d1 = w(0);
        float d2 = w(1);
        float d3 = w(2);

        // 若奇异值过于接近，说明分解高度退化，难以稳定区分不同运动假设。
        if(d1/d2<1.00001 || d2/d3<1.00001)
        {
            return false;
        }

        // vR: 8 个旋转候选
        // vt: 8 个平移方向候选
        // vn: 8 个平面法向候选
        // 当前实现最终只显式验证 R/t，但法向量同样是理论分解的一部分。
        vector<Eigen::Matrix3f> vR;
        vector<Eigen::Vector3f> vt, vn;
        vR.reserve(8);
        vt.reserve(8);
        vn.reserve(8);

        // 第一类构造：n' = [x1, 0, x3]^T，一共 4 种符号组合。
        float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
        float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
        float x1[] = {aux1,aux1,-aux1,-aux1};
        float x3[] = {aux3,-aux3,aux3,-aux3};

        // 情形一：d' = d2。
        float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

        float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
        float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

        for(int i=0; i<4; i++)
        {
            Eigen::Matrix3f Rp;
            Rp.setZero();
            Rp(0,0) = ctheta;
            Rp(0,2) = -stheta[i];
            Rp(1,1) = 1.f;
            Rp(2,0) = stheta[i];
            Rp(2,2) = ctheta;

            // 通过 U * Rp * V^T 恢复到原坐标系中的旋转。
            Eigen::Matrix3f R = s*U*Rp*Vt;
            vR.push_back(R);

            // tp 是分解公式下的平移方向中间量，最终还要乘 U 并单位化。
            Eigen::Vector3f tp;
            tp(0) = x1[i];
            tp(1) = 0;
            tp(2) = -x3[i];
            tp *= d1-d3;

            Eigen::Vector3f t = U*tp;
            vt.push_back(t / t.norm());

            // 同时构造与该运动假设对应的平面法向量。
            Eigen::Vector3f np;
            np(0) = x1[i];
            np(1) = 0;
            np(2) = x3[i];

            // 强制法向量的 z 分量为正，统一朝向约定。
            Eigen::Vector3f n = V*np;
            if(n(2) < 0)
                n = -n;
            vn.push_back(n);
        }

        // 情形二：d' = -d2，再产生另外 4 组解。
        float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

        float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
        float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

        for(int i=0; i<4; i++)
        {
            Eigen::Matrix3f Rp;
            Rp.setZero();
            Rp(0,0) = cphi;
            Rp(0,2) = sphi[i];
            Rp(1,1) = -1;
            Rp(2,0) = sphi[i];
            Rp(2,2) = -cphi;

            Eigen::Matrix3f R = s*U*Rp*Vt;
            vR.push_back(R);

            Eigen::Vector3f tp;
            tp(0) = x1[i];
            tp(1) = 0;
            tp(2) = x3[i];
            tp *= d1+d3;

            Eigen::Vector3f t = U*tp;
            vt.push_back(t / t.norm());

            Eigen::Vector3f np;
            np(0) = x1[i];
            np(1) = 0;
            np(2) = x3[i];

            Eigen::Vector3f n = V*np;
            if(n(2) < 0)
                n = -n;
            vn.push_back(n);
        }


        // 在 8 组候选中寻找最优解，并同时记录次优解做“唯一性”判断。
        int bestGood = 0;
        int secondBestGood = 0;
        int bestSolutionIdx = -1;
        float bestParallax = -1;
        vector<cv::Point3f> bestP3D;
        vector<bool> bestTriangulated;

        // 与论文中直接依赖可见性约束不同，这里采用更稳健的工程策略：
        // 对所有假设都实际做三角化，然后用“有效点数 + 视差”来决定优劣。
        for(size_t i=0; i<8; i++)
        {
            float parallaxi;
            vector<cv::Point3f> vP3Di;
            vector<bool> vbTriangulatedi;
            int nGood = CheckRT(vR[i],vt[i],mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K,vP3Di, 4.0*mSigma2, vbTriangulatedi, parallaxi);

            if(nGood>bestGood)
            {
                secondBestGood = bestGood;
                bestGood = nGood;
                bestSolutionIdx = i;
                bestParallax = parallaxi;
                bestP3D = vP3Di;
                bestTriangulated = vbTriangulatedi;
            }
            else if(nGood>secondBestGood)
            {
                secondBestGood = nGood;
            }
        }


        // 只有当最优解明显优于次优解、视差充足、且通过检验的点足够多时，
        // 才接受该单应矩阵运动解。
        if(secondBestGood<0.75*bestGood && bestParallax>=minParallax && bestGood>minTriangulated && bestGood>0.9*N)
        {
            T21 = Sophus::SE3f(vR[bestSolutionIdx], vt[bestSolutionIdx]);
            vbTriangulated = bestTriangulated;

            // 这里保留最佳三角化标记供后续使用。
            // bestP3D 在本实现中虽然被记录，但没有在此处写回 vP3D。
            return true;
        }

        return false;
    }


    /**
     * @brief 对关键点坐标做平移和缩放归一化
     * @param vKeys               输入关键点
     * @param vNormalizedPoints   输出归一化点
     * @param T                   输出归一化矩阵
     *
     * 该归一化与 Hartley 归一化思想相近，作用是降低线性求解中的数值不稳定：
     * 1. 先把点集中心平移到原点附近；
     * 2. 再分别按 x / y 方向的平均绝对偏差缩放。
     */
    void TwoViewReconstruction::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, Eigen::Matrix3f &T)
    {
        float meanX = 0;
        float meanY = 0;
        const int N = vKeys.size();

        vNormalizedPoints.resize(N);

        // 先计算原始点集的质心。
        for(int i=0; i<N; i++)
        {
            meanX += vKeys[i].pt.x;
            meanY += vKeys[i].pt.y;
        }

        meanX = meanX/N;
        meanY = meanY/N;

        float meanDevX = 0;
        float meanDevY = 0;

        // 将点集平移到以质心为中心的新坐标系，并统计平均绝对偏差。
        for(int i=0; i<N; i++)
        {
            vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
            vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

            meanDevX += fabs(vNormalizedPoints[i].x);
            meanDevY += fabs(vNormalizedPoints[i].y);
        }

        meanDevX = meanDevX/N;
        meanDevY = meanDevY/N;

        // 按各轴平均绝对偏差的倒数进行缩放，使点坐标量级更接近 1。
        float sX = 1.0/meanDevX;
        float sY = 1.0/meanDevY;

        for(int i=0; i<N; i++)
        {
            vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
            vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
        }

        // 构造齐次坐标下的归一化变换矩阵。
        T.setZero();
        T(0,0) = sX;
        T(1,1) = sY;
        T(0,2) = -meanX*sX;
        T(1,2) = -meanY*sY;
        T(2,2) = 1.f;
    }

    /**
     * @brief 检验给定 [R|t] 假设是否能支持稳定的两视图重建
     * @param R                 候选旋转
     * @param t                 候选平移方向
     * @param vKeys1            参考帧关键点
     * @param vKeys2            当前帧关键点
     * @param vMatches12        匹配对
     * @param vbMatchesInliers  几何模型内点标记
     * @param K                 相机内参矩阵
     * @param vP3D              输出三维点，索引与参考帧关键点对齐
     * @param th2               重投影误差平方阈值
     * @param vbGood            输出标记，表示该点是否通过了全部严格检查
     * @param parallax          输出代表性视差角
     * @return                  通过检验的点数
     *
     * 该函数是模型验证的核心：
     * 1. 根据 [R|t] 三角化所有内点匹配；
     * 2. 检查三维点是否有限；
     * 3. 检查点是否位于两相机前方；
     * 4. 检查两帧重投影误差；
     * 5. 统计有效点数和视差角。
     */
    int TwoViewReconstruction::CheckRT(const Eigen::Matrix3f &R, const Eigen::Vector3f &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                                       const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                                       const Eigen::Matrix3f &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
    {
        // 从内参矩阵中拆出针孔模型参数，便于后面重投影。
        const float fx = K(0,0);
        const float fy = K(1,1);
        const float cx = K(0,2);
        const float cy = K(1,2);

        // 输出数组按参考帧关键点大小初始化，方便直接用 frame1 索引回填。
        vbGood = vector<bool>(vKeys1.size(),false);
        vP3D.resize(vKeys1.size());

        // 保存通过检查点的视差余弦，用于后面估计代表性视差角。
        vector<float> vCosParallax;
        vCosParallax.reserve(vKeys1.size());

        // 相机 1 作为参考坐标系，其投影矩阵为 K[I|0]。
        Eigen::Matrix<float,3,4> P1;
        P1.setZero();
        P1.block<3,3>(0,0) = K;

        // 相机 1 光心位于原点。
        Eigen::Vector3f O1;
        O1.setZero();

        // 相机 2 的投影矩阵为 K[R|t]。
        Eigen::Matrix<float,3,4> P2;
        P2.block<3,3>(0,0) = R;
        P2.block<3,1>(0,3) = t;
        P2 = K * P2;

        // 由世界到相机的外参反推出相机 2 在参考坐标系中的光心位置。
        Eigen::Vector3f O2 = -R.transpose() * t;

        int nGood=0;

        for(size_t i=0, iend=vMatches12.size();i<iend;i++)
        {
            // 只验证模型内点，离群点没有必要参与三角化。
            if(!vbMatchesInliers[i])
                continue;

            const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
            const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];

            // 构造两帧中的齐次像素坐标。
            Eigen::Vector3f p3dC1;
            Eigen::Vector3f x_p1(kp1.pt.x, kp1.pt.y, 1);
            Eigen::Vector3f x_p2(kp2.pt.x, kp2.pt.y, 1);

            // 在线性意义下三角化三维点，结果以相机 1 坐标系表示。
            GeometricTools::Triangulate(x_p1, x_p2, P1, P2, p3dC1);


            // 数值异常的点直接丢弃。
            if(!isfinite(p3dC1(0)) || !isfinite(p3dC1(1)) || !isfinite(p3dC1(2)))
            {
                vbGood[vMatches12[i].first]=false;
                continue;
            }

            // 由两相机光心指向该点的视线向量，计算视差夹角。
            Eigen::Vector3f normal1 = p3dC1 - O1;
            float dist1 = normal1.norm();

            Eigen::Vector3f normal2 = p3dC1 - O2;
            float dist2 = normal2.norm();

            float cosParallax = normal1.dot(normal2) / (dist1*dist2);

            // 若视差足够大，就要求点必须位于两相机前方。
            // 当 cosParallax 非常接近 1 时，点几乎在无穷远处，此时深度符号会比较不稳定，
            // 代码放宽了这类点的正深度限制。
            if(p3dC1(2)<=0 && cosParallax<0.99998)
                continue;

            // 把点变换到相机 2 坐标系，再做同样的正深度检查。
            Eigen::Vector3f p3dC2 = R * p3dC1 + t;

            if(p3dC2(2)<=0 && cosParallax<0.99998)
                continue;

            // 计算在图像 1 中的重投影误差。
            float im1x, im1y;
            float invZ1 = 1.0/p3dC1(2);
            im1x = fx*p3dC1(0)*invZ1+cx;
            im1y = fy*p3dC1(1)*invZ1+cy;

            float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

            if(squareError1>th2)
                continue;

            // 计算在图像 2 中的重投影误差。
            float im2x, im2y;
            float invZ2 = 1.0/p3dC2(2);
            im2x = fx*p3dC2(0)*invZ2+cx;
            im2y = fy*p3dC2(1)*invZ2+cy;

            float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

            if(squareError2>th2)
                continue;

            // 走到这里，说明该点满足当前 [R|t] 假设下的全部几何条件。
            vCosParallax.push_back(cosParallax);
            vP3D[vMatches12[i].first] = cv::Point3f(p3dC1(0), p3dC1(1), p3dC1(2));
            nGood++;

            // 只有视差不是近似 0 的点，才认为它真的适合用于稳定三角化。
            if(cosParallax<0.99998)
                vbGood[vMatches12[i].first]=true;
        }

        if(nGood>0)
        {
            sort(vCosParallax.begin(),vCosParallax.end());

            // 升序后，前面的元素对应更大的视差角。
            // 这里取前 50 个“高视差点”中的最保守一个，避免只看最大值过于敏感。
            size_t idx = min(50,int(vCosParallax.size()-1));
            parallax = acos(vCosParallax[idx])*180/CV_PI;
        }
        else
            parallax=0;

        return nGood;
    }

    /**
     * @brief 分解本质矩阵得到两个旋转候选和一个平移方向
     * @param E   本质矩阵
     * @param R1  输出旋转候选 1
     * @param R2  输出旋转候选 2
     * @param t   输出单位平移方向
     *
     * 标准结论：
     * 1. E 的 SVD 可写为 U * diag(s, s, 0) * V^T；
     * 2. 借助固定矩阵 W，可得到两个旋转解；
     * 3. 平移方向来自 U 的第三列；
     * 4. 再结合 t 的正负号形成 4 组位姿假设。
     */
    void TwoViewReconstruction::DecomposeE(const Eigen::Matrix3f &E, Eigen::Matrix3f &R1, Eigen::Matrix3f &R2, Eigen::Vector3f &t)
    {

        // 对本质矩阵做 SVD，后续的标准分解公式都依赖 U、V^T。
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Matrix3f U = svd.matrixU();
        Eigen::Matrix3f Vt = svd.matrixV().transpose();

        // 平移方向是 U 的第三列，只能恢复到尺度未知的单位方向。
        t = U.col(2);
        t = t / t.norm();

        // W 是本质矩阵分解中的固定辅助矩阵。
        Eigen::Matrix3f W;
        W.setZero();
        W(0,1) = -1;
        W(1,0) = 1;
        W(2,2) = 1;

        // 第一组旋转解。
        R1 = U * W * Vt;
        if(R1.determinant() < 0)
            R1 = -R1;

        // 第二组旋转解。
        R2 = U * W.transpose() * Vt;
        if(R2.determinant() < 0)
            R2 = -R2;
    }

} //namespace ORB_SLAM
