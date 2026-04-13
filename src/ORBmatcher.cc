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


#include "ORBmatcher.h"

#include<limits.h>

#include<opencv2/core/core.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include<stdint-gcc.h>

using namespace std;

namespace ORB_SLAM3
{

    // 较宽松的描述子距离阈值。常用于已有几何约束时的投影匹配。
    const int ORBmatcher::TH_HIGH = 100;
    // 较严格的描述子距离阈值。常用于 BoW / 初始化 / 融合等更依赖外观一致性的场景。
    const int ORBmatcher::TH_LOW = 50;
    // 方向一致性直方图的 bin 数。360 度会被均匀划分到 30 个区间。
    const int ORBmatcher::HISTO_LENGTH = 30;

    /**
     * @brief 构造 ORB 匹配器
     * @param nnratio   最近邻比值测试阈值
     * @param checkOri  是否启用方向一致性检查
     */
    ORBmatcher::ORBmatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
    {
    }

    /**
     * @brief              在当前帧中搜索投影后的地图点匹配
     * @param F            当前帧
     * @param vpMapPoints  待匹配地图点集合
     * @param th           搜索半径倍率
     * @param bFarPoints   是否过滤远点
     * @param thFarPoints  远点深度阈值
     * @return             匹配成功的数量
     *
     * 这是局部地图跟踪阶段的核心匹配函数之一。流程可以概括为：
     * 1. 地图点已经在跟踪线程中被预测到了当前帧的投影位置；
     * 2. 在投影附近、对应尺度层上搜索候选特征；
     * 3. 通过 ORB 汉明距离找到最佳与次佳候选；
     * 4. 用最近邻比值测试和尺度一致性过滤歧义匹配；
     * 5. 如果是双目 / 双鱼眼，还会同时补全另一侧相机的观测。
     */
    int ORBmatcher::SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th, const bool bFarPoints, const float thFarPoints)
    {
        // nmatches 统计总匹配数；left/right 仅用于区分左右目观测统计。
        int nmatches=0, left = 0, right = 0;

        // 当 th != 1 时，需要对默认搜索半径进行额外缩放。
        const bool bFactor = th!=1.0;

        // 逐个处理所有候选地图点。
        for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++)
        {
            MapPoint* pMP = vpMapPoints[iMP];

            // 如果该点既没有被预测到左目，也没有被预测到右目，则当前帧没法用它做匹配。
            if(!pMP->mbTrackInView && !pMP->mbTrackInViewR)
                continue;

            // 可选的远点过滤，用于限制只跟踪一定深度范围内的地图点。
            if(bFarPoints && pMP->mTrackDepth>thFarPoints)
                continue;

            // 坏点直接跳过。
            if(pMP->isBad())
                continue;

            // 先处理当前帧左目 / 单目图像中的投影匹配。
            if(pMP->mbTrackInView)
            {
                // 跟踪线程已经根据投影深度预测出了最可能出现的尺度层。
                const int &nPredictedLevel = pMP->mnTrackScaleLevel;

                // 搜索窗口半径由视角余弦决定。
                // 视角越接近地图点法向，投影不确定性越小，窗口就可以更小。
                float r = RadiusByViewingCos(pMP->mTrackViewCos);

                if(bFactor)
                    r*=th;

                // 仅在预测投影位置附近、预测尺度层附近搜索候选特征。
                const vector<size_t> vIndices =
                        F.GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY,r*F.mvScaleFactors[nPredictedLevel],nPredictedLevel-1,nPredictedLevel);

                if(!vIndices.empty()){
                    // 地图点使用其代表描述子与当前帧候选特征做比较。
                    const cv::Mat MPdescriptor = pMP->GetDescriptor();

                    // 保存最佳和次佳描述子距离，同时记录二者所在尺度层。
                    int bestDist=256;
                    int bestLevel= -1;
                    int bestDist2=256;
                    int bestLevel2 = -1;
                    int bestIdx =-1 ;

                    // 在局部窗口里寻找最佳和次佳匹配。
                    for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
                    {
                        const size_t idx = *vit;

                        // 如果该特征已经绑定了一个有观测支持的地图点，就不再抢占它。
                        if(F.mvpMapPoints[idx])
                            if(F.mvpMapPoints[idx]->Observations()>0)
                                continue;

                        // 在双目但未分左右数组的情形下，进一步检查右目投影误差。
                        if(F.Nleft == -1 && F.mvuRight[idx]>0)
                        {
                            const float er = fabs(pMP->mTrackProjXR-F.mvuRight[idx]);
                            if(er>r*F.mvScaleFactors[nPredictedLevel])
                                continue;
                        }

                        const cv::Mat &d = F.mDescriptors.row(idx);

                        const int dist = DescriptorDistance(MPdescriptor,d);

                        if(dist<bestDist)
                        {
                            bestDist2=bestDist;
                            bestDist=dist;
                            bestLevel2 = bestLevel;
                            bestLevel = (F.Nleft == -1) ? F.mvKeysUn[idx].octave
                                                        : (idx < F.Nleft) ? F.mvKeys[idx].octave
                                                                          : F.mvKeysRight[idx - F.Nleft].octave;
                            bestIdx=idx;
                        }
                        else if(dist<bestDist2)
                        {
                            bestLevel2 = (F.Nleft == -1) ? F.mvKeysUn[idx].octave
                                                         : (idx < F.Nleft) ? F.mvKeys[idx].octave
                                                                           : F.mvKeysRight[idx - F.Nleft].octave;
                            bestDist2=dist;
                        }
                    }

                    // 先用宽松阈值过滤明显差的描述子，再做最近邻比值测试。
                    // 只有当最佳和次佳来自同一尺度层时，比值测试才有可比性。
                    if(bestDist<=TH_HIGH)
                    {
                        if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                            continue;

                        if(bestLevel!=bestLevel2 || bestDist<=mfNNratio*bestDist2){
                            // 通过外观与尺度筛选后，把最佳特征与该地图点建立关联。
                            F.mvpMapPoints[bestIdx]=pMP;

                            // 如果当前帧已经知道左-右特征对应关系，则同步把右目对应点也绑定上。
                            if(F.Nleft != -1 && F.mvLeftToRightMatch[bestIdx] != -1){ //Also match with the stereo observation at right camera
                                F.mvpMapPoints[F.mvLeftToRightMatch[bestIdx] + F.Nleft] = pMP;
                                nmatches++;
                                right++;
                            }

                            nmatches++;
                            left++;
                        }
                    }
                }
            }

            // 如果当前帧含右目，并且该地图点也被预测到了右目视野，则继续在右目中搜索。
            if(F.Nleft != -1 && pMP->mbTrackInViewR){
                const int &nPredictedLevel = pMP->mnTrackScaleLevelR;
                if(nPredictedLevel != -1){
                    float r = RadiusByViewingCos(pMP->mTrackViewCosR);

                    // true 表示仅在右目特征中搜索。
                    const vector<size_t> vIndices =
                            F.GetFeaturesInArea(pMP->mTrackProjXR,pMP->mTrackProjYR,r*F.mvScaleFactors[nPredictedLevel],nPredictedLevel-1,nPredictedLevel,true);

                    if(vIndices.empty())
                        continue;

                    const cv::Mat MPdescriptor = pMP->GetDescriptor();

                    // 与左目分支相同，记录最佳 / 次佳匹配与对应尺度层。
                    int bestDist=256;
                    int bestLevel= -1;
                    int bestDist2=256;
                    int bestLevel2 = -1;
                    int bestIdx =-1 ;

                    // 在右目候选中寻找最佳和次佳匹配。
                    for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
                    {
                        const size_t idx = *vit;

                        // 右目特征若已经绑定到已有地图点，则跳过。
                        if(F.mvpMapPoints[idx + F.Nleft])
                            if(F.mvpMapPoints[idx + F.Nleft]->Observations()>0)
                                continue;


                        const cv::Mat &d = F.mDescriptors.row(idx + F.Nleft);

                        const int dist = DescriptorDistance(MPdescriptor,d);

                        if(dist<bestDist)
                        {
                            bestDist2=bestDist;
                            bestDist=dist;
                            bestLevel2 = bestLevel;
                            bestLevel = F.mvKeysRight[idx].octave;
                            bestIdx=idx;
                        }
                        else if(dist<bestDist2)
                        {
                            bestLevel2 = F.mvKeysRight[idx].octave;
                            bestDist2=dist;
                        }
                    }

                    // 右目分支同样执行距离阈值和比值测试。
                    if(bestDist<=TH_HIGH)
                    {
                        if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                            continue;

                        // 如果右目特征在系统中已有“右到左”对应关系，也同步补上左目观测。
                        if(F.Nleft != -1 && F.mvRightToLeftMatch[bestIdx] != -1){ //Also match with the stereo observation at right camera
                            F.mvpMapPoints[F.mvRightToLeftMatch[bestIdx]] = pMP;
                            nmatches++;
                            left++;
                        }


                        // 把当前地图点绑定到右目最佳特征。
                        F.mvpMapPoints[bestIdx + F.Nleft]=pMP;
                        nmatches++;
                        right++;
                    }
                }
            }
        }
        return nmatches;
    }

    /**
     * @brief         根据视角余弦值返回搜索半径
     * @param viewCos 地图点法向与当前观测方向夹角的余弦值
     * @return        推荐使用的搜索半径倍率
     *
     * viewCos 越大表示视角越正，地图点投影越稳定，因此允许使用更小的搜索窗口。
     */
    float ORBmatcher::RadiusByViewingCos(const float &viewCos)
    {
        if(viewCos>0.998)
            return 2.5;
        else
            return 4.0;
    }

    /**
     * @brief                  使用词袋模型在关键帧与普通帧之间搜索匹配
     * @param pKF              参考关键帧
     * @param F                当前帧
     * @param vpMapPointMatches 输出当前帧中每个特征对应的地图点
     * @return                 匹配成功的数量
     *
     * 核心思想：
     * 1. 只在同一个 BoW 结点中的 ORB 特征之间比较描述子；
     * 2. 使用最佳 / 次佳描述子距离做最近邻比值测试；
     * 3. 最后再用方向直方图保留主方向一致的匹配。
     */
    int ORBmatcher::SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)
    {
        // 关键帧中的地图点与特征是一一对应的，这里取出所有已观测到的地图点。
        const vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();

        // 输出数组大小与当前帧特征数一致，初值均为空。
        vpMapPointMatches = vector<MapPoint*>(F.N,static_cast<MapPoint*>(NULL));

        // DBoW2 的 FeatureVector 记录“词袋结点 -> 落入该结点的特征索引列表”。
        const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;

        int nmatches=0;

        // 用于后续方向一致性检验的旋转直方图。
        vector<int> rotHist[HISTO_LENGTH];

        // 提前为每个 bin 预留容量，减少大量 push_back 带来的重复扩容。
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f/HISTO_LENGTH;

        // 通过同时遍历两个 FeatureVector，只比较落在同一词袋结点中的特征。
        DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
        DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
        DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
        DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

        // 类似双指针归并：
        // 只有当两个词袋结点编号相等时，才说明这两个结点里的特征具备直接比较的意义。
        while(KFit != KFend && Fit != Fend)
        {
            if(KFit->first == Fit->first)
            {
                // 当前结点内，KFit->second / Fit->second 分别是关键帧和当前帧的特征索引。
                const vector<unsigned int> vIndicesKF = KFit->second;
                const vector<unsigned int> vIndicesF = Fit->second;

                for(size_t iKF=0; iKF<vIndicesKF.size(); iKF++)
                {
                    const unsigned int realIdxKF = vIndicesKF[iKF];

                    MapPoint* pMP = vpMapPointsKF[realIdxKF];

                    // 没有关联地图点或地图点已失效时，不需要做投影跟踪匹配。
                    if(!pMP)
                        continue;

                    if(pMP->isBad())
                        continue;

                    // 取出关键帧该特征的 ORB 描述子。
                    const cv::Mat &dKF= pKF->mDescriptors.row(realIdxKF);

                    // 单目 / 左右目分别维护最佳与次佳距离。
                    int bestDist1=256;
                    int bestIdxF =-1 ;
                    int bestDist2=256;

                    int bestDist1R=256;
                    int bestIdxFR =-1 ;
                    int bestDist2R=256;

                    // 遍历当前 BoW 结点内属于当前帧的所有候选特征，
                    // 在这个小集合中找出和关键帧特征最相似的那个。
                    for(size_t iF=0; iF<vIndicesF.size(); iF++)
                    {
                        if(F.Nleft == -1){
                            const unsigned int realIdxF = vIndicesF[iF];

                            // 当前帧该特征已经匹配到别的地图点，则不再参与竞争。
                            if(vpMapPointMatches[realIdxF])
                                continue;

                            const cv::Mat &dF = F.mDescriptors.row(realIdxF);

                            // 比较描述子汉明距离，记录最佳与次佳。
                            const int dist =  DescriptorDistance(dKF,dF);

                            if(dist<bestDist1)
                            {
                                bestDist2=bestDist1;
                                bestDist1=dist;
                                bestIdxF=realIdxF;
                            }
                            else if(dist<bestDist2)
                            {
                                bestDist2=dist;
                            }
                        }
                        else{
                            const unsigned int realIdxF = vIndicesF[iF];

                            if(vpMapPointMatches[realIdxF])
                                continue;

                            const cv::Mat &dF = F.mDescriptors.row(realIdxF);

                            const int dist =  DescriptorDistance(dKF,dF);

                            // 双目情况下，左目特征和右目特征分别维护一套最佳 / 次佳距离。
                            if(realIdxF < F.Nleft && dist<bestDist1){
                                bestDist2=bestDist1;
                                bestDist1=dist;
                                bestIdxF=realIdxF;
                            }
                            else if(realIdxF < F.Nleft && dist<bestDist2){
                                bestDist2=dist;
                            }

                            if(realIdxF >= F.Nleft && dist<bestDist1R){
                                bestDist2R=bestDist1R;
                                bestDist1R=dist;
                                bestIdxFR=realIdxF;
                            }
                            else if(realIdxF >= F.Nleft && dist<bestDist2R){
                                bestDist2R=dist;
                            }
                        }

                    }

                    if(bestDist1<=TH_LOW)
                    {
                        // 左目匹配通过距离阈值后，再执行最近邻比值测试。
                        if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                        {
                            vpMapPointMatches[bestIdxF]=pMP;

                            const cv::KeyPoint &kp =
                                    (!pKF->mpCamera2) ? pKF->mvKeysUn[realIdxKF] :
                                    (realIdxKF >= pKF -> NLeft) ? pKF -> mvKeysRight[realIdxKF - pKF -> NLeft]
                                                                : pKF -> mvKeys[realIdxKF];

                            if(mbCheckOrientation)
                            {
                                // 记录匹配对的旋转差，后续只保留主方向上的匹配。
                                cv::KeyPoint &Fkp =
                                        (!pKF->mpCamera2 || F.Nleft == -1) ? F.mvKeys[bestIdxF] :
                                        (bestIdxF >= F.Nleft) ? F.mvKeysRight[bestIdxF - F.Nleft]
                                                              : F.mvKeys[bestIdxF];

                                float rot = kp.angle-Fkp.angle;
                                if(rot<0.0)
                                    rot+=360.0f;
                                int bin = round(rot*factor);
                                if(bin==HISTO_LENGTH)
                                    bin=0;
                                assert(bin>=0 && bin<HISTO_LENGTH);
                                rotHist[bin].push_back(bestIdxF);
                            }
                            nmatches++;
                        }

                        if(bestDist1R<=TH_LOW)
                        {
                            // 右目分支同理。
                            if(static_cast<float>(bestDist1R)<mfNNratio*static_cast<float>(bestDist2R) || true)
                            {
                                vpMapPointMatches[bestIdxFR]=pMP;

                                const cv::KeyPoint &kp =
                                        (!pKF->mpCamera2) ? pKF->mvKeysUn[realIdxKF] :
                                        (realIdxKF >= pKF -> NLeft) ? pKF -> mvKeysRight[realIdxKF - pKF -> NLeft]
                                                                    : pKF -> mvKeys[realIdxKF];

                                if(mbCheckOrientation)
                                {
                                    cv::KeyPoint &Fkp =
                                            (!F.mpCamera2) ? F.mvKeys[bestIdxFR] :
                                            (bestIdxFR >= F.Nleft) ? F.mvKeysRight[bestIdxFR - F.Nleft]
                                                                   : F.mvKeys[bestIdxFR];

                                    float rot = kp.angle-Fkp.angle;
                                    if(rot<0.0)
                                        rot+=360.0f;
                                    int bin = round(rot*factor);
                                    if(bin==HISTO_LENGTH)
                                        bin=0;
                                    assert(bin>=0 && bin<HISTO_LENGTH);
                                    rotHist[bin].push_back(bestIdxFR);
                                }
                                nmatches++;
                            }
                        }
                    }

                }

                KFit++;
                Fit++;
            }
            else if(KFit->first < Fit->first)
            {
                // 谁的词袋结点编号小，谁就跳到对方结点附近，避免无效遍历。
                KFit = vFeatVecKF.lower_bound(Fit->first);
            }
            else
            {
                Fit = F.mFeatVec.lower_bound(KFit->first);
            }
        }

        if(mbCheckOrientation)
        {
            // 只保留旋转差直方图中最大的三个 bin，对离群方向匹配进行剔除。
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i==ind1 || i==ind2 || i==ind3)
                    continue;

                // 当前 bin 不属于主方向峰值，说明这些匹配的相对旋转方向不稳定，
                // 逐个撤销这些匹配。
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    vpMapPointMatches[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                    nmatches--;
                }
            }
        }

        return nmatches;
    }

    /**
     * @brief             使用 Sim3 变换将地图点投影到关键帧中并搜索匹配
     * @param pKF         目标关键帧
     * @param Scw         从世界坐标到相机坐标的 Sim3 变换
     * @param vpPoints    待投影地图点
     * @param vpMatched   输出匹配结果
     * @param th          搜索半径倍率
     * @param ratioHamming 描述子阈值倍率
     * @return            匹配成功的数量
     *
     * 用于闭环检测阶段。已知一个近似 Sim3 后，把候选地图点投影到关键帧图像中，
     * 再通过尺度、可见性和描述子相似性来确认匹配。
     */
    int ORBmatcher::SearchByProjection(KeyFrame* pKF, Sophus::Sim3f &Scw, const vector<MapPoint*> &vpPoints,
                                       vector<MapPoint*> &vpMatched, int th, float ratioHamming)
    {
        // 相机内参在手工投影或半径估计时会用到。
        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;

        // Sim3 中含尺度，转换为 SE3 时需要把平移除以 scale。
        Sophus::SE3f Tcw = Sophus::SE3f(Scw.rotationMatrix(),Scw.translation()/Scw.scale());
        Eigen::Vector3f Ow = Tcw.inverse().translation();

        // 把已经匹配过的地图点放入集合，避免重复分配。
        set<MapPoint*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
        spAlreadyFound.erase(static_cast<MapPoint*>(NULL));

        int nmatches=0;

        // 对每个候选地图点：投影、可见性检查、尺度检查、描述子匹配。
        for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
        {
            MapPoint* pMP = vpPoints[iMP];

            // 坏点和已匹配点直接跳过。
            if(pMP->isBad() || spAlreadyFound.count(pMP))
                continue;

            // 取地图点世界坐标。
            Eigen::Vector3f p3Dw = pMP->GetWorldPos();

            // 变换到目标关键帧相机坐标系。
            Eigen::Vector3f p3Dc = Tcw * p3Dw;

            // 相机前方深度必须为正。
            if(p3Dc(2)<0.0)
                continue;

            // 投影到图像平面。
            const Eigen::Vector2f uv = pKF->mpCamera->project(p3Dc);

            // 投影必须落在图像内。
            if(!pKF->IsInImage(uv(0),uv(1)))
                continue;

            // 地图点被创建时记录了其可稳定观测的距离区间，只在该尺度不变范围内才尝试匹配。
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            Eigen::Vector3f PO = p3Dw-Ow;
            const float dist = PO.norm();

            if(dist<minDistance || dist>maxDistance)
                continue;

            // 视角不能偏离地图点法向过大，否则描述子外观变化会太明显。
            Eigen::Vector3f Pn = pMP->GetNormal();

            if(PO.dot(Pn)<0.5*dist)
                continue;

            int nPredictedLevel = pMP->PredictScale(dist,pKF);

            // 依据预测尺度决定搜索半径。
            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF->GetFeaturesInArea(uv(0),uv(1),radius);

            if(vIndices.empty())
                continue;

            // 在局部窗口中寻找描述子距离最近的关键点。
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = 256;
            int bestIdx = -1;

            // 遍历投影窗口中的所有候选关键点，选出描述子距离最小的那个。
            for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
            {
                const size_t idx = *vit;
                if(vpMatched[idx])
                    continue;

                const int &kpLevel= pKF->mvKeysUn[idx].octave;

                if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            // 通过严格距离阈值后，接受该匹配。
            if(bestDist<=TH_LOW*ratioHamming)
            {
                vpMatched[bestIdx]=pMP;
                nmatches++;
            }

        }

        return nmatches;
    }

    /**
     * @brief              使用 Sim3 投影地图点并记录其来源关键帧
     * @param pKF          目标关键帧
     * @param Scw          Sim3 变换
     * @param vpPoints     候选地图点
     * @param vpPointsKFs  每个候选地图点对应的来源关键帧
     * @param vpMatched    输出匹配到的地图点
     * @param vpMatchedKF  输出匹配点对应的来源关键帧
     * @param th           搜索半径倍率
     * @param ratioHamming 描述子阈值倍率
     * @return             匹配成功的数量
     *
     * 逻辑与上一个 SearchByProjection 很接近，只是额外保留“这个匹配来自哪个关键帧”。
     */
    int ORBmatcher::SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, const std::vector<KeyFrame*> &vpPointsKFs,
                                       std::vector<MapPoint*> &vpMatched, std::vector<KeyFrame*> &vpMatchedKF, int th, float ratioHamming)
    {
        // 相机内参。
        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;

        // Sim3 去掉尺度后转换成相机位姿。
        Sophus::SE3f Tcw = Sophus::SE3f(Scw.rotationMatrix(),Scw.translation()/Scw.scale());
        Eigen::Vector3f Ow = Tcw.inverse().translation();

        // 已经被匹配到的地图点不再重复参与竞争。
        set<MapPoint*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
        spAlreadyFound.erase(static_cast<MapPoint*>(NULL));

        int nmatches=0;

        // 对每个候选地图点重复执行“投影 + 局部最佳匹配”流程。
        for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
        {
            MapPoint* pMP = vpPoints[iMP];
            KeyFrame* pKFi = vpPointsKFs[iMP];

            // 跳过坏点和重复点。
            if(pMP->isBad() || spAlreadyFound.count(pMP))
                continue;

            // 世界坐标 -> 相机坐标。
            Eigen::Vector3f p3Dw = pMP->GetWorldPos();

            // Transform into Camera Coords.
            Eigen::Vector3f p3Dc = Tcw * p3Dw;

            // Depth must be positive
            if(p3Dc(2)<0.0)
                continue;

            // 这里直接用 pinhole 公式投影，得到像素坐标。
            const float invz = 1/p3Dc(2);
            const float x = p3Dc(0)*invz;
            const float y = p3Dc(1)*invz;

            const float u = fx*x+cx;
            const float v = fy*y+cy;

            // Point must be inside the image
            if(!pKF->IsInImage(u,v))
                continue;

            // 只在该地图点的有效观测距离范围内尝试匹配。
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            Eigen::Vector3f PO = p3Dw-Ow;
            const float dist = PO.norm();

            if(dist<minDistance || dist>maxDistance)
                continue;

            // 视角过斜时直接剔除。
            Eigen::Vector3f Pn = pMP->GetNormal();

            if(PO.dot(Pn)<0.5*dist)
                continue;

            int nPredictedLevel = pMP->PredictScale(dist,pKF);

            // 依据尺度预测在局部窗口中搜索。
            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

            if(vIndices.empty())
                continue;

            // 找到局部窗口内最相似的 ORB 特征。
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = 256;
            int bestIdx = -1;

            // 在投影窗口中寻找当前地图点最相近的关键点，并同步保留其来源关键帧。
            for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
            {
                const size_t idx = *vit;
                if(vpMatched[idx])
                    continue;

                const int &kpLevel= pKF->mvKeysUn[idx].octave;

                if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if(bestDist<=TH_LOW*ratioHamming)
            {
                vpMatched[bestIdx] = pMP;
                // 附带记录该地图点原本来自哪个关键帧。
                vpMatchedKF[bestIdx] = pKFi;
                nmatches++;
            }

        }

        return nmatches;
    }

    /**
     * @brief               单目初始化阶段的两帧匹配
     * @param F1            第一帧
     * @param F2            第二帧
     * @param vbPrevMatched 第一帧特征在第二帧中的预测位置
     * @param vnMatches12   输出 F1 -> F2 的匹配索引
     * @param windowSize    搜索窗口半径
     * @return              匹配成功的数量
     *
     * 这是单目初始化时的专用匹配策略。它与普通 BoW/投影匹配不同：
     * 1. 只使用第 0 层特征，避免尺度变化干扰；
     * 2. 以预测位置为中心做局部窗口搜索；
     * 3. 同时维护正向和反向唯一性，避免多个点抢同一个匹配。
     */
    int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)
    {
        int nmatches=0;

        // vnMatches12[i] = j 表示 F1 的第 i 个特征匹配到 F2 的第 j 个特征。
        vnMatches12 = vector<int>(F1.mvKeysUn.size(),-1);

        // 方向一致性直方图。
        vector<int> rotHist[HISTO_LENGTH];

        // 每个 bin 预留空间，避免初始化匹配中频繁扩容。
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f/HISTO_LENGTH;

        // vMatchedDistance / vnMatches21 用于保证 F2 中一个特征只被最好的一方占用。
        vector<int> vMatchedDistance(F2.mvKeysUn.size(),INT_MAX);
        vector<int> vnMatches21(F2.mvKeysUn.size(),-1);

        // 遍历第一帧所有候选特征，尝试为每个特征在第二帧中找到唯一对应点。
        for(size_t i1=0, iend1=F1.mvKeysUn.size(); i1<iend1; i1++)
        {
            cv::KeyPoint kp1 = F1.mvKeysUn[i1];
            int level1 = kp1.octave;

            // 初始化阶段只使用最底层特征，减少尺度不确定性。
            if(level1>0)
                continue;

            // 在上一轮估计的位置附近做局部搜索。
            vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1);

            if(vIndices2.empty())
                continue;

            cv::Mat d1 = F1.mDescriptors.row(i1);

            // 记录最佳和次佳距离。
            int bestDist = INT_MAX;
            int bestDist2 = INT_MAX;
            int bestIdx2 = -1;

            // 只在预测窗口内搜索，这个循环就是对局部候选做最佳/次佳比较。
            for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
            {
                size_t i2 = *vit;

                cv::Mat d2 = F2.mDescriptors.row(i2);

                int dist = DescriptorDistance(d1,d2);

                // 如果 F2 的这个特征已经被更近的描述子匹配过，就直接跳过。
                if(vMatchedDistance[i2]<=dist)
                    continue;

                if(dist<bestDist)
                {
                    bestDist2=bestDist;
                    bestDist=dist;
                    bestIdx2=i2;
                }
                else if(dist<bestDist2)
                {
                    bestDist2=dist;
                }
            }

            if(bestDist<=TH_LOW)
            {
                // 通过距离阈值后，再执行最近邻比值测试。
                if(bestDist<(float)bestDist2*mfNNratio)
                {
                    // 如果 bestIdx2 之前已经被别的 F1 特征占用，则撤销旧匹配，保留更优者。
                    if(vnMatches21[bestIdx2]>=0)
                    {
                        vnMatches12[vnMatches21[bestIdx2]]=-1;
                        nmatches--;
                    }
                    vnMatches12[i1]=bestIdx2;
                    vnMatches21[bestIdx2]=i1;
                    vMatchedDistance[bestIdx2]=bestDist;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        // 保存旋转差，用于后续整体方向一致性筛选。
                        float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(i1);
                    }
                }
            }

        }

        if(mbCheckOrientation)
        {
            // 初始化匹配也会剔除方向直方图中的离群 bin。
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i==ind1 || i==ind2 || i==ind3)
                    continue;

                // 该方向 bin 不是主峰，说明其中大部分匹配不满足整体旋转一致性，逐个剔除。
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    int idx1 = rotHist[i][j];
                    if(vnMatches12[idx1]>=0)
                    {
                        vnMatches12[idx1]=-1;
                        nmatches--;
                    }
                }
            }

        }

        // 用当前的匹配结果更新下一轮搜索的预测中心。
        for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
            if(vnMatches12[i1]>=0)
                vbPrevMatched[i1]=F2.mvKeysUn[vnMatches12[i1]].pt;

        return nmatches;
    }

    /**
     * @brief            使用 BoW 在两个关键帧之间搜索匹配
     * @param pKF1       关键帧 1
     * @param pKF2       关键帧 2
     * @param vpMatches12 输出从关键帧 1 到关键帧 2 的匹配地图点
     * @return           匹配成功的数量
     *
     * 该函数与 KeyFrame-Frame 版本类似，但这里两端都是关键帧，
     * 且只对已经关联地图点的特征做匹配，主要用于回环 / 共视关系建立。
     */
    int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)
    {
        // 取出两个关键帧的关键点、词袋向量、地图点关联和描述子。
        const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeysUn;
        const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
        const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
        const cv::Mat &Descriptors1 = pKF1->mDescriptors;

        const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn;
        const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
        const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
        const cv::Mat &Descriptors2 = pKF2->mDescriptors;

        // 输出结果大小按关键帧 1 的地图点数初始化。
        vpMatches12 = vector<MapPoint*>(vpMapPoints1.size(),static_cast<MapPoint*>(NULL));

        // 标记关键帧 2 的哪些特征已经被占用，避免一对多匹配。
        vector<bool> vbMatched2(vpMapPoints2.size(),false);

        // 方向一致性直方图。
        vector<int> rotHist[HISTO_LENGTH];

        // 预留容量，减少在回环候选较多时的动态扩容。
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);

        const float factor = 1.0f/HISTO_LENGTH;

        int nmatches = 0;

        DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
        DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
        DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
        DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

        // 两个关键帧的词袋结点按编号有序，while 循环通过同步推进只比较相同结点。
        while(f1it != f1end && f2it != f2end)
        {
            if(f1it->first == f2it->first)
            {
                // 只比较位于同一词袋结点中的特征。
                for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
                {
                    const size_t idx1 = f1it->second[i1];
                    if(pKF1 -> NLeft != -1 && idx1 >= pKF1 -> mvKeysUn.size()){
                        // 双目情况下，跳过不在统一索引范围内的异常索引。
                        continue;
                    }

                    MapPoint* pMP1 = vpMapPoints1[idx1];
                    if(!pMP1)
                        continue;
                    if(pMP1->isBad())
                        continue;

                    const cv::Mat &d1 = Descriptors1.row(idx1);

                    // 维护关键帧 2 中的最佳和次佳匹配。
                    int bestDist1=256;
                    int bestIdx2 =-1 ;
                    int bestDist2=256;

                    // 扫描当前词袋结点在关键帧 2 中的全部候选，寻找最佳与次佳外观匹配。
                    for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                    {
                        const size_t idx2 = f2it->second[i2];

                        if(pKF2 -> NLeft != -1 && idx2 >= pKF2 -> mvKeysUn.size()){
                            continue;
                        }

                        MapPoint* pMP2 = vpMapPoints2[idx2];

                        // 目标特征已被匹配、没有地图点或者地图点失效时直接跳过。
                        if(vbMatched2[idx2] || !pMP2)
                            continue;

                        if(pMP2->isBad())
                            continue;

                        const cv::Mat &d2 = Descriptors2.row(idx2);

                        int dist = DescriptorDistance(d1,d2);

                        // 保留最佳和次佳描述子距离，用于比值测试。
                        if(dist<bestDist1)
                        {
                            bestDist2=bestDist1;
                            bestDist1=dist;
                            bestIdx2=idx2;
                        }
                        else if(dist<bestDist2)
                        {
                            bestDist2=dist;
                        }
                    }

                    if(bestDist1<TH_LOW)
                    {
                        // 通过最近邻比值测试后，接受匹配。
                        if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                        {
                            vpMatches12[idx1]=vpMapPoints2[bestIdx2];
                            vbMatched2[bestIdx2]=true;

                            if(mbCheckOrientation)
                            {
                                // 记录关键点旋转差，用于方向一致性过滤。
                                float rot = vKeysUn1[idx1].angle-vKeysUn2[bestIdx2].angle;
                                if(rot<0.0)
                                    rot+=360.0f;
                                int bin = round(rot*factor);
                                if(bin==HISTO_LENGTH)
                                    bin=0;
                                assert(bin>=0 && bin<HISTO_LENGTH);
                                rotHist[bin].push_back(idx1);
                            }
                            nmatches++;
                        }
                    }
                }

                f1it++;
                f2it++;
            }
            else if(f1it->first < f2it->first)
            {
                // 与 Frame 版相同，谁落后谁跳。
                f1it = vFeatVec1.lower_bound(f2it->first);
            }
            else
            {
                f2it = vFeatVec2.lower_bound(f1it->first);
            }
        }

        if(mbCheckOrientation)
        {
            // 只保留主方向上的匹配，剔除方向离群项。
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i==ind1 || i==ind2 || i==ind3)
                    continue;

                // 撤销所有落在非主峰 bin 中的匹配。
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    vpMatches12[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                    nmatches--;
                }
            }
        }

        return nmatches;
    }

    /**
     * @brief               搜索可用于三角化的新匹配对
     * @param pKF1          关键帧 1
     * @param pKF2          关键帧 2
     * @param vMatchedPairs 输出匹配索引对
     * @param bOnlyStereo   是否只使用具备立体深度的点
     * @param bCoarse       是否启用粗匹配模式
     * @return              匹配成功的数量
     *
     * 该函数用于生成“尚未有地图点关联”的新特征对，供后续三角化生成新地图点。
     * 因此它会特别跳过已经有关联 MapPoint 的特征，并加入极线约束来保证几何一致性。
     */
    int ORBmatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2,
                                           vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo, const bool bCoarse)
    {
        const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
        const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;

        // 计算关键帧 1 的相机中心在关键帧 2 图像中的极点位置。
        Sophus::SE3f T1w = pKF1->GetPose();
        Sophus::SE3f T2w = pKF2->GetPose();
        Sophus::SE3f Tw2 = pKF2->GetPoseInverse(); // for convenience
        Eigen::Vector3f Cw = pKF1->GetCameraCenter();
        Eigen::Vector3f C2 = T2w * Cw;

        Eigen::Vector2f ep = pKF2->mpCamera->project(C2);
        Sophus::SE3f T12;
        Sophus::SE3f Tll, Tlr, Trl, Trr;
        Eigen::Matrix3f R12; // for fastest computation
        Eigen::Vector3f t12; // for fastest computation

        GeometricCamera* pCamera1 = pKF1->mpCamera, *pCamera2 = pKF2->mpCamera;

        if(!pKF1->mpCamera2 && !pKF2->mpCamera2){
            // 单目 / 双目统一相机情况下，只需要一套左-左变换。
            T12 = T1w * Tw2;
            R12 = T12.rotationMatrix();
            t12 = T12.translation();
        }
        else{
            // 若存在左右目，则预先计算左左、左右、右左、右右四种相机组合之间的相对位姿。
            Sophus::SE3f Tr1w = pKF1->GetRightPose();
            Sophus::SE3f Twr2 = pKF2->GetRightPoseInverse();
            Tll = T1w * Tw2;
            Tlr = T1w * Twr2;
            Trl = Tr1w * Tw2;
            Trr = Tr1w * Twr2;
        }

        Eigen::Matrix3f Rll = Tll.rotationMatrix(), Rlr  = Tlr.rotationMatrix(), Rrl  = Trl.rotationMatrix(), Rrr  = Trr.rotationMatrix();
        Eigen::Vector3f tll = Tll.translation(), tlr = Tlr.translation(), trl = Trl.translation(), trr = Trr.translation();

        // 只为“还没有地图点”的特征寻找匹配，并且只在相同 BoW 结点内比较，减少复杂度。
        int nmatches=0;
        vector<bool> vbMatched2(pKF2->N,false);
        vector<int> vMatches12(pKF1->N,-1);

        vector<int> rotHist[HISTO_LENGTH];

        // 三角化候选可能很多，预分配 bin 空间。
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);

        const float factor = 1.0f/HISTO_LENGTH;

        DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
        DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
        DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
        DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

        // 与 BoW 匹配相同，使用双指针仅在同一词袋结点内寻找候选配对。
        while(f1it!=f1end && f2it!=f2end)
        {
            if(f1it->first == f2it->first)
            {
                // 遍历关键帧 1 当前词袋结点内的每个“尚无地图点”的特征。
                for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
                {
                    const size_t idx1 = f1it->second[i1];

                    MapPoint* pMP1 = pKF1->GetMapPoint(idx1);

                    // 若该特征已经有地图点，则不需要再为它三角化新点。
                    if(pMP1)
                    {
                        continue;
                    }

                    // 判断该特征是否具备立体观测。
                    const bool bStereo1 = (!pKF1->mpCamera2 && pKF1->mvuRight[idx1]>=0);

                    if(bOnlyStereo)
                        if(!bStereo1)
                            continue;

                    const cv::KeyPoint &kp1 = (pKF1 -> NLeft == -1) ? pKF1->mvKeysUn[idx1]
                                                                    : (idx1 < pKF1 -> NLeft) ? pKF1 -> mvKeys[idx1]
                                                                                             : pKF1 -> mvKeysRight[idx1 - pKF1 -> NLeft];

                    const bool bRight1 = (pKF1 -> NLeft == -1 || idx1 < pKF1 -> NLeft) ? false
                                                                                       : true;

                    const cv::Mat &d1 = pKF1->mDescriptors.row(idx1);

                    // Triangulation 阶段只需要一个当前最佳匹配，不做次佳比值测试，
                    // 因为后面还有极线约束和方向一致性帮助过滤。
                    int bestDist = TH_LOW;
                    int bestIdx2 = -1;

                    // 遍历关键帧 2 同一词袋结点内的所有候选特征，
                    // 用描述子距离和极线约束共同挑出最适合三角化的一对。
                    for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                    {
                        size_t idx2 = f2it->second[i2];

                        MapPoint* pMP2 = pKF2->GetMapPoint(idx2);

                        // 已匹配过、或者目标特征已有地图点，则跳过。
                        if(vbMatched2[idx2] || pMP2)
                            continue;

                        const bool bStereo2 = (!pKF2->mpCamera2 &&  pKF2->mvuRight[idx2]>=0);

                        if(bOnlyStereo)
                            if(!bStereo2)
                                continue;

                        const cv::Mat &d2 = pKF2->mDescriptors.row(idx2);

                        const int dist = DescriptorDistance(d1,d2);

                        // 超过阈值或不比当前最好更优时直接跳过。
                        if(dist>TH_LOW || dist>bestDist)
                            continue;

                        const cv::KeyPoint &kp2 = (pKF2 -> NLeft == -1) ? pKF2->mvKeysUn[idx2]
                                                                        : (idx2 < pKF2 -> NLeft) ? pKF2 -> mvKeys[idx2]
                                                                                                 : pKF2 -> mvKeysRight[idx2 - pKF2 -> NLeft];
                        const bool bRight2 = (pKF2 -> NLeft == -1 || idx2 < pKF2 -> NLeft) ? false
                                                                                           : true;

                        if(!bStereo1 && !bStereo2 && !pKF1->mpCamera2)
                        {
                            // 单目-单目时，若匹配点过于靠近极点，则三角化容易退化，故跳过。
                            const float distex = ep(0)-kp2.pt.x;
                            const float distey = ep(1)-kp2.pt.y;
                            if(distex*distex+distey*distey<100*pKF2->mvScaleFactors[kp2.octave])
                            {
                                continue;
                            }
                        }

                        if(pKF1->mpCamera2 && pKF2->mpCamera2){
                            // 如果两边都有左右目，需要根据特征来自左目还是右目切换相机组合。
                            if(bRight1 && bRight2){
                                R12 = Rrr;
                                t12 = trr;
                                T12 = Trr;

                                pCamera1 = pKF1->mpCamera2;
                                pCamera2 = pKF2->mpCamera2;
                            }
                            else if(bRight1 && !bRight2){
                                R12 = Rrl;
                                t12 = trl;
                                T12 = Trl;

                                pCamera1 = pKF1->mpCamera2;
                                pCamera2 = pKF2->mpCamera;
                            }
                            else if(!bRight1 && bRight2){
                                R12 = Rlr;
                                t12 = tlr;
                                T12 = Tlr;

                                pCamera1 = pKF1->mpCamera;
                                pCamera2 = pKF2->mpCamera2;
                            }
                            else{
                                R12 = Rll;
                                t12 = tll;
                                T12 = Tll;

                                pCamera1 = pKF1->mpCamera;
                                pCamera2 = pKF2->mpCamera;
                            }

                        }

                        // 粗匹配模式下跳过严格极线约束；否则要求满足极线几何。
                        if(bCoarse || pCamera1->epipolarConstrain(pCamera2,kp1,kp2,R12,t12,pKF1->mvLevelSigma2[kp1.octave],pKF2->mvLevelSigma2[kp2.octave])) // MODIFICATION_2
                        {
                            bestIdx2 = idx2;
                            bestDist = dist;
                        }
                    }

                    if(bestIdx2>=0)
                    {
                        // 找到合格匹配后，先记录索引，后面统一转成 pair 输出。
                        const cv::KeyPoint &kp2 = (pKF2 -> NLeft == -1) ? pKF2->mvKeysUn[bestIdx2]
                                                                        : (bestIdx2 < pKF2 -> NLeft) ? pKF2 -> mvKeys[bestIdx2]
                                                                                                     : pKF2 -> mvKeysRight[bestIdx2 - pKF2 -> NLeft];
                        vMatches12[idx1]=bestIdx2;
                        nmatches++;

                        if(mbCheckOrientation)
                        {
                            // 记录方向差，用于最后剔除方向离群匹配。
                            float rot = kp1.angle-kp2.angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(idx1);
                        }
                    }
                }

                f1it++;
                f2it++;
            }
            else if(f1it->first < f2it->first)
            {
                // 双指针加 lower_bound 跳表式推进。
                f1it = vFeatVec1.lower_bound(f2it->first);
            }
            else
            {
                f2it = vFeatVec2.lower_bound(f1it->first);
            }
        }

        if(mbCheckOrientation)
        {
            // 方向一致性过滤。
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i==ind1 || i==ind2 || i==ind3)
                    continue;

                // 非主方向峰值中的匹配被认为更可能是误匹配，统一撤销。
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    vMatches12[rotHist[i][j]]=-1;
                    nmatches--;
                }
            }

        }

        // 把稀疏索引数组压缩成输出匹配对列表。
        vMatchedPairs.clear();
        vMatchedPairs.reserve(nmatches);

        // vMatches12 中大量元素仍为 -1，这个循环负责只收集真正配对成功的项。
        for(size_t i=0, iend=vMatches12.size(); i<iend; i++)
        {
            if(vMatches12[i]<0)
                continue;
            vMatchedPairs.push_back(make_pair(i,vMatches12[i]));
        }

        return nmatches;
    }

    /**
     * @brief            将地图点投影到关键帧并尝试融合
     * @param pKF        目标关键帧
     * @param vpMapPoints 候选地图点集合
     * @param th         搜索半径倍率
     * @param bRight     是否投影到右目
     * @return           成功融合的数量
     *
     * 融合的含义是：如果投影后的地图点在目标关键帧中找到了一个已经存在的观测，
     * 就判断它们是不是同一个物理点；如果是，则选择保留观测更多的那个地图点。
     */
    int ORBmatcher::Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th, const bool bRight)
    {
        GeometricCamera* pCamera;
        Sophus::SE3f Tcw;
        Eigen::Vector3f Ow;

        // 根据 bRight 决定使用左目还是右目的位姿和相机模型。
        if(bRight){
            Tcw = pKF->GetRightPose();
            Ow = pKF->GetRightCameraCenter();
            pCamera = pKF->mpCamera2;
        }
        else{
            Tcw = pKF->GetPose();
            Ow = pKF->GetCameraCenter();
            pCamera = pKF->mpCamera;
        }

        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;
        const float &bf = pKF->mbf;

        int nFused=0;

        const int nMPs = vpMapPoints.size();

        // 这些计数变量主要用于调试统计，逻辑上不影响融合结果。
        int count_notMP = 0, count_bad=0, count_isinKF = 0, count_negdepth = 0, count_notinim = 0, count_dist = 0, count_normal=0, count_notidx = 0, count_thcheck = 0;

        // 逐个把候选地图点投影到关键帧上，检查它们是否和现有观测重复。
        for(int i=0; i<nMPs; i++)
        {
            MapPoint* pMP = vpMapPoints[i];

            // 空指针直接跳过。
            if(!pMP)
            {
                count_notMP++;
                continue;
            }

            // 坏点或已经在该关键帧中出现过的点不需要再融合。
            if(pMP->isBad())
            {
                count_bad++;
                continue;
            }
            else if(pMP->IsInKeyFrame(pKF))
            {
                count_isinKF++;
                continue;
            }

            Eigen::Vector3f p3Dw = pMP->GetWorldPos();
            Eigen::Vector3f p3Dc = Tcw * p3Dw;

            // 相机前方深度必须为正。
            if(p3Dc(2)<0.0f)
            {
                count_negdepth++;
                continue;
            }

            const float invz = 1/p3Dc(2);

            const Eigen::Vector2f uv = pCamera->project(p3Dc);

            // 投影必须落在图像可见区域。
            if(!pKF->IsInImage(uv(0),uv(1)))
            {
                count_notinim++;
                continue;
            }

            const float ur = uv(0)-bf*invz;

            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            Eigen::Vector3f PO = p3Dw-Ow;
            const float dist3D = PO.norm();

            // 距离必须落在地图点的尺度不变范围内。
            if(dist3D<minDistance || dist3D>maxDistance) {
                count_dist++;
                continue;
            }

            // 观测方向不能与地图点法向偏差过大。
            Eigen::Vector3f Pn = pMP->GetNormal();

            if(PO.dot(Pn)<0.5*dist3D)
            {
                count_normal++;
                continue;
            }

            int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

            // 根据尺度预测决定局部搜索半径。
            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF->GetFeaturesInArea(uv(0),uv(1),radius,bRight);

            if(vIndices.empty())
            {
                count_notidx++;
                continue;
            }

            // 在局部窗口内寻找描述子最接近的关键点。

            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = 256;
            int bestIdx = -1;

            // 遍历投影窗口里的全部关键点，综合尺度、重投影误差和描述子距离筛出最佳者。
            for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
            {
                size_t idx = *vit;
                const cv::KeyPoint &kp = (pKF -> NLeft == -1) ? pKF->mvKeysUn[idx]
                                                              : (!bRight) ? pKF -> mvKeys[idx]
                                                                          : pKF -> mvKeysRight[idx];

                const int &kpLevel= kp.octave;

                // 候选关键点的尺度层要与预测层匹配。
                if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                    continue;

                if(pKF->mvuRight[idx]>=0)
                {
                    // 双目观测时，使用三维重投影误差（左 x/y + 右目视差）做卡方检验。
                    const float &kpx = kp.pt.x;
                    const float &kpy = kp.pt.y;
                    const float &kpr = pKF->mvuRight[idx];
                    const float ex = uv(0)-kpx;
                    const float ey = uv(1)-kpy;
                    const float er = ur-kpr;
                    const float e2 = ex*ex+ey*ey+er*er;

                    if(e2*pKF->mvInvLevelSigma2[kpLevel]>7.8)
                        continue;
                }
                else
                {
                    // 单目观测时，仅使用二维重投影误差做卡方检验。
                    const float &kpx = kp.pt.x;
                    const float &kpy = kp.pt.y;
                    const float ex = uv(0)-kpx;
                    const float ey = uv(1)-kpy;
                    const float e2 = ex*ex+ey*ey;

                    if(e2*pKF->mvInvLevelSigma2[kpLevel]>5.99)
                        continue;
                }

                if(bRight) idx += pKF->NLeft;

                const cv::Mat &dKF = pKF->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            // 若描述子距离足够小，则认为该地图点在关键帧里找到了对应观测。
            // 若该观测位置本来就挂着另一个地图点，则执行“保留观测更多的那个点”的替换策略；
            // 否则直接把该地图点加入关键帧观测中。
            if(bestDist<=TH_LOW)
            {
                MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
                if(pMPinKF)
                {
                    if(!pMPinKF->isBad())
                    {
                        if(pMPinKF->Observations()>pMP->Observations())
                            pMP->Replace(pMPinKF);
                        else
                            pMPinKF->Replace(pMP);
                    }
                }
                else
                {
                    pMP->AddObservation(pKF,bestIdx);
                    pKF->AddMapPoint(pMP,bestIdx);
                }
                nFused++;
            }
            else
                count_thcheck++;

        }

        return nFused;
    }

    /**
     * @brief             使用 Sim3 变换将地图点投影到关键帧中并尝试融合
     * @param pKF         目标关键帧
     * @param Scw         Sim3 变换
     * @param vpPoints    候选地图点
     * @param th          搜索半径倍率
     * @param vpReplacePoint 输出需要替换成的已有地图点
     * @return            成功融合的数量
     */
    int ORBmatcher::Fuse(KeyFrame *pKF, Sophus::Sim3f &Scw, const vector<MapPoint *> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint)
    {
        // 相机内参。
        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;

        // Sim3 去掉尺度后转换成 SE3 形式。
        Sophus::SE3f Tcw = Sophus::SE3f(Scw.rotationMatrix(),Scw.translation()/Scw.scale());
        Eigen::Vector3f Ow = Tcw.inverse().translation();

        // 已经在关键帧里的地图点不用再投影查重。
        const set<MapPoint*> spAlreadyFound = pKF->GetMapPoints();

        int nFused=0;

        const int nPoints = vpPoints.size();

        // 对每个候选点执行：投影 -> 可见性检查 -> 描述子匹配 -> 融合/替换。
        for(int iMP=0; iMP<nPoints; iMP++)
        {
            MapPoint* pMP = vpPoints[iMP];

            // 坏点或已在关键帧中的点直接跳过。
            if(pMP->isBad() || spAlreadyFound.count(pMP))
                continue;

            // 世界坐标 -> 相机坐标。
            Eigen::Vector3f p3Dw = pMP->GetWorldPos();

            // Transform into Camera Coords.
            Eigen::Vector3f p3Dc = Tcw * p3Dw;

            // Depth must be positive
            if(p3Dc(2)<0.0f)
                continue;

            // 投影到图像平面。
            const Eigen::Vector2f uv = pKF->mpCamera->project(p3Dc);

            // 投影点必须在图像内。
            if(!pKF->IsInImage(uv(0),uv(1)))
                continue;

            // 距离范围和视角都要合理，否则不尝试匹配。
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            Eigen::Vector3f PO = p3Dw-Ow;
            const float dist3D = PO.norm();

            if(dist3D<minDistance || dist3D>maxDistance)
                continue;

            // 法向一致性检查。
            Eigen::Vector3f Pn = pMP->GetNormal();

            if(PO.dot(Pn)<0.5*dist3D)
                continue;

            // 预测尺度层，缩小搜索范围。
            const int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

            // 局部搜索窗口。
            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF->GetFeaturesInArea(uv(0),uv(1),radius);

            if(vIndices.empty())
                continue;

            // 在局部范围中寻找最相似的 ORB 描述子。

            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = INT_MAX;
            int bestIdx = -1;

            // 这一轮在投影窗口内逐个比较描述子，找出最可能对应当前地图点的关键点。
            for(vector<size_t>::const_iterator vit=vIndices.begin(); vit!=vIndices.end(); vit++)
            {
                const size_t idx = *vit;
                const int &kpLevel = pKF->mvKeysUn[idx].octave;

                if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF->mDescriptors.row(idx);

                int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            // 若找到合格匹配：
            // 1. 该位置已有地图点，则把“待替换对象”写到 vpReplacePoint 中；
            // 2. 否则直接把该地图点加入关键帧观测。
            if(bestDist<=TH_LOW)
            {
                MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
                if(pMPinKF)
                {
                    if(!pMPinKF->isBad())
                        vpReplacePoint[iMP] = pMPinKF;
                }
                else
                {
                    pMP->AddObservation(pKF,bestIdx);
                    pKF->AddMapPoint(pMP,bestIdx);
                }
                nFused++;
            }
        }

        return nFused;
    }

    /**
     * @brief            基于 Sim3 约束在两个关键帧之间搜索匹配
     * @param pKF1       关键帧 1
     * @param pKF2       关键帧 2
     * @param vpMatches12 输入输出匹配数组
     * @param S12        从关键帧 2 到关键帧 1 的 Sim3 变换
     * @param th         搜索半径倍率
     * @return           匹配成功的数量
     *
     * 该函数采用“双向投影 + 一致性交叉验证”：
     * 1. 把 KF1 中的地图点投影到 KF2；
     * 2. 再把 KF2 中的地图点投影回 KF1；
     * 3. 只有双向都互相选中对方的匹配，才最终接受。
     */
    int ORBmatcher::SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const Sophus::Sim3f &S12, const float th)
    {
        const float &fx = pKF1->fx;
        const float &fy = pKF1->fy;
        const float &cx = pKF1->cx;
        const float &cy = pKF1->cy;

        // 两个关键帧各自从世界到相机的位姿。
        Sophus::SE3f T1w = pKF1->GetPose();
        Sophus::SE3f T2w = pKF2->GetPose();

        // S12 是 2->1，S21 则是 1->2。
        Sophus::Sim3f S21 = S12.inverse();

        const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
        const int N1 = vpMapPoints1.size();

        const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
        const int N2 = vpMapPoints2.size();

        vector<bool> vbAlreadyMatched1(N1,false);
        vector<bool> vbAlreadyMatched2(N2,false);

        // 输入数组中已有的匹配关系先标记出来，避免重复搜索。
        for(int i=0; i<N1; i++)
        {
            MapPoint* pMP = vpMatches12[i];
            if(pMP)
            {
                vbAlreadyMatched1[i]=true;
                int idx2 = get<0>(pMP->GetIndexInKeyFrame(pKF2));
                if(idx2>=0 && idx2<N2)
                    vbAlreadyMatched2[idx2]=true;
            }
        }

        vector<int> vnMatch1(N1,-1);
        vector<int> vnMatch2(N2,-1);

        // 第一轮：把 KF1 中的地图点变换到 KF2 并搜索局部最佳匹配。
        for(int i1=0; i1<N1; i1++)
        {
            MapPoint* pMP = vpMapPoints1[i1];

            // 无地图点、已匹配、坏点都无需处理。
            if(!pMP || vbAlreadyMatched1[i1])
                continue;

            if(pMP->isBad())
                continue;

            // 世界坐标 -> KF1 相机坐标 -> 通过 Sim3 到 KF2 相机坐标。
            Eigen::Vector3f p3Dw = pMP->GetWorldPos();
            Eigen::Vector3f p3Dc1 = T1w * p3Dw;
            Eigen::Vector3f p3Dc2 = S21 * p3Dc1;

            // 只有相机前方点才可见。
            if(p3Dc2(2)<0.0)
                continue;

            const float invz = 1.0/p3Dc2(2);
            const float x = p3Dc2(0)*invz;
            const float y = p3Dc2(1)*invz;

            const float u = fx*x+cx;
            const float v = fy*y+cy;

            // 投影必须在 KF2 图像范围内。
            if(!pKF2->IsInImage(u,v))
                continue;

            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            const float dist3D = p3Dc2.norm();

            // 距离需要落入地图点的尺度不变范围。
            if(dist3D<minDistance || dist3D>maxDistance )
                continue;

            // 根据投影距离预测尺度层。
            const int nPredictedLevel = pMP->PredictScale(dist3D,pKF2);

            // 在预测位置附近搜索。
            const float radius = th*pKF2->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u,v,radius);

            if(vIndices.empty())
                continue;

            // 局部窗口内选描述子距离最小者。
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = INT_MAX;
            int bestIdx = -1;

            // 这个循环只处理 KF2 中投影邻域内的候选关键点，
            // 最终得到“KF1 的第 i1 个地图点在 KF2 中最像谁”。
            for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
            {
                const size_t idx = *vit;

                const cv::KeyPoint &kp = pKF2->mvKeysUn[idx];

                if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF2->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if(bestDist<=TH_HIGH)
            {
                vnMatch1[i1]=bestIdx;
            }
        }

        // 第二轮：把 KF2 中的地图点投影到 KF1，做对称搜索。
        for(int i2=0; i2<N2; i2++)
        {
            MapPoint* pMP = vpMapPoints2[i2];

            if(!pMP || vbAlreadyMatched2[i2])
                continue;

            if(pMP->isBad())
                continue;

            Eigen::Vector3f p3Dw = pMP->GetWorldPos();
            Eigen::Vector3f p3Dc2 = T2w * p3Dw;
            Eigen::Vector3f p3Dc1 = S12 * p3Dc2;

            // 只接受深度为正的点。
            if(p3Dc1(2)<0.0)
                continue;

            const float invz = 1.0/p3Dc1(2);
            const float x = p3Dc1(0)*invz;
            const float y = p3Dc1(1)*invz;

            const float u = fx*x+cx;
            const float v = fy*y+cy;

            // 投影必须在 KF1 内。
            if(!pKF1->IsInImage(u,v))
                continue;

            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            const float dist3D = p3Dc1.norm();

            // 距离范围检查。
            if(dist3D<minDistance || dist3D>maxDistance)
                continue;

            // 预测尺度层。
            const int nPredictedLevel = pMP->PredictScale(dist3D,pKF1);

            // 搜索局部窗口。
            const float radius = th*pKF1->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF1->GetFeaturesInArea(u,v,radius);

            if(vIndices.empty())
                continue;

            // 选局部窗口内的最佳描述子匹配。
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = INT_MAX;
            int bestIdx = -1;

            // 与上一轮对称，这里得到“KF2 的第 i2 个地图点在 KF1 中最像谁”。
            for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
            {
                const size_t idx = *vit;

                const cv::KeyPoint &kp = pKF1->mvKeysUn[idx];

                if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF1->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if(bestDist<=TH_HIGH)
            {
                vnMatch2[i2]=bestIdx;
            }
        }

        // 双向一致性检查：只有 i1 -> idx2 且 idx2 -> i1 才最终成立。
        int nFound = 0;

        // 遍历第一轮匹配结果，用第二轮结果做交叉验证。
        for(int i1=0; i1<N1; i1++)
        {
            int idx2 = vnMatch1[i1];

            if(idx2>=0)
            {
                int idx1 = vnMatch2[idx2];
                if(idx1==i1)
                {
                    vpMatches12[i1] = vpMapPoints2[idx2];
                    nFound++;
                }
            }
        }

        return nFound;
    }

    /**
     * @brief            将上一帧地图点投影到当前帧并搜索匹配
     * @param CurrentFrame 当前帧
     * @param LastFrame    上一帧
     * @param th           搜索半径倍率
     * @param bMono        是否单目模式
     * @return             匹配成功的数量
     *
     * 用途：Tracking 中基于匀速模型或前一帧位姿的连续跟踪。
     */
    int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
    {
        int nmatches = 0;

        // 方向一致性直方图。
        vector<int> rotHist[HISTO_LENGTH];

        // 提前为每个方向 bin 分配空间。
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f/HISTO_LENGTH;

        // 当前帧位姿及其相机中心。
        const Sophus::SE3f Tcw = CurrentFrame.GetPose();
        const Eigen::Vector3f twc = Tcw.inverse().translation();

        // 上一帧位姿以及当前相机中心在上一帧坐标系下的位置。
        const Sophus::SE3f Tlw = LastFrame.GetPose();
        const Eigen::Vector3f tlc = Tlw * twc;

        // 根据前后向运动方向决定搜索尺度层范围。
        const bool bForward = tlc(2)>CurrentFrame.mb && !bMono;
        const bool bBackward = -tlc(2)>CurrentFrame.mb && !bMono;

        // 逐个处理上一帧中已经关联了地图点的特征。
        for(int i=0; i<LastFrame.N; i++)
        {
            MapPoint* pMP = LastFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!LastFrame.mvbOutlier[i])
                {
                    // 把地图点投影到当前帧。
                    Eigen::Vector3f x3Dw = pMP->GetWorldPos();
                    Eigen::Vector3f x3Dc = Tcw * x3Dw;

                    const float xc = x3Dc(0);
                    const float yc = x3Dc(1);
                    const float invzc = 1.0/x3Dc(2);

                    // 深度必须为正。
                    if(invzc<0)
                        continue;

                    Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dc);

                    // 投影点必须落在当前帧图像范围内。
                    if(uv(0)<CurrentFrame.mnMinX || uv(0)>CurrentFrame.mnMaxX)
                        continue;
                    if(uv(1)<CurrentFrame.mnMinY || uv(1)>CurrentFrame.mnMaxY)
                        continue;

                    // 以“上一帧该特征所在尺度层”作为当前搜索的尺度先验。
                    int nLastOctave = (LastFrame.Nleft == -1 || i < LastFrame.Nleft) ? LastFrame.mvKeys[i].octave
                                                                                     : LastFrame.mvKeysRight[i - LastFrame.Nleft].octave;

                    // 搜索半径与尺度层相关。
                    float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];

                    vector<size_t> vIndices2;

                    // 前向运动时更可能出现在更细层，后向运动时更可能出现在更粗层。
                    if(bForward)
                        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, nLastOctave);
                    else if(bBackward)
                        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, 0, nLastOctave);
                    else
                        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, nLastOctave-1, nLastOctave+1);

                    if(vIndices2.empty())
                        continue;

                    const cv::Mat dMP = pMP->GetDescriptor();

                    int bestDist = 256;
                    int bestIdx2 = -1;

                    // 逐个遍历当前帧左目候选特征，选出描述子距离最小的那个。
                    for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                    {
                        const size_t i2 = *vit;

                        // 当前帧该特征若已绑定有充分观测的地图点，则跳过。
                        if(CurrentFrame.mvpMapPoints[i2])
                            if(CurrentFrame.mvpMapPoints[i2]->Observations()>0)
                                continue;

                        // 单目双视差情形下，进一步检查右目投影误差。
                        if(CurrentFrame.Nleft == -1 && CurrentFrame.mvuRight[i2]>0)
                        {
                            const float ur = uv(0) - CurrentFrame.mbf*invzc;
                            const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
                            if(er>radius)
                                continue;
                        }

                        const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                        const int dist = DescriptorDistance(dMP,d);

                        if(dist<bestDist)
                        {
                            bestDist=dist;
                            bestIdx2=i2;
                        }
                    }

                    if(bestDist<=TH_HIGH)
                    {
                        // 接受匹配并把地图点挂到当前帧对应特征上。
                        CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                        nmatches++;

                        if(mbCheckOrientation)
                        {
                            // 记录旋转差，用于最后方向一致性筛选。
                            cv::KeyPoint kpLF = (LastFrame.Nleft == -1) ? LastFrame.mvKeysUn[i]
                                                                        : (i < LastFrame.Nleft) ? LastFrame.mvKeys[i]
                                                                                                : LastFrame.mvKeysRight[i - LastFrame.Nleft];

                            cv::KeyPoint kpCF = (CurrentFrame.Nleft == -1) ? CurrentFrame.mvKeysUn[bestIdx2]
                                                                           : (bestIdx2 < CurrentFrame.Nleft) ? CurrentFrame.mvKeys[bestIdx2]
                                                                                                             : CurrentFrame.mvKeysRight[bestIdx2 - CurrentFrame.Nleft];
                            float rot = kpLF.angle-kpCF.angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdx2);
                        }
                    }

                    // 如果当前帧是双目 / 双鱼眼，还会尝试在右目中补一个匹配。
                    if(CurrentFrame.Nleft != -1){
                        Eigen::Vector3f x3Dr = CurrentFrame.GetRelativePoseTrl() * x3Dc;
                        Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dr);

                        int nLastOctave = (LastFrame.Nleft == -1 || i < LastFrame.Nleft) ? LastFrame.mvKeys[i].octave
                                             : LastFrame.mvKeysRight[i - LastFrame.Nleft].octave;

                        // Search in a window. Size depends on scale
                        float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];

                        vector<size_t> vIndices2;

                        if(bForward)
                            vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, nLastOctave, -1,true);
                        else if(bBackward)
                            vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, 0, nLastOctave, true);
                        else
                            vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, nLastOctave-1, nLastOctave+1, true);

                        const cv::Mat dMP = pMP->GetDescriptor();

                        int bestDist = 256;
                        int bestIdx2 = -1;

                        // 右目分支同样在局部窗口中逐个比较描述子，寻找最优匹配。
                        for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                        {
                            const size_t i2 = *vit;
                            if(CurrentFrame.mvpMapPoints[i2 + CurrentFrame.Nleft])
                                if(CurrentFrame.mvpMapPoints[i2 + CurrentFrame.Nleft]->Observations()>0)
                                    continue;

                            const cv::Mat &d = CurrentFrame.mDescriptors.row(i2 + CurrentFrame.Nleft);

                            const int dist = DescriptorDistance(dMP,d);

                            if(dist<bestDist)
                            {
                                bestDist=dist;
                                bestIdx2=i2;
                            }
                        }

                        if(bestDist<=TH_HIGH)
                        {
                            CurrentFrame.mvpMapPoints[bestIdx2 + CurrentFrame.Nleft]=pMP;
                            nmatches++;
                            if(mbCheckOrientation)
                            {
                                // 右目匹配同样记录方向差。
                                cv::KeyPoint kpLF = (LastFrame.Nleft == -1) ? LastFrame.mvKeysUn[i]
                                                                            : (i < LastFrame.Nleft) ? LastFrame.mvKeys[i]
                                                                                                    : LastFrame.mvKeysRight[i - LastFrame.Nleft];

                                cv::KeyPoint kpCF = CurrentFrame.mvKeysRight[bestIdx2];

                                float rot = kpLF.angle-kpCF.angle;
                                if(rot<0.0)
                                    rot+=360.0f;
                                int bin = round(rot*factor);
                                if(bin==HISTO_LENGTH)
                                    bin=0;
                                assert(bin>=0 && bin<HISTO_LENGTH);
                                rotHist[bin].push_back(bestIdx2  + CurrentFrame.Nleft);
                            }
                        }

                    }
                }
            }
        }

        // 只保留方向直方图中最主要的三个峰。
        if(mbCheckOrientation)
        {
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i!=ind1 && i!=ind2 && i!=ind3)
                {
                    // 所有落在非主方向峰值的匹配都会被撤销。
                    for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                    {
                        CurrentFrame.mvpMapPoints[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                        nmatches--;
                    }
                }
            }
        }

        return nmatches;
    }

    /**
     * @brief              将关键帧中的地图点投影到当前帧并搜索匹配
     * @param CurrentFrame 当前帧
     * @param pKF          参考关键帧
     * @param sAlreadyFound 已经匹配到的地图点集合
     * @param th           搜索半径倍率
     * @param ORBdist      描述子距离阈值
     * @return             匹配成功的数量
     *
     * 用途：重定位阶段。已知一个候选关键帧后，把它的地图点投影到当前帧里寻找支持。
     */
    int ORBmatcher::SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const set<MapPoint*> &sAlreadyFound, const float th , const int ORBdist)
    {
        int nmatches = 0;

        // 当前帧位姿和相机中心。
        const Sophus::SE3f Tcw = CurrentFrame.GetPose();
        Eigen::Vector3f Ow = Tcw.inverse().translation();

        // 方向一致性直方图。
        vector<int> rotHist[HISTO_LENGTH];

        // 预先给每个 bin 分配空间，避免重定位候选较多时频繁扩容。
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f/HISTO_LENGTH;

        // 取出参考关键帧中的地图点。
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        // 遍历参考关键帧的所有地图点，依次尝试在当前帧里为它们找到投影匹配。
        for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMPs[i];

            if(pMP)
            {
                if(!pMP->isBad() && !sAlreadyFound.count(pMP))
                {
                    // 投影到当前帧。
                    Eigen::Vector3f x3Dw = pMP->GetWorldPos();
                    Eigen::Vector3f x3Dc = Tcw * x3Dw;

                    const Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dc);

                    // 投影必须落在图像范围内。
                    if(uv(0)<CurrentFrame.mnMinX || uv(0)>CurrentFrame.mnMaxX)
                        continue;
                    if(uv(1)<CurrentFrame.mnMinY || uv(1)>CurrentFrame.mnMaxY)
                        continue;

                    // 通过相机中心到地图点的距离预测尺度层。
                    Eigen::Vector3f PO = x3Dw-Ow;
                    float dist3D = PO.norm();

                    const float maxDistance = pMP->GetMaxDistanceInvariance();
                    const float minDistance = pMP->GetMinDistanceInvariance();

                    // 地图点必须落在它的稳定观测距离范围内。
                    if(dist3D<minDistance || dist3D>maxDistance)
                        continue;

                    int nPredictedLevel = pMP->PredictScale(dist3D,&CurrentFrame);

                    // 在投影位置附近搜索候选特征。
                    const float radius = th*CurrentFrame.mvScaleFactors[nPredictedLevel];

                    const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, nPredictedLevel-1, nPredictedLevel+1);

                    if(vIndices2.empty())
                        continue;

                    const cv::Mat dMP = pMP->GetDescriptor();

                    int bestDist = 256;
                    int bestIdx2 = -1;

                    // 遍历投影窗口中的全部候选特征，选择描述子最近者。
                    for(vector<size_t>::const_iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
                    {
                        const size_t i2 = *vit;

                        // 当前帧已经绑定地图点的特征不再抢占。
                        if(CurrentFrame.mvpMapPoints[i2])
                            continue;

                        const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                        const int dist = DescriptorDistance(dMP,d);

                        if(dist<bestDist)
                        {
                            bestDist=dist;
                            bestIdx2=i2;
                        }
                    }

                    if(bestDist<=ORBdist)
                    {
                        // 满足距离阈值则接受匹配。
                        CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                        nmatches++;

                        if(mbCheckOrientation)
                        {
                            // 保存旋转差，后续只保留主方向。
                            float rot = pKF->mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdx2);
                        }
                    }

                }
            }
        }

        if(mbCheckOrientation)
        {
            // 方向一致性过滤。
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i!=ind1 && i!=ind2 && i!=ind3)
                {
                    // 非主峰方向上的匹配统一清空。
                    for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                    {
                        CurrentFrame.mvpMapPoints[rotHist[i][j]]=NULL;
                        nmatches--;
                    }
                }
            }
        }

        return nmatches;
    }

    /**
     * @brief      从方向直方图中找出元素数最多的三个 bin
     * @param histo 方向直方图数组
     * @param L     直方图长度
     * @param ind1  最大 bin 索引
     * @param ind2  第二大 bin 索引
     * @param ind3  第三大 bin 索引
     *
     * 该函数用于方向一致性筛选。经验上，真正正确的匹配通常会在某几个相近方向差上形成主峰，
     * 而错误匹配更容易分散在其它 bin 中。
     */
    void ORBmatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
    {
        // 记录前三大 bin 中的元素个数。
        int max1=0;
        int max2=0;
        int max3=0;

        // 单次遍历直方图，维护前三大峰值及其下标。
        for(int i=0; i<L; i++)
        {
            const int s = histo[i].size();

            // 每访问一个 bin，就尝试把它插入“当前前三大”的有序结果里。
            if(s>max1)
            {
                max3=max2;
                max2=max1;
                max1=s;
                ind3=ind2;
                ind2=ind1;
                ind1=i;
            }
            else if(s>max2)
            {
                max3=max2;
                max2=s;
                ind3=ind2;
                ind2=i;
            }
            else if(s>max3)
            {
                max3=s;
                ind3=i;
            }
        }

        // 如果第二大峰远小于第一大峰，则只保留第一峰；
        // 若第三大峰也太小，则只保留前两峰。
        if(max2<0.1f*(float)max1)
        {
            ind2=-1;
            ind3=-1;
        }
        else if(max3<0.1f*(float)max1)
        {
            ind3=-1;
        }
    }


    /**
     * @brief      计算两个 ORB 描述子的汉明距离
     * @param a    描述子 a
     * @param b    描述子 b
     * @return     两个描述子不同 bit 的数量
     *
     * ORB 描述子长度为 32 字节 = 256 bit。
     * 本实现把它们按 8 个 32 位整数读取，先做异或，再用位运算并行统计置位个数，
     * 比逐 bit 统计更高效。
     */
    int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
    {
        // 以 int32_t 指针方式读取，每次处理 32 bit。
        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();

        int dist=0;

        // 256 bit 描述子共分成 8 组 32 bit，该循环每轮统计其中一组的不同 bit 数。
        for(int i=0; i<8; i++, pa++, pb++)
        {
            // 异或后，值为 1 的 bit 表示两个描述子在该位不同。
            unsigned  int v = *pa ^ *pb;

            // 以下是经典的 parallel bit count（并行位计数）写法：
            // 通过几轮位运算，把 32 bit 中的 1 的数量压缩统计出来。
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        // dist 越小，两个 ORB 描述子越相似。
        return dist;
    }

} //namespace ORB_SLAM
