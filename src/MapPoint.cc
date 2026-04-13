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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM3
{

// 全局地图点 ID 计数器。所有 MapPoint 共享。
long unsigned int MapPoint::nNextId=0;
// 用于保护某些涉及跨对象位置写入的全局互斥量。
mutex MapPoint::mGlobalMutex;

/**
 * @brief 默认构造函数
 *
 * 该构造函数主要服务于反序列化等“先创建对象、后补充内容”的场景，
 * 因此这里只初始化最基本的计数器和标志位。
 */
MapPoint::MapPoint():
    mnFirstKFid(0), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL))
{
    mpReplaced = static_cast<MapPoint*>(NULL);
}

/**
 * @brief       根据三维坐标创建地图点
 * @param Pos   地图点世界坐标
 * @param pRefKF 参考关键帧
 * @param pMap  所属地图
 *
 * 这是建图阶段最常见的地图点构造方式。构造时只确定：
 * 1. 三维位置；
 * 2. 初始参考关键帧；
 * 3. 所属地图与唯一 ID。
 *
 * 法向量、代表性描述子、尺度距离范围会在加入观测后进一步更新。
 */
MapPoint::MapPoint(const Eigen::Vector3f &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap),
    mnOriginMapId(pMap->GetId())
{
    // 写入世界坐标。内部会加锁，避免与其他线程中的几何访问冲突。
    SetWorldPos(Pos);

    // 初始时还没有足够观测来估计平均视线方向，因此先置零。
    mNormalVector.setZero();

    // 跟踪线程相关的“是否在视野内”标志先初始化为 false。
    mbTrackInViewR = false;
    mbTrackInView = false;

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    // 从地图级计数器分配唯一 ID。
    mnId=nNextId++;
}

/**
 * @brief         以逆深度参数化形式创建地图点
 * @param invDepth 初始逆深度
 * @param uv_init  初始像素坐标
 * @param pRefKF   参考关键帧
 * @param pHostKF  主机关键帧
 * @param pMap     所属地图
 *
 * 该构造方式通常出现在需要保留逆深度参数进行优化的场景。
 * 与直接给定三维坐标不同，这里暂时只记录参数化信息，世界坐标稍后才能恢复。
 */
MapPoint::MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame* pRefKF, KeyFrame* pHostKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap),
    mnOriginMapId(pMap->GetId())
{
    // 保存逆深度参数化所需的全部初始量。
    mInvDepth=invDepth;
    mInitU=(double)uv_init.x;
    mInitV=(double)uv_init.y;
    mpHostKF = pHostKF;

    // 尚未得到稳定几何时，法向量也无法确定。
    mNormalVector.setZero();

    // Worldpos is not set
    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    // 为该逆深度点分配唯一 ID。
    mnId=nNextId++;
}

/**
 * @brief       根据普通帧中的单个特征创建地图点
 * @param Pos   地图点世界坐标
 * @param pMap  所属地图
 * @param pFrame 产生该点的普通帧
 * @param idxF  特征在帧中的索引
 *
 * 该构造函数主要用于 Tracking 阶段临时生成的 VO 点。它会立即利用当前帧已有的 ORB 信息：
 * 1. 根据观测相机中心初始化平均视线方向；
 * 2. 根据该特征所在金字塔层推断尺度不变距离范围；
 * 3. 直接复制该特征的 ORB 描述子，作为初始代表性描述子。
 */
MapPoint::MapPoint(const Eigen::Vector3f &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap), mnOriginMapId(pMap->GetId())
{
    // 写入三维位置。
    SetWorldPos(Pos);

    Eigen::Vector3f Ow;
    // 若是单目/RGB-D，或者当前索引属于左目特征，则使用主相机中心；
    // 否则使用右目相机中心。这样初始化的视线方向才对应真实观测相机。
    if(pFrame -> Nleft == -1 || idxF < pFrame -> Nleft){
        Ow = pFrame->GetCameraCenter();
    }
    else{
        Eigen::Matrix3f Rwl = pFrame->GetRwc();
        Eigen::Vector3f tlr = pFrame->GetRelativePoseTlr().translation();
        Eigen::Vector3f twl = pFrame->GetOw();

        Ow = Rwl * tlr + twl;
    }
    // 初始平均观测方向就是“相机中心 -> 地图点”的单位向量。
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector / mNormalVector.norm();

    // 当前距离会与 ORB 金字塔层级共同决定该点的尺度不变观测范围。
    Eigen::Vector3f PC = mWorldPos - Ow;
    const float dist = PC.norm();

    // 读取该特征所在的 octave。ORB 特征是在尺度金字塔中提取的，
    // octave 越大，说明该点是在更粗糙的图像层被检测到，通常对应更远的观测距离。
    const int level = (pFrame -> Nleft == -1) ? pFrame->mvKeysUn[idxF].octave
                                              : (idxF < pFrame -> Nleft) ? pFrame->mvKeys[idxF].octave
                                                                         : pFrame -> mvKeysRight[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    // 经验上把“当前距离 * 当前层尺度”记为最大稳定观测距离；
    // 再除以最顶层尺度，得到最小稳定观测距离。
    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    // 直接复制该特征的 ORB 描述子作为初值。
    // ORB 描述子是经过方向补偿的 BRIEF，具有较好的旋转不变性。
    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    // 分配唯一 ID。
    mnId=nNextId++;
}

/**
 * @brief       设置世界坐标
 * @param Pos   新的三维位置
 */
void MapPoint::SetWorldPos(const Eigen::Vector3f &Pos) {
    // 先锁全局，再锁位置，避免多个地图点位置更新时出现交叉死锁风险。
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    mWorldPos = Pos;
}

/**
 * @brief 获取世界坐标
 * @return 当前三维位置
 */
Eigen::Vector3f MapPoint::GetWorldPos() {
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos;
}

/**
 * @brief 获取平均观测方向
 * @return 单位化法向/平均视线方向
 */
Eigen::Vector3f MapPoint::GetNormal() {
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector;
}


/**
 * @brief 获取参考关键帧
 * @return 当前参考关键帧
 */
KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

/**
 * @brief       添加一个关键帧观测
 * @param pKF   观测该点的关键帧
 * @param idx   该点在关键帧中的特征索引
 *
 * `mObservations[pKF]` 保存的是一个二元组：
 * 1. 第一个元素表示左目/单目特征索引；
 * 2. 第二个元素表示右目特征索引；
 * 3. 若某一侧没有观测，则对应位置为 -1。
 */
void MapPoint::AddObservation(KeyFrame* pKF, int idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    tuple<int,int> indexes;

    // 若该关键帧已经观测过当前点，则在原有左右目索引基础上补齐；
    // 否则以 (-1,-1) 开始建立新的观测项。
    if(mObservations.count(pKF)){
        indexes = mObservations[pKF];
    }
    else{
        indexes = tuple<int,int>(-1,-1);
    }

    // 双目鱼眼/双相机模式下，右目特征在 KeyFrame 中的全局索引会排在左目之后；
    // 因此通过 idx 是否越过 NLeft 来判断当前观测属于左目还是右目。
    if(pKF -> NLeft != -1 && idx >= pKF -> NLeft){
        get<1>(indexes) = idx;
    }
    else{
        get<0>(indexes) = idx;
    }

    // 更新该关键帧对应的观测记录。
    mObservations[pKF]=indexes;

    // 统计总观测数 nObs 时，采用的是“有效测量数”而不是关键帧数。
    // 传统双目（mpCamera2 == NULL）中，如果左目特征存在合法右目视差，
    // 这一条观测可提供两次测量，因此记为 2；其余情况记为 1。
    if(!pKF->mpCamera2 && pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
}

/**
 * @brief       删除某个关键帧对该点的观测
 * @param pKF   要删除的关键帧
 *
 * 若删除后剩余观测过少，则当前地图点会被判定为不稳定并进一步置坏。
 */
void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            tuple<int,int> indexes = mObservations[pKF];
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            // 左目/单目索引存在时，先从总观测数中扣掉对应贡献。
            // 对传统双目模型，如果左目特征有合法右目匹配，这一观测原本计为 2。
            if(leftIndex != -1){
                if(!pKF->mpCamera2 && pKF->mvuRight[leftIndex]>=0)
                    nObs-=2;
                else
                    nObs--;
            }
            // 双相机模型中右目会单独作为一个观测计数，因此需要额外减 1。
            if(rightIndex != -1){
                nObs--;
            }

            // 从观测表中删除该关键帧。
            mObservations.erase(pKF);

            // 若删掉的是参考关键帧，则尝试把剩余观测中的某个关键帧作为新参考。
            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            // 经验规则：观测过少的点通常不够稳定，保留意义不大。
            if(nObs<=2)
                bBad=true;
        }
    }

    // 在锁外执行坏点设置，避免嵌套持锁过深。
    if(bBad)
        SetBadFlag();
}


/**
 * @brief 获取当前所有观测
 * @return 关键帧到左右目索引的映射表副本
 */
std::map<KeyFrame*, std::tuple<int,int>>  MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

/**
 * @brief 获取总观测数
 * @return 当前 nObs
 */
int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

/**
 * @brief 将当前地图点标记为坏点
 *
 * 该函数的目的不是立即释放内存，而是把该点从系统的有效关联关系中“摘掉”：
 * 1. 标记 `mbBad = true`；
 * 2. 清空自己的观测表；
 * 3. 让所有观测它的关键帧删除对它的引用；
 * 4. 从所属地图的点集合中移除。
 */
void MapPoint::SetBadFlag()
{
    map<KeyFrame*, tuple<int,int>> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        // 先在自身内部完成状态切换，并把观测表复制出来。
        // 后续对 KeyFrame 的修改放到锁外执行，减少锁持有时间。
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*, tuple<int,int>>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        int leftIndex = get<0>(mit -> second), rightIndex = get<1>(mit -> second);
        // 通知关键帧删除对应索引上的地图点匹配关系。
        if(leftIndex != -1){
            pKF->EraseMapPointMatch(leftIndex);
        }
        if(rightIndex != -1){
            pKF->EraseMapPointMatch(rightIndex);
        }
    }

    // 最后把自己从地图容器中摘除。
    mpMap->EraseMapPoint(this);
}

/**
 * @brief 获取替换当前点的地图点
 * @return 若已被其他点取代，则返回替换者
 */
MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

/**
 * @brief       用另一个地图点替换当前点
 * @param pMP   目标地图点
 *
 * 常用于地图点融合。核心流程是：
 * 1. 把当前点的全部观测关系迁移给目标点；
 * 2. 若目标点在某关键帧里已经存在，则删除重复观测；
 * 3. 累加可见/找到计数；
 * 4. 重新计算目标点的代表性描述子；
 * 5. 将当前点从地图中移除。
 */
void MapPoint::Replace(MapPoint* pMP)
{
    // 自替换没有意义，直接返回。
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,tuple<int,int>> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        // 先把当前点冻结为坏点，并把所有观测搬到局部变量中，
        // 后续再逐个修改相关关键帧与目标地图点。
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<KeyFrame*,tuple<int,int>>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        tuple<int,int> indexes = mit -> second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if(!pMP->IsInKeyFrame(pKF))
        {
            // 目标点尚未被该关键帧观测时，直接把关键帧里的匹配重定向到目标点，
            // 并让目标点接管该观测关系。
            if(leftIndex != -1){
                pKF->ReplaceMapPointMatch(leftIndex, pMP);
                pMP->AddObservation(pKF,leftIndex);
            }
            if(rightIndex != -1){
                pKF->ReplaceMapPointMatch(rightIndex, pMP);
                pMP->AddObservation(pKF,rightIndex);
            }
        }
        else
        {
            // 如果目标点本来就在该关键帧中，则当前点的这份观测属于重复信息，
            // 只需要从关键帧中删除当前点对应的匹配即可，避免同一关键帧挂两个点。
            if(leftIndex != -1){
                pKF->EraseMapPointMatch(leftIndex);
            }
            if(rightIndex != -1){
                pKF->EraseMapPointMatch(rightIndex);
            }
        }
    }
    // 继承当前点累积的统计量，保留目标点的历史稳定性信息。
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    // 合并完观测后，目标点的“中心描述子”可能改变，需要重新计算。
    pMP->ComputeDistinctiveDescriptors();

    // 当前点彻底从地图集合中移除。
    mpMap->EraseMapPoint(this);
}

/**
 * @brief 判断地图点是否为坏点
 * @return 若已失效则返回 true
 */
bool MapPoint::isBad()
{
    unique_lock<mutex> lock1(mMutexFeatures,std::defer_lock);
    unique_lock<mutex> lock2(mMutexPos,std::defer_lock);
    // 同时锁住特征和几何信息，避免读取状态时看到不一致的中间结果。
    lock(lock1, lock2);

    return mbBad;
}

/**
 * @brief       增加可见次数
 * @param n     增量
 */
void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

/**
 * @brief       增加找到次数
 * @param n     增量
 */
void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

/**
 * @brief 获取匹配成功率
 * @return `mnFound / mnVisible`
 */
float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

/**
 * @brief 计算最具代表性的 ORB 描述子
 *
 * ORB 描述子来自 ORB 特征提取器，其核心是“带方向补偿的 BRIEF”：
 * 1. 先用灰度质心法得到关键点方向；
 * 2. 再把 BRIEF 采样模板按该方向旋转；
 * 3. 对若干像素对做灰度比较，得到 256 bit 二值描述子。
 *
 * 同一个物理点会被不同关键帧、不同尺度层、不同视角反复观测，
 * 因而会对应多个略有差异的 ORB 描述子。这里通过两两汉明距离构造距离矩阵，
 * 选择“到其他描述子中值距离最小”的那个，作为当前 MapPoint 的代表性描述子。
 */
void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,tuple<int,int>> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        // 坏点不再需要维护代表性描述子。
        if(mbBad)
            return;
        // 先复制观测表，再在锁外遍历关键帧，减少持锁时间。
        observations=mObservations;
    }

    // 没有任何观测时无法计算描述子。
    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad()){
            tuple<int,int> indexes = mit -> second;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            // 一个关键帧中可能既有左目观测，也有右目观测；
            // 两边各自的描述子都可以参与“中心描述子”的投票。
            if(leftIndex != -1){
                vDescriptors.push_back(pKF->mDescriptors.row(leftIndex));
            }
            if(rightIndex != -1){
                vDescriptors.push_back(pKF->mDescriptors.row(rightIndex));
            }
        }
    }

    // 如果所有观测关键帧都已经失效，则同样无法得到有效描述子。
    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    // Distances[i][j] 表示第 i 个和第 j 个 ORB 描述子的汉明距离。
    // ORB 是二值描述子，因此匹配代价就是不同 bit 的个数。
    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            // ORBmatcher::DescriptorDistance 内部就是针对 ORB 二值串计算汉明距离。
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        // 把第 i 个描述子到所有描述子的距离复制出来并排序。
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());

        // 选中值而不是均值，是为了降低极端离群观测的影响。
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        // clone() 很重要：避免仅保存对 KeyFrame 描述子矩阵某一行的浅引用。
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

/**
 * @brief 获取代表性描述子
 * @return 代表性 ORB 描述子的拷贝
 */
cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

/**
 * @brief       获取地图点在指定关键帧中的左右目索引
 * @param pKF   指定关键帧
 * @return      `(左目索引, 右目索引)`，若不存在则为 `(-1,-1)`
 */
tuple<int,int> MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return tuple<int,int>(-1,-1);
}

/**
 * @brief       判断指定关键帧是否观测到该点
 * @param pKF   指定关键帧
 * @return      若存在观测则返回 true
 */
bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

/**
 * @brief 更新平均观测方向与尺度不变距离
 *
 * 这一步是 MapPoint 与 ORB 金字塔联系最紧的一处：
 * 1. 平均观测方向来自所有相机中心到该点的单位视线平均；
 * 2. 尺度距离范围来自“参考关键帧中观测该点的尺度层”。
 *
 * 直观地说：
 * 如果一个点在参考关键帧中只能在高层金字塔被检测到，说明它在图像中较小，
 * 往往对应更远的观测距离；反之若出现在低层，则通常离相机更近。
 */
void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,tuple<int,int>> observations;
    KeyFrame* pRefKF;
    Eigen::Vector3f Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        // 坏点无需更新；同时复制出当前的观测、参考帧和位置，
        // 便于后续在锁外完成较重的计算。
        if(mbBad)
            return;
        observations = mObservations;
        pRefKF = mpRefKF;
        Pos = mWorldPos;
    }

    // 没有观测无法估计平均视线和尺度范围。
    if(observations.empty())
        return;

    Eigen::Vector3f normal;
    normal.setZero();
    int n=0;
    for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        tuple<int,int> indexes = mit -> second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        // 左目/单目观测：使用主相机中心到地图点的方向。
        if(leftIndex != -1){
            Eigen::Vector3f Owi = pKF->GetCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal = normal + normali / normali.norm();
            n++;
        }
        // 右目观测：使用右相机中心到地图点的方向。
        if(rightIndex != -1){
            Eigen::Vector3f Owi = pKF->GetRightCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal = normal + normali / normali.norm();
            n++;
        }
    }

    // 以参考关键帧左相机中心到地图点的距离作为尺度范围的基准距离。
    Eigen::Vector3f PC = Pos - pRefKF->GetCameraCenter();
    const float dist = PC.norm();

    // 读取参考关键帧中该地图点对应特征的金字塔层级。
    tuple<int ,int> indexes = observations[pRefKF];
    int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    int level;
    if(pRefKF -> NLeft == -1){
        level = pRefKF->mvKeysUn[leftIndex].octave;
    }
    else if(leftIndex != -1){
        level = pRefKF -> mvKeys[leftIndex].octave;
    }
    else{
        level = pRefKF -> mvKeysRight[rightIndex - pRefKF -> NLeft].octave;
    }

    //const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        // 这里沿用 ORB-SLAM 的经典经验模型：
        // 1. 当前距离乘以参考尺度，得到最大稳定观测距离；
        // 2. 再除以金字塔最顶层尺度，得到最小稳定观测距离。
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        // 平均法向 = 所有单位观测方向的均值。
        mNormalVector = normal/n;
    }
}

/**
 * @brief         直接设置法向量
 * @param normal  新的平均观测方向
 */
void MapPoint::SetNormalVector(const Eigen::Vector3f& normal)
{
    unique_lock<mutex> lock3(mMutexPos);
    mNormalVector = normal;
}

/**
 * @brief 获取最小尺度不变距离
 * @return `0.8 * mfMinDistance`
 *
 * 乘以 0.8 是为了给匹配留下少量容差。
 */
float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f * mfMinDistance;
}

/**
 * @brief 获取最大尺度不变距离
 * @return `1.2 * mfMaxDistance`
 *
 * 乘以 1.2 同样是经验容差，使搜索范围略微放宽。
 */
float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f * mfMaxDistance;
}

/**
 * @brief             根据当前距离预测关键帧中的尺度层
 * @param currentDist 当前相机到地图点的距离
 * @param pKF         目标关键帧
 * @return            预测的 ORB 金字塔层级
 *
 * 公式：
 * `nScale = ceil(log(mfMaxDistance / currentDist) / log(scaleFactor))`
 *
 * 含义是把“当前距离相对于参考最大距离的比例”映射回 ORB 金字塔层级。
 * 距离越大，比例越小，预测层通常越高；距离越近，则预测层越低。
 */
int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        // 若当前距离接近 mfMaxDistance，则 ratio 接近 1；
        // 若当前距离更小，则 ratio > 1，对应更低层/更清晰的图像。
        ratio = mfMaxDistance/currentDist;
    }

    // 利用关键帧预存的 log(scaleFactor) 将比例换算成尺度层级。
    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    // 最终把结果限制在合法金字塔层范围内。
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

/**
 * @brief             根据当前距离预测普通帧中的尺度层
 * @param currentDist 当前相机到地图点的距离
 * @param pF          目标普通帧
 * @return            预测的 ORB 金字塔层级
 */
int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        // 与 KeyFrame 版本相同，只是这里使用的是 Frame 的金字塔参数。
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}

/**
 * @brief 打印当前地图点的观测信息
 *
 * 主要用于调试，输出该地图点被哪些关键帧观测到。
 */
void MapPoint::PrintObservations()
{
    cout << "MP_OBS: MP " << mnId << endl;
    for(map<KeyFrame*,tuple<int,int>>::iterator mit=mObservations.begin(), mend=mObservations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKFi = mit->first;
        tuple<int,int> indexes = mit->second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
        // 当前实现没有把左右目索引打印出来，但这里先解析它们，便于调试扩展。
        cout << "--OBS in KF " << pKFi->mnId << " in map " << pKFi->GetMap()->GetId() << endl;
    }
}

/**
 * @brief 获取所属地图
 * @return 当前地图指针
 */
Map* MapPoint::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

/**
 * @brief       更新所属地图
 * @param pMap  新地图
 */
void MapPoint::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

/**
 * @brief             序列化前预处理
 * @param spKF        需要保留的关键帧集合
 * @param spMP        需要保留的地图点集合
 *
 * 目的有两个：
 * 1. 把指针关系转成 ID 关系，便于写盘；
 * 2. 过滤掉不会被一起保存的关联对象，避免加载后出现悬空引用。
 */
void MapPoint::PreSave(set<KeyFrame*>& spKF,set<MapPoint*>& spMP)
{
    // 先备份“被谁替换”的关系，若替换者也会被一起保存，则记录其 ID。
    mBackupReplacedId = -1;
    if(mpReplaced && spMP.find(mpReplaced) != spMP.end())
        mBackupReplacedId = mpReplaced->mnId;

    // 重新构造基于 ID 的观测表备份。
    mBackupObservationsId1.clear();
    mBackupObservationsId2.clear();
    // Save the id and position in each KF who view it
    for(std::map<KeyFrame*,std::tuple<int,int> >::const_iterator it = mObservations.begin(), end = mObservations.end(); it != end; ++it)
    {
        KeyFrame* pKFi = it->first;
        if(spKF.find(pKFi) != spKF.end())
        {
            // 只有会被一起保存的关键帧，其观测关系才有必要写入备份表。
            mBackupObservationsId1[it->first->mnId] = get<0>(it->second);
            mBackupObservationsId2[it->first->mnId] = get<1>(it->second);
        }
        else
        {
            // 若某个关键帧不会被保存，则顺便删除这条观测，避免恢复后留下无效关系。
            EraseObservation(pKFi);
        }
    }

    // Save the id of the reference KF
    if(spKF.find(mpRefKF) != spKF.end())
    {
        // 参考关键帧同样通过 ID 保存。
        mBackupRefKFId = mpRefKF->mnId;
    }
}

/**
 * @brief              反序列化后恢复指针关系
 * @param mpKFid       关键帧 ID 到指针的映射表
 * @param mpMPid       地图点 ID 到指针的映射表
 */
void MapPoint::PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<long unsigned int, MapPoint*>& mpMPid)
{
    // 先恢复参考关键帧。
    mpRefKF = mpKFid[mBackupRefKFId];
    if(!mpRefKF)
    {
        cout << "ERROR: MP without KF reference " << mBackupRefKFId << "; Num obs: " << nObs << endl;
    }
    // 默认没有替换者；若备份中记录了替换关系，则尝试恢复。
    mpReplaced = static_cast<MapPoint*>(NULL);
    if(mBackupReplacedId>=0)
    {
        map<long unsigned int, MapPoint*>::iterator it = mpMPid.find(mBackupReplacedId);
        if (it != mpMPid.end())
            mpReplaced = it->second;
    }

    // 依据备份 ID 表重建“关键帧指针 -> 左右目索引”的观测表。
    mObservations.clear();

    for(map<long unsigned int, int>::const_iterator it = mBackupObservationsId1.begin(), end = mBackupObservationsId1.end(); it != end; ++it)
    {
        KeyFrame* pKFi = mpKFid[it->first];
        map<long unsigned int, int>::const_iterator it2 = mBackupObservationsId2.find(it->first);
        std::tuple<int, int> indexes = tuple<int,int>(it->second,it2->second);
        if(pKFi)
        {
           // 只有在关键帧指针恢复成功时，才建立观测关系。
           mObservations[pKFi] = indexes;
        }
    }

    // 清空备份表，避免长期占用内存，也防止后续误用旧数据。
    mBackupObservationsId1.clear();
    mBackupObservationsId2.clear();
}

} //namespace ORB_SLAM
