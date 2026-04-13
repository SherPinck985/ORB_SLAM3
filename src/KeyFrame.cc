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

#include "KeyFrame.h"
#include "Converter.h"
#include "ImuTypes.h"
#include<mutex>

namespace ORB_SLAM3
{

// 全局关键帧编号计数器。每创建一个新关键帧，id 递增一次。
long unsigned int KeyFrame::nNextId=0;

/**
 * @brief 默认构造函数
 *
 * 该构造函数主要服务于序列化/反序列化流程。
 * 这里把各类成员初始化为空值或默认值，但不构造任何地图关系。
 */
KeyFrame::KeyFrame():
        mnFrameId(0),  mTimeStamp(0), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
        mfGridElementWidthInv(0), mfGridElementHeightInv(0),
        mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
        mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnMergeQuery(0), mnMergeWords(0), mnBAGlobalForKF(0),
        fx(0), fy(0), cx(0), cy(0), invfx(0), invfy(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
        mbf(0), mb(0), mThDepth(0), N(0), mvKeys(static_cast<vector<cv::KeyPoint> >(NULL)), mvKeysUn(static_cast<vector<cv::KeyPoint> >(NULL)),
        mvuRight(static_cast<vector<float> >(NULL)), mvDepth(static_cast<vector<float> >(NULL)), mnScaleLevels(0), mfScaleFactor(0),
        mfLogScaleFactor(0), mvScaleFactors(0), mvLevelSigma2(0), mvInvLevelSigma2(0), mnMinX(0), mnMinY(0), mnMaxX(0),
        mnMaxY(0), mPrevKF(static_cast<KeyFrame*>(NULL)), mNextKF(static_cast<KeyFrame*>(NULL)), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
        mbToBeErased(false), mbBad(false), mHalfBaseline(0), mbCurrentPlaceRecognition(false), mnMergeCorrectedForKF(0),
        NLeft(0),NRight(0), mnNumberOfOpt(0), mbHasVelocity(false)
{

}

/**
 * @brief 由 Frame 构造关键帧
 * @param F      当前帧
 * @param pMap   所属地图
 * @param pKFDB  关键帧数据库
 *
 * 该函数会把 Frame 中已经完成的视觉前端结果固化下来，包括：
 * 1. ORB 关键点、方向、尺度层和描述子；
 * 2. 相机内参与图像边界；
 * 3. 初始位姿、BoW、地图点匹配关系；
 * 4. IMU 预积分、偏置、速度等惯性信息；
 * 5. 特征网格索引。
 *
 * ORB 提取本身发生在 Frame/ORBextractor 中，而不是在 KeyFrame 构造函数里。
 * 关键帧这里只是“拷贝并固化”这些结果，使其成为长期存在的地图结点。
 */
KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    bImu(pMap->isImuInitialized()), mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK_(F.mK_), mPrevKF(NULL), mNextKF(NULL), mpImuPreintegrated(F.mpImuPreintegrated),
    mImuCalib(F.mImuCalib), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mDistCoef(F.mDistCoef), mbNotErase(false), mnDataset(F.mnDataset),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap), mbCurrentPlaceRecognition(false), mNameFile(F.mNameFile), mnMergeCorrectedForKF(0),
    mpCamera(F.mpCamera), mpCamera2(F.mpCamera2),
    mvLeftToRightMatch(F.mvLeftToRightMatch),mvRightToLeftMatch(F.mvRightToLeftMatch), mTlr(F.GetRelativePoseTlr()),
    mvKeysRight(F.mvKeysRight), NLeft(F.Nleft), NRight(F.Nright), mTrl(F.GetRelativePoseTrl()), mnNumberOfOpt(0), mbHasVelocity(false)
{
    // 为当前关键帧分配全局唯一 id。
    mnId=nNextId++;

    // 复制 Frame 中已经构建好的网格索引。
    // 后续通过 GetFeaturesInArea() 做局部特征查询时直接使用它。
    mGrid.resize(mnGridCols);
    if(F.Nleft != -1)  mGridRight.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        if(F.Nleft != -1) mGridRight[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++){
            mGrid[i][j] = F.mGrid[i][j];
            if(F.Nleft != -1){
                mGridRight[i][j] = F.mGridRight[i][j];
            }
        }
    }



    if(!F.HasVelocity()) {
        // 非惯性或当前帧未估出速度时，置零并打上“无速度”标记。
        mVw.setZero();
        mbHasVelocity = false;
    }
    else
    {
        // 若前端已估计出速度，直接拷贝到关键帧中。
        mVw = F.GetVelocity();
        mbHasVelocity = true;
    }

    // 复制当前 IMU 偏置，并设置位姿。
    // SetPose 会同时更新 mTcw / mTwc / mRcw / mRwc 以及 IMU 世界坐标位置。
    mImuBias = F.mImuBias;
    SetPose(F.GetPose());

    // 记录当前关键帧最初所属地图的 id，后续保存/加载和多地图管理会用到。
    mnOriginMapId = pMap->GetId();
}

/**
 * @brief 计算 BoW 表示
 *
 * ORB 特征提取器已经为关键帧保存了 ORB 描述子 `mDescriptors`。
 * 这里进一步把描述子投影到 ORB 词典树上，生成：
 * 1. `mBowVec`：词袋向量，用于整体相似度比较；
 * 2. `mFeatVec`：特征到词典节点的索引，用于快速匹配。
 */
void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        // OpenCV 的描述子矩阵先拆成单个描述子的向量形式，便于词典树处理。
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);

        // 将描述子映射到词典树中。
        // 这里使用第 4 层节点构造特征向量，默认假设词典树共 6 层。
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

/**
 * @brief 设置关键帧位姿
 * @param Tcw 世界到相机的位姿
 */
void KeyFrame::SetPose(const Sophus::SE3f &Tcw)
{
    unique_lock<mutex> lock(mMutexPose);

    // 保存世界到相机的位姿。
    mTcw = Tcw;
    // 同步缓存旋转矩阵，避免后续反复从 Sophus 对象中拆解。
    mRcw = mTcw.rotationMatrix();

    // 同时计算逆位姿和相机到世界旋转矩阵。
    mTwc = mTcw.inverse();
    mRwc = mTwc.rotationMatrix();

    if (mImuCalib.mbIsSet) // TODO Use a flag instead of the OpenCV matrix
    {
        // 若 IMU 外参已知，则由相机位姿和 Tcb 求出 IMU 在世界系下的位置。
        // mTcb 表示相机到 IMU 的外参，因此 IMU 原点在世界系中的位置为：
        // Owb = Rwc * tcb + twc
        mOwb = mRwc * mImuCalib.mTcb.translation() + mTwc.translation();
    }
}

/**
 * @brief 设置关键帧速度
 * @param Vw 世界坐标系下速度
 */
void KeyFrame::SetVelocity(const Eigen::Vector3f &Vw)
{
    unique_lock<mutex> lock(mMutexPose);
    mVw = Vw;
    mbHasVelocity = true;
}

/**
 * @brief 获取世界到相机位姿
 * @return Tcw
 */
Sophus::SE3f KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTcw;
}

/**
 * @brief 获取相机到世界位姿
 * @return Twc
 */
Sophus::SE3f KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTwc;
}

/**
 * @brief 获取相机中心
 * @return 相机中心在世界坐标系下的位置
 */
Eigen::Vector3f KeyFrame::GetCameraCenter(){
    unique_lock<mutex> lock(mMutexPose);
    return mTwc.translation();
}

/**
 * @brief 获取 IMU 位置
 * @return IMU 在世界坐标系下的位置
 */
Eigen::Vector3f KeyFrame::GetImuPosition()
{
    unique_lock<mutex> lock(mMutexPose);
    return mOwb;
}

/**
 * @brief 获取 IMU 旋转
 * @return IMU 在世界系下的旋转矩阵
 */
Eigen::Matrix3f KeyFrame::GetImuRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    // IMU 姿态 = 相机到世界位姿 * 相机到 IMU 外参。
    return (mTwc * mImuCalib.mTcb).rotationMatrix();
}

/**
 * @brief 获取 IMU 位姿
 * @return IMU 在世界系下的完整位姿
 */
Sophus::SE3f KeyFrame::GetImuPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTwc * mImuCalib.mTcb;
}

/**
 * @brief 获取旋转矩阵
 * @return 世界到相机旋转矩阵
 */
Eigen::Matrix3f KeyFrame::GetRotation(){
    unique_lock<mutex> lock(mMutexPose);
    return mRcw;
}

/**
 * @brief 获取平移向量
 * @return 世界到相机位姿中的平移部分
 */
Eigen::Vector3f KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTcw.translation();
}

/**
 * @brief 获取速度
 * @return 世界系速度
 */
Eigen::Vector3f KeyFrame::GetVelocity()
{
    unique_lock<mutex> lock(mMutexPose);
    return mVw;
}

/**
 * @brief 查询速度是否有效
 * @return true 表示关键帧已保存有效速度
 */
bool KeyFrame::isVelocitySet()
{
    unique_lock<mutex> lock(mMutexPose);
    return mbHasVelocity;
}

/**
 * @brief 添加或更新一条共视连接
 * @param pKF    目标关键帧
 * @param weight 共享地图点权重
 */
void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        // 新连接直接插入；旧连接若权重变化则更新；若完全一致则不做任何事。
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

/**
 * @brief 按共视权重重排相连关键帧
 *
 * `mConnectedKeyFrameWeights` 是无序 map。
 * 本函数把它整理成按权重从大到小排序的数组，方便后续快速取最强共视邻居。
 */
void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());

    // 先把 map 转成 pair 数组，pair.first 存权重，pair.second 存关键帧指针。
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    // 按权重升序排序，后面通过 push_front 逆置成降序结果。
    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        // 坏关键帧不再参与共视排序。
        if(!vPairs[i].second->isBad())
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

/**
 * @brief 获取所有连接关键帧
 * @return 连接关键帧集合
 */
set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

/**
 * @brief 获取按权重排序后的共视关键帧
 * @return 共视关键帧数组
 */
vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

/**
 * @brief 获取前 N 个最强共视关键帧
 * @param N 数量上限
 * @return 共视关键帧数组
 */
vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

/**
 * @brief 获取权重大于阈值的共视关键帧
 * @param w 权重阈值
 * @return 满足条件的关键帧列表
 */
vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
    {
        return vector<KeyFrame*>();
    }

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);

    if(it==mvOrderedWeights.end() && mvOrderedWeights.back() < w)
    {
        return vector<KeyFrame*>();
    }
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

/**
 * @brief 获取与指定关键帧的共视权重
 * @param pKF 目标关键帧
 * @return 权重，不存在则返回 0
 */
int KeyFrame::GetWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

/**
 * @brief 统计当前关键帧关联到的地图点数量
 * @return 地图点数量
 */
int KeyFrame::GetNumberMPs()
{
    unique_lock<mutex> lock(mMutexFeatures);
    int numberMPs = 0;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        // 空指针不计入数量。
        if(!mvpMapPoints[i])
            continue;
        numberMPs++;
    }
    return numberMPs;
}

/**
 * @brief 在指定索引处挂接地图点
 * @param pMP 地图点
 * @param idx 特征点索引
 */
void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

/**
 * @brief 按特征点索引删除地图点匹配
 * @param idx 特征点索引
 */
void KeyFrame::EraseMapPointMatch(const int &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

/**
 * @brief 按地图点指针删除与当前关键帧的匹配关系
 * @param pMP 地图点
 */
void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    // 一个地图点在双目关键帧中可能同时被左目和右目观测，因此会返回两个索引。
    tuple<size_t,size_t> indexes = pMP->GetIndexInKeyFrame(this);
    size_t leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    if(leftIndex != -1)
        mvpMapPoints[leftIndex]=static_cast<MapPoint*>(NULL);
    if(rightIndex != -1)
        mvpMapPoints[rightIndex]=static_cast<MapPoint*>(NULL);
}


/**
 * @brief 替换指定索引上的地图点
 * @param idx 特征点索引
 * @param pMP 新地图点
 */
void KeyFrame::ReplaceMapPointMatch(const int &idx, MapPoint* pMP)
{
    mvpMapPoints[idx]=pMP;
}

/**
 * @brief 获取当前关键帧观测到的所有有效地图点
 * @return 地图点集合
 */
set<MapPoint*> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

/**
 * @brief 统计跟踪到的地图点数量
 * @param minObs 最小观测次数阈值
 * @return 满足条件的地图点数
 */
int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    // 若要求最少观测次数，则只统计被足够多关键帧支持的点。
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

/**
 * @brief 获取每个特征点对应的地图点数组
 * @return 地图点数组副本
 */
vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

/**
 * @brief 获取指定索引上的地图点
 * @param idx 特征点索引
 * @return 地图点指针
 */
MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

/**
 * @brief 更新当前关键帧的共视连接
 * @param upParent 是否允许更新父关键帧
 *
 * 该函数是构建共视图的核心：
 * 1. 遍历当前关键帧观测到的所有 MapPoint；
 * 2. 统计这些点还被哪些其他关键帧观测到；
 * 3. 用共同观测数量作为权重建立关键帧连接；
 * 4. 对连接按权重排序；
 * 5. 若这是首次连接，则选择最强共视关键帧作为父结点，接入生成树。
 */
void KeyFrame::UpdateConnections(bool upParent)
{
    map<KeyFrame*,int> KFcounter;

    vector<MapPoint*> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        // 先拷贝一份地图点列表，避免后续长时间持有特征互斥锁。
        vpMP = mvpMapPoints;
    }

    // 遍历当前关键帧中的所有地图点。
    // 对每个点，统计它还被哪些关键帧观测到，从而累积共视权重。
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();

        for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            // 跳过自己、坏关键帧和不在同一张地图中的关键帧。
            if(mit->first->mnId==mnId || mit->first->isBad() || mit->first->GetMap() != mpMap)
                continue;
            KFcounter[mit->first]++;

        }
    }

    // 正常情况下，关键帧至少应与若干其他关键帧共享地图点。
    // 若为空，说明当前关键帧没有有效共视关系，直接返回。
    if(KFcounter.empty())
        return;

    // 共视权重阈值。大于等于该值时，认为值得在共视图中显式连接。
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    if(!upParent)
        cout << "UPDATE_CONN: current KF " << mnId << endl;
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(!upParent)
            cout << "  UPDATE_CONN: KF " << mit->first->mnId << " ; num matches: " << mit->second << endl;

        // 记录权重最大的关键帧。
        // 若没有任何关键帧超过阈值，后面至少还会保留这条最强连接。
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }

        // 只有足够强的共视关系才直接加入共视图，并同步通知对方也连回当前关键帧。
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    if(vPairs.empty())
    {
        // 若没有任何连接超过阈值，仍然强制保留与最强共视关键帧的连接，
        // 否则当前关键帧会在图中孤立。
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    // 对有效连接按权重排序，后续前插后得到降序结果。
    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        // 保存完整的连接权重表。
        mConnectedKeyFrameWeights = KFcounter;
        // 保存按权重降序排列后的关键帧列表与权重列表。
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());


        if(mbFirstConnection && mnId!=mpMap->GetInitKFid())
        {
            // 除初始化关键帧外，每个关键帧第一次建立连接时，
            // 都会选择最强共视关键帧作为生成树父结点。
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }
}

/**
 * @brief 在生成树中添加一个子关键帧
 * @param pKF 子关键帧
 */
void KeyFrame::AddChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

/**
 * @brief 在生成树中删除一个子关键帧
 * @param pKF 子关键帧
 */
void KeyFrame::EraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

/**
 * @brief 修改父关键帧
 * @param pKF 新父关键帧
 */
void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    if(pKF == this)
    {
        cout << "ERROR: Change parent KF, the parent and child are the same KF" << endl;
        throw std::invalid_argument("The parent and child can not be the same");
    }

    // 更新父结点指针，并把自己登记到新父结点的子结点集合中。
    mpParent = pKF;
    pKF->AddChild(this);
}

/**
 * @brief 获取所有子关键帧
 * @return 子关键帧集合
 */
set<KeyFrame*> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

/**
 * @brief 获取父关键帧
 * @return 父关键帧
 */
KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

/**
 * @brief 判断某关键帧是否是当前关键帧的子结点
 * @param pKF 目标关键帧
 * @return true 表示是子结点
 */
bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

/**
 * @brief 设置首次连接标志
 * @param bFirst 标志值
 */
void KeyFrame::SetFirstConnection(bool bFirst)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbFirstConnection=bFirst;
}

/**
 * @brief 添加闭环边
 * @param pKF 闭环关键帧
 */
void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);

    // 一旦参与闭环，通常不希望该关键帧被轻易删除。
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

/**
 * @brief 获取闭环边
 * @return 闭环边集合
 */
set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

/**
 * @brief 添加地图融合边
 * @param pKF 融合相关关键帧
 */
void KeyFrame::AddMergeEdge(KeyFrame* pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);

    // 参与融合边的关键帧也不允许被立即删除。
    mbNotErase = true;
    mspMergeEdges.insert(pKF);
}

/**
 * @brief 获取地图融合边
 * @return 融合边集合
 */
set<KeyFrame*> KeyFrame::GetMergeEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspMergeEdges;
}

/**
 * @brief 设置“不允许删除”标志
 */
void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

/**
 * @brief 清除“不允许删除”标志
 *
 * 若此前已经收到删除请求，则在允许删除后立即真正执行删除。
 */
void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

/**
 * @brief 将关键帧标记为坏关键帧并从图结构中移除
 *
 * 删除过程包含多个步骤：
 * 1. 从所有相连关键帧的共视图中删除自己；
 * 2. 从所有 MapPoint 的观测关系中删除自己；
 * 3. 清空自身连接信息；
 * 4. 重新维护生成树，避免子关键帧失去父结点；
 * 5. 从地图和关键帧数据库中移除自己。
 */
void KeyFrame::SetBadFlag()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==mpMap->GetInitKFid())
        {
            // 初始化关键帧是整张地图的根，不能被删。
            return;
        }
        else if(mbNotErase)
        {
            // 当前关键帧暂时不允许删除，则只记录“待删除”，以后在合适时机再删。
            mbToBeErased = true;
            return;
        }
    }

    // 先通知所有与自己相连的关键帧删除这条共视连接。
    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
    {
        mit->first->EraseConnection(this);
    }

    // 再从所有地图点的观测列表中移除当前关键帧。
    for(size_t i=0; i<mvpMapPoints.size(); i++)
    {
        if(mvpMapPoints[i])
        {
            mvpMapPoints[i]->EraseObservation(this);
        }
    }

    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // 下面开始重建生成树关系。
        // 初始候选父结点集合只包含当前关键帧原本的父结点。
        set<KeyFrame*> sParentCandidates;
        if(mpParent)
            sParentCandidates.insert(mpParent);

        // 迭代地为每个子关键帧寻找新的父结点。
        // 规则是：在“候选父结点集合”中找与某个子关键帧共视权重最大的那一对，
        // 然后先把这一对子关系固定，再把该子关键帧加入候选父结点集合。
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                if(pKF->isBad())
                    continue;

                // 查询该子关键帧的所有共视关键帧，看是否与某个候选父结点有连接。
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                // 选出当前最可靠的“子结点-新父结点”组合并重连。
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // 若还有子关键帧没找到更好的父结点，则直接挂到当前关键帧的原父结点下。
        if(!mspChildrens.empty())
        {
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }
        }

        if(mpParent){
            // 把自己从原父结点的子集合中删掉。
            mpParent->EraseChild(this);

            // 保存当前关键帧相对于父关键帧的位姿。
            // 这个相对位姿在某些后处理或可视化场景下还会用到。
            mTcp = mTcw * mpParent->GetPoseInverse();
        }
        mbBad = true;
    }

    // 最后从地图和关键帧数据库中彻底移除当前关键帧。
    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
}

/**
 * @brief 查询关键帧是否已被标坏
 * @return true 表示坏关键帧
 */
bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

/**
 * @brief 删除一条共视连接
 * @param pKF 待删除连接的关键帧
 */
void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    // 删除连接后，需要重新整理共视关键帧排序结果。
    if(bUpdate)
        UpdateBestCovisibles();
}


/**
 * @brief 在给定区域内查找特征点
 * @param x      查询中心 x
 * @param y      查询中心 y
 * @param r      查询半径
 * @param bRight 是否在右目中查询
 * @return 命中的特征点索引
 *
 * 这里依赖规则网格做粗筛选，再用简单的矩形距离做细筛选。
 * 它是 ORB 匹配加速的重要基础函数。
 */
vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r, const bool bRight) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    // 当前实现直接把查询半径作为 x/y 两个方向的边界。
    float factorX = r;
    float factorY = r;

    // 先把查询圆覆盖到的网格范围算出来。
    const int nMinCellX = max(0,(int)floor((x-mnMinX-factorX)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+factorX)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-factorY)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+factorY)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    // 只遍历命中的少量网格，而不是扫描全图特征点。
    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            // 根据是否查询右目，选择左目或右目的网格。
            const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                // 双目鱼眼时左目和右目关键点分开存；普通情况统一使用 mvKeysUn。
                const cv::KeyPoint &kpUn = (NLeft == -1) ? mvKeysUn[vCell[j]]
                                                         : (!bRight) ? mvKeys[vCell[j]]
                                                                     : mvKeysRight[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                // 最后再检查落在半径 r 的范围内。
                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

/**
 * @brief 判断点是否在图像有效区域内
 * @param x 像素 x
 * @param y 像素 y
 * @return true 表示在图像边界内
 */
bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

/**
 * @brief 将带深度的特征点反投影到世界坐标系
 * @param i   特征点索引
 * @param x3D 输出三维点
 * @return true 表示反投影成功
 *
 * `mvDepth` 与 `mvKeys` / `mvKeysUn` / `mDescriptors` 是按索引一一对应的。
 * 这些 ORB 特征都是前端通过 ORBextractor 提取好的；这里利用其中的像素坐标和深度，
 * 把点从图像坐标恢复为世界坐标系下的三维点。
 */
bool KeyFrame::UnprojectStereo(int i, Eigen::Vector3f &x3D)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        // 用针孔模型把像素坐标反投影到当前相机坐标系。
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        Eigen::Vector3f x3Dc(x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        // 再由相机坐标系变换到世界坐标系。
        x3D = mRwc * x3Dc + mTwc.translation();
        return true;
    }
    else
        // 深度无效时，无法反投影。
        return false;
}

/**
 * @brief 计算场景深度统计量
 * @param q 分位因子
 * @return 对应分位深度
 *
 * 常见用法是 `q=2`，此时返回中值深度。
 * 这在单目初始化和判断基线充分性时很有用。
 */
float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    if(N==0)
        return -1.0;

    vector<MapPoint*> vpMapPoints;
    Eigen::Matrix3f Rcw;
    Eigen::Vector3f tcw;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);

        // 拷贝地图点列表和当前位姿，避免长时间持锁。
        vpMapPoints = mvpMapPoints;
        tcw = mTcw.translation();
        Rcw = mRcw;
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    Eigen::Matrix<float,1,3> Rcw2 = Rcw.row(2);
    float zcw = tcw(2);
    for(int i=0; i<N; i++) {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            Eigen::Vector3f x3Dw = pMP->GetWorldPos();

            // 地图点在当前相机坐标系下的 z 值即为该点深度。
            float z = Rcw2.dot(x3Dw) + zcw;
            vDepths.push_back(z);
        }
    }

    // 排序后返回指定分位位置的深度值。
    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

/**
 * @brief 设置新的 IMU 偏置
 * @param b 新偏置
 */
void KeyFrame::SetNewBias(const IMU::Bias &b)
{
    unique_lock<mutex> lock(mMutexPose);
    mImuBias = b;
    if(mpImuPreintegrated)
        // 当前关键帧若持有从前驱积分到自己的预积分量，也要同步更新偏置。
        mpImuPreintegrated->SetNewBias(b);
}

/**
 * @brief 获取陀螺仪偏置
 * @return 陀螺仪偏置
 */
Eigen::Vector3f KeyFrame::GetGyroBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return Eigen::Vector3f(mImuBias.bwx, mImuBias.bwy, mImuBias.bwz);
}

/**
 * @brief 获取加速度计偏置
 * @return 加速度计偏置
 */
Eigen::Vector3f KeyFrame::GetAccBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return Eigen::Vector3f(mImuBias.bax, mImuBias.bay, mImuBias.baz);
}

/**
 * @brief 获取完整 IMU 偏置
 * @return 偏置结构体
 */
IMU::Bias KeyFrame::GetImuBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return mImuBias;
}

/**
 * @brief 获取当前所属地图
 * @return 地图指针
 */
Map* KeyFrame::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

/**
 * @brief 更新当前所属地图
 * @param pMap 新地图
 */
void KeyFrame::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

/**
 * @brief 序列化前保存所有需要的 id 关系
 * @param spKF  允许保存的关键帧集合
 * @param spMP  允许保存的地图点集合
 * @param spCam 允许保存的相机集合
 *
 * 由于指针无法直接安全地序列化，这里会把关键帧内部所有重要指针关系先转成 id。
 */
void KeyFrame::PreSave(set<KeyFrame*>& spKF,set<MapPoint*>& spMP, set<GeometricCamera*>& spCam)
{
    // 备份每个特征点关联的地图点 id。
    // 若该位置没有地图点，则保存为 -1。
    mvBackupMapPointsId.clear();
    mvBackupMapPointsId.reserve(N);
    for(int i = 0; i < N; ++i)
    {

        if(mvpMapPoints[i] && spMP.find(mvpMapPoints[i]) != spMP.end()) // Checks if the element is not null
            mvBackupMapPointsId.push_back(mvpMapPoints[i]->mnId);
        else // If the element is null his value is -1 because all the id are positives
            mvBackupMapPointsId.push_back(-1);
    }

    // 备份共视连接关系：关键帧 id -> 权重。
    mBackupConnectedKeyFrameIdWeights.clear();
    for(std::map<KeyFrame*,int>::const_iterator it = mConnectedKeyFrameWeights.begin(), end = mConnectedKeyFrameWeights.end(); it != end; ++it)
    {
        if(spKF.find(it->first) != spKF.end())
            mBackupConnectedKeyFrameIdWeights[it->first->mnId] = it->second;
    }

    // 备份父关键帧 id。
    mBackupParentId = -1;
    if(mpParent && spKF.find(mpParent) != spKF.end())
        mBackupParentId = mpParent->mnId;

    // 备份所有子关键帧 id。
    mvBackupChildrensId.clear();
    mvBackupChildrensId.reserve(mspChildrens.size());
    for(KeyFrame* pKFi : mspChildrens)
    {
        if(spKF.find(pKFi) != spKF.end())
            mvBackupChildrensId.push_back(pKFi->mnId);
    }

    // 备份闭环边关键帧 id。
    mvBackupLoopEdgesId.clear();
    mvBackupLoopEdgesId.reserve(mspLoopEdges.size());
    for(KeyFrame* pKFi : mspLoopEdges)
    {
        if(spKF.find(pKFi) != spKF.end())
            mvBackupLoopEdgesId.push_back(pKFi->mnId);
    }

    // 备份地图融合边关键帧 id。
    mvBackupMergeEdgesId.clear();
    mvBackupMergeEdgesId.reserve(mspMergeEdges.size());
    for(KeyFrame* pKFi : mspMergeEdges)
    {
        if(spKF.find(pKFi) != spKF.end())
            mvBackupMergeEdgesId.push_back(pKFi->mnId);
    }

    // 备份主相机与副相机 id。
    mnBackupIdCamera = -1;
    if(mpCamera && spCam.find(mpCamera) != spCam.end())
        mnBackupIdCamera = mpCamera->GetId();

    mnBackupIdCamera2 = -1;
    if(mpCamera2 && spCam.find(mpCamera2) != spCam.end())
        mnBackupIdCamera2 = mpCamera2->GetId();

    // 备份 IMU 时间链与预积分。
    mBackupPrevKFId = -1;
    if(mPrevKF && spKF.find(mPrevKF) != spKF.end())
        mBackupPrevKFId = mPrevKF->mnId;

    mBackupNextKFId = -1;
    if(mNextKF && spKF.find(mNextKF) != spKF.end())
        mBackupNextKFId = mNextKF->mnId;

    if(mpImuPreintegrated)
        mBackupImuPreintegrated.CopyFrom(mpImuPreintegrated);
}

/**
 * @brief 反序列化后恢复运行时指针关系
 * @param mpKFid  关键帧 id -> 指针
 * @param mpMPid  地图点 id -> 指针
 * @param mpCamId 相机 id -> 指针
 */
void KeyFrame::PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<long unsigned int, MapPoint*>& mpMPid, map<unsigned int, GeometricCamera*>& mpCamId){
    // 反序列化出来后，许多只适合运行时保存的指针关系还没恢复。
    // 下面逐项根据备份的 id 把它们重建回来。

    // 重新设置位姿，使缓存旋转矩阵和 IMU 位置同步更新。
    SetPose(mTcw);

    // 恢复 Trl。
    mTrl = mTlr.inverse();

    // 恢复每个特征点对应的地图点指针。
    mvpMapPoints.clear();
    mvpMapPoints.resize(N);
    for(int i=0; i<N; ++i)
    {
        if(mvBackupMapPointsId[i] != -1)
            mvpMapPoints[i] = mpMPid[mvBackupMapPointsId[i]];
        else
            mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
    }

    // 恢复共视图连接。
    mConnectedKeyFrameWeights.clear();
    for(map<long unsigned int, int>::const_iterator it = mBackupConnectedKeyFrameIdWeights.begin(), end = mBackupConnectedKeyFrameIdWeights.end();
        it != end; ++it)
    {
        KeyFrame* pKFi = mpKFid[it->first];
        mConnectedKeyFrameWeights[pKFi] = it->second;
    }

    // 恢复父关键帧。
    if(mBackupParentId>=0)
        mpParent = mpKFid[mBackupParentId];

    // 恢复子关键帧集合。
    mspChildrens.clear();
    for(vector<long unsigned int>::const_iterator it = mvBackupChildrensId.begin(), end = mvBackupChildrensId.end(); it!=end; ++it)
    {
        mspChildrens.insert(mpKFid[*it]);
    }

    // 恢复闭环边。
    mspLoopEdges.clear();
    for(vector<long unsigned int>::const_iterator it = mvBackupLoopEdgesId.begin(), end = mvBackupLoopEdgesId.end(); it != end; ++it)
    {
        mspLoopEdges.insert(mpKFid[*it]);
    }

    // 恢复地图融合边。
    mspMergeEdges.clear();
    for(vector<long unsigned int>::const_iterator it = mvBackupMergeEdgesId.begin(), end = mvBackupMergeEdgesId.end(); it != end; ++it)
    {
        mspMergeEdges.insert(mpKFid[*it]);
    }

    // 恢复主相机和副相机指针。
    if(mnBackupIdCamera >= 0)
    {
        mpCamera = mpCamId[mnBackupIdCamera];
    }
    else
    {
        cout << "ERROR: There is not a main camera in KF " << mnId << endl;
    }
    if(mnBackupIdCamera2 >= 0)
    {
        mpCamera2 = mpCamId[mnBackupIdCamera2];
    }

    // 恢复惯性时间链和预积分数据。
    if(mBackupPrevKFId != -1)
    {
        mPrevKF = mpKFid[mBackupPrevKFId];
    }
    if(mBackupNextKFId != -1)
    {
        mNextKF = mpKFid[mBackupNextKFId];
    }
    mpImuPreintegrated = &mBackupImuPreintegrated;


    // 备份容器只在加载恢复时使用，恢复后立即清空。
    mvBackupMapPointsId.clear();
    mBackupConnectedKeyFrameIdWeights.clear();
    mvBackupChildrensId.clear();
    mvBackupLoopEdgesId.clear();

    // 共视图权重 map 已恢复，但排序缓存还需要重建。
    UpdateBestCovisibles();
}

/**
 * @brief 将地图点投影到带畸变图像平面
 * @param pMP 地图点
 * @param kp  输出像素点
 * @param u   输出像素 x
 * @param v   输出像素 y
 * @return true 表示投影成功
 */
bool KeyFrame::ProjectPointDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v)
{

    // 先取地图点在世界坐标系中的位置。
    Eigen::Vector3f P = pMP->GetWorldPos();

    // 变换到当前相机坐标系。
    Eigen::Vector3f Pc = mRcw * P + mTcw.translation();
    float &PcX = Pc(0);
    float &PcY = Pc(1);
    float &PcZ = Pc(2);

    // 只接受位于相机前方的点。
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // 先用理想针孔模型投影。
    float invz = 1.0f/PcZ;
    u=fx*PcX*invz+cx;
    v=fy*PcY*invz+cy;

    // 粗略检查投影点是否落在图像边界内。
    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // 归一化坐标。
    float x = (u - cx) * invfx;
    float y = (v - cy) * invfy;
    float r2 = x * x + y * y;
    float k1 = mDistCoef.at<float>(0);
    float k2 = mDistCoef.at<float>(1);
    float p1 = mDistCoef.at<float>(2);
    float p2 = mDistCoef.at<float>(3);
    float k3 = 0;
    if(mDistCoef.total() == 5)
    {
        k3 = mDistCoef.at<float>(4);
    }

    // 径向畸变。
    float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    // 切向畸变。
    x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
    y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

    // 再乘回像素尺度，得到最终带畸变像素坐标。
    float u_distort = x_distort * fx + cx;
    float v_distort = y_distort * fy + cy;

    u = u_distort;
    v = v_distort;

    kp = cv::Point2f(u, v);

    return true;
}

/**
 * @brief 将地图点投影到去畸变图像平面
 * @param pMP 地图点
 * @param kp  输出像素点
 * @param u   输出像素 x
 * @param v   输出像素 y
 * @return true 表示投影成功
 */
bool KeyFrame::ProjectPointUnDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v)
{

    // 世界坐标系下的 3D 点。
    Eigen::Vector3f P = pMP->GetWorldPos();

    // 转到当前相机坐标系。
    Eigen::Vector3f Pc = mRcw * P + mTcw.translation();
    float &PcX = Pc(0);
    float &PcY= Pc(1);
    float &PcZ = Pc(2);

    // 深度必须为正。
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // 直接按针孔模型投影到去畸变平面。
    const float invz = 1.0f/PcZ;
    u = fx * PcX * invz + cx;
    v = fy * PcY * invz + cy;

    // 检查是否落在图像有效区域中。
    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    kp = cv::Point2f(u, v);

    return true;
}

/**
 * @brief 获取右到左的相对位姿
 * @return Trl
 */
Sophus::SE3f KeyFrame::GetRelativePoseTrl()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTrl;
}

/**
 * @brief 获取左到右的相对位姿
 * @return Tlr
 */
Sophus::SE3f KeyFrame::GetRelativePoseTlr()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTlr;
}

/**
 * @brief 获取右相机位姿
 * @return 右相机的 Tcw
 */
Sophus::SE3<float> KeyFrame::GetRightPose() {
    unique_lock<mutex> lock(mMutexPose);

    // 右相机位姿 = 右相机相对左相机位姿 * 左相机位姿。
    return mTrl * mTcw;
}

/**
 * @brief 获取右相机逆位姿
 * @return 右相机的 Twc
 */
Sophus::SE3<float> KeyFrame::GetRightPoseInverse() {
    unique_lock<mutex> lock(mMutexPose);

    // 右相机到世界 = 左相机到世界 * 左到右相机相对位姿。
    return mTwc * mTlr;
}

/**
 * @brief 获取右相机中心
 * @return 右相机中心在世界坐标系下的位置
 */
Eigen::Vector3f KeyFrame::GetRightCameraCenter() {
    unique_lock<mutex> lock(mMutexPose);

    return (mTwc * mTlr).translation();
}

/**
 * @brief 获取右相机旋转矩阵
 * @return 世界到右相机的旋转矩阵
 */
Eigen::Matrix<float,3,3> KeyFrame::GetRightRotation() {
    unique_lock<mutex> lock(mMutexPose);

    return (mTrl.so3() * mTcw.so3()).matrix();
}

/**
 * @brief 获取右相机平移
 * @return 世界到右相机位姿中的平移部分
 */
Eigen::Vector3f KeyFrame::GetRightTranslation() {
    unique_lock<mutex> lock(mMutexPose);
    return (mTrl * mTcw).translation();
}

/**
 * @brief 设置 ORB 词典
 * @param pORBVoc ORB 词典
 */
void KeyFrame::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBvocabulary = pORBVoc;
}

/**
 * @brief 设置关键帧数据库
 * @param pKFDB 关键帧数据库
 */
void KeyFrame::SetKeyFrameDatabase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

} //namespace ORB_SLAM
