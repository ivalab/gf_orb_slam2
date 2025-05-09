/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "slam_utility/io.h"
#include <thread>

namespace ORB_SLAM2
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
      mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
      mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
      mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
      mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
      mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
      mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
      mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
      mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
      mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
      mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}


Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
             ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K,
             cv::Mat &K_left, cv::Mat &distCoef_left, cv::Mat &R_left, cv::Mat &P_left,
             cv::Mat &K_right, cv::Mat &distCoef_right, cv::Mat &R_right, cv::Mat &P_right,
             const float &bf, const float &thDepth)
    :mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),
      mK_ori(K_left.clone()), mDistCoef(distCoef_left.clone()), mR(R_left.clone()), mP(P_left.clone()),
      mK_right(K_right.clone()), mDistCoef_right(distCoef_right.clone()), mR_right(R_right.clone()), mP_right(P_right.clone()),
      mbf(bf), mThDepth(thDepth), mpORBvocabulary(voc), mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    // std::thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    // std::thread threadRight(&Frame::ExtractORB,this,1,imRight);
    // threadLeft.join();
    // threadRight.join();
    if (true) {
        thread threadLeft(&ORBextractor::ComputePyramid,
                          this->mpORBextractorLeft, imLeft);
        thread threadRight(&ORBextractor::ComputePyramid,
                           this->mpORBextractorRight, imRight);
        threadLeft.join();
        threadRight.join();
        // load feature points
        bool flag = FeatureIO::loadFeaturePoints(
            mTimeStamp, mvKeys, mDescriptors, mvKeysRight, mDescriptorsRight);
        std::cout << "load feature" << std::endl;
        if (!flag) {
            std::cout << "Could not load feature for frame: "
                      << std::setprecision(20) << mTimeStamp << "\n";
            return;
        }
    } else {
        std::thread threadLeft(&Frame::ExtractORB, this, 0, imLeft);
        std::thread threadRight(&Frame::ExtractORB, this, 1, imRight);
        threadLeft.join();
        threadRight.join();
        // save feature points
        FeatureIO::saveFeaturePoints(mTimeStamp, mvKeys, mDescriptors,
                                     mvKeysRight, mDescriptorsRight);
    }

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    //    cout << "number of keypoints detected: left = " << mvKeys.size() << "; right = " << mvKeysRight.size() << endl;

#ifdef ALTER_STEREO_MATCHING
    //
    UndistortKeyPointsStereo();

#ifndef DELAYED_STEREO_MATCHING
    ComputeStereoMatches_Undistorted(false);
#endif

#else

    //    UndistortKeyPoints();
    mvKeysUn=mvKeys;
    ComputeStereoMatches();

#endif

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

    mvbOutlier = vector<bool>(N,false);
    mvbCandidate = vector<bool>(N,true);
  //  mvbJacobBuilt = vector<bool>(N,false);
    mvbGoodFeature = vector<bool>(N,false);
    //
    mvpMatchScore = vector<int>(N,static_cast<int>(999));

    mvStereoMatched = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
#ifdef ALTER_STEREO_MATCHING
        ComputeImageBoundsStereo(imLeft);
#else
        ComputeImageBounds(imLeft);
#endif

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }
    //    cout << "K = " << K << endl;

    mb = mbf/fx;
    //    cout << "-------- mbf = " << mbf << "; fx = " << fx << "; mb = " << mb << endl;

    AssignFeaturesToGrid();
}

Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
             ORBextractor *extractorLeft, ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K,
             cv::Mat &K_left, cv::Mat &distCoef_left, cv::Mat &R_left, cv::Mat &P_left,
             cv::Mat &K_right, cv::Mat &distCoef_right, cv::Mat &R_right, cv::Mat &P_right,
             const float &bf, const float &thDepth,
             const std::vector<cv::KeyPoint> &vKeys_left, const std::vector<cv::KeyPoint> &vKeys_right,
             const cv::Mat &mDisc_left, const cv::Mat &mDisc_right)
    : mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),
      mK_ori(K_left.clone()), mDistCoef(distCoef_left.clone()), mR(R_left.clone()), mP(P_left.clone()),
      mK_right(K_right.clone()), mDistCoef_right(distCoef_right.clone()), mR_right(R_right.clone()), mP_right(P_right.clone()),
      mbf(bf), mThDepth(thDepth), mpORBvocabulary(voc), mpReferenceKF(static_cast<KeyFrame *>(NULL)),
      mvKeys(vKeys_left), mvKeysRight(vKeys_right), mDescriptors(mDisc_left), mDescriptorsRight(mDisc_right)
{
    // Frame ID
    mnId = nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    //    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    //    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    //    threadLeft.join();
    //    threadRight.join();

    // NOTE
    // only build pyramid, while other detection results are directly copied from input parameters
    thread threadLeft(&ORBextractor::ComputePyramid, this->mpORBextractorLeft, imLeft);
    thread threadRight(&ORBextractor::ComputePyramid, this->mpORBextractorRight, imRight);
    threadLeft.join();
    threadRight.join();

    N = mvKeys.size();

    if (mvKeys.empty())
        return;

    //    cout << "number of keypoints detected: left = " << mvKeys.size() << "; right = " << mvKeysRight.size() << endl;

#ifdef ALTER_STEREO_MATCHING
    //
    UndistortKeyPointsStereo();

#ifndef DELAYED_STEREO_MATCHING
    ComputeStereoMatches_Undistorted(false);
#endif

#else

    //    UndistortKeyPoints();
    mvKeysUn = mvKeys;
    ComputeStereoMatches();

#endif

    mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));

    mvbOutlier = vector<bool>(N, false);
    mvbCandidate = vector<bool>(N, true);
   // mvbJacobBuilt = vector<bool>(N, false);
    mvbGoodFeature = vector<bool>(N, false);
    //
    mvpMatchScore = vector<int>(N, static_cast<int>(999));

    mvStereoMatched = vector<bool>(N, false);

    // This is done only for the first Frame (or after a change in the calibration)
    if (mbInitialComputations)
    {
#ifdef ALTER_STEREO_MATCHING
        ComputeImageBoundsStereo(imLeft);
#else
        ComputeImageBounds(imLeft);
#endif

        mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
        mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

        fx = K.at<float>(0, 0);
        fy = K.at<float>(1, 1);
        cx = K.at<float>(0, 2);
        cy = K.at<float>(1, 2);
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;

        mbInitialComputations = false;
    }
    //    cout << "K = " << K << endl;

    mb = mbf / fx;
    //    cout << "-------- mbf = " << mbf << "; fx = " << fx << "; mb = " << mb << endl;

    AssignFeaturesToGrid();
}


Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
             ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K,
             cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),
      mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
      mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    threadLeft.join();
    threadRight.join();

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    //    cout << "number of keypoints detected: left = " << mvKeys.size() << "; right = " << mvKeysRight.size() << endl;

    UndistortKeyPoints();

    ComputeStereoMatches();
    int stereo_count = 0;
    for (int i=0; i<mvDepth.size(); ++i) {
        if (fabs(mvDepth[i] + 1) > 0.0001) {
            //            cout << mvDepth[i] << ", ";
            stereo_count ++;
        }
    }
    //    cout << endl;
    std::cout << "Frame: number of stereo matchings = " << stereo_count << "......................" << std::endl;

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

    mvbOutlier = vector<bool>(N,false);
    mvbCandidate = vector<bool>(N,true);
   // mvbJacobBuilt = vector<bool>(N,false);
    mvbGoodFeature = vector<bool>(N,false);
    //
    mvpMatchScore = vector<int>(N,static_cast<int>(999));

    mvStereoMatched = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();

}


Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp,
             ORBextractor* extractor, ORBVocabulary* voc, cv::Mat &K,
             cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
      mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

    mvbOutlier = vector<bool>(N,false);
    mvbCandidate = vector<bool>(N,true);
  //  mvbJacobBuilt = vector<bool>(N,false);
    mvbGoodFeature = vector<bool>(N,false);
    //
    mvpMatchScore = vector<int>(N,static_cast<int>(999));

    mvStereoMatched = vector<bool>(N,false);


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}


Frame::Frame(const cv::Mat &imGray, const double &timeStamp,
             ORBextractor* extractor, ORBVocabulary* voc, cv::Mat &K,
             cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
      mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);
    mvbCandidate = vector<bool>(N,true);
  //  mvbJacobBuilt = vector<bool>(N,false);
    mvbGoodFeature = vector<bool>(N,false);
    //
    mvpMatchScore = vector<int>(N,static_cast<int>(999));

    mvStereoMatched = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}


void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

//
cv::Mat Frame::getTwc() {
    //
    cv::Mat Rwc = mTcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*mTcw.rowRange(0,3).col(3);
    //
    cv::Mat Twc = cv::Mat::eye(4,4,CV_32F);

    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    twc.copyTo(Twc.rowRange(0,3).col(3));

    return Twc;
}

void Frame::ExportToYML(cv::FileStorage &fs)
{

    if (!fs.isOpened())
        return;

    write(fs, "nNextId", double(nNextId));
    write(fs, "mnId", double(mnId));
    write(fs, "mTimeStamp", mTimeStamp);
    write(fs, "mK", mK);

    // only export detection results from right frame
    write(fs, "mvKeys", mvKeys);
    write(fs, "mDescriptors", mDescriptors);
    // export detection results from both frames
    write(fs, "mvKeysRight", mvKeysRight);
    write(fs, "mDescriptorsRight", mDescriptorsRight);
    //    write(fs, "mvuRight", mvuRight);
    //    write(fs, "mvDepth", mvDepth);
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY = Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

    // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void Frame::UndistortKeyPointsStereo()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        mvKeysRightUn=mvKeysRight;
        return;
    }

    //    cout << "mK = " << mK << endl;
    //    cout << "mDistCoef = " << mDistCoef << endl;
    //    cout << "mR = " << mR << endl;
    //    cout << "mP = " << mP << endl;

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }
    // Undistort points
    mat=mat.reshape(2);
#ifdef USE_FISHEYE_DISTORTION
    cv::fisheye::undistortPoints(mat,mat,mK_ori,mDistCoef,mR,mP);
#else
    cv::undistortPoints(mat,mat,mK_ori,mDistCoef,mR,mP);
#endif
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }

    //    cout << "finish undistort " << N << " left keypoints!" << endl;

    /* Right */
    //
    // Fill matrix with points
    cv::Mat matR(mvKeysRight.size(),2,CV_32F);
    for(int i=0; i<mvKeysRight.size(); i++)
    {
        matR.at<float>(i,0)=mvKeysRight[i].pt.x;
        matR.at<float>(i,1)=mvKeysRight[i].pt.y;
    }
    // Undistort points
    matR=matR.reshape(2);
#ifdef USE_FISHEYE_DISTORTION
    cv::fisheye::undistortPoints(matR,matR,mK_right,mDistCoef_right,mR_right,mP_right);
#else
    cv::undistortPoints(matR,matR,mK_right,mDistCoef_right,mR_right,mP_right);
#endif
    matR=matR.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysRightUn.resize(mvKeysRight.size());
    for(int i=0; i<mvKeysRight.size(); i++)
    {
        cv::KeyPoint kp = mvKeysRight[i];
        kp.pt.x=matR.at<float>(i,0);
        kp.pt.y=matR.at<float>(i,1);
        mvKeysRightUn[i]=kp;
    }

    //    cout << "finish undistort " << mvKeysRight.size() << " right keypoints!" << endl;
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
#ifdef USE_FISHEYE_DISTORTION
    cv::fisheye::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
#else
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
#endif
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        //        cv::Mat mat(4,2,CV_32F);
        //        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        //        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        //        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        //        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        cv::Mat mat(8,2,CV_32F);
        mat.at<float>(0,0)=0.0;
        mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols;
        mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0;
        mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols;
        mat.at<float>(3,1)=imLeft.rows;
        //
        mat.at<float>(4,0)=0.0;
        mat.at<float>(4,1)=float(imLeft.rows) / 2.0;
        mat.at<float>(5,0)=float(imLeft.cols) / 2.0;
        mat.at<float>(5,1)=0.0;
        mat.at<float>(6,0)=float(imLeft.cols) / 2.0;
        mat.at<float>(6,1)=imLeft.rows;
        mat.at<float>(7,0)=imLeft.cols;
        mat.at<float>(7,1)=float(imLeft.rows) / 2.0;

        // Undistort corners
        mat=mat.reshape(2);
#ifdef USE_FISHEYE_DISTORTION
        cv::fisheye::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
#else
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
#endif
        mat=mat.reshape(1);

        //        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        //        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        //        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        //        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

        mnMinX = INT_MAX;
        mnMaxX = INT_MIN;
        mnMinY = INT_MAX;
        mnMaxY = INT_MIN;
        //
        for (int i=0; i<mat.rows; ++i) {
            if (mnMinX > mat.at<float>(i,0))
                mnMinX = floor( mat.at<float>(i,0) );
            if (mnMinY > mat.at<float>(i,1))
                mnMinY = floor( mat.at<float>(i,1) );
            //
            if (mnMaxX < mat.at<float>(i,0))
                mnMaxX = ceil( mat.at<float>(i,0) );
            if (mnMaxY < mat.at<float>(i,1))
                mnMaxY = ceil( mat.at<float>(i,1) );
        }

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeImageBoundsStereo(const cv::Mat &imLeft) {

    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(8,2,CV_32F);
        mat.at<float>(0,0)=0.0;
        mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols;
        mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0;
        mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols;
        mat.at<float>(3,1)=imLeft.rows;
        //
        mat.at<float>(4,0)=0.0;
        mat.at<float>(4,1)=float(imLeft.rows) / 2.0;
        mat.at<float>(5,0)=float(imLeft.cols) / 2.0;
        mat.at<float>(5,1)=0.0;
        mat.at<float>(6,0)=float(imLeft.cols) / 2.0;
        mat.at<float>(6,1)=imLeft.rows;
        mat.at<float>(7,0)=imLeft.cols;
        mat.at<float>(7,1)=float(imLeft.rows) / 2.0;

        // Undistort corners
        mat=mat.reshape(2);
#ifdef USE_FISHEYE_DISTORTION
        cv::fisheye::undistortPoints(mat,mat,mK_ori,mDistCoef,mR,mP);
#else
        cv::undistortPoints(mat,mat,mK_ori,mDistCoef,mR,mP);
#endif
        mat=mat.reshape(1);

        mnMinX = INT_MAX;
        mnMaxX = INT_MIN;
        mnMinY = INT_MAX;
        mnMaxY = INT_MIN;
        //
        for (int i=0; i<mat.rows; ++i) {
            if (mnMinX > mat.at<float>(i,0))
                mnMinX = floor( mat.at<float>(i,0) );
            if (mnMinY > mat.at<float>(i,1))
                mnMinY = floor( mat.at<float>(i,1) );
            //
            if (mnMaxX < mat.at<float>(i,0))
                mnMaxX = ceil( mat.at<float>(i,0) );
            if (mnMaxY < mat.at<float>(i,1))
                mnMaxY = ceil( mat.at<float>(i,1) );
        }

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        //        const float r = 10.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {

            //            cout << bestDist << "; ";

            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
#ifdef CUDA_ACC_FAST
            cv::cuda::GpuMat gMat = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduL - w, scaleduL + w + 1);
            cv::Mat IL(gMat.rows, gMat.cols, gMat.type(), gMat.data, gMat.step);
#else
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
#endif
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
#ifdef CUDA_ACC_FAST
                cv::cuda::GpuMat gMat = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);
                cv::Mat IR(gMat.rows, gMat.cols, gMat.type(), gMat.data, gMat.step);
#else
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
#endif
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL] = mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }
    //    cout << endl;

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}

bool Frame::ComputeStereoMatch_OnePoint(const int iL, const int thOrbDist, const float minD, const float maxD) {

    if (iL < 0 || iL >= this->mvpMapPoints.size())
        return false;

    // only stereo matching points with associated 3D map points
    if (this->mvpMapPoints[iL] == NULL || this->mvStereoMatched[iL] == true) {
        return false;
    }

    this->mvStereoMatched[iL] = true;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    const cv::KeyPoint &kpL = mvKeysUn[iL];
    const int &levelL = kpL.octave;
    const float &vL = kpL.pt.y;
    const float &uL = kpL.pt.x;

    if (vL < 0 || vL > nRows-1)
        return false;

    const vector<size_t> &vCandidates = mvRowIndices[int(vL)];

    if(vCandidates.empty())
        return false;

    const float minU = uL-maxD;
    const float maxU = uL-minD;

    if(maxU<0)
        return false;

    int bestDist = ORBmatcher::TH_HIGH;
    size_t bestIdxR = 0;

    const cv::Mat &dL = mDescriptors.row(iL);

    // Compare descriptor to right keypoints
    for(size_t iC=0; iC<vCandidates.size(); iC++)
    {
        const size_t iR = vCandidates[iC];
        const cv::KeyPoint &kpR = mvKeysRightUn[iR];

        if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
            continue;

        const float &uR = kpR.pt.x;

        if(uR>=minU && uR<=maxU)
        {
            const cv::Mat &dR = mDescriptorsRight.row(iR);
            const int dist = ORBmatcher::DescriptorDistance(dL,dR);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdxR = iR;
            }
        }
    }

    // Subpixel match by correlation
    if(bestDist<thOrbDist)
    {
        float bestuR = mvKeysRightUn[bestIdxR].pt.x;
        float disparity = (uL-bestuR);
        if(disparity>=minD && disparity<maxD)
        {
            if(disparity<=0)
            {
                disparity=0.01;
                bestuR = uL-0.01;
            }
            mvDepth[iL] = mbf/disparity;
            mvuRight[iL] = bestuR;
            mvDistIdx.push_back(pair<int,int>(bestDist,iL));

            return true;
        }

    }

    return false;
}


int Frame::ComputeStereoMatches_Undistorted(bool isOnline)
{
    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    if (mvRowIndices.size() != nRows) {
        PrepareStereoCandidates();
        std::cout << "redo the stereo candidate preparation!" << std::endl;
    }

    // Set limits for search
    const float minZ = mb;

    int nmatched = 0;

    for(int iL=0; iL<N; iL++)
    {

#ifdef DELAYED_STEREO_MATCHING
        if (isOnline) {
            // only stereo matching points with associated 3D map points
            if (this->mvpMapPoints[iL] == NULL || this->mvStereoMatched[iL] == true) {
                continue ;
            }
        }
        else {
            // stereo matching the rest of points
            if (this->mvStereoMatched[iL] == true) {
                continue ;
            }
        }
#endif

        float minD = 0;
        float maxD = mbf/minZ;
        this->mvStereoMatched[iL] = true;

        const cv::KeyPoint &kpL = mvKeysUn[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        if (vL < 0 || vL > nRows-1)
            continue;

        const vector<size_t> &vCandidates = mvRowIndices[int(vL)];

        if(vCandidates.empty())
            continue;

        //        cout << "Before - minD: " << minD << " maxD: " << maxD << ";   ";
        // reduce the range of stereo matching for map-matched points
        if (this->mvpMapPoints[iL] != NULL) {
            MapPoint * pMP = this->mvpMapPoints[iL];
            if (!pMP->isBad()) {
                cv::Mat Pw = pMP->GetWorldPos(), Pc;
                if (WorldToCameraPoint(Pw, Pc) == true) {
                    //
                    float disp = float(mbf) / Pc.at<float>(2);
                    minD = std::max(disp - float(DISPARITY_THRES), 0.0f);
                    maxD = std::min(disp + float(DISPARITY_THRES), float(mbf)/float(mb));
                }
            }
        }
        //        cout << "After - minD: " << minD << " maxD: " << maxD << endl;
        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<mnMinX)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRightUn[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            float bestuR = mvKeysRightUn[bestIdxR].pt.x;
            float disparity = (uL-bestuR);
            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                mvDistIdx.push_back(pair<int,int>(bestDist,iL));
            }

        }

        nmatched ++;
    }

    if (!isOnline) {

        //
        if (mvDistIdx.empty())
            return nmatched;

        sort(mvDistIdx.begin(),mvDistIdx.end());
        const float median = mvDistIdx[mvDistIdx.size()/2].first;
        const float thDist = 1.5f*1.4f*median;
        // reduce the number of stereo matchings being dropped
        //        const float thDist = 3.0f*median;

        for(int i=mvDistIdx.size()-1;i>=0;i--)
        {
            if(mvDistIdx[i].first<thDist)
                break;
            else
            {
                mvuRight[mvDistIdx[i].second]=-1;
                mvDepth[mvDistIdx[i].second]=-1;
                nmatched --;
            }
        }
    }

    return nmatched;
}

int Frame::ComputeStereoMatches_Undistorted_ByBucketing(bool isOnline)
{
    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    if (mvRowIndices.size() != nRows) {
        PrepareStereoCandidates();
        std::cout << "redo the stereo candidate preparation!" << std::endl;
    }

    // Set limits for search
    const float minZ = mb;

    int nmatched = 0;

    std::vector<unsigned int> unmatched;
    Frame::GetUnMatchedKPbyBucketing(this, unmatched);

    //    for(int iL=0; iL<N; iL++)
    for(int iL:unmatched)
    {

#ifdef DELAYED_STEREO_MATCHING
        if (isOnline) {
            // only stereo matching points with associated 3D map points
            if (this->mvpMapPoints[iL] == NULL || this->mvStereoMatched[iL] == true) {
                continue ;
            }
        }
        else {
            // stereo matching the rest of points
            if (this->mvStereoMatched[iL] == true) {
                continue ;
            }
        }
#endif

        float minD = 0;
        float maxD = mbf/minZ;
        this->mvStereoMatched[iL] = true;

        const cv::KeyPoint &kpL = mvKeysUn[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        if (vL < 0 || vL > nRows-1)
            continue;

        const vector<size_t> &vCandidates = mvRowIndices[int(vL)];

        if(vCandidates.empty())
            continue;

        //        cout << "Before - minD: " << minD << " maxD: " << maxD << ";   ";
        // reduce the range of stereo matching for map-matched points
        if (this->mvpMapPoints[iL] != NULL) {
            MapPoint * pMP = this->mvpMapPoints[iL];
            if (!pMP->isBad()) {
                cv::Mat Pw = pMP->GetWorldPos(), Pc;
                if (WorldToCameraPoint(Pw, Pc) == true) {
                    //
                    float disp = float(mbf) / Pc.at<float>(2);
                    minD = std::max(disp - float(DISPARITY_THRES), 0.0f);
                    maxD = std::min(disp + float(DISPARITY_THRES), float(mbf)/float(mb));
                }
            }
        }
        //        cout << "After - minD: " << minD << " maxD: " << maxD << endl;
        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<mnMinX)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRightUn[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            float bestuR = mvKeysRightUn[bestIdxR].pt.x;
            float disparity = (uL-bestuR);
            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                mvDistIdx.push_back(pair<int,int>(bestDist,iL));
            }

        }

        nmatched ++;
    }

    if (!isOnline) {
        sort(mvDistIdx.begin(),mvDistIdx.end());
        const float median = mvDistIdx[mvDistIdx.size()/2].first;
        const float thDist = 1.5f*1.4f*median;
        // reduce the number of stereo matchings being dropped
        //        const float thDist = 3.0f*median;

        for(int i=mvDistIdx.size()-1;i>=0;i--)
        {
            if(mvDistIdx[i].first<thDist)
                break;
            else
            {
                mvuRight[mvDistIdx[i].second]=-1;
                mvDepth[mvDistIdx[i].second]=-1;
                nmatched --;
            }
        }
    }

    return nmatched;
}

void Frame::GetUnMatchedKPbyBucketing(const Frame *pFrame, std::vector<unsigned int> &vUnMatchedKeyPoints)
{
    //    int32_t max_features = BUCKET_FEATURE_NUM;
    float bucket_width = FRAME_GRID_COLS;
    float bucket_height = FRAME_GRID_ROWS;

    // find max values
    float u_max = -99999,   v_max = -99999;
    float u_min = 99999,    v_min = 99999;

    //    if(mbTrackLossAlert = 2){
    //        for(unsigned int i=0; i!=pFrame->N; i++){
    //            if(pFrame->mvpMapPoints[i] != NULL)
    //                vUnMatchedKeyPoints.push_back(i);
    //        }
    //        return;
    //    }

    //    int32_t inlier_num = 0;
    for(auto it = pFrame->mvKeysUn.begin(); it != pFrame->mvKeysUn.end(); it++)  {
        //        for(int i= 0; i<pFrame->mvKeysUn.size(); i++)  {
        //        MapPoint* pMP = pFrame->mvpMapPoints[i];
        //        if(pMP) {
        //            if (pFrame->mvbOutlier[i] == false && pFrame->mvbCandidate[i] == true) {
        // kpUn.pt.x, kpUn.pt.y;
        //                cv::KeyPoint kpUn = pFrame->mvKeysUn[i];

        //
        if (it->pt.x > u_max)
            u_max = it->pt.x;
        if (it->pt.y > v_max)
            v_max = it->pt.y;
        //
        if (it->pt.x < u_min)
            u_min = it->pt.x;
        if (it->pt.y < v_min)
            v_min = it->pt.y;
        //
        //                inlier_num ++;
        //            }
        //        }
    }

    //    std::cout << "u_max = " << u_max << "; "  << "v_max = " << v_max << "; " << std::endl;
    //    std::cout << "u_min = " << u_min << "; "  << "v_min = " << v_min << "; " << std::endl;
    //    std::cout<<std::endl<<"STEP 1 FINISHED, "<<std::endl;
    // allocate number of buckets needed
    int32_t bucket_cols = (int32_t)floor( (u_max - u_min) / float(bucket_width) )  + 1;
    int32_t bucket_rows = (int32_t)floor( (v_max - v_min) / float(bucket_height) ) + 1;
    Grid *buckets = new Grid[bucket_cols*bucket_rows];

    //    std::cout << "bucket_cols = " << bucket_cols << "; "  << "bucket_rows = " << bucket_rows << "; " << std::endl;
    //    int32_t max_features = (int32_t)floor( float(inlier_num) * this->ratio_good_inlier_predef / float(bucket_cols*bucket_rows) );
    //    std::cout<<std::endl<<"STEP 2 FINISHED, "<<std::endl;
    // assign all keypoints to the buckets
    for(int i = 0; i != pFrame->mvKeysUn.size(); i++)  {
        //    for(auto itKpUn = pFrame->mvKeysUn.begin(); itKpUn!=pFrame->mvKeysUn.end(); i++, itKpUn++)  {
        //        MapPoint* pMP = pFrame->mvpMapPoints[i];
        //        if(pMP) {
        //            if (pFrame->mvbOutlier[i] == false && pFrame->mvbCandidate[i] == true) {
        //                std::cout << "enter one map point" << std::endl;
        // kpUn.pt.x, kpUn.pt.y;
        //                cv::KeyPoint kpUn = pFrame->mvKeysUn[i];
        int32_t u = (int32_t)floor( float(pFrame->mvKeysUn[i].pt.x - u_min) / float(bucket_width) );
        int32_t v = (int32_t)floor( float(pFrame->mvKeysUn[i].pt.y - v_min) / float(bucket_height) );
        //                std::cout << "u = " << u << "; v = " << v << std::endl;
        if (pFrame->mvpMapPoints[i] == NULL)    // not matched to any local MapPoint yet
        {
            buckets[ v * bucket_cols + u ].kp_unmatched_indices.push_back(i);
        }
        else
        {
            buckets[ v * bucket_cols + u ].matched_size ++;
        }
        //            }
        //        }
    }
    //    int32_t max_features = (int32_t)ceil( float(this->num_good_inlier_predef) / float(bucket_cols*bucket_rows) ) + 1;

    float num_mached_kpts = 0;  // total #matched_features
    float num_matched_buckets = 0;  // #matched_grids
    for (size_t i=0; i<bucket_cols*bucket_rows; i++)
    {
        if(buckets[i].matched_size>0)
        {
            num_matched_buckets ++;
            num_mached_kpts += buckets[i].matched_size;
        }

    }
    unsigned int min_matched_features = (unsigned int)ceil(num_mached_kpts / num_matched_buckets / 2.0);
    min_matched_features = max(min_matched_features, (unsigned int)BUCKET_MIN_FEATURES_PER_GRID);
    //    std::cout << "fill in content for buckets!" << std::endl;

    // refill p_matched from buckets
    //    size_t total_num = 0;
    //    bool stop_bucketing = false;
    vUnMatchedKeyPoints.clear();
    for (size_t i=0; i<bucket_cols*bucket_rows; i++) {

        //        if (stop_bucketing == true)
        //            break ;

        if (buckets[i].matched_size < min_matched_features)
        {
            vUnMatchedKeyPoints.insert(vUnMatchedKeyPoints.end(), buckets[i].kp_unmatched_indices.begin(),  buckets[i].kp_unmatched_indices.end());
        }

        // shuffle bucket indices randomly
        //        std::random_shuffle(buckets[i].begin(),buckets[i].end());

        // add up to max_features features from this bucket to p_matched
        //        size_t k=0;
        //        for (vector<size_t>::iterator it=buckets[i].begin(); it!=buckets[i].end(); it++) {
        //            //
        //            //            std::cout << "select match " << *it << " from bucket " << i << std::endl;
        //            obsLmk tmpLmk(*it, 1);
        //            mpBucketed.push_back(tmpLmk);
        //            k++;
        //            total_num ++;
        //            //
        //            if (total_num >= this->num_good_inlier_predef) {
        //                stop_bucketing = true;
        //                break ;
        //            }
        //            if (k >= max_features)
        //                break;
        //        }
    }

    //    std::cout << "feature bucketed = " << total_num << std::endl;
    //    std::cout << "done with bucketing!" << std::endl;
    //    std::cout<<"ALL FINISHED"<<std::endl;
    // free buckets
    delete []buckets;
}


void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}



void Frame::plotStereoDetection(const cv::Mat &imLeft, const cv::Mat &imRight,
                                cv::Mat &imRes) {

    // create new image to modify it
    cv::Mat img_l_aux,img_l_aux1,img_l_aux2;
    //cv::Mat img_l_aux;
    imLeft.copyTo( img_l_aux2 );
    imRight.copyTo( img_l_aux1 );
    if( img_l_aux1.channels() == 1 )
        cv::cvtColor(img_l_aux1, img_l_aux1, cv::COLOR_GRAY2BGR);
    if( img_l_aux2.channels() == 1 )
        cv::cvtColor(img_l_aux2, img_l_aux2, cv::COLOR_GRAY2BGR);

    assert(mvKeys.size() == mvKeysRight.size());

    //    cv::RNG rand_gen( 0xFFFFFFFF );
    for (size_t i=0; i<mvKeys.size(); ++i) {
        //        int icolor = (unsigned) rand_gen;
        //        const cv::Scalar color_rnd = cv::Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
        //
        cv::circle( img_l_aux2, mvKeys[i].pt, 3.0, cv::Scalar(0,255,255), 1.5);
        //        cv::putText(img_l_aux2,
        //                    std::to_string(mvKeys[i].class_id),
        //                    mvKeys[i].pt, // Coordinates
        //                    cv::FONT_HERSHEY_PLAIN, // Font
        //                    1.0, // Scale. 2.0 = 2x bigger
        //                    color_rnd, // Color
        //                    1); // Thickness
        //
        cv::circle( img_l_aux1, mvKeysRight[i].pt, 3.0, cv::Scalar(0,255,0), 1.5);
        //        cv::putText(img_l_aux1,
        //                    std::to_string(mvKeysRight[i].class_id),
        //                    mvKeysRight[i].pt, // Coordinates
        //                    cv::FONT_HERSHEY_PLAIN, // Font
        //                    1.0, // Scale. 2.0 = 2x bigger
        //                    color_rnd, // Color
        //                    1); // Thickness
    }

    cv::hconcat(img_l_aux2,img_l_aux1,img_l_aux);
    //    if( img_l_aux.channels() == 1 )
    //        cvtColor(img_l_aux,img_l_aux,CV_GRAY2BGR);
    cv::putText(img_l_aux,
                std::to_string((mvKeys.size())),
                cv::Point(25, 25), // Coordinates
                cv::FONT_HERSHEY_PLAIN, // Font
                2.0, // Scale. 2.0 = 2x bigger
                cv::Scalar(0,0,255), // Color
                2); // Thickness

    img_l_aux.copyTo(imRes);
}



void Frame::plotStereoMatching(const cv::Mat &imLeft, const cv::Mat &imRight,
                               cv::Mat &imRes) {

    // create new image to modify it
    cv::Mat img_l_aux,img_l_aux1,img_l_aux2;
    //cv::Mat img_l_aux;
    imLeft.copyTo( img_l_aux2 );
    imRight.copyTo( img_l_aux1 );
    if( img_l_aux1.channels() == 1 )
        cv::cvtColor(img_l_aux1, img_l_aux1, cv::COLOR_GRAY2BGR);
    if( img_l_aux2.channels() == 1 )
        cv::cvtColor(img_l_aux2, img_l_aux2, cv::COLOR_GRAY2BGR);

    assert(mvKeys.size() == mvKeysRight.size());

    int match_num = 0;
    cv::RNG rand_gen( 0xFFFFFFFF );
    for (size_t i=0; i<mvKeys.size(); ++i) {
        int icolor = (unsigned) rand_gen;
        const cv::Scalar color_rnd = cv::Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
        //
        if (mvuRight[i] == -1) {
            //
            cv::circle( img_l_aux2, mvKeys[i].pt, 3.0, cv::Scalar(255,255,255), 1.5);
        }
        else {
            //
            cv::circle( img_l_aux2, mvKeys[i].pt, 3.0, color_rnd, 1.5);
            cv::Point2f match_pt;
            match_pt.x = mvuRight[i];
            match_pt.y = mvKeys[i].pt.y;
            cv::circle( img_l_aux1, match_pt, 3.0, color_rnd, 1.5);
            //
            match_num ++;
        }
    }

    cv::hconcat(img_l_aux2,img_l_aux1,img_l_aux);
    //    if( img_l_aux.channels() == 1 )
    //        cvtColor(img_l_aux,img_l_aux,CV_GRAY2BGR);
    cv::putText(img_l_aux,
                std::to_string(match_num),
                cv::Point(25, 25), // Coordinates
                cv::FONT_HERSHEY_PLAIN, // Font
                2.0, // Scale. 2.0 = 2x bigger
                cv::Scalar(0,0,255), // Color
                2); // Thickness

    img_l_aux.copyTo(imRes);
}

} // namespace ORB_SLAM2
