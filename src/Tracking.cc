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

#include <unistd.h>
#include <iostream>
#include <fstream>

#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"

#include "Optimizer.h"
#include "PnPsolver.h"

#include <iostream>
#include <fstream>

#include <mutex>

using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB,
                   const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0), num_good_constr_predef(200)
{

    //    this->num_good_feature_found = 0;
    //    cout << strSettingPath << endl;
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    //    cout << "what" << endl;
    //
    // Load camera parameters from settings file
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    //    cout << "what" << endl;

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);
    //    cout << "what" << endl;

    mbf = fSettings["Camera.bf"];
    camera_fps = fSettings["Camera.fps"];
    if(camera_fps==0)
        camera_fps=30;
    cout << "camera fps: " << camera_fps << endl;

    if (sensor != System::STEREO) {
        // default code in ORB-SLAM2
        cv::Mat DistCoef(4,1,CV_32F);
#ifdef USE_FISHEYE_DISTORTION
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.k3"];
        DistCoef.at<float>(3) = fSettings["Camera.k4"];
#else
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if(k3!=0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
#endif
        DistCoef.copyTo(mDistCoef);

        cout << endl << "Camera Parameters: " << endl;
        cout << "- fx: " << fx << endl;
        cout << "- fy: " << fy << endl;
        cout << "- cx: " << cx << endl;
        cout << "- cy: " << cy << endl;
#ifdef USE_FISHEYE_DISTORTION
        cout << "- k1: " << DistCoef.at<float>(0) << endl;
        cout << "- k2: " << DistCoef.at<float>(1) << endl;
        cout << "- k3: " << DistCoef.at<float>(2) << endl;
        cout << "- k4: " << DistCoef.at<float>(3) << endl;
#else
        cout << "- k1: " << DistCoef.at<float>(0) << endl;
        cout << "- k2: " << DistCoef.at<float>(1) << endl;
        if(DistCoef.rows==5)
            cout << "- k3: " << DistCoef.at<float>(4) << endl;
        cout << "- p1: " << DistCoef.at<float>(2) << endl;
        cout << "- p2: " << DistCoef.at<float>(3) << endl;
#endif

        if(sensor==System::RGBD)
        {
            mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
            cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;

            mDepthMapFactor = fSettings["DepthMapFactor"];
            if(fabs(mDepthMapFactor)<1e-5)
                mDepthMapFactor=1;
            else
                mDepthMapFactor = 1.0f/mDepthMapFactor;
        }
    }
    else {
        // modified stereo with undistortion of both cameras
        // Load camera parameters from settings file
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fSettings["LEFT.K"] >> K_l;
        if (K_l.type() != CV_32F)
            K_l.convertTo(K_l, CV_32F);
        fSettings["RIGHT.K"] >> K_r;
        if (K_r.type() != CV_32F)
            K_r.convertTo(K_r, CV_32F);

        fSettings["LEFT.P"] >> P_l;
        if (P_l.type() != CV_32F)
            P_l.convertTo(P_l, CV_32F);
        fSettings["RIGHT.P"] >> P_r;
        if (P_r.type() != CV_32F)
            P_r.convertTo(P_r, CV_32F);

        fSettings["LEFT.R"] >> R_l;
        if (R_l.type() != CV_32F)
            R_l.convertTo(R_l, CV_32F);
        fSettings["RIGHT.R"] >> R_r;
        if (R_r.type() != CV_32F)
            R_r.convertTo(R_r, CV_32F);

        fSettings["LEFT.D"] >> D_l;
        if (D_l.type() != CV_32F)
            D_l.convertTo(D_l, CV_32F);
        fSettings["RIGHT.D"] >> D_r;
        if (D_r.type() != CV_32F)
            D_r.convertTo(D_r, CV_32F);

        int rows_l = fSettings["LEFT.height"];
        int cols_l = fSettings["LEFT.width"];
        int rows_r = fSettings["RIGHT.height"];
        int cols_r = fSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return ;
        }

        K_l.copyTo(mK_ori);
        D_l.copyTo(mDistCoef);
        R_l.copyTo(mR);
        P_l.copyTo(mP);
        //
        K_r.copyTo(mK_right);
        D_r.copyTo(mDistCoef_right);
        R_r.copyTo(mR_right);
        P_r.copyTo(mP_right);

#ifndef ALTER_STEREO_MATCHING
        //
#ifdef USE_FISHEYE_DISTORTION
        cv::fisheye::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0,3).colRange(0,3), cv::Size(cols_l,rows_l), CV_32F, mMap1_l, mMap2_l);
        cv::fisheye::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0,3).colRange(0,3), cv::Size(cols_r,rows_r), CV_32F, mMap1_r, mMap2_r);
#else
        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0,3).colRange(0,3), cv::Size(cols_l,rows_l), CV_32F, mMap1_l, mMap2_l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0,3).colRange(0,3), cv::Size(cols_r,rows_r), CV_32F, mMap1_r, mMap2_r);
#endif

#endif

        cout << endl << "Camera Parameters: " << endl;
        cout << "- fx: " << mK.at<float>(0,0) << endl;
        cout << "- fy: " << mK.at<float>(1,1) << endl;
        cout << "- cx: " << mK.at<float>(0,2) << endl;
        cout << "- cy: " << mK.at<float>(1,2) << endl;
#ifdef USE_FISHEYE_DISTORTION
        cout << "- k1: " << mDistCoef.at<float>(0) << endl;
        cout << "- k2: " << mDistCoef.at<float>(1) << endl;
        cout << "- k3: " << mDistCoef.at<float>(2) << endl;
        cout << "- k4: " << mDistCoef.at<float>(3) << endl;
#else
        cout << "- k1: " << mDistCoef.at<float>(0) << endl;
        cout << "- k2: " << mDistCoef.at<float>(1) << endl;
        if(mDistCoef.rows==5)
            cout << "- k3: " << mDistCoef.at<float>(4) << endl;
        cout << "- p1: " << mDistCoef.at<float>(2) << endl;
        cout << "- p2: " << mDistCoef.at<float>(3) << endl;
#endif

        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }


    int nRows= fSettings["Camera2.nRows"];
    int nCols= fSettings["Camera2.nCols"];
    mObsHandler = new Observability(fx, fy, nRows, nCols, cx, cy, 0, 0, sensor);
    if (sensor == System::STEREO || sensor == System::RGBD)
        mObsHandler->camera.bf = mbf;

#ifdef SPARSE_KEYFRAME_COND
    // Sparse config for key-frames, suited for large-scale (outdoor) environment
    mMinFrames = camera_fps;
    mMaxFrames = camera_fps * 5; // 10;
#else
    // Dense (default) config for key-frames, suited for small-scale environment
    mMinFrames = 0;
    mMaxFrames = camera_fps;
    //    mMaxFrames = 18*camera_fps/30;
    //  #ifdef GROUND_TRUTH_GEN_MODE
    //      mMaxFrames = camera_fps * 0.2;
    //  #endif

#endif

#ifdef ENABLE_ANTICIPATION_IN_GRAPH
    //NOTE for fast-mo simulation, the budget could be less than real-time value, 
    // therefore is set explicitly (via System::SetBudgetPerFrame)
    // mVFrameInteval = VIRTUAL_FRAME_STEP * time_track_budget;
    //
    //NOTE for real-time navigation, simply set the budget as 1/camera_fps will do
    mVFrameInteval = VIRTUAL_FRAME_STEP / camera_fps;
    
#endif

    mFrameAfterInital = 0;
    mbTrackLossAlert = 0;
    nFrameSinceLast = 0;

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;
    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    // default
    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR) {
        //        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
        mpIniORBextractor = new ORBextractor(2000,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    }

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

#ifdef DISABLE_RELOC
    std::cout << "Tracking: relocalization disabled!" << std::endl;
#else
    std::cout << "Tracking: relocalization enabled!" << std::endl;
#endif

#ifdef PRED_WITH_ODOM
    std::cout << "Tracking: pred with odom enabled!" << std::endl;

    // @TODO (yanwei) even though, constant velocity model is used if
    // disturbed gt pose or planner fails to predict new camera pose,
    // see @line1873 for code detail.
#else
    std::cout << "Tracking: constant velocity motion model is used in tracking!" << std::endl;
#endif

#ifdef ENABLE_PLANNER_PREDICTION
    std::cout << "Tracking: trajectory planner prediction enabled!" << std::endl;
#endif

#ifdef USE_INFO_MATRIX
    mCurrentInfoMat.set_size(7, 7);
#elif defined USE_HYBRID_MATRIX
    mCurrentInfoMat.set_size(13, 13);
#endif
    mCurrentInfoMat.zeros();

    mnInitStereo = 0;


#ifdef INIT_WITH_ARUCHO
    //
    mTw_align = cv::Mat::eye(4,4,CV_32F);
    mTw_align.at<float>(0, 3) = fSettings["InitialAnchor.x"];
    mTw_align.at<float>(1, 3) = fSettings["InitialAnchor.y"];
    mTw_align.at<float>(2, 3) = fSettings["InitialAnchor.z"];
    //
    float qw = fSettings["InitialAnchor.qw"];
    float qx = fSettings["InitialAnchor.qx"];
    float qy = fSettings["InitialAnchor.qy"];
    float qz = fSettings["InitialAnchor.qz"];
    arma::rowvec qw_align = {qw, qx, qy, qz};
    cv::Mat Rw_align = cv::Mat(3, 3, CV_32F);
    QUAT2DCM_float(qw_align, Rw_align);
    Rw_align.copyTo(mTw_align.rowRange(0,3).colRange(0,3));
    std::cout << "Transform hard-coded in world frame: " << mTw_align << std::endl;
    /*
float init_bias_x = fSettings["InitialBias.x"];
float init_bias_y = fSettings["InitialBias.y"];
float init_bias_z = fSettings["InitialBias.z"];
float init_bias_qw = fSettings["InitialBias.qw"];
float init_bias_qx = fSettings["InitialBias.qx"];
float init_bias_qy = fSettings["InitialBias.qy"];
float init_bias_qz = fSettings["InitialBias.qz"];
mT_bias = cv::Mat::eye(4,4,CV_32F);
mT_bias.at<float>(0, 3) = init_bias_x;
mT_bias.at<float>(1, 3) = init_bias_y;
mT_bias.at<float>(2, 3) = init_bias_z;
//
arma::rowvec q_bias = {init_bias_qw, init_bias_qx, init_bias_qy, init_bias_qz};
q_bias = qNormalize(q_bias);
cv::Mat R_bias = cv::Mat(3, 3, CV_32F);
QUAT2DCM_float(q_bias, R_bias);
R_bias.copyTo(mT_bias.rowRange(0,3).colRange(0,3));
*/
    //
    mpCharuco = new ChArUco(mK, mDistCoef, "../tools/charuco_param.yml",
                            false, true);
#endif

    //#ifdef PRED_WITH_ODOM
    //    mvOdomBuf.reserve(800);
    //    //
    //#endif
    
#if defined ENABLE_PLANNER_PREDICTION
    mpStatePred = new TrajectoryStatePredictor();
    
    // TODO instead hard-coding body to camera transform, subscribe to tf and query the transform 
    Tc2b = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat Rmat(3,3,CV_32F);
    QUAT2DCM_float(-0.500, 0.500, -0.500, 0.500, Rmat);
    Rmat.copyTo(Tc2b.rowRange(0,3).colRange(0,3));
    Tc2b.at<float>(0,3) = 0.094;
    Tc2b.at<float>(1,3) = -0.074;
    Tc2b.at<float>(2,3) = 0.280;

    Tb2c = Tc2b.inv();
#endif

#if defined PRED_WITH_ODOM

    Tc2b = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat Rmat(3,3,CV_32F);
    QUAT2DCM_float(-0.500, 0.500, -0.500, 0.500, Rmat);
    Rmat.copyTo(Tc2b.rowRange(0,3).colRange(0,3));
    Tc2b.at<float>(0,3) = 0.150;
    Tc2b.at<float>(1,3) = -0.013;
    Tc2b.at<float>(2,3) = 0.310;
    Tb2c = Tc2b.inv();

#endif
    

#if defined DELAYED_MAP_MATCHING && defined LOCAL_SEARCH_USING_HASHING
    mvpQueriedFeatures.resize(NUM_TOTAL_HASHTABLES);
    std::fill(mvpQueriedFeatures.begin(), mvpQueriedFeatures.begin() + NUM_ACTIVE_HASHTABLES, NUM_ADDITIONAL_FEATURES + NUM_CANDIDATE_FEATURES);
    std::fill(mvpQueriedFeatures.begin() + NUM_ACTIVE_HASHTABLES, mvpQueriedFeatures.end(), 0);
#endif

}


Tracking::~Tracking() {
#ifdef REALTIME_TRAJ_LOGGING
    f_realTimeTrack.close();
#endif
}

void Tracking::SetRealTimeFileStream(string fNameRealTimeTrack)
{
    f_realTimeTrack.open(fNameRealTimeTrack.c_str());
    f_realTimeTrack << fixed;
    f_realTimeTrack << "#TimeStamp Tx Ty Tz Qx Qy Qz Qw" << std::endl;
}

void Tracking::updateORBExtractor() {

    assert(mpORBextractorLeft != NULL);

    // take the budget number of input arg as feature extraction constr
    float fScaleFactor = mpORBextractorLeft->GetScaleFactor();
    int nLevels = mpORBextractorLeft->GetLevels();
    int fIniThFAST = mpORBextractorLeft->GetInitThres();
    int fMinThFAST = mpORBextractorLeft->GetMinThres();

    //
    delete mpORBextractorLeft;
    mpORBextractorLeft = new ORBextractor(this->num_good_constr_predef / 2,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::STEREO) {
        assert(mpORBextractorRight != NULL);
        //
        delete mpORBextractorRight;
        mpORBextractorRight = new ORBextractor(this->num_good_constr_predef / 2,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    }
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

void Tracking::SetHashHandler(HASHING::MultiIndexHashing *pHashHandler)
{
    mpHashMethod = pHashHandler;
}

cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    logCurrentFrame.setZero();

    arma::wall_clock timer_mod;
    timer_mod.tic();

#ifdef ENABLE_WHITE_BALANCE

    // Debug
    cv::equalizeHist(imRectLeft, mImGray);
    cv::Mat imGrayRight;
    cv::equalizeHist(imRectRight, imGrayRight);
    //

    // mpBalancer->balanceWhite(imRectLeft, mImGray);
    // cv::Mat imGrayRight;
    // mpBalancer->balanceWhite(imRectRight, imGrayRight);

#else

    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

#endif

#ifndef ALTER_STEREO_MATCHING
    // undistort the image as in the original ORB pipeline
    cv::remap(mImGray, mImGray, mMap1_l, mMap2_l, cv::INTER_LINEAR);
    cv::remap(imGrayRight, imGrayRight, mMap1_r, mMap2_r, cv::INTER_LINEAR);
#endif

    logCurrentFrame.time_rectification = timer_mod.toc();

    timer_mod.tic();

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

#ifdef SIMU_MOTION_BLUR

    // Update kernel size for a normalized box filter
    int kernel_size = 5; // 9; // 15;
    cv::Mat kernel = cv::Mat::ones( kernel_size, kernel_size, CV_32F )/ (float)(kernel_size*kernel_size);

    cv::filter2D(mImGray, mImGray, -1, kernel);
    cv::filter2D(imGrayRight, imGrayRight, -1, kernel);

#endif


    //    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    //

    mCurrentFrame = Frame(mImGray, imGrayRight, timestamp,
                          mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK,
                          mK_ori, mDistCoef, mR, mP,
                          mK_right, mDistCoef_right, mR_right, mP_right,
                          mbf, mThDepth);

    logCurrentFrame.time_ORB_extraction = timer_mod.toc();

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    logCurrentFrame.setZero();

    arma::wall_clock timer_mod;
    timer_mod.tic();

#ifdef ENABLE_WHITE_BALANCE

    // Debug
    cv::equalizeHist(imRGB, mImGray);
    cv::Mat imDepth = imD;

#else

    mImGray = imRGB;
    cv::Mat imDepth = imD;

#endif

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    logCurrentFrame.time_ORB_extraction = timer_mod.toc();

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    logCurrentFrame.setZero();

    arma::wall_clock timer_mod;
    timer_mod.tic();

#ifdef ENABLE_WHITE_BALANCE

    // Debug
    cv::equalizeHist(im, mImGray);

#else

    mImGray = im;

#endif


    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    logCurrentFrame.time_ORB_extraction = timer_mod.toc();

    Track();

    return mCurrentFrame.mTcw.clone();
}


//
void Tracking::BufferingOdom(const double & timeStamp,
                             const double & tx, const double & ty, const double & tz,
                             const double & qw, const double & qx, const double & qy, const double & qz) {
    unique_lock<mutex> lock(mMutexOdomBuf);
    mvOdomBuf.push_back(OdometryLog(timeStamp, tx, ty, tz, qw, qx, qy, qz));
}

void Tracking::ResetOdomBuffer() {
    unique_lock<mutex> lock(mMutexOdomBuf);
    mvOdomBuf.clear();
}

void Tracking::PredictingOdom(const double & time_prev, const double & time_curr,
                              cv::Mat & T_se) {

    unique_lock<mutex> lock(mMutexOdomBuf);
    int n = mvOdomBuf.size(), i;
    //cout << "mvOdomBuf.size() = " << n ;
    cv::Mat T_st, T_ed;

    auto lower = std::lower_bound( mvOdomBuf.begin(), mvOdomBuf.end(), time_prev, OdomLogComparator() );
    auto upper = std::upper_bound( mvOdomBuf.begin(), mvOdomBuf.end(), time_curr, OdomLogComparator() );

    if (lower == mvOdomBuf.end()){
        mvOdomBuf.clear();
        return ;
    }
    lower->matrixCast(T_st);

    if (upper == mvOdomBuf.end()){
        mvOdomBuf.clear();
        return ;
    }
    upper->matrixCast(T_ed);

    // relative transform between i_st & i_ed
    // cv::Mat Ttmp = (Tb2c * T_ed * T_st.inv() * Tc2b);
    cv::Mat Ttmp = (Tb2c * T_ed.inv() * T_st * Tc2b);
    Ttmp.copyTo(T_se);

    //mvOdomBuf.clear();
    mvOdomBuf.erase(mvOdomBuf.begin(), upper);
}


/*
void Tracking::BufferingOdom(const nav_msgs::Odometry::ConstPtr& msg) {
  unique_lock<mutex> lock(mMutexOdomBuf);
  mvOdomBuf.push_back(OdomLog(msg->header.stamp.toSec(),
                  msg->twist.twist.linear.x,
                  msg->twist.twist.linear.y,
                  msg->twist.twist.linear.z,
                  msg->twist.twist.angular.x,
                  msg->twist.twist.angular.y,
                  msg->twist.twist.angular.z));
}

void Tracking::PredictingOdom(const double & time_prev, const double & time_curr,
                  cv::Mat & T_se) {

  unique_lock<mutex> lock(mMutexOdomBuf);
  int n = mvOdomBuf.size(), i;
  cv::Mat T_st, T_ed;
  if (n == 0)
    return ;

  //cout << "mvOdomBuf.size() = " << n ;
  bool is_valid = false;
  for (i=1; i<n; ++i) {
    if (mvOdomBuf[i].time_stamp > time_prev) {
      is_valid = true;
      break ;
    }
  }
  //cout << " start idx = " << i ;
  if (!is_valid){
    mvOdomBuf.clear();
    return ;
  }

  mvOdomBuf[i-1].matrixCast(T_st);

  // T_se = Tb2c * T_ed * T_st.inv() * Tc2b;
  cv::Mat Ttmp = (Tc2b * T_ed.inv() * T_st * Tb2c);
  Ttmp.copyTo(T_se);

  mvOdomBuf.clear();
}
*/


void Tracking::Track()
{
    if (mState == NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    arma::wall_clock timer_all, timer_mod;

    mLastProcessedState = mState;
#ifdef ENABLE_ANTICIPATION_IN_BUDGET
    // Update visible map points number
    mNumVisibleMpt = 0;
#endif

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if (mState == NOT_INITIALIZED)
    {
        if (mSensor == System::STEREO || mSensor == System::RGBD)
        {
#if defined ALTER_STEREO_MATCHING && defined DELAYED_STEREO_MATCHING
            mCurrentFrame.PrepareStereoCandidates();
            mCurrentFrame.ComputeStereoMatches_Undistorted(false);
#endif
            StereoInitialization();
        }
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if (mState != OK)
            return;
    }
    else
    {
        timer_all.tic();

        mFrameAfterInital++;

        // set up the time log struct
        logCurrentFrame.frame_time_stamp = mCurrentFrame.mTimeStamp;

        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if (!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if (mState == OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

#if defined DELAYED_STEREO_MATCHING
                mCurrentFrame.PrepareStereoCandidates();
#endif

                if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                {
                    timer_mod.tic();
                    bOK = TrackReferenceKeyFrame();
                    //                    cout << "number of visible map at TrackReferenceKeyFrame: " << mNumVisibleMpt << endl;
                    if (!bOK)
                        cout << "Track loss at func TrackReferenceKeyFrame !!!" << endl;
                    logCurrentFrame.time_track_frame = timer_mod.toc();
                }
                else
                {
                    timer_mod.tic();
                    bOK = TrackWithMotionModel();
                    //                    cout << "number of visible map at TrackWithMotionModel: " << mNumVisibleMpt << endl;
                    if (!bOK)
                        cout << "Track loss at func TrackWithMotionModel !!!" << endl;
                    logCurrentFrame.time_track_motion = timer_mod.toc();
                    if (!bOK)
                    {
                        timer_mod.tic();
                        bOK = TrackReferenceKeyFrame();
                        if (!bOK)
                            cout << "Track loss at func TrackReferenceKeyFrame !!!" << endl;
                        logCurrentFrame.time_track_frame = timer_mod.toc();
                    }
                }
            }
            else
            {
#ifdef DISABLE_RELOC
                // do nothing
#else

#if defined DELAYED_STEREO_MATCHING
                if(mSensor==System::STEREO || mSensor==System::RGBD) {
                    mCurrentFrame.PrepareStereoCandidates();
                }
#endif

                bOK = Relocalization();
                if (!bOK)
                    cout << "Track loss at func Relocalization !!!" << endl;
#endif
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated

            if (mState == LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if (!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if (!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint *> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if (!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if (bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if (mbVO)
                        {
                            for (int i = 0; i < mCurrentFrame.N; i++)
                            {
                                if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if (bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }


        // cout << "Time cost of motion based tracking = " << logCurrentFrame.time_track_motion << endl;
        // cout << "Time cost of ref-frame tracking = " << logCurrentFrame.time_track_frame << endl;

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        timer_mod.tic();

        if(!mbOnlyTracking)
        {
            if(bOK) {
#ifdef GOOD_FEATURE_MAP_MATCHING
                if ( logCurrentFrame.lmk_num_motion*2 > num_good_constr_predef ||
                     logCurrentFrame.lmk_num_frame*2 > num_good_constr_predef ) {
                    // Skip map tracking
                    bOK = true;
                }
                else
                    bOK = TrackLocalMap();
#else
                bOK = TrackLocalMap();
#endif

                if (!bOK)
                    cout << "Track loss at func TrackLocalMap !!!" << endl;
            }
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO) {
                bOK = TrackLocalMap();
            }
        }

        logCurrentFrame.time_track_map = timer_mod.toc();

#ifdef ENABLE_TIME_CONTROL_FOR_TRACKING
        logCurrentFrame.lmk_hash_dynamics = mpHashMethod->updateDynamics(logCurrentFrame.time_track_map - MAX_TRACK_LOCALMAP_TIME);
#endif

        //	cout << "Time cost of local map tracking = " << logCurrentFrame.time_track_map << endl;

        if(bOK)
            mState = OK;
        else
            mState = LOST;

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if (bOK)
        {
            timer_mod.tic();

            // Update motion model
            if (!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
                mVelocity = mCurrentFrame.mTcw * LastTwc;
            }
            else
                mVelocity = cv::Mat();

            // VIZ only
            //        if (mCurrentFrame.mnId % 20 == 0)
            // PlotFrameWithPointMatches();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            logCurrentFrame.time_update_motion = timer_mod.toc();

            //            timer_mod.tic();

            // NOTE
            // When good feature being used, only the selected inliers will be sent into local BA

            //            // Clean VO matches
            //            for(int i=0; i<mCurrentFrame.N; i++)
            //            {
            //                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            //                if(pMP)
            //                    if(pMP->Observations()<1)
            //                    {
            //                        mCurrentFrame.mvbOutlier[i] = false;
            //                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            //                    }
            //            }

            //            // Delete temporal MapPoints
            //            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            //            {
            //                MapPoint* pMP = *lit;
            //                delete pMP;
            //            }
            //            mlpTemporalPoints.clear();

            //            // Check if we need to insert a new keyframe
            //            if(NeedNewKeyFrame()) {
            //                CreateNewKeyFrame();
            //                cout << "New key frame inserted!" << endl;
            //            }

            //            // We allow points with high innovation (considererd outliers by the Huber Function)
            //            // pass to the new keyframe, so that bundle adjustment will finally decide
            //            // if they are outliers or not. We don't want next frame to estimate its position
            //            // with those points so we discard them in the frame.
            //            for(int i=0; i<mCurrentFrame.N;i++)
            //            {
            //                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
            //                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            //            }

            //            logCurrentFrame.time_create_kf = timer_mod.toc();
        }

        // Reset if the camera get lost soon after initialization
        if (mState == LOST)
        {
            if (mpMap->KeyFramesInMap() <= 5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        //        if(!mCurrentFrame.mpReferenceKF)
        //            mCurrentFrame.mpReferenceKF = mpReferenceKF;

#if defined GOOD_FEATURE_MAP_MATCHING || defined ONLINE_TABLE_SELECTION
        // predict the pose at next frame
        if (bOK) {
            // cout << "SE3 of last frame: " << mLastFrame.mTcw << endl;
            if(!mLastFrame.mTcw.empty()) {
                mObsHandler->updatePWLSVec(mLastFrame.mTimeStamp, mLastFrame.mTcw,
                                           mCurrentFrame.mTimeStamp, mCurrentFrame.getTwc());
            }
            mObsHandler->predictPWLSVec( (mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp), 2 );
        }
#endif

        // Since we search for additional matches after publishing the camera state,
        // the creation of last frame should be postponed after the additional matching.
        //        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    //    if(!mCurrentFrame.mTcw.empty())
    //    {
    //        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
    //        mlRelativeFramePoses.push_back(Tcr);
    //        mlpReferences.push_back(mpReferenceKF);
    //        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
    //        mlbLost.push_back(mState==LOST);
    //    }
    //    else
    //    {
    //        // This can happen if tracking is lost
    //        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
    //        mlpReferences.push_back(mlpReferences.back());
    //        mlFrameTimes.push_back(mlFrameTimes.back());
    //        mlbLost.push_back(mState==LOST);
    //    }

#ifdef REALTIME_TRAJ_LOGGING
    if(!mCurrentFrame.mTcw.empty())
    {
        // Write the real time tracking result to file
        FramePose poseTmp = FramePose(mCurrentFrame.mTimeStamp, mCurrentFrame.mTcw);

        double timestamp = poseTmp.time_stamp;
        cv::Mat Homm = poseTmp.homm;
        cv::Mat R = Homm.rowRange(0,3).colRange(0,3).t();
        cv::Mat t = - R * Homm.rowRange(0,3).col(3);

        //
        // TODO
        // check the output R & T are identical with the ones send to visualizer
        //
        //        std::cout << twc - t << std::endl;
        //        std::cout << Rwc - R << std::endl;

        vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);

        f_realTimeTrack << setprecision(6) << timestamp << setprecision(7)
                        << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
                        << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }
#endif
    // unlock the map states?
    //    lock.unlock();

    //
    timer_mod.tic();
    if (mState == OK) {
        //        UpdateReferenceKeyFrames();

#ifdef GROUND_TRUTH_GEN_MODE
        double timeCost_sofar = timer_all.toc() + logCurrentFrame.time_ORB_extraction,
                timeCost_rest = 1.0 / (camera_fps * 0.2) - timeCost_sofar - 0.002;
        // Compute the good feature for local Map
        // std::cout << "======= total time to proc 1 frame = " << 1.0 / (camera_fps * 0.2)
        // 	  << "; already taken " << timeCost_sofar
        // 	  << " with ORB time " << logCurrentFrame.time_ORB_extraction
        // 	  << " ; left " << timeCost_rest << " =======" << std::endl;
#else
        double timeCost_sofar = timer_all.toc() + logCurrentFrame.time_ORB_extraction,
                timeCost_rest = 1.0 / (camera_fps) - timeCost_sofar - 0.002;
        // Compute the good feature for local Map
        // std::cout << "======= total time to proc 1 frame = " << 1.0 / (camera_fps)
        // 	  << "; already taken " << timeCost_sofar
        // 	  << " with ORB time " << logCurrentFrame.time_ORB_extraction
        // 	  << " ; left " << timeCost_rest << " =======" << std::endl;
#endif

        if (mObsHandler != NULL) {
            // predict info matrix for visible local points
            mObsHandler->mnFrameId = mCurrentFrame.nNextId;

            mObsHandler->mBoundXInFrame = (ORB_SLAM2::Frame::mnMaxX - ORB_SLAM2::Frame::mnMinX) * 0.1; // 20;
            mObsHandler->mBoundYInFrame = (ORB_SLAM2::Frame::mnMaxY - ORB_SLAM2::Frame::mnMinY) * 0.1; // 20;
            mObsHandler->mBoundDepth = 0;
        }


#ifdef PRECOMPUTE_WITH_MOTION_MODEL
        // predict info matrix for visible local points at next frame
        mObsHandler->mnFrameId = mCurrentFrame.nNextId;
        //  set pred_horizon to 1, so that the NEXT pose is used to compute jacobian
        std::thread thread_Predict(&Tracking::PredictJacobianNextFrame, this, timeCost_rest, 1);
#endif

        // Check if we need to insert a new keyframe
        //#ifdef DELAYED_STEREO_MATCHING
        //        if(NeedNewKeyFrame_Experimental()) {
        //#else
        //        if(NeedNewKeyFrame()) {
        //#endif

#ifdef SPARSE_KEYFRAME_COND
        if(NeedNewKeyFrame_Temp()) {
#else
        if(NeedNewKeyFrame()) {
#endif

#ifdef DELAYED_MAP_MATCHING
            //
#ifdef ONLINE_TABLE_SELECTION
            // additional search routine that supports online table selection
            // add additional local points into the left pool
            if (mObsHandler != NULL) {
                //            idx_startVizCheck = mObsHandler->mLeftMapPoints.size();
                // mvpLeftMapPointsByHashing = additionalMPsByHashing + complement of MPsByCovisibility
                for(vector<MapPoint*>::iterator itMP=mvpLeftMapPointsByHashing.begin(), itEndMP=mvpLeftMapPointsByHashing.end(); itMP!=itEndMP; itMP++)
                {
                    MapPoint* pMP = *itMP;
                    if(!pMP)        // already matched or outliers
                        continue;
                    if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)   // check if selected yet (avoid pushing back same MapPoints multiple times)
                        continue;
                    if(!pMP->isBad())
                    {
                        mObsHandler->mLeftMapPoints.push_back(pMP);
                        pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    }
                }

                // TODO: deal with mObsHandler->mbNeedVizCheck
                mObsHandler->mbNeedVizCheck = true;
            }

            // update rest time
            timeCost_rest = 1.0 / camera_fps - timer_all.toc() - 0.002;

            // search for additional matches
            if(SearchAdditionalMatchesInFrame(timeCost_rest, mCurrentFrame) && mbMapHashTriggered && mbMapHashOTS){

                // update time_rest
                timeCost_rest = 1.0 / camera_fps - timer_all.toc() - 0.002;

                // build information matrix for additional points
                mObsHandler->pFrame = &mCurrentFrame;

                // Add by Yipu
                // for online selection, it is necessary to set frame id back to CURRENT ID
                // while for pre-computation, the frame id will be the NEXT ID
                mObsHandler->mKineIdx = 0;
                mObsHandler->mnFrameId = mCurrentFrame.mnId;
                mObsHandler->runMatrixBuilding(ORB_SLAM2::FRAME_INFO_MATRIX, timeCost_rest * 0.4, USE_MULTI_THREAD, false);
                //
                //cout<<"complete building matrix, time cost:"<< timer.toc() <<endl;
                //timer.tic();
                // TODO @wye
                if(UpdateQueryNumByHashTable(timeCost_rest * 0.4)) {
                    //cout<<"Complete updating table index using "<<mObsHandler->mLeftMapPoints.size()<<" points, time cost:"<< timer.toc() <<endl;
                }
                else {
                    //cout<<"In-complete updating table index, time cost:"<< timer.toc() <<endl;
                }
            }
#else
            // default additional search routine
            SearchAdditionalMatchesInFrame(timeCost_rest, mCurrentFrame);
#endif
            // if left some time, run stereo matching
#ifdef ALTER_STEREO_MATCHING
            timer_mod.tic();
            // perform stereo matching
            int nStereo = 0;
            if(mSensor==System::STEREO) {
#ifdef LOCAL_SEARCH_USING_HASHING
                nStereo = mCurrentFrame.ComputeStereoMatches_Undistorted_ByBucketing(false);
#else
                nStereo = mCurrentFrame.ComputeStereoMatches_Undistorted(false);
#endif
            }
            //            std::cout << "func Track: number of new points being stereo matched = " << nStereo << "             " << std::endl;
            logCurrentFrame.time_stereo_post = timer_mod.toc();
#endif

#endif

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            CreateNewKeyFrame();
            nFrameSinceLast = 0;
        }
        else
            nFrameSinceLast ++;

        // reset active
#ifdef ONLINE_TABLE_SELECTION
        if(mbMapHashTriggered && mbMapHashOTS) {
            for(auto it=mvpMapPointsByHashing.begin(), itend=mvpMapPointsByHashing.end(); it!=itend; it++){
                MapPoint* pMP = *it;
                if(!pMP)        // already matched or outliers
                    continue;
                //            pMP->queriedByHashing = false;
                std::fill(pMP->mvbActiveHashTables.begin(), pMP->mvbActiveHashTables.end(), false);
            }
        }
#endif


#ifdef PRECOMPUTE_WITH_MOTION_MODEL
        thread_Predict.join();
        // PredictJacobianNextFrame(MATRIX_BUDGET_PREDICT, 1);
#endif

        // Delete temporal MapPoints
        for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
        {
            MapPoint* pMP = *lit;
            delete pMP;
        }
        mlpTemporalPoints.clear();

        // We allow points with high innovation (considererd outliers by the Huber Function)
        // pass to the new keyframe, so that bundle adjustment will finally decide
        // if they are outliers or not. We don't want next frame to estimate its position
        // with those points so we discard them in the frame.
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(pMP) {
                if(pMP->Observations()<1)
                {
                    mCurrentFrame.mvbOutlier[i] = false;
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                }
                //
                if (mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }

            //            if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
            //                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
        }

        mLastFrame = Frame(mCurrentFrame);

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

    }
    logCurrentFrame.time_post_proc = timer_mod.toc();

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else if (mlRelativeFramePoses.size() > 0)
    {
        // This can happen if tracking is lost
        if (!mlRelativeFramePoses.empty())
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        if (!mlpReferences.empty())
            mlpReferences.push_back(mlpReferences.back());
        if (!mlFrameTimes.empty())
            mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

    // push the time log of current frame into the vector
    mFrameTimeLog.push_back(logCurrentFrame);
    //    cout << "Done with tracking!" << endl;

    // std::cout << "Time cost break down: " << std::endl
    //           << "ORB extraction " << logCurrentFrame.time_ORB_extraction << "; "
    //           << "Init tracking " << logCurrentFrame.time_track_motion + logCurrentFrame.time_track_frame << "; "
    //           << "Map tracking " << logCurrentFrame.time_track_map << "; "
    //           << "Post proc. " << logCurrentFrame.time_post_proc << std::endl;

}


void Tracking::StereoInitialization()
{
    if (mCurrentFrame.N > THRES_INIT_MPT_NUM)
    {
        //
#ifdef INIT_WITH_ARUCHO
        // query the pose from ChArUco
        cv::Mat Twc_aruco = cv::Mat::eye(4,4,CV_32F);
        if (mpCharuco->process(mImGray, Twc_aruco)) {
            // Set Frame pose to the origin
            cv::Mat Tcw_aruco = Twc_aruco.inv();
            std::cout << "Transform established from ChArUco: " << Tcw_aruco << std::endl;
            //
            Tcw_aruco = mTw_align * Tcw_aruco;
            std::cout << "Further aligned to world frame: " << Tcw_aruco << std::endl;
            mCurrentFrame.SetPose(Tcw_aruco.inv());
        }
        else {
            // require re-init
            std::cout << "No valid transform established from ChArUco! Please make sure ChArUco board is visible!" << std::endl;
            return ;
        }
#else
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
#endif

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;

        cout << "func StereoInitialization: done!" << endl;
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>THRES_INIT_MPT_NUM)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=THRES_INIT_MPT_NUM)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,SRH_WINDOW_SIZE_INIT);
        /*
#ifdef ENABLE_LARGE_SEARCH_WINDOW
    int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,3*SRH_WINDOW_SIZE_INIT);
#else
    int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,SRH_WINDOW_SIZE_INIT);
#endif
*/

        // Check if there are enough correspondences
        if(nmatches<THRES_INIT_MPT_NUM)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

#ifdef GROUND_TRUTH_GEN_MODE
    Optimizer::GlobalBundleAdjustemnt(mpMap,40);
#else
    Optimizer::GlobalBundleAdjustemnt(mpMap,20);
#endif

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    double t = 0.;
    arma::wall_clock timer;
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;

#ifdef LOCAL_SEARCH_USING_HASHING
                timer.tic();
                mpHashMethod->insert(pRep);
                t += timer.toc();
#endif
            }
        }
    }

    logCurrentFrame.time_hash_insert += t;
}


bool Tracking::TrackReferenceKeyFrame()
{
    arma::wall_clock timer;

    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

#ifdef ENABLE_ANTICIPATION_IN_BUDGET
    mNumVisibleMpt = mpReferenceKF->GetMatchNum();
#endif

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    double time_stereo = 0;
#ifdef DELAYED_STEREO_MATCHING
    timer.tic();
    mnInitStereo = mCurrentFrame.ComputeStereoMatches_Undistorted(true);
    cout << "func TrackReferenceKeyFrame: number of points being stereo matched = " << mnInitStereo << "             "  << endl;
    time_stereo = timer.toc();
#endif
    //
    logCurrentFrame.time_stereo_frame = time_stereo;

    //    if(nmatches<15)
    if(nmatches<10)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    logCurrentFrame.lmk_num_frame = nmatches;

    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

bool Tracking::PredictRelMotionFromBuffer(const double & time_prev, const double & time_curr,
                                          cv::Mat & T_rel) {

    if (mvOdomBuf.empty() || time_curr < mvOdomBuf[0].time_stamp && time_prev > mvOdomBuf.back().time_stamp)
        return false;
//    cout << "current time: " << time_curr << " vs. " << mvOdomBuf[0].time_stamp << endl;
//    cout << "previous time: " << time_prev << " vs. " << mvOdomBuf.back().time_stamp << endl;

    int Nodom = mvOdomBuf.size();
    // std::cout << "mOdomTrackIdx = " << mOdomTrackIdx << "; Nodom = " << Nodom << std::endl;
    if (mOdomTrackIdx < 0 || mOdomTrackIdx >= Nodom)
        return false;

//        cout << mvOdomBuf[mOdomTrackIdx].time_stamp  << " vs. " << time_prev << endl;
    while (mvOdomBuf[mOdomTrackIdx].time_stamp < time_prev) {
        // move forward to the odom wrt time_prev
        mOdomTrackIdx ++;
        if (mOdomTrackIdx == Nodom)
            return false;
    }

    cv::Mat Twc_base;
//    cv::Mat Rwc = mvOdomBuf[mOdomTrackIdx].Tcw.rowRange(0, 3).colRange(0, 3).t();
//    cv::Mat twc = -Rwc * mvOdomBuf[mOdomTrackIdx].Tcw.rowRange(0, 3).col(3);
//    Twc_base = cv::Mat::eye(4, 4, CV_32F);
//    Rwc.copyTo(Twc_base.rowRange(0, 3).colRange(0, 3));
//    twc.copyTo(Twc_base.rowRange(0, 3).col(3));
    Twc_base = mvOdomBuf[mOdomTrackIdx].Twc;
//        cout << "Twc_base = " << Twc_base << endl;
    //
    for (int i = mOdomTrackIdx; i < Nodom; ++i) {
        //            cout << setprecision(12) << mvOdomBuf[i].first << " vs. " << pKF->mTimeStamp + vn * VIRTUAL_KF_INTEVAL << endl;
        // find the closest odom to pKF->mTimeStamp + VIRTUAL_KF_INTEVAL * (vn+1)
        if (mvOdomBuf[i].time_stamp >= time_curr) {
            // jump to the timestamp close to current frame
            mOdomTrackIdx = i;
            //                    Tcw_base = mvOdomBuf[i].second.inv();
            // Trel = Tcw_current * Twc_prev
            T_rel = mvOdomBuf[i].Tcw * Twc_base;
            // cv::Mat Ttmp = (Tb2c * T_ed.inv() * T_st * Tc2b);
            return true;
        }
    }

    return false;
}

bool Tracking::TrackWithMotionModel()
{
    arma::wall_clock timer;

    ORBmatcher matcher(0.9,true);

#ifdef TIMECOST_VERBOSE
    timer.tic();
#endif
    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();
#ifdef TIMECOST_VERBOSE
    cout << "Time Cost of Updating Last Frame = " << timer.toc() << endl;
#endif

    //#ifdef PRED_WITH_ODOM
    //    cout << "Previous mVelocity = " << endl << mVelocity << endl;
    //    //
    //    // NOTE
    //    // When pre-loading the full odom from file, query the relative motion with the following blcok
    //    //
    //    cv::Mat mVelocity_tmp;
    //    if (PredictRelMotionFromBuffer(mLastFrame.mTimeStamp, mCurrentFrame.mTimeStamp, mVelocity_tmp)) {
    //        mVelocity = mVelocity_tmp;
    //    }
    //    else {
    //        // do nothing
    //    }
    //    //
    //    cout << "Current mVelocity = " << endl << mVelocity << endl;
    //#endif

#ifdef PRED_WITH_ODOM
            // cout << "Previous mVelocity = " << endl << mVelocity << endl;
            //
            // NOTE
            // When connecting to PiPS planner, simply call recall function to query the desired relative motion
            cv::Mat mVelocity_tmp = cv::Mat(4,4,CV_32F);
#ifdef ENABLE_PLANNER_PREDICTION
            if (PredictRelMotionFromCallback(mLastFrame.mTimeStamp,
                                             mCurrentFrame.mTimeStamp,
                                             mVelocity_tmp)) {
                // add body to camera transform
                mVelocity = (Tb2c * mVelocity_tmp * Tc2b);
#else
            if (PredictRelMotionFromBuffer(mLastFrame.mTimeStamp,
                                           mCurrentFrame.mTimeStamp,
                                           mVelocity_tmp)) {
#endif
                // cout << "mVelocity_tmp = " << endl << mVelocity_tmp << endl;
                mVelocity = (Tb2c * mVelocity_tmp * Tc2b);
                // mVelocity = mVelocity_tmp;
            }
            else {
                // do nothing
            }
            // cout << "Current mVelocity = " << endl << mVelocity << endl;
#endif



    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

#ifdef TIMECOST_VERBOSE
    timer.tic();
#endif

    // Project points seen in previous frame
    int th;
    // if(mSensor!=System::STEREO)
    if (mSensor == System::MONOCULAR)
        th=15;
    else
        th=7;
    //    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);
#ifdef ENABLE_LARGE_SEARCH_WINDOW
    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th * 3, mSensor == System::MONOCULAR, mNumVisibleMpt);
#else
    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR, mNumVisibleMpt);
#endif

#ifdef TIMECOST_VERBOSE
    cout << "Time Cost of Feature Searching to Last Frame = " << timer.toc() << endl;
#endif

    double time_stereo = 0;
#ifdef DELAYED_STEREO_MATCHING
    timer.tic();
#ifdef LOCAL_SEARCH_USING_HASHING
    mnInitStereo = mCurrentFrame.ComputeStereoMatches_Undistorted_ByBucketing(true);
#else
    mnInitStereo = mCurrentFrame.ComputeStereoMatches_Undistorted(true);
#endif
    //    cout << "func TrackWithMotionModel: number of points being stereo matched = " << nStereo << "             "  << endl;
    time_stereo = timer.toc();
#endif
    //
    logCurrentFrame.time_stereo_motion = time_stereo;
    // cout << "Time Cost of Stereo Matching = " << time_stereo << endl;

#ifdef TIMECOST_VERBOSE
    timer.tic();
#endif
    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR, mNumVisibleMpt);
    }

    if(nmatches<20)
        return false;
#ifdef TIMECOST_VERBOSE
    cout << "Time Cost of Searching for Additional Matches = " << timer.toc() << endl;
#endif

#ifdef TIMECOST_VERBOSE
    timer.tic();
#endif
    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);
#ifdef TIMECOST_VERBOSE
    cout << "Time Cost of Pose Optimization = " << timer.toc() << endl;
#endif

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    logCurrentFrame.lmk_num_motion = nmatches;

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    arma::wall_clock timer;

    timer.tic();
    UpdateLocalMap();
    double time_upd_ref = timer.toc();

    timer.tic();
    int mnMatchesInit = SearchLocalPoints();
    double time_srh_ref = timer.toc();

    double time_stereo = 0;
#ifdef DELAYED_STEREO_MATCHING
    timer.tic();
    int nStereo = mCurrentFrame.ComputeStereoMatches_Undistorted(true);
    //    cout << "func TrackLocalMap: number of points being stereo matched = " << nStereo << "             "  << endl;
    time_stereo = timer.toc();
#endif

    timer.tic();
    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;
    double time_opt = timer.toc();

    timer.tic();
    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            // else if(mSensor==System::STEREO)
            else if(mSensor!=System::MONOCULAR)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }
    double time_post = timer.toc();

    logCurrentFrame.lmk_num_map = mnMatchesInliers;
    logCurrentFrame.lmk_num_BA = mnMatchesInit;
    logCurrentFrame.time_match = time_upd_ref + time_srh_ref + time_stereo;
    logCurrentFrame.time_optim = time_opt + time_post;
    //
    logCurrentFrame.time_stereo_map = time_stereo;

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    //    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
    //        return false;

    //    if(mnMatchesInliers<30)
    //        return false;
    //    else
    //        return true;

    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<25)
        return false;

    if(mnMatchesInliers<15)
        return false;
    else
        return true;
}





// TODO
// TODO
// TODO
// inject max vol to label the good map points from the rest
void Tracking::PredictJacobianNextFrame(const double time_for_predict, const size_t pred_horizon) {

    if (time_for_predict <= 0)
    {
        // std::cout << "too little budget available!" << std::endl;
        return ;
    }
    if (mObsHandler == NULL)
    {
        std::cout << "invalid mObsHandler!" << std::endl;
        return ;
    }

    //    arma::wall_clock timer;
    //    timer.tic();

    //    // find visible map points
    //    std::vector<MapPoint*> tmpMapPoints;
    //    for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    //    {
    //        KeyFrame* pKF = *itKF;
    //        vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

    //        for(vector<MapPoint*>::iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
    //        {
    //            MapPoint* pMP = *itMP;
    //            if(!pMP)
    //                continue;
    //            if(pMP->mnUsedForLocalMap==mCurrentFrame.mnId)
    //                continue;
    //            if(!pMP->isBad()) {
    //                //                cv::Mat Pw = pMP->GetWorldPos();
    //                //                if ( mObsHandler->visible_Point_To_Frame(Pw, mObsHandler->kinematic[1].Tcw) == true ) {
    //                //                    tmpMapPoints.push_back(pMP);
    //                //                }
    //                pMP->mnUsedForLocalMap=mCurrentFrame.mnId;
    //                tmpMapPoints.push_back(pMP);
    //            }
    //        }
    //    }
    //    //    std::cout << "func RunMapPointsSelection: number of local map points before visibility check = " << tmpMapPoints.size() << std::endl;

    double time_used = 0;
    mObsHandler->mMapPoints = &(mCurrentFrame.mvpMapPoints);

    //    // select map points with low score
    //    //    size_t num_map_point_selected = 4000; // 2000; //
    //    double time_used = timer.toc();

    //    mObsHandler->mMapPoints = &tmpMapPoints;
    mObsHandler->mKineIdx = pred_horizon;

#ifdef USE_INFO_MATRIX
    mObsHandler->runMatrixBuilding(ORB_SLAM2::MAP_INFO_MATRIX, time_for_predict - time_used, USE_MULTI_THREAD, true);
#elif defined USE_HYBRID_MATRIX
    mObsHandler->runMatrixBuilding(ORB_SLAM2::MAP_HYBRID_MATRIX, time_for_predict - time_used, USE_MULTI_THREAD, true);
#elif defined USE_OBSERVABILITY_MATRIX
    // TODO
#endif

    //    double time_map_bound = timer.toc();
    //    logCurrentFrame.time_mat_pred = time_map_bound;

    //    std::cout << "func PredictJacobianNextFrame: actual time used  = " << time_map_bound << std::endl;

}


void Tracking::BucketingMatches(const Frame *pFrame, vector<GoodPoint> & mpBucketed) {

    //    int32_t max_features = BUCKET_FEATURE_NUM;
    float bucket_width = BUCKET_WIDTH;
    float bucket_height = BUCKET_HEIGHT;

    // find max values
    float u_max = -99999,   v_max = -99999;
    float u_min = 99999,    v_min = 99999;
    int32_t inlier_num = 0;
    for(int i= 0; i<pFrame->mvpMapPoints.size(); i++)  {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP) {
            if (pFrame->mvbOutlier[i] == false && pFrame->mvbCandidate[i] == true) {
                // kpUn.pt.x, kpUn.pt.y;
                cv::KeyPoint kpUn = pFrame->mvKeysUn[i];
                //
                if (kpUn.pt.x > u_max)
                    u_max = kpUn.pt.x;
                if (kpUn.pt.y > v_max)
                    v_max = kpUn.pt.y;
                //
                if (kpUn.pt.x < u_min)
                    u_min = kpUn.pt.x;
                if (kpUn.pt.y < v_min)
                    v_min = kpUn.pt.y;
                //
                inlier_num ++;
            }
        }
    }

    //    std::cout << "u_max = " << u_max << "; "  << "v_max = " << v_max << "; " << std::endl;
    //    std::cout << "u_min = " << u_min << "; "  << "v_min = " << v_min << "; " << std::endl;

    // allocate number of buckets needed
    int32_t bucket_cols = (int32_t)floor( (u_max - u_min) / float(bucket_width) )  + 1;
    int32_t bucket_rows = (int32_t)floor( (v_max - v_min) / float(bucket_height) ) + 1;
    vector<size_t> *buckets = new vector<size_t>[bucket_cols*bucket_rows];

    //    std::cout << "bucket_cols = " << bucket_cols << "; "  << "bucket_rows = " << bucket_rows << "; " << std::endl;
    //    int32_t max_features = (int32_t)floor( float(inlier_num) * this->ratio_good_inlier_predef / float(bucket_cols*bucket_rows) );
    int32_t max_features = (int32_t)ceil( float(this->num_good_constr_predef / 2) / float(bucket_cols*bucket_rows) ) + 1;

    // assign matches to their buckets
    for(int i= 0; i<pFrame->mvpMapPoints.size(); i++)  {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP) {
            if (pFrame->mvbOutlier[i] == false && pFrame->mvbCandidate[i] == true) {
                //                std::cout << "enter one map point" << std::endl;
                // kpUn.pt.x, kpUn.pt.y;
                cv::KeyPoint kpUn = pFrame->mvKeysUn[i];

                //                std::cout << kpUn.pt.x << "; " << kpUn.pt.y << ";" << bucket_width << "; " << bucket_height << std::endl;

                int32_t u = (int32_t)floor( float(kpUn.pt.x - u_min) / float(bucket_width) );
                int32_t v = (int32_t)floor( float(kpUn.pt.y - v_min) / float(bucket_height) );

                //                std::cout << "u = " << u << "; v = " << v << std::endl;
                buckets[ v * bucket_cols + u ].push_back( static_cast<size_t>(i) );
            }
        }
    }
    //    std::cout << "fill in content for buckets!" << std::endl;

    // refill p_matched from buckets
    size_t total_num = 0;
    bool stop_bucketing = false;
    mpBucketed.clear();
    for (size_t i=0; i<bucket_cols*bucket_rows; i++) {

        if (stop_bucketing == true)
            break ;

        // shuffle bucket indices randomly
        std::random_shuffle(buckets[i].begin(),buckets[i].end());

        // add up to max_features features from this bucket to p_matched
        size_t k=0;
        for (vector<size_t>::iterator it=buckets[i].begin(); it!=buckets[i].end(); it++) {
            //
            //            std::cout << "select match " << *it << " from bucket " << i << std::endl;
            GoodPoint tmpLmk(*it, 1);
            mpBucketed.push_back(tmpLmk);
            k++;
            total_num ++;
            //
            if (total_num >= this->num_good_constr_predef / 2) {
                stop_bucketing = true;
                break ;
            }
            if (k >= max_features)
                break;
        }
    }

    //    std::cout << "feature bucketed = " << total_num << std::endl;
    //    std::cout << "done with bucketing!" << std::endl;

    // free buckets
    delete []buckets;
}


void Tracking::LongLivedMatches(const Frame *pFrame, vector<GoodPoint> & mpLongLived) {

    mpLongLived.clear();
    for(size_t i= 0; i<pFrame->mvpMapPoints.size(); i++)  {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP) {
            if (pFrame->mvbOutlier[i] == false && pFrame->mvbCandidate[i] == true) {
                GoodPoint tmpLmk(static_cast<size_t>(i), pMP->mnVisible);
                mpLongLived.push_back( tmpLmk );
            }
        }
    }

    std::sort(mpLongLived.begin(), mpLongLived.end(), GoodPoint::rankObsScore_descend);

    //
    if( mpLongLived.size() > this->num_good_constr_predef / 2) {
        mpLongLived.erase(mpLongLived.begin() + this->num_good_constr_predef / 2, mpLongLived.end());
    }
}



bool Tracking::NeedNewKeyFrame_Temp()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested()) {
        cout << "local mapping is freezed; dropping from KF insertion!" << endl;
        return false;
    }
    unsigned long nKFs = mpMap->KeyFramesInMap();

    // cout << mCurrentFrame.mnId << "; " << mnLastRelocFrameId << "; " << mMaxFrames << "; " << nKFs << endl;
    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if ( mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames ) {
        cout << "not enough frames have passed from last relocalisation; dropping from KF insertion!" << endl;
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;

    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    bool mbFastTurning = false;
    if(!mVelocity.empty()) {
        vector<float> q = ORB_SLAM2::Converter::toQuaternion(mVelocity.rowRange(0,3).colRange(0,3));
        // std::cout << "rotation vel = " << q[3] << std::endl;
        if (q[3] < 0.9998)
            mbFastTurning = true;
    }

#ifdef GROUND_TRUTH_GEN_MODE
    if(mnMatchesInliers < 80 || mnMatchesInliers<nRefMatches*0.25 || nFrameSinceLast > camera_fps / 2 || mbFastTurning) // 100)
    {
#else
    if(mnMatchesInliers < 80 || mnMatchesInliers<nRefMatches*0.25 || nFrameSinceLast > camera_fps || mbFastTurning) // 100)
    {
#endif
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

bool Tracking::NeedNewKeyFrame_Experimental()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.5f; // 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c = false && (mnMatchesInliers<nRefMatches*0.25) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            return false;
        }
    }
    else
        return false;
}

bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;
    unsigned long nKFs = mpMap->KeyFramesInMap();

    //    cout << mCurrentFrame.mnId << "; " << mnLastRelocFrameId << "; " << mMaxFrames << "; " << nKFs << endl;
    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if ( mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames )
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;

    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    // NOTE
    // this is the key condition for stereo / RGBD keyframe insertion
    // while setting a sensitive condition here resolves track loss & reloc,
    // the time cost could easily exploded;
    // for GF, an adaptive threshold is set according to good feature budget.
#if defined GOOD_FEATURE_MAP_MATCHING || defined RANDOM_FEATURE_MAP_MATCHING || defined LONGLIVE_FEATURE_MAP_MATCHING
    bool bNeedToInsertClose = (nTrackedClose * 3 < num_good_constr_predef * 0.6f) && (nNonTrackedClose > mCurrentFrame.N * 0.15f);
#else
    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);
#endif

    /*
#ifdef SPARSE_KEYFRAME_COND
    bNeedToInsertClose = (nTrackedClose<70) && (nNonTrackedClose>100);
#endif
*/

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    // cout << c1a << "; "  << c1b << "; "  << c1c << "; "  << c2 << "; " << endl;

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

#ifdef ENABLE_ANTICIPATION_IN_GRAPH

    // only update visible_mpt_num at this point
    pKF->mNumVisibleMpt = mNumVisibleMpt;

    // Update average matching number
    mpLocalMapper->mParam.avg_match_num = (mpLocalMapper->mParam.avg_match_num +
                                           logCurrentFrame.lmk_num_BA) / 2.0;

					   
#ifdef ENABLE_PLANNER_PREDICTION
    pKF->mvTrel.clear();
    size_t vn = 1;
    cv::Mat mVelocity_tmp = cv::Mat(4,4,CV_32F);
    
    while (vn <= VIRTUAL_FRAME_NUM) {
      // std::cout << "vn = " << vn << "; mVFrameInteval = " << mVFrameInteval << std::endl;
      if (PredictRelMotionFromCallback(pKF->mTimeStamp, pKF->mTimeStamp + vn * mVFrameInteval, mVelocity_tmp)) {
        pKF->mvTrel.push_back(Tb2c * mVelocity_tmp * Tc2b);
      }
      vn ++;
    } 
    
#else
    // NOTE
    // When pre-loading the full odom from file, query the relative motion with the following block
    //
    pKF->mvTrel.clear();
    size_t Nodom = mvOdomBuf.size(), vn = 0;
    // cout << "Nodom = " << Nodom << endl;
    //    cout << setprecision(12) << "pKF->mTimeStamp + VIRTUAL_KF_INTEVAL = " << pKF->mTimeStamp + VIRTUAL_KF_INTEVAL << endl;
    //    cout << setprecision(12) << "mvOdomPlanned[0].first = " << mvOdomPlanned[0].first << endl;
    //    cout << setprecision(12) << "mvOdomPlanned.back().first = " << mvOdomPlanned.back().first << endl;
    if (!mvOdomBuf.empty() &&
        pKF->mTimeStamp + mVFrameInteval >= mvOdomBuf[0].time_stamp &&
        pKF->mTimeStamp + mVFrameInteval <= mvOdomBuf.back().time_stamp)
    {
        // when given valid KF
        cv::Mat Twc_base;
        cv::Mat T_tmp;
        for (size_t i = mOdomLBAIdx; i < Nodom; ++i) {
            // cout << setprecision(12) << mvOdomBuf[i].time_stamp << " vs. " << pKF->mTimeStamp + vn * mVFrameInteval << endl;
            // find the closest odom to pKF->mTimeStamp + VIRTUAL_KF_INTEVAL * (vn+1)
            if (mvOdomBuf[i].time_stamp >= pKF->mTimeStamp + vn * mVFrameInteval) {
                if (vn == 0) {
                    mOdomLBAIdx = i;
                    //                    Tcw_base = mvOdomPlanned[i].second.inv();
                    cv::Mat Rwc = mvOdomBuf[i].Tcw.rowRange(0, 3).colRange(0, 3).t();
                    cv::Mat twc = -Rwc * mvOdomBuf[i].Tcw.rowRange(0, 3).col(3);
                    Twc_base = cv::Mat::eye(4, 4, CV_32F);
                    Rwc.copyTo(Twc_base.rowRange(0, 3).colRange(0, 3));
                    twc.copyTo(Twc_base.rowRange(0, 3).col(3));
                    //                    Tcw_base.rowRange(0, 3).colRange(0, 3) = mvOdomPlanned[i].second.rowRange(0, 3).colRange(0, 3).t();
                    //                    Tcw_base.rowRange(0, 3).col(3) = -mvOdomPlanned[i].second.rowRange(0, 3).colRange(0, 3) * mvOdomPlanned[i].second.rowRange(0, 3).col(3);

                    vn ++;
                    continue ;
                }

                // Trel = Tcw_current * Twc_prev
                cv::Mat mTrel = mvOdomBuf[i].Tcw * Twc_base;
                pKF->mvTrel.push_back(mTrel);
                vn ++;
                //                std::cout << setprecision(15) << "timeStamp planned: " << mvOdomPlanned[mOdomLBAIdx].first << " to " << mvOdomPlanned[i].first << std::endl;
                //                std::cout << setprecision(15) << "timeStamp actual: " << pKF->mTimeStamp << std::endl;
                //                std::cout << "mVel planned: " << std::endl
                //                          << mTrel << std::endl;
                //                std::cout << "mVel actual: " << std::endl
                //                          << mVelocity << std::endl;
            }
            if (vn > VIRTUAL_FRAME_NUM)
                break ;
        }
    }
#endif

    //    if (mpLocalMapper->mParam.match_ratio_log.size() < NUM_HISTORICAL_BUDGET) {
    //        // no fully loaded yet
    //        mpLocalMapper->mParam.match_ratio_log.push_back(double(logCurrentFrame.lmk_num_BA) / double(mNumVisibleMpt));
    //        mpLocalMapper->mParam.match_ratio_idx ++;
    //    }
    //    else {
    //        // start ring buffering
    //        mpLocalMapper->mParam.match_ratio_log[mpLocalMapper->mParam.match_ratio_idx % NUM_HISTORICAL_BUDGET] =
    //                double(logCurrentFrame.lmk_num_BA) / double(mNumVisibleMpt);
    //        mpLocalMapper->mParam.match_ratio_idx ++;
    //    }
    //    std::cout << "num of map used: " << mNumVisibleMpt
    //              << "; num of matching found: " << logCurrentFrame.lmk_num_BA
    //              << "; matching ratio: " << double(logCurrentFrame.lmk_num_BA) / double(mNumVisibleMpt) << std::endl;
#endif

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

bool Tracking::SearchAdditionalMatchesInFrame(const double time_for_search, Frame & F) {

    bool completed = true;
    if (mObsHandler == NULL || mObsHandler->mLeftMapPoints.size() == 0)
        return !completed;

    arma::wall_clock timer;
    timer.tic();

    if (mObsHandler->mbNeedVizCheck) {
        //
        for(vector<MapPoint*>::iterator vit=mObsHandler->mLeftMapPoints.begin(), vend=mObsHandler->mLeftMapPoints.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP->mnLastFrameSeen == F.mnId)
                continue;
            if(pMP->isBad())
                continue;

            if (timer.toc() > time_for_search / 2.0) {
                // std::cout << "func SearchAdditionalMatchesInFrame: early stop in visibility check!" << std::endl;
                mObsHandler->mLeftMapPoints.erase(vit, vend);
                completed = false;
                break;
            }

            // Project (this fills MapPoint variables for matching)
            if(F.isInFrustum(pMP,0.5))
            {
                pMP->IncreaseVisible();
                //
#ifdef ENABLE_ANTICIPATION_IN_BUDGET
                mNumVisibleMpt ++;
#endif
            }
        }
    }

    double time_so_far = timer.toc();

    ORBmatcher matcher(0.8);
    double th = 0.5; // 1; // 0.8; // 0.2; //
    if (mbTrackLossAlert == 1) {
        std::cout << "func SearchAdditionalMatchesInFrame: increase searching range to avoid track loss !!!" << std::endl;
        th = 1; // 2; // 1.6; // 0.4; //
    }

    int nMatched = matcher.SearchByProjection_Budget(F,mObsHandler->mLeftMapPoints,th,time_for_search-time_so_far);

    //    std::cout << "func SearchAdditionalMatchesInFrame: found " << nMatched
    //              << " additional matches from " << mObsHandler->mLeftMapPoints.size()
    //              << " map points, with total time cost = " << timer.toc() << std::endl;

    logCurrentFrame.lmk_num_BA += nMatched;

    return completed;
}

int Tracking::SearchLocalPoints()
{
    int nAlreadyMatched = mnInitStereo;
    //    int nAlreadyMatched = 0;

    arma::wall_clock timer;

#ifdef GOOD_FEATURE_MAP_MATCHING
    //    mCurrentInfoMat.zeros();
    mCurrentInfoMat = arma::eye( size(mCurrentInfoMat) ) * 0.00001;
    if (mFrameAfterInital > camera_fps * TIME_INIT_TRACKING && mCurrentFrame.mnId >= mnLastRelocFrameId+2) {
        //    cout << "update pose info in obs class" << endl;
        // NOTE
        // there is no need to do motion prediction again, since it's already be
        // predicted and somewhat optimzed in the 1st stage of pose tracking
        if(!mLastFrame.mTcw.empty()) {
            mObsHandler->updatePWLSVec(mLastFrame.mTimeStamp, mLastFrame.mTcw,
                                       mCurrentFrame.mTimeStamp, mCurrentFrame.getTwc());
        }
        //    cout << "propagate pose info in obs class" << endl;
        // NOTE
        // instead of using actual time between consequtive frames, we construct virtual frame with slightly longer horizon;
        // the motivation being: reducing the size of matrix to 2-segments (which is the minimim-size); meanwhile preserving the spectral property
        mObsHandler->predictPWLSVec( (mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp), 1 );

        mObsHandler->mKineIdx = 0;
        mObsHandler->mnFrameId = mCurrentFrame.mnId;

#ifdef FRAME_MATCHING_INFO_PRIOR
        //    mObsHandler->mMapPoints = &mCurrentFrame.mvpMapPoints;
        mObsHandler->pFrame = &mCurrentFrame;
        // compute info matrix for frame-by-frame matches
#ifdef USE_INFO_MATRIX
        mObsHandler->runMatrixBuilding(ORB_SLAM2::FRAME_INFO_MATRIX, MATRIX_BUDGET_REALTIME, USE_MULTI_THREAD, false);
#elif defined USE_HYBRID_MATRIX
        mObsHandler->runMatrixBuilding(ORB_SLAM2::FRAME_HYBRID_MATRIX, MATRIX_BUDGET_REALTIME, USE_MULTI_THREAD, false);
#elif defined USE_OBSERVABILITY_MATRIX
        // TODO
#endif

#endif
    }
#endif
    //    cout << mCurrentInfoMat << endl << endl;

    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;

#if defined GOOD_FEATURE_MAP_MATCHING && defined FRAME_MATCHING_INFO_PRIOR
                if (pMP->updateAtFrameId == mCurrentFrame.mnId)
                    mCurrentInfoMat += pMP->ObsMat;
#endif

                //#ifdef ENABLE_ANTICIPATION_IN_BUDGET
                //                mNumVisibleMpt ++;
                //#endif

                nAlreadyMatched += 2;
            }
        }
    }
    //    cout << mCurrentInfoMat << endl << endl;

    int nToMatch=0;

#if defined GOOD_FEATURE_MAP_MATCHING || defined RANDOM_FEATURE_MAP_MATCHING ||  defined LONGLIVE_FEATURE_MAP_MATCHING
    //
    mObsHandler->mLeftMapPoints.clear();
    mObsHandler->mbNeedVizCheck = false;
    double time_total_match = 0.015; // 1.0; //
    int num_to_match = this->num_good_constr_predef - nAlreadyMatched; // 50;  //
    if (num_to_match <= 0) {
        // skip the rest
        for (size_t i=0; i<mvpLocalMapPoints.size(); ++i) {
            if (mvpLocalMapPoints[i] == NULL)
                continue ;
            if (mvpLocalMapPoints[i]->isBad())
                continue ;
            if(mvpLocalMapPoints[i]->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if (mvpLocalMapPoints[i]->mbTrackInView == false)
                continue ;
            //
            mObsHandler->mLeftMapPoints.push_back(mvpLocalMapPoints[i]);
        }
        mObsHandler->mbNeedVizCheck = true;
        //
        return nAlreadyMatched;
    }

    double time_Viz = 0;
    timer.tic();
    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;

        time_Viz = timer.toc();
        if (time_Viz > time_total_match / 2.0) {
            //
            mObsHandler->mLeftMapPoints = vector<MapPoint*>(vit, vend);
            mvpLocalMapPoints.erase(vit, vend);
            mObsHandler->mbNeedVizCheck = true;
            //
            break ;
        }

        // Project (this fills MapPoint variables for matching)
        if (mCurrentFrame.isInFrustum(pMP, 0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
            //
#ifdef ENABLE_ANTICIPATION_IN_BUDGET
            mNumVisibleMpt ++;
#endif
        }
    }

    //    cout << "time_Viz = " << time_Viz << "; mvpLocalMapPoints.size() = " << mvpLocalMapPoints.size() << endl;

#else
    // Project points in frame and check its visibility
    for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend; vit++)
    {
        MapPoint *pMP = *vit;
        if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if (pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if (mCurrentFrame.isInFrustum(pMP, 0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
            //
#ifdef ENABLE_ANTICIPATION_IN_BUDGET
            mNumVisibleMpt ++;
#endif
        }
    }

#endif
    //
    // TODO
    // Here is the place to inject Obs computation; to reduce the load of computation, we might need to approximate exact point Obs with region;
    // Following are the time cost of each step in TrackLocalMap:
    //
    int nMatched = 0;
    if(nToMatch>0)
    {

#ifdef GOOD_FEATURE_MAP_MATCHING

        timer.tic();

        ORBmatcher matcher(0.8);

        int th = 1;  // 1.5; // NOTE try increase the window size for vicon seq
        if(mSensor==System::RGBD)
            th=3;

        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2) {
            th=5;
            nMatched = matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
        }
        else if (mFrameAfterInital <= camera_fps * TIME_INIT_TRACKING || nToMatch < 400) { // 800)
            nMatched = matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
            //            cout << "nToMatch = " << nToMatch << "; nMatched = " << nMatched << endl;
        }
        else {
            // TEST
            // try computing Jacobian for each map point
            mObsHandler->mMapPoints = &mvpLocalMapPoints;
#ifdef USE_INFO_MATRIX
            mObsHandler->runMatrixBuilding(ORB_SLAM2::MAP_INFO_MATRIX, (time_total_match-time_Viz)/2, USE_MULTI_THREAD, false);
#elif defined USE_HYBRID_MATRIX
            mObsHandler->runMatrixBuilding(ORB_SLAM2::MAP_HYBRID_MATRIX, (time_total_match-time_Viz)/2, USE_MULTI_THREAD, false);
#elif defined USE_OBSERVABILITY_MATRIX
            // TODO
#endif

            double time_Mat_Online = timer.toc();
            logCurrentFrame.time_mat_online = time_Mat_Online;
            //            std::cout << "func SearchReferencePointsInFrustum: time cost of matrix building = " << time_Mat_Online << endl;

#ifdef USE_INFO_MATRIX
            nMatched = mObsHandler->runActiveMapMatching(&mCurrentFrame, ORB_SLAM2::FRAME_INFO_MATRIX, mCurrentInfoMat,
                                                         th,matcher,num_to_match,time_total_match-time_Mat_Online-time_Viz);
#elif defined USE_HYBRID_MATRIX
            nMatched = mObsHandler->runActiveMapMatching(&mCurrentFrame, ORB_SLAM2::FRAME_HYBRID_MATRIX, mCurrentInfoMat,
                                                         th,matcher,num_to_match,time_total_match-time_Mat_Online-time_Viz);
#elif defined USE_OBSERVABILITY_MATRIX
            // TODO
#endif

        }
        double time_Match = timer.toc();
        //        std::cout << "func SearchReferencePointsInFrustum: found " << nMatched << " constraints (in total "
        //                  << nMatched + nAlreadyMatched <<  ") in " << time_Match * 1000 << " ms                                 " << endl;
        //        std::cout << "func SearchReferencePointsInFrustum: time cost of active matching = " << time_Match << "; matched feature = " << nMatched << endl;

#elif defined RANDOM_FEATURE_MAP_MATCHING ||  defined LONGLIVE_FEATURE_MAP_MATCHING

        timer.tic();

        ORBmatcher matcher(0.8);

        int th = 1;  // 1.5; // NOTE try increase the window size for vicon seq

        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2) {
            th=5;
            nMatched = matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
        }
        else if (mFrameAfterInital <= camera_fps * TIME_INIT_TRACKING || nToMatch < 400) { // 800)
            nMatched = matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
            cout << "nToMatch = " << nToMatch << "; nMatched = " << nMatched << endl;
        }
        else {
            // TEST
            // try computing Jacobian for each map point
            // print out the time cost
            // double time_total_match = 1.0; // 0.009; //

            mObsHandler->mMapPoints = &mvpLocalMapPoints;

            //
            // int num_to_match = this->num_good_inlier_predef - nMatchesFound; // 50;  //
#ifdef RANDOM_FEATURE_MAP_MATCHING
            nMatched = mObsHandler->runBaselineMapMatching(&mCurrentFrame, ORB_SLAM2::BASELINE_RANDOM,
                                                           th,matcher,num_to_match,time_total_match-time_Viz);
#else
            nMatched = mObsHandler->runBaselineMapMatching(&mCurrentFrame, ORB_SLAM2::BASELINE_LONGLIVE,
                                                           th,matcher,num_to_match,time_total_match-time_Viz);
#endif

        }
        double time_Match = timer.toc();
        std::cout << "func SearchReferencePointsInFrustum: time cost of active matching = " << time_Match << "; matched feature = " << nMatched << endl;


#else

        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;

#ifdef ENABLE_LARGE_SEARCH_WINDOW
        th *= 3;
#endif

        nMatched = matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);

#endif
    }

    return nAlreadyMatched + nMatched;
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    //    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
#ifdef LOCAL_SEARCH_USING_HASHING
    arma::wall_clock timer;
    timer.tic();
    //    UpdateReferencePointsByHashing();
    //        if(mCurrentFrame.mTimeStamp - timestamp_first_frame > 5.0)

    //    if (mpLocalMapper->stopRequested() || mpLocalMapper->isStopped())
    //    {
    //        UpdateLocalPointsByHashing(eLocalMapSet::CovisOnly);
    //    }
    //    else
    //    {

    if (mFrameAfterInital >= camera_fps * TIME_INIT_TRACKING || mvpLocalMapPoints.size() > MAP_SIZE_TRIGGER_HASHING )// || logCurrentFrame.lmk_num_frame > this->num_good_constr_predef)
    {
        // cout<<"Collect Local Map with Multi-Index Hasing!"<<endl;
        UpdateLocalPointsByHashing(eLocalMapSet::Combined);
    }
    else
    {
        //	    cout<<"Collect Local Map with Co-Visibility!"<<endl;
        UpdateLocalPointsByHashing(eLocalMapSet::CovisOnly);
    }
    //    }

    logCurrentFrame.time_hash_query = timer.toc();

#else
    UpdateLocalPoints();
#endif

    logCurrentFrame.lmk_localmap_comb = mvpLocalMapPoints.size();

#ifndef DISABLE_MAP_VIZ
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
#endif
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    unsigned long minFrameId = mCurrentFrame.mnId; // - 2; // - 1; //
    if (mbTrackLossAlert == 2)
        minFrameId = 0;

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {

#if defined GOOD_FEATURE_MAP_BOUND || defined RANDOM_MAP_BOUND
                //                if (pMP->goodAtFrameId > 0 || mvpLocalMapPoints.size() < 2000) {
                //                    acceptMapPoint = true;
                //                }
                if (pMP->goodAtFrameId >= minFrameId)
                {
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                }
                else
                {
                    // for those unselected map points, we still keep them and try matching them after current pose being published
                    mvpLocalAdditionalPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                }
#else
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
#endif

            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty()) {
        cout << "keyframeCounter is empty!" << endl;
        return;
    }

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            //            std::cout << pParent << std::endl;
            //            std::cout << pParent->mnTrackReferenceForFrame << std::endl;
            //            std::cout << mCurrentFrame.mnId << std::endl;

            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    cout << mpKeyFrameDB->mvpKFset.size() << endl;

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty()) {
        cout << "failed to find reloc candidate!" << endl;
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;

                    // cerr<<"start relocalize......"<<endl;
#ifdef LOCAL_SEARCH_USING_HASHING
                    // arma::wall_clock timer_relicalization;
                    // timer_relicalization.tic();
                    vector<MapPoint*> vpMapPoints;
                    KeyFrame* pKeyFrame = vpCandidateKFs[i];
                    vector<KeyFrame*> vpKeyFrames = pKeyFrame->GetBestCovisibilityKeyFrames(10);
                    for(vector<KeyFrame*>::iterator itKF=vpKeyFrames.begin(), itEndKF=vpKeyFrames.end(); itKF!=itEndKF; itKF++)
                    {
                        KeyFrame* pKF = *itKF;
                        vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

                        for(vector<MapPoint*>::iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
                        {
                            MapPoint* pMP = *itMP;
                            if(!pMP)        // already matched or outliers
                                continue;
                            if(pMP->mnIdRelocalized==mCurrentFrame.mnId)   // check if selected yet (avoid pushing back same MapPoints multiple times)
                                continue;
                            if(!pMP->isBad())
                            {
                                vpMapPoints.push_back(pMP);
                                pMP->mnIdRelocalized = mCurrentFrame.mnId;
                            }
                        }
                    }
                    mpLocalMapper->UpdateHashTables(vpMapPoints);

                    std::fill(mvpQueriedFeatures.begin(), mvpQueriedFeatures.begin() + NUM_ACTIVE_HASHTABLES, NUM_ADDITIONAL_FEATURES + NUM_CANDIDATE_FEATURES);
                    std::fill(mvpQueriedFeatures.begin() + NUM_ACTIVE_HASHTABLES, mvpQueriedFeatures.end(), 0);
#endif
                    // cerr<<"done relocalize, time cost = "<< timer_relicalization.toc() << "sec ......"<<endl;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        cout << "Reloc failed!" << endl;
        return false;
    }
    else
    {
        int vld_cnt = 0;
        for (int io = 0; io < mCurrentFrame.N; io++)
            if (mCurrentFrame.mvpMapPoints[io])
                vld_cnt++;
        cout << "Reloc succeed with " << vld_cnt << " valid map points!" << endl;

        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);

#ifdef USE_FISHEYE_DISTORTION
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.k3"];
    DistCoef.at<float>(3) = fSettings["Camera.k4"];
#else
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
#endif

    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}


// Hash ----------------------------------------------------------------------------
#ifdef LOCAL_SEARCH_USING_HASHING

void Tracking::UpdateLocalPointsByHashing(eLocalMapSet eLocalMap)
{
#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::UpdateReferencePointsByHashing()");
#endif

    //#ifdef TIMECOST_VERBOSE
    arma::wall_clock timer/*, timer1*/;
    double t;
    //#endif
    mbMapHashOTS = false;

    timer.tic();

    if (eLocalMap == eLocalMapSet::CovisOnly || eLocalMap == eLocalMapSet::Combined) {
        //
#ifdef TIMECOST_VERBOSE
        timer.tic();
#endif
        // grab the local map from co-visibility
        mvpLocalMapPoints.clear();

        for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
        {
            KeyFrame* pKF = *itKF;
            vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

            for(vector<MapPoint*>::iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
            {
                MapPoint* pMP = *itMP;
                if(!pMP)        // already matched or outliers
                    continue;
                if(pMP->mnIdCoVisible==mCurrentFrame.mnId)   // check if selected yet (avoid pushing back same MapPoints multiple times)
                    continue;
                if(!pMP->isBad())
                {
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnIdCoVisible = mCurrentFrame.mnId;
                    //                    pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;

                    //                    // flow from co-visibility => maphash
                    //                    timer1.tic();
                    //                    mpHashMethod->insert(pMP);
                    //                    t += timer1.toc();
                }
            }
        }

        logCurrentFrame.lmk_localmap_covis = mvpLocalMapPoints.size();
        StoreLocalMapPointsByCoVis(mvpLocalMapPoints);

#ifdef TIMECOST_VERBOSE
        double time_1 = timer.toc();
        std::cout << "func UpdateReferencePointsByHashing: size of co-vis map = " << mvpLocalMapPoints.size() << std::endl;
        std::cout << "func UpdateReferencePointsByHashing: time cost of pulling out co-visibility local map = " << time_1 << std::endl;
#endif

        if (mvpLocalMapPoints.size() < /*th_trigger_hashing*/MAP_SIZE_TRIGGER_HASHING) {
            // update flag for final local map
            for(vector<MapPoint*>::iterator itMP=mvpLocalMapPoints.begin(), itEndMP=mvpLocalMapPoints.end(); itMP!=itEndMP; itMP++)
            {
                MapPoint* pMP = *itMP;
                if(!pMP || pMP->isBad())    // not matched yet
                    continue;
                if(pMP->mnTrackReferenceForFrame != mCurrentFrame.mnId)   //
                    pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;

                //
                //                pMP->matchedByTrackLocalMap = false;
            }

            //            std::cout << "func UpdateReferencePointsByHashing: size of co-vis map = " << mvpLocalMapPoints.size() << std::endl;
            return ;
        }

    }

    //    cerr<<"Time cost:"<<timer.toc()<<", ---------------------------------"<<endl;

    if (eLocalMap == eLocalMapSet::HashOnly || eLocalMap == eLocalMapSet::Combined) {
        mbMapHashTriggered = true;
        //
        // grab the local map from map hash
        cv::Mat mDescriptors = mCurrentFrame.mDescriptors;
        //    std::vector<MapPoint*> vpMP = mCurrentFrame.mvpMapPoints;
        std::vector<unsigned int> vKPU; // keypoint indices which need to be matched to LocalMapPoints

        //#ifdef TIMECOST_VERBOSE
        // timer.tic();
        //#endif

        Frame::GetUnMatchedKPbyBucketing(&mCurrentFrame, vKPU);

        //    bool hit[pMIH->getHashTableSize()][pMIH->getBucketNum()] = {0}; // only works for false initialization
        //    std::vector<std::vector<size_t> > hitid;
        //    hitid.clear();

        mvpLeftMapPointsByHashing.clear();
        mvpMapPointsByHashing.clear();

        //        std::cout << "before iterating vKPU" << std::endl;
        t = 0.;
        for(auto i:vKPU) {
            //
            unsigned int* pDesc = mDescriptors.ptr<unsigned int>(i);
            std::vector<MapPoint*> candidates;
            std::vector<MapPoint*> additional_candidates;

            //            std::cout << "before query vKPU" << std::endl;

#ifdef ONLINE_TABLE_SELECTION
            mbMapHashOTS = true;
#endif

            timer.tic();
#ifdef DELAYED_MAP_MATCHING
            mpHashMethod->query(pDesc, candidates, additional_candidates, mvpQueriedFeatures);
            mvpLeftMapPointsByHashing.insert(mvpLeftMapPointsByHashing.end(), additional_candidates.begin(), additional_candidates.end());
#else
            // retrieve up to NUM_CANDIDATE_FEATURES x NUM_ACTIVE_HASHTABLES candidates
            mpHashMethod->query(pDesc, candidates, NUM_ACTIVE_HASHTABLES, NUM_CANDIDATE_FEATURES);
#endif
            t += timer.toc();

            for(vector<MapPoint*>::iterator itMP=candidates.begin(), itEndMP=candidates.end(); itMP!=itEndMP; itMP++)
            {
                MapPoint* pMP = *itMP;
                if(!pMP)
                    continue;
                if(pMP->mnIdMapHashed==mCurrentFrame.mnId)
                    continue;
                if(!pMP->isBad())
                {
                    mvpMapPointsByHashing.push_back(pMP);
                    pMP->mnIdMapHashed = mCurrentFrame.mnId;
                }
            }
        }

        logCurrentFrame.time_hash_query = t;
        logCurrentFrame.lmk_localmap_hash = mvpMapPointsByHashing.size();

        //        std::cout << "done iterating vKPU" << std::endl;
        //        std::sort(mvpHashMapPoints.begin(), mvpHashMapPoints.end(), sortMapPoint_ascend);

#ifdef TIMECOST_VERBOSE
        double time_2 = timer.toc();
        std::cout << "func UpdateReferencePointsByHashing: size of hashed map = " << mvpMapPointsByHashing.size() << std::endl;
        std::cout << "func UpdateReferencePointsByHashing: time cost of pulling out hash local map = " << time_2 << std::endl;
#endif
    }


#ifdef TIMECOST_VERBOSE
    timer.tic();
#endif

    if (eLocalMap == eLocalMapSet::Combined) {
        // set operation of both local maps
        //
        vector<MapPoint *> mvpInterMapPoints;
        for(vector<MapPoint*>::iterator itMP=mvpLocalMapPoints.begin(), itEndMP=mvpLocalMapPoints.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP || pMP->isBad())    // not matched yet
                continue;
            if(pMP->mnIdMapHashed == mCurrentFrame.mnId) {
                mvpInterMapPoints.push_back(pMP);
            }
            else {
                mvpLeftMapPointsByHashing.push_back(pMP);
            }
        }

        // copy from the intersected result
        mvpLocalMapPoints.clear();
        mvpLocalMapPoints.swap(mvpInterMapPoints);
    }
    else {

        if (eLocalMap == eLocalMapSet::HashOnly) {
            // simply copy from hashed result
            mvpLocalMapPoints.clear();
            mvpLocalMapPoints.swap(mvpMapPointsByHashing);
        }
    }

    // update flag for final local map
    for(vector<MapPoint*>::iterator itMP=mvpLocalMapPoints.begin(), itEndMP=mvpLocalMapPoints.end(); itMP!=itEndMP; itMP++)
    {
        MapPoint* pMP = *itMP;
        if(!pMP || pMP->isBad())    // not matched yet
            continue;
        if(pMP->mnTrackReferenceForFrame != mCurrentFrame.mnId)
            pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;

        //        pMP->matchedByTrackLocalMap = false;
    }


#ifdef TIMECOST_VERBOSE
    double time_3 = timer.toc();
    std::cout << "func UpdateReferencePointsByHashing: size of final local map = " << mvpLocalMapPoints.size() << std::endl;
    std::cout << "func UpdateReferencePointsByHashing: time cost of set operations on both local maps = " << time_3 << std::endl;
#endif

#ifdef MAPHASH_VERBOSE
    if(mbMapHashTriggered)
        cerr << "MapHash is triggered";
    if(mbMapHashOTS)
        cerr << " and Online Table Selection is triggered as well";
    cerr << "!!!-------------"<<endl;
#endif

}

bool Tracking::UpdateQueryNumByHashTable(const double time_limit)
{
    // data struct for table-to-MapPoint querying
    std::vector<std::vector<MapPoint*> > mvvpMapPointsPerTable(NUM_TOTAL_HASHTABLES, std::vector<MapPoint*>());
    std::vector<arma::mat> InfoMatrices(NUM_TOTAL_HASHTABLES, arma::eye( size(mCurrentInfoMat) ) * EPS);
    arma::wall_clock timer;
    timer.tic();

    // sample subset of mvpMapPointsByHashing here to reduce the time cost
    //    std::srand(std::time(0));
    //    std::vector<int> viRandSamples(NUM_RANDOM_SAMPLES);
    //    std::iota(viRandSamples.begin(), viRandSamples.end(), 0);

    // calculate infoMat for each hash table
    //    for(auto it=mvpMapPointsByHashing.begin(), itend=mvpMapPointsByHashing.end(); it!=itend; it++){
    for(size_t i=0; i<mCurrentFrame.mvpMapPoints.size(); ++i){
        //
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        //
        if(!pMP)        // already matched or outliers
            continue;
        if(pMP->isBad())
            continue;

        if(pMP->mnLastFrameSeen != mCurrentFrame.mnId)   // check if visible
            continue;

        if(pMP->mnIdSelected == mCurrentFrame.mnId)   // check if selected yet (avoid pushing back same MapPoints multiple times)
            continue;

        // get rid of the mvbJacobain flag check; be consistent with the rest of the pipeline
        if (pMP->updateAtFrameId != mCurrentFrame.mnId)
            continue ;

        if ( mCurrentFrame.mvbOutlier[i] == false && mCurrentFrame.mvbCandidate[i] == true ) {

            const cv::Mat mDescriptors = mCurrentFrame.mDescriptors;

            // update activeHTs flags for each matched MapPoints
            // const void *pDesc1 = mDescriptors.ptr<unsigned int>(i);    // desc in current frame
            const cv::Mat &dKF = mDescriptors.row(i);
            // void *pDesc2 = pMP->GetDescriptor().ptr<unsigned int>(0);    // desc of matched MapPoint
            const cv::Mat &dMP = pMP->GetDescriptor();
            // const void *pDesc2 = &dMP;
            const void *pDesc1, *pDesc2;
            //
            if(mpHashMethod->getSubstringLen() == 8) {
                pDesc1 = dKF.ptr<int8_t>();
                pDesc2 = dMP.ptr<int8_t>();
            }
            else if(mpHashMethod->getSubstringLen() == 16) {
                pDesc1 = dKF.ptr<int16_t>();
                pDesc2 = dMP.ptr<int16_t>();
            }
            else {
                cerr<<"ERROR: unsupported substring length!!!"<<endl;
                exit(-1);
            }

            // cout << "getSubstringLen = " << mpHashMethod->getSubstringLen() << endl;
            // cout << "mDescriptors.cols = " << mDescriptors.cols << endl;
            // cout << "dKF.cols = " << dKF.cols << endl;
            // cout << "dMP.cols = " << dMP.cols << endl;
            // cout << "full desc. dist. = " << ORBmatcher::DescriptorDistance(dKF, dMP) << endl;
            // cout << "pMP = " << pMP << endl;
            // cout << "pMP->mvbActiveHashTables.size() = " << pMP->mvbActiveHashTables.size() << endl;

            for(int k=0; k<NUM_TOTAL_HASHTABLES; k++){

                // DEBUG
                // cout << pDesc1 << "; " << ((int8_t *)pDesc1)[k] << endl;
                // cout << pDesc2 << "; " << ((int8_t *)pDesc2)[k] << endl;
                // cout << "mvbActiveHashTables[k] = " << pMP->mvbActiveHashTables[k] << endl;

                if(mpHashMethod->getSubstringLen() == 8){
                    if(((int8_t *)pDesc1)[k] == ((int8_t *)pDesc2)[k])
                        pMP->mvbActiveHashTables[k] = true;
                }
                else if(mpHashMethod->getSubstringLen() == 16)
                {
                    if(((int16_t *)pDesc1)[k] == ((int16_t *)pDesc2)[k])
                        pMP->mvbActiveHashTables[k] = true;
                }
                else{
                    cerr<<"ERROR: unsupported substring length!!!"<<endl;
                    exit(-1);
                }

                if(pMP->mvbActiveHashTables[k]){
                    //                                        if (pMP->updateAtFrameId == mCurrentFrame.mnId){
                    //                    try {
                    // NOTE
                    // for some reason the obsmatrix would occassionaly go corrupted
                    // at this stage (while everything is fine when build the matrix)
                    // as a consequence, we have to do the stupic check here to get
                    // around; later on a careful debug is required!
                    //
                    if (pMP->ObsMat.n_cols != DIMENSION_OF_STATE_MODEL) {
                        //                        cout << "something went wrong with Obs Matrix!!!" << endl;
                        continue;
                    }
                    InfoMatrices[k] = InfoMatrices[k] + pMP->ObsMat;
                    mvvpMapPointsPerTable[k].push_back(pMP);
                    //
                }
            }

            pMP->mnIdSelected = mCurrentFrame.mnId;
        }
    }

    //    cout<<fixed<<setprecision(6)<<mCurrentFrame.mTimeStamp<<",";
    //    for(int k=0; k!=NUM_TOTAL_HASHTABLES; k++){
    //        //            cout << "sum info matrix for table " << k << endl;
    //        //            cout << InfoMatrices[k]/* << endl*/;
    //        for(int i=0; i!=InfoMatrices[k].n_rows; i++)
    //            for(int j=0; j!=InfoMatrices[k].n_cols; j++)
    //                cout<<InfoMatrices[k](i,j)<<",";
    //    }
    //    cout<<endl;
    //    cout<<"Time taken to calculate the info mat for each hashtable:"<<timer.toc()<<endl;

    // data struct for left hash table indices
    std::vector<int> mviHashTableIndices(NUM_TOTAL_HASHTABLES);
    std::iota(mviHashTableIndices.begin(), mviHashTableIndices.end(), 0);
    //    std::random_shuffle ( mviHashTableIndices.begin(), mviHashTableIndices.end() );

#ifdef TRACKING_VERBOSE
    cout << "init: " ;
    for(int i=0; i<mviHashTableIndices.size(); ++i)
        std::cout << mviHashTableIndices[i] << ' ';
    cout<<endl;
#endif

    // data struct for updating #additional_candidates per table
    std::vector<int> queriedfeatures(NUM_TOTAL_HASHTABLES);
    std::fill(queriedfeatures.begin(), queriedfeatures.end(), 0);

    arma::mat CurMat = arma::eye( size(mCurrentInfoMat) ) * 0.00001;

    bool completed = false;

#ifdef MAPHASH_OTS_VERBOSE
    cerr<<fixed<<setprecision(6)<<mCurrentFrame.mTimeStamp<<","<<setprecision(0);
#endif

    while(!completed){

        // check time_rest
        if(time_limit - timer.toc() < 0)
            break;

        std::vector<double> mvdDet(mviHashTableIndices.size());

        // calculate CurDet for each hash table
        for(int k=0; k!=mviHashTableIndices.size(); k++)
            mvdDet[k] = ORB_SLAM2::logDet(CurMat + InfoMatrices[mviHashTableIndices[k]]);

        // get the table ID with max logDet
        int max_id = std::distance(mvdDet.begin(), std::max_element(mvdDet.begin(), mvdDet.end()));
        int max_tableid = mviHashTableIndices[max_id];

#ifdef MAPHASH_OTS_VERBOSE
        cerr<<"(" << max_tableid << ": " << mvdDet[max_id] << "),";
#endif

        // update CurMat
        CurMat += InfoMatrices[max_tableid];

        // update
        queriedfeatures[max_tableid] = NUM_ADDITIONAL_FEATURES + NUM_CANDIDATE_FEATURES;

        // remove MapPoints in the selected table
        mviHashTableIndices[max_id] = mviHashTableIndices.back();
        //
        for(auto it=mvvpMapPointsPerTable[max_tableid].begin(), itend = mvvpMapPointsPerTable[max_tableid].end(); it!=itend; it++){
            MapPoint* pMP = *it;

            // make sure no shitty hash records are utilized
            if(!pMP)
                continue;
            if(pMP->isBad())
                continue;

            if(pMP->mnIdSelected != mCurrentFrame.mnId) // in case of deleting the same point multiple times
                continue;

            // NOTE
            // for some reason the obsmatrix would occassionaly go corrupted
            // at this stage (while everything is fine when build the matrix)
            // as a consequence, we have to do the stupic check here to get
            // around; later on a careful debug is required!
            //
            if (pMP->ObsMat.n_cols != DIMENSION_OF_STATE_MODEL)
                continue;

            //                for(int k=0; k!=NUM_TOTAL_HASHTABLES; k++){
            // NOTE
            // the last table is not processed here, since it is the one
            // being selected (therefore won't be used in next iteration)
            //
            for(int k=0; k<mviHashTableIndices.size()/*-1*/; k++){
                // for each hash table that this mappoint is queried from
                if(pMP->mvbActiveHashTables[mviHashTableIndices[k]]){
                    pMP->mvbActiveHashTables[mviHashTableIndices[k]] = false;    // reset

                    if(k != mviHashTableIndices.size()-1)
                        InfoMatrices[mviHashTableIndices[k]] = InfoMatrices[mviHashTableIndices[k]] - pMP->ObsMat;
                }
            }

            pMP->mnIdSelected = 0;
        }

        mviHashTableIndices.pop_back();

#ifdef TRACKING_VERBOSE
        cout << "iter " << NUM_TOTAL_HASHTABLES - mviHashTableIndices.size() << ": ";
        for(int i=0; i<mviHashTableIndices.size(); ++i)
            std::cout << mviHashTableIndices[i] << ' ';
        cout<<endl;
#endif

        // add by Yipu
        // I guess the next step would be using the logDet
        // as the stopping condition instead?
        // select best NUM_ACTIVE_HASHTABLES(12) hash tables to query additional points
        //            if(mviHashTableIndices.size() <= NUM_TOTAL_HASHTABLES-NUM_ACTIVE_HASHTABLES)
        //                completed = true;
        if(NUM_TOTAL_HASHTABLES - mviHashTableIndices.size() == MAX_NUM_HASHTABLES){
            // TODO(optional): increase the upper limit for number of hash tables
            // if(mvdDet[max_id] < MIN_DETERMINE_THRESHOLD)

            completed = true;
            continue;
        }

        if(NUM_TOTAL_HASHTABLES - mviHashTableIndices.size() >= MIN_NUM_HASHTABLES && mvdDet[max_id] >= /*th_logDet*/MIN_DETERMINE_THRESHOLD)
            completed = true;

    }

    // update only if all the NUM_ACTIVE_HASHTABLES(12) hash tables are determined
    if(completed)
        mvpQueriedFeatures.swap(queriedfeatures);

#ifdef MAPHASH_OTS_VERBOSE
    cerr<<"---------"<<endl;
#endif

    return completed;
}

#endif


// DEBUG ----------------------------------------------------------------------------

void Tracking::PlotFrameWithPointMatches() {

    // create new image to modify it
    cv::Mat img_l_aux;
    mImGray.copyTo( img_l_aux );
    if( img_l_aux.channels() == 1 )
        cv::cvtColor(img_l_aux,img_l_aux,CV_GRAY2BGR);

    // Variables
    cv::Point       p;
    double          thick = 2.0; // 1.0;
    size_t match_counter = 0;
    std::string frame_wid;
    long time_stamp = mCurrentFrame.mTimeStamp * pow(10, 9);

    vector<GoodPoint>  mPtObs;

    // Collect the mpSorted
    for(size_t i=0; i<mCurrentFrame.mvpMapPoints.size(); i++)  {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if(pMP) {
            //            if (mCurrentFrame.mvbOutlier[i] == false) {
            GoodPoint tmpLmk(static_cast<size_t>(i), double(mCurrentFrame.mvpMatchScore[i]));
            mPtObs.push_back( tmpLmk );
            //            }
        }
    }

    // Sort features according to the orb matching score
    //    std::sort(mPtObs.begin(), mPtObs.end(), GoodPoint::rankObsScore_ascend);


    // for(size_t i=0; i<mCurrentFrame.mvKeys.size(); i++)  {
    //     cv::KeyPoint kpUn = mCurrentFrame.mvKeys[i];
    //     p = cv::Point( int(kpUn.pt.x),
    //                    int(kpUn.pt.y) );
    //     //
    //     cv::drawMarker( img_l_aux, p, cv::Scalar(255,0,0), cv::MARKER_CROSS, 10, 2);
    // }

    //
    for(size_t i =0; i<mPtObs.size(); i++)
    {
        cv::KeyPoint kpUn = mCurrentFrame.mvKeys[mPtObs[i].idx];
        p = cv::Point( int(kpUn.pt.x),
                       int(kpUn.pt.y) );
        //
        //        if(mCurrentFrame.mvbOutlier[mPtObs[i].idx])
        //        {
        //            cv::circle( img_l_aux, p, 3.0, cv::Scalar(0,0,255), thick);
        //        }
        //        else
        //        {
        //            cv::circle( img_l_aux, p, 3.0, cv::Scalar(0,255,0), thick);
        //        }
        cv::circle( img_l_aux, p, 3.0, cv::Scalar(0,255,0), thick);

        match_counter ++;

        //
        //        if (match_counter == 50)
        //        {
        //            frame_wid = "/mnt/DATA/tmp/ScreenShot/";
        //            frame_wid += std::to_string(time_stamp);
        //            cv::imwrite( frame_wid + "_m050.png", img_l_aux );
        //        }
        //        else if (match_counter == 100)
        //        {
        //            frame_wid = "/mnt/DATA/tmp/ScreenShot/";
        //            frame_wid += std::to_string(time_stamp);
        //            cv::imwrite( frame_wid + "_m100.png", img_l_aux );
        //        }
        //        else if (match_counter == 150)
        //        {
        //            frame_wid = "/mnt/DATA/tmp/ScreenShot/";
        //            frame_wid += std::to_string(time_stamp);
        //            cv::imwrite( frame_wid + "_m150.png", img_l_aux );
        //        }
    }

    // cv::putText(img_l_aux,
    //             "Keypoints Detected: " + std::to_string(mCurrentFrame.N) + "; Good Features Matched: " + std::to_string(match_counter),
    //             cv::Point(10, 470), // Coordinates
    //             cv::FONT_HERSHEY_PLAIN, // Font
    //             1.5, // Scale. 2.0 = 2x bigger
    //             cv::Scalar(0,0,255), // Color
    //             2.0); // Thickness
    cv::putText(img_l_aux,
                "Features Matched: " + std::to_string(match_counter),
                cv::Point(10, 470), // Coordinates
                cv::FONT_HERSHEY_PLAIN, // Font
                1.5, // Scale. 2.0 = 2x bigger
                cv::Scalar(0,0,255), // Color
                2.0); // Thickness

    //
    frame_wid = "/mnt/DATA/tmp/Demo/ORB_blur/";
    frame_wid += std::to_string(time_stamp);
    cv::imwrite( frame_wid + ".png", img_l_aux );

    //    ++ frame_counter;
}

} // namespace ORB_SLAM2
