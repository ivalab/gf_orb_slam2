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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_srvs/SetBool.h>

#include <opencv2/core/core.hpp>
#include "../../../../include/System.h"
#include "../../../../include/MapPublisher.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

//
//#include "path_smoothing_ros/cubic_spline_interpolator.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/publisher.h>


using namespace std;

// #define MAP_PUBLISH

#define FRAME_WITH_INFO_PUBLISH


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM), tfli_(tf_buffer_) {
#ifdef MAP_PUBLISH
      mnMapRefreshCounter = 0;
#endif
    }

    bool HandlePauseRequest(std_srvs::SetBool::Request&  req,
                            std_srvs::SetBool::Response& res);

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    
    void GrabOdom(const nav_msgs::Odometry::ConstPtr& msg);

    void GrabPath(const nav_msgs::Path::ConstPtr    & msg);

    void PostProcess(const ros::Time& time, const geometry_msgs::Pose& wTc);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    
    double timeStamp;
    cv::Mat Tmat;

    ros::ServiceServer service;
    bool paused = false;


    ros::Publisher mpCameraPosePublisher, mpCameraPoseInIMUPublisher;
    //    ros::Publisher mpDensePathPub;
    
    bool enable_map_to_odom_tf{false};
    ros::Publisher camera_path_publisher, camera_pose_publisher;
    nav_msgs::Path camera_path;
    std::string map_frame_{"map"};
    std::string odom_frame_{"odom"};
    std::string camera_frame_{"camera"};
    std::string base_frame_{"base_footprint"};
    std::string fixed_frame_{"fixed"};
    tf2_ros::StaticTransformBroadcaster static_br_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformBroadcaster tfbr_;
    tf2_ros::TransformListener tfli_;
    
#ifdef MAP_PUBLISH
    size_t mnMapRefreshCounter;
    ORB_SLAM2::MapPublisher* mpMapPub;
#endif

#ifdef FRAME_WITH_INFO_PUBLISH
    ros::Publisher mpFrameWithInfoPublisher;
#endif

    // Added by yanwei, save tracking latency
    std::vector<double> vTimesTrack;
    std::vector<pair<double, double> > vStampedTimesTrack;

    void saveStats(const std::string& path_traj)
    {
        // Tracking time statistics
        sort(vTimesTrack.begin(),vTimesTrack.end());
        float totaltime = 0;
        int proccIm = vTimesTrack.size();
        for(int ni=0; ni<proccIm; ni++)
        {
            totaltime+=vTimesTrack[ni];
        }

        // save to stats
        {
            std::ofstream myfile(path_traj + "_stats.txt");
            myfile << std::setprecision(6) << -1 << " "
                << proccIm << " "
                << totaltime / proccIm << " "
                << vTimesTrack[proccIm/2] << " "
                << vTimesTrack.front() << " "
                << vTimesTrack.back() << " ";
            myfile.close();

            myfile.open(path_traj + "_Log_Latency.txt");
            myfile << std::setprecision(20);
            for (const auto& m : vStampedTimesTrack)
            {
                myfile << m.first << " " << m.second << "\n";
            }
            myfile.close();
        }
    }

    Eigen::Matrix4d Ros2Eigen(const geometry_msgs::Transform& tf);
    Eigen::Matrix4d Ros2Eigen(const geometry_msgs::Pose& pose);
    geometry_msgs::Transform Eigen2Ros(const Eigen::Matrix4d& pose);
    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::start();

    if(argc != 10)
    {
        cerr << endl << "Usage: rosrun gf_orb_slam2 Stereo path_to_vocabulary path_to_settings budget_per_frame "
             << " do_rectify do_viz "
             << " topic_img_l topic_img_r path_to_traj path_to_map" << endl;
        ros::shutdown();
        return 1;
    }

    bool do_viz;
    stringstream s1(argv[5]);
    s1 >> boolalpha >> do_viz;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,do_viz);

    SLAM.SetConstrPerFrame(std::atoi(argv[3]));

    // convert budget input from ms to sec
    // SLAM.SetBudgetPerFrame(FLAGS_budget_per_frame*1e-3);
    

#ifdef LOGGING_KF_LIST
    std::string fNameRealTimeBA = std::string(argv[8]) + "_Log_BA.txt";
    std::cout << std::endl << "Saving BA Log to Log_BA.txt" << std::endl;
#endif

#ifdef REALTIME_TRAJ_LOGGING
    std::string fNameRealTimeTrack = std::string(argv[8]) + "_AllFrameTrajectory.txt";
    std::cout << std::endl << "Saving AllFrame Trajectory to AllFrameTrajectory.txt" << std::endl;
    SLAM.SetRealTimeFileStream(fNameRealTimeTrack);
#endif

#ifdef ENABLE_MAP_IO
    SLAM.LoadMap(std::string(argv[9]));
    // SLAM.LoadMap(std::string(argv[8]) + "_Map/");
    //
    SLAM.ForceRelocTracker();
    // SLAM.ForceInitTracker();
#endif

    ImageGrabber igb(&SLAM);

    stringstream s2(argv[4]);
    s2 >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {
	// Load settings related to stereo calibration
	cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
	if(!fsSettings.isOpened())
	{
	    cerr << "ERROR: Wrong path to settings" << endl;
	    return -1;
	}

	cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
	fsSettings["LEFT.K"] >> K_l;
	fsSettings["RIGHT.K"] >> K_r;

	fsSettings["LEFT.P"] >> P_l;
	fsSettings["RIGHT.P"] >> P_r;

	fsSettings["LEFT.R"] >> R_l;
	fsSettings["RIGHT.R"] >> R_r;

	fsSettings["LEFT.D"] >> D_l;
	fsSettings["RIGHT.D"] >> D_r;

	int rows_l = fsSettings["LEFT.height"];
	int cols_l = fsSettings["LEFT.width"];
	int rows_r = fsSettings["RIGHT.height"];
	int cols_r = fsSettings["RIGHT.width"];

	if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
		rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
	{
	    cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
	    return -1;
	}

#ifdef USE_FISHEYE_DISTORTION
	cv::fisheye::initUndistortRectifyMap(K_l,D_l,R_l,P_l,cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
	cv::fisheye::initUndistortRectifyMap(K_r,D_r,R_r,P_r,cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
	cout << "finish creating equidistant rectification map!" << endl;
#else
	cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
	cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
	cout << "finish creating rad-tan rectification map!" << endl;
#endif

    }

    ros::NodeHandle nh;

    // message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    // message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "camera/right/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, argv[6], 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, argv[7], 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;

    // NOTE
    // default sync config for real stereo rig
    // message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    // gazebo simulated stereo rig
    message_filters::Synchronizer<sync_pol> sync(sync_pol(2), left_sub, right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));
    
    //
    ros::Subscriber sub = nh.subscribe("/odom", 100, &ImageGrabber::GrabOdom, &igb);
    // ros::Subscriber sub = nh.subscribe("/desired_path", 100, &ImageGrabber::GrabPath, &igb);
    //    igb.mpDensePathPub = nh.advertise<nav_msgs::Path>("/dense_path", 100);
    
    // TODO
    // figure out the proper queue size
    igb.mpCameraPosePublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ORB_SLAM/camera_pose", 100);
    igb.mpCameraPoseInIMUPublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ORB_SLAM/camera_pose_in_imu", 100);

    // Service to pause the node.
    igb.service = nh.advertiseService("/pause_slam", &ImageGrabber::HandlePauseRequest, &igb);
    igb.paused = false;


    if (igb.enable_map_to_odom_tf)
    {
        igb.camera_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/gfgg/pose", 10);
        igb.camera_path_publisher = nh.advertise<nav_msgs::Path>("/gfgg/path", 10);
    }

#ifdef FRAME_WITH_INFO_PUBLISH
    igb.mpFrameWithInfoPublisher = nh.advertise<sensor_msgs::Image>("ORB_SLAM/frame_with_info", 100);
#endif

#ifdef MAP_PUBLISH
    igb.mpMapPub = new ORB_SLAM2::MapPublisher(SLAM.mpMap);
    ROS_INFO_STREAM("Initialized map publisher");
#endif

    while(ros::ok())
        ros::spin();
    // ros::spin();

    cout << "ros_stereo: done with spin!" << endl;

    // save stats
    igb.saveStats(std::string(argv[8])); 

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM( std::string(argv[8]) + "_KeyFrameTrajectory.txt" );
    SLAM.SaveTrackingLog( std::string(argv[8]) + "_Log.txt" );
#ifdef LOCAL_BA_TIME_LOGGING
    SLAM.SaveMappingLog( std::string(argv[8]) + "_Log_Mapping.txt" );
#endif
    //
    // SLAM.SaveLmkLog( std::string(argv[8]) + "_Log_lmk.txt" );

/*
#ifdef ENABLE_MAP_IO

// #ifdef MAP_PUBLISH
// publish map points
//  for (size_t i=0; i<10; ++i)
//   igb.mpMapPub->Refresh();
// #endif
  
  SLAM.SaveMap(std::string(argv[9]));
  // SLAM.SaveMap(std::string(argv[8]) + "_Map/");
  
#endif
*/

    std::cout << "Finished saving!" << std::endl;

    ros::shutdown();

    cout << "ros_stereo: done with ros Shutdown!" << endl;

    // Stop all threads
    SLAM.Shutdown();
    cout << "ros_stereo: done with SLAM Shutdown!" << endl;

    return 0;
}

bool ImageGrabber::HandlePauseRequest(std_srvs::SetBool::Request&  req,
                                      std_srvs::SetBool::Response& res) {
    paused      = req.data;
    res.success = true;
    res.message = paused ? "GFGG Paused" : "GFGG Resumed";
    return true;
}

void ImageGrabber::GrabOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    /*
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    */
    
    // TODO
    timeStamp = msg->header.stamp.toSec();
    
    mpSLAM->mpTracker->BufferingOdom(
                timeStamp,
                msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z,
                msg->pose.pose.orientation.w,
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z
                );
}

void ImageGrabber::GrabPath(const nav_msgs::Path::ConstPtr& msg) {
    /*
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    */

    //    // densify the path
    //    // create a cubic spline interpolator
    //    nav_msgs::Path path_dense;
    //    // pointsPerUnit, skipPoints, useEndConditions, useMiddleConditions);
    //    path_smoothing::CubicSplineInterpolator csi(double(100.0),
    //                                                (unsigned int)0,
    //                                                true,
    //                                                true);
    //    csi.interpolatePath(*msg, path_dense);

    size_t N = msg->poses.size();
    //    ROS_INFO("Size of path: before [%d] vs. after [%d]", msg->poses.size(), N);
    for (size_t i=0; i<N; ++i) {

        timeStamp = msg->poses[i].header.stamp.toSec();
        mpSLAM->mpTracker->BufferingOdom(
                    timeStamp,
                    msg->poses[i].pose.position.x,
                    msg->poses[i].pose.position.y,
                    msg->poses[i].pose.position.z,
                    msg->poses[i].pose.orientation.w,
                    msg->poses[i].pose.orientation.x,
                    msg->poses[i].pose.orientation.y,
                    msg->poses[i].pose.orientation.z
                    );
    }

    //    mpDensePathPub.publish(path_dense);
}



void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
#ifdef ENABLE_CLOSED_LOOP
    // @NOTE (yanwei) throw the first few garbage images from gazebo
    static size_t skip_imgs = 0;
    if (skip_imgs < 10)
    {
        ++skip_imgs;
        return;
    }
#endif

    if (paused) {
        ROS_WARN_STREAM("GFGG has paused, waiting to resume.");
        return;
    }

double latency_trans = ros::Time::now().toSec() - msgLeft->header.stamp.toSec();
// ROS_INFO("ORB-SLAM Initial Latency: %.03f sec", ros::Time::now().toSec() - msgLeft->header.stamp.toSec());

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
	cv_ptrLeft = cv_bridge::toCvShare(msgLeft, "mono8");
    }
    catch (cv_bridge::Exception& e)
    {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
	cv_ptrRight = cv_bridge::toCvShare(msgRight, "mono8");
    }
    catch (cv_bridge::Exception& e)
    {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
    }

// ROS_INFO("ORB-SLAM Image Transmision Latency: %.03f sec", ros::Time::now().toSec() - cv_ptrLeft->header.stamp.toSec());

    cv::Mat pose;
    //
    if(do_rectify)
    {
	cv::Mat imLeft, imRight;
	cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
	cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
	//        cv::Mat out;
	//        cv::hconcat(imLeft, imRight, out);
	//        cv::imwrite( "./stereo_input.png", out );

	pose = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
	//        cv::Mat image_match;
	//        mpSLAM->mpTracker->mCurrentFrame.plotStereoMatching(imLeft, imRight, image_match);
	//        cv::imwrite( "./stereo_matching.png", image_match );
    }
    else
    {
	pose = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

    double latency_total = ros::Time::now().toSec() - cv_ptrLeft->header.stamp.toSec();
    {
        const double track_latency = latency_total - latency_trans;
        vTimesTrack.emplace_back(track_latency);
        vStampedTimesTrack.emplace_back(cv_ptrLeft->header.stamp.toSec(), track_latency);
    }
// ROS_INFO("ORB-SLAM Tracking Latency: %.03f sec", ros::Time::now().toSec() - cv_ptrLeft->header.stamp.toSec());
// ROS_INFO("Image Transmision Latency: %.03f sec; Total Tracking Latency: %.03f sec", latency_trans, latency_total);
    if (pose.empty())
    {
        return;
    }
    // ROS_INFO("Pose Tracking Latency: %.03f sec", latency_total - latency_trans);

/*
    // std::cout << "broadcast pose!" << std::endl;

    /// broadcast tf
    // global left handed coordinate system 
    static cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
    static cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);
    // matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
    static const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
				    -1, 1,-1, 1,
				    -1,-1, 1, 1,
				    1, 1, 1, 1);

    //prev_pose * T = pose
    cv::Mat translation =  (pose * pose_prev.inv()).mul(flipSign);
    world_lh = world_lh * translation;
    pose_prev = pose.clone();

    tf::Matrix3x3 tf3d;
    tf3d.setValue(pose.at<float>(0,0), pose.at<float>(0,1), pose.at<float>(0,2),
		  pose.at<float>(1,0), pose.at<float>(1,1), pose.at<float>(1,2),
		  pose.at<float>(2,0), pose.at<float>(2,1), pose.at<float>(2,2));

    tf::Vector3 cameraTranslation_rh( world_lh.at<float>(0,3),world_lh.at<float>(1,3), - world_lh.at<float>(2,3) );

    //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
    const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
					    0, 0, 1,
					    1, 0, 0);

    static tf::TransformBroadcaster br;

    tf::Matrix3x3 globalRotation_rh = tf3d;
    tf::Vector3 globalTranslation_rh = cameraTranslation_rh * rotation270degXZ;

    tf::Quaternion tfqt;
    globalRotation_rh.getRotation(tfqt);

    double aux = tfqt[0];
    tfqt[0]=-tfqt[2];
    tfqt[2]=tfqt[1];
    tfqt[1]=aux;

    tf::Transform transform;
    transform.setOrigin(globalTranslation_rh);
    transform.setRotation(tfqt);

    br.sendTransform(tf::StampedTransform(transform, cv_ptrLeft->header.stamp, "map", "camera_pose"));
  */
    
    
    /// broadcast campose pose message
    // camera pose
    /*
	tf::Matrix3x3 R( 0,  0,  1,
			-1,  0,  0,
			0, -1,  0);
	tf::Transform T ( R * tf::Matrix3x3( transform.getRotation() ), R * transform.getOrigin() );
  */

    cv::Mat Rwc = pose.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*pose.rowRange(0,3).col(3);
    tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
		    Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
		    Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
    tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

    tf::Transform tfTcw(M,V);
    geometry_msgs::Transform gmTwc;
    tf::transformTFToMsg(tfTcw, gmTwc);
	
    geometry_msgs::Pose camera_pose;
    camera_pose.position.x = gmTwc.translation.x;
    camera_pose.position.y = gmTwc.translation.y;
    camera_pose.position.z = gmTwc.translation.z;
    camera_pose.orientation = gmTwc.rotation;
    
    geometry_msgs::PoseWithCovarianceStamped camera_odom;
    camera_odom.header.frame_id = "odom";
    camera_odom.header.stamp = cv_ptrLeft->header.stamp;
    camera_odom.pose.pose = camera_pose;
    
    mpCameraPosePublisher.publish(camera_odom);

    if (enable_map_to_odom_tf)
    {
        camera_frame_ = msgLeft->header.frame_id;
        PostProcess(cv_ptrLeft->header.stamp, camera_pose);
    }

//
// by default, an additional transform is applied to make camera pose and body frame aligned
// which is assumed in msf
#ifdef INIT_WITH_ARUCHO
    tf::Matrix3x3 Ric(   0, -1, 0,
			    0, 0, -1,
			    1, 0, 0);
/*  tf::Matrix3x3 Ric(   0, 0, 1,
			    -1, 0, 0,
			    0, -1, 0);*/
	tf::Transform tfTiw ( tf::Matrix3x3( tfTcw.getRotation() ) * Ric, tfTcw.getOrigin() );
#else
    tf::Matrix3x3 Ric( 0,  0,  1,
			-1,  0,  0,
			0,  -1,  0);
      tf::Transform tfTiw ( Ric * tf::Matrix3x3( tfTcw.getRotation() ), Ric * tfTcw.getOrigin() );
#endif

    geometry_msgs::Transform gmTwi;
	tf::transformTFToMsg(tfTiw, gmTwi);
	
	geometry_msgs::Pose camera_pose_in_imu;
	camera_pose_in_imu.position.x = gmTwi.translation.x;
	camera_pose_in_imu.position.y = gmTwi.translation.y;
	camera_pose_in_imu.position.z = gmTwi.translation.z;
	camera_pose_in_imu.orientation = gmTwi.rotation;
    
    geometry_msgs::PoseWithCovarianceStamped camera_odom_in_imu;
	camera_odom_in_imu.header.frame_id = "map";
	camera_odom_in_imu.header.stamp = cv_ptrLeft->header.stamp;
	camera_odom_in_imu.pose.pose = camera_pose_in_imu;
    
	mpCameraPoseInIMUPublisher.publish(camera_odom_in_imu);

/*
    tf::Matrix3x3 Ric( 0,  0,  1,
			-1,  0,  0,
			0,  -1,  0);
    
    tf::Matrix3x3 Rbi( 0,  -1,  0,
			0,  0,  -1,
			1,  0,  0);
    
	tf::Transform tfTiw ( Ric * tf::Matrix3x3( tfTcw.getRotation() ) * Rbi, Ric * tfTcw.getOrigin() );
    geometry_msgs::Transform gmTwi;
	tf::transformTFToMsg(tfTiw, gmTwi);
	
	geometry_msgs::Pose camera_pose_in_imu;
	camera_pose_in_imu.position.x = gmTwi.translation.x;
	camera_pose_in_imu.position.y = gmTwi.translation.y;
	camera_pose_in_imu.position.z = gmTwi.translation.z;
	camera_pose_in_imu.orientation = gmTwi.rotation;
    
    geometry_msgs::PoseWithCovarianceStamped camera_odom_in_imu;
	camera_odom_in_imu.header.frame_id = "odom";
	camera_odom_in_imu.header.stamp = cv_ptrLeft->header.stamp;
	camera_odom_in_imu.pose.pose = camera_pose_in_imu;
    
	mpCameraPoseInIMUPublisher.publish(camera_odom_in_imu);
*/
    
#ifdef FRAME_WITH_INFO_PUBLISH
    if (mpSLAM != NULL && mpSLAM->mpFrameDrawer != NULL) {
        cv::Mat fr_info_cv = mpSLAM->mpFrameDrawer->DrawFrame();
        cv_bridge::CvImage out_msg;
        out_msg.header   = cv_ptrLeft->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
        out_msg.image    = fr_info_cv; // Your cv::Mat
        mpFrameWithInfoPublisher.publish(out_msg.toImageMsg());
    }
#endif


#ifdef MAP_PUBLISH
    if (mnMapRefreshCounter % 30 == 1) {
	  // publish map points
	  mpMapPub->Refresh();
    }
    mnMapRefreshCounter ++;
#endif

}

void ImageGrabber::PostProcess(const ros::Time& time, const geometry_msgs::Pose& wTc)
{
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = fixed_frame_;
        pose_msg.header.stamp = time;
        pose_msg.pose = wTc;
        camera_pose_publisher.publish(pose_msg);

        camera_path.header = pose_msg.header;
        camera_path.poses.push_back(pose_msg);
        camera_path_publisher.publish(camera_path);
    }

     // Define map that aligns with base_frame
    static bool is_map_defined = false;
    static Eigen::Matrix4d mTw = Eigen::Matrix4d::Identity();
    if (!is_map_defined) {
        geometry_msgs::TransformStamped base_to_cam = tf_buffer_.lookupTransform(
            base_frame_, camera_frame_, time, ros::Duration(0.2));
        mTw = Ros2Eigen(base_to_cam.transform);
    //   writer_.Write("camera_extrinsic", Tmw);

        base_to_cam.header.frame_id = map_frame_;
        base_to_cam.header.stamp = time;
        base_to_cam.child_frame_id = fixed_frame_;
        static_br_.sendTransform(base_to_cam);
        is_map_defined = true;
    }

    // Publish map to odom.
    const Eigen::Matrix4d mTc = mTw * Ros2Eigen(wTc);
    const geometry_msgs::TransformStamped cam_to_odom =
        tf_buffer_.lookupTransform(
            camera_frame_, odom_frame_, time, ros::Duration(0.2));
    geometry_msgs::TransformStamped map_to_odom;
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.header.stamp = time + ros::Duration(0.5);
    map_to_odom.child_frame_id = odom_frame_;
    map_to_odom.transform = Eigen2Ros(mTc * Ros2Eigen(cam_to_odom.transform));
    tfbr_.sendTransform(map_to_odom);
}

Eigen::Matrix4d ImageGrabber::Ros2Eigen(const geometry_msgs::Transform& tf)
{
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out.topLeftCorner(3, 3) = Eigen::Quaterniond(
        tf.rotation.w,
        tf.rotation.x,
        tf.rotation.y,
        tf.rotation.z
    ).toRotationMatrix();
    out.topRightCorner(3, 1) = Eigen::Vector3d(tf.translation.x, tf.translation.y, tf.translation.z);
    return out;
}
Eigen::Matrix4d ImageGrabber::Ros2Eigen(const geometry_msgs::Pose& pose)
{
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out.topLeftCorner(3, 3) = Eigen::Quaterniond(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z
    ).toRotationMatrix();
    out.topRightCorner(3, 1) = Eigen::Vector3d(
        pose.position.x,
        pose.position.y,
        pose.position.z);
    return out;
    
}
geometry_msgs::Transform ImageGrabber::Eigen2Ros(const Eigen::Matrix4d& pose)
{
    geometry_msgs::Transform out;
    out.translation.x = pose(0, 3);
    out.translation.y = pose(1, 3);
    out.translation.z = pose(2, 3);
    const Eigen::Quaterniond quat(
        Eigen::Matrix3d(pose.topLeftCorner(3, 3)));
    out.rotation.x = quat.x();
    out.rotation.y = quat.y();
    out.rotation.z = quat.z();
    out.rotation.w = quat.w();
    return out;
}