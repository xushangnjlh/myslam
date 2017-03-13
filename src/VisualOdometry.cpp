/*
 * <mySLAM, a simple slam learnt from ORB-SLAM and gaoxiang's book.>
 * Copyright (C) 2017  <Shang XU> <xushangnjlh@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "myslam/VisualOdometry.h"
#include "myslam/g2o_types.h"
#include <boost/timer.hpp>
#include <algorithm>

namespace myslam
{

VisualOdometry::VisualOdometry():
mState(INITIALIZING), mMap(new Map), mCurrentFrame(nullptr), mReferenceFrame(nullptr), 
mnInliers(0), mnLost(0), mMatcher(new cv::flann::LshIndexParams(5, 10, 2))
{
  mnFeatures = Config::get<int>("nFeatures");
  mScaleFactor = Config::get<double>("scaleFactor");
  mnLevel = Config::get<int>("nLevels");
  mMatchRatio = Config::get<double>("matchRatio");
  mnMaxLost = Config::get<int>("nMaxLost");
  mnMinInlier = Config::get<int>("nMinInlier");
  mKeyFrameMinR = Config::get<double>("KeyFrameMinR");
  mKeyFrameMint = Config::get<double>("KeyFrameMint");
  mMapPointTh = Config::get<double>("MapPointTh");
  mpORB = cv::ORB::create(mnFeatures, mScaleFactor, mnLevel);
}

VisualOdometry::~VisualOdometry()
{
  
}

bool VisualOdometry::AddFrame(Frame::Ptr frame)
{
  switch(mState)
  {
    case INITIALIZING:
    {
      mState = OK;
      mCurrentFrame = mReferenceFrame = frame;
      ExtractKeyPoints();
      ComputeDescripter();
      // this first frame is KeyFrame, add all MapPoints
      AddKeyFrame();
      break;
    }
    case OK:
    {
      mCurrentFrame = frame;
      mCurrentFrame->mTcw = mReferenceFrame->mTcw;
      ExtractKeyPoints();
      ComputeDescripter();
      FeatureMatch();
      PoseEstimateByPnP();
      if(CheckPose())
      {
	mCurrentFrame->mTcw = mEstimatedTcw*mReferenceFrame->mTcw;
	mReferenceFrame = mCurrentFrame;
	PoseOptimization();
	mnLost = 0;
	if(CheckKeyFrame())
	{
	  AddKeyFrame();
	}
      }
      else
      {
	mnLost++;
	if(mnLost>mnMaxLost)
	{
	  mState = LOST;
	}
	return false;
      }
      break;
    }
    case LOST:
    {
      cout << "Tracking is lost! " << endl;
      break;
    }
  }
  return true;
}

void VisualOdometry::ExtractKeyPoints()
{
  boost::timer timer;
  mpORB->detect(mCurrentFrame->mColor, mvKeyPointsCur);
  cout << "ExtractKeyPoints time: " << timer.elapsed() << endl;
}

void VisualOdometry::ComputeDescripter()
{
  boost::timer timer;
  mpORB->compute(mCurrentFrame->mColor, mvKeyPointsCur, mDescriptorCur);
  cout << "ComputeDescripter time: " << timer.elapsed() << endl;
}

void VisualOdometry::AddKeyFrame()
{
  if(mMap->mmKeyFrames.empty()) // first frame - add all MapPoints
  {
    for(size_t i=0; i<mvKeyPointsCur.size();i++)
    {
      double d = mCurrentFrame->FindDepth();
      if(d<0)
      {
	continue;
      }
      Vector3d mp_world = mCurrentFrame->mpCamera->pixel2world(mvKeyPointsCur[i], mCurrentFrame->mTcw, d);
      Vector3d mp_camera_norm = mp_world - mCurrentFrame->GetCameraCenter();
      mp_camera_norm.normalize();
      MapPoint::Ptr mapPoint = MapPoint::CreateMapPoint(mp_world, 
							mp_camera_norm, 
							mDescriptorCur.col(i).clone(), 
							mCurrentFrame.get()
						       );
      mMap->InsertMapPoint(mapPoint);
    }
  }
  mMap->InsertKeyFrame(mCurrentFrame);
  mReferenceFrame = mCurrentFrame;
}

void VisualOdometry::FeatureMatch()
{
  boost::timer timer;
  vector<cv::DMatch> matches;
  
  Mat DescriptorQuery_MP;
  vector<MapPoint::Ptr> vMapPointsInFrame;
  
  for(auto& p_index : mMap->mmMapPoints){
    MapPoint::Ptr& p = p_index.second();
    if(mCurrentFrame->IsInFrame(p->mWorldPos))
    {
      p->mnVisible++;
      vMapPointsInFrame.push_back(p);
      DescriptorQuery_MP.push_back(p->mDescriptor);
    }
  }
  
  mMatcher.match(DescriptorQuery_MP, mDescriptorCur, matches);
  // use lambda expressions in c++11 
  // https://en.wikipedia.org/wiki/Anonymous_function#C.2B.2B_.28since_C.2B.2B11.29
  float min_distance = std::min_element(matches.begin(), 
					matches.end(), 
					[](const cv::DMatch& d1, const cv::DMatch& d2)->float
					{
					  return d1.distance < d2.distance;
					})->distance;
  mvpMatchedMP.clear();
  mvMatchedKPIndex.clear();
  for(cv::DMatch& m : matches)
  {
    if (m.distance() < max<float>( min_distance*mMatchRatio, 30.0))
    {
      mvpMatchedMP.push_back(vMapPointsInFrame[m.queryIdx]);
      mvMatchedKPIndex.push_back(m.trainIdx);
    }
  }
  cout << "number of good matches: " << mvpMatchedMP.size() << endl;
  cout << "FeatureMatch time: " << timer.elapsed() << endl;
}

void VisualOdometry::PoseEstimateByPnP()
{
  vector<cv::Point3f> vMapPoints;
  vector<cv::Point2f> vKeyPoints;
  for(MapPoint::Ptr mp : mvpMatchedMP)
  {
    vMapPoints.pop_back(mp->GetPositionCV());
  }
  for(int index : mvMatchedKPIndex)
  {
    vKeyPoints.push_back(mvKeyPointsCur[index].pt);
  }
  
  Mat K = (cv::Mat_<double>(3,3) << 
				mReferenceFrame->mpCamera->fx_, 0, mReferenceFrame->mpCamera->cx_,
				0, mCurrentFrame->mpCamera->fy, mReferenceFrame->mpCamera->cy_,
				0,0,1
	  );
  Mat rvec,tvec, inliers;
  cv::solvePnPRansac ( vMapPoints, vKeyPoints, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
  mnInliers = inliers.rows;
  cout << "pnp inliers: " << mnInliers;
  mEstimatedTcw = SE3( SO3(rvec.at<double>(0,0), rvec.at<double>(0,1), rvec.at<double>(0,2)),
		       Vector3d(tvec.at<double>(0,0)), tvec.at<double>(0,1), tvec.at<double>(0,2)
  );
  cout << "EstimatedTcwByPnP: " << mEstimatedTcw.matrix() << endl;
  
  // g2o
  g2o::SparseOptimizer optimizer;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
  Block::LinearSolverType* linearSolver;
  linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
  Block* blockSolver = new Block(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);
  
  // vertex (pose of Tcw) this one-edge graph has only one vertex and mnInliners edges
  g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
  pose->setId(0);
  pose->setEstimate(g2o::SE3Quat(mEstimatedTcw.rotation_matrix(), mEstimatedTcw.translation()));
  optimizer.addVertex(pose);
  
  // edge of projection measurement
  for(size_t i=0; i<mnInliers; i++)
  {
    int inlierId = inliers.at<int>(i,0);
    EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
    edge->setId(i);
    edge->setVertex(0,pose);
    edge->camera_ = mCurrentFrame->mpCamera.get();
    edge->point_ = Vector3d(vMapPoints[inlierId].x, vMapPoints[inlierId].y, vMapPoints[inlierId].z);
    edge->setMeasurement(Vector2d(vKeyPoints[inlierId].x, vKeyPoints[inlierId].y));
    edge->setInformation(Eigen::Matrix2d::Identity());
    optimizer.addEdge(edge);
    vMapPoints[inlierId]->mnMatched++;
  }
}



}