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
mState(INITIALIZING), mMap(new Map), mCurrentFrame(nullptr), mReferenceFrame(nullptr), mnPnPInliers(0),
mnBAInliers(0), mnLost(0), mMatcher(new cv::flann::LshIndexParams(5, 10, 2))
{
  mnFeatures = Config::get<int>("nFeatures");
  mScaleFactor = Config::get<double>("scaleFactor");
  mnLevel = Config::get<int>("nLevels");
  mMatchRatio = Config::get<double>("matchRatio");
  mnMaxLost = Config::get<int>("nMaxLost");
  mnMinPnPInlier = Config::get<int>("nMinPnPInlier");
  mnMinBAInlier = Config::get<int>("nMinBAInlier");
  mMaxMotion = Config::get<double>("MaxMotion");
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
      if(CheckPose()) // enough inlier and no brutal motion
      {
	mCurrentFrame->mTcw = mEstimatedTcw;
	UpdateMapPoints();
	mnLost = 0; // only the lost frames are consecutive, counts for mnLost
	if(CheckKeyFrame())
	{
	  AddKeyFrame();
	}
	mReferenceFrame = mCurrentFrame;
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

bool VisualOdometry::CheckKeyFrame()
{
  SE3 Tcr = mEstimatedTcw*mReferenceFrame->mTcw.inverse();
  Sophus::Vector6d se3 = Tcr.log();
  Vector3d t = se3.head<3>();
  Vector3d r = se3.tail<3>();
  if(t.norm() > mKeyFrameMint || r.norm() > mKeyFrameMinR)
    return true;
  else
    return false;
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
      // all MapPoints in the Map, which can be visible in current Frame, set mnVisible++
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
  // convert MapPoints and KeyPoints towards cv::Point3f and cv::Point2f format
  // in order to use solvePnPRansac (minimize reprojection error from objectPoints to imagePoints, to get inital pose)
  vector<cv::Point3f> vMapPoints;
  vector<cv::Point2f> vKeyPoints;
  for(MapPoint::Ptr mp : mvpMatchedMP)
  {
    vMapPoints.push_back(mp->GetPositionCV());
  }
  for(int index : mvMatchedKPIndex)
  {
    vKeyPoints.push_back(mvKeyPointsCur[index].pt);
  }
  
  Mat K = (cv::Mat_<double>(3,3) << 
				mReferenceFrame->mpCamera->fx_, 0, mReferenceFrame->mpCamera->cx_,
				0, mReferenceFrame->mpCamera->fy_, mReferenceFrame->mpCamera->cy_,
				0,0,1
	  );
  Mat rvec,tvec, inliers;
  cv::solvePnPRansac ( vMapPoints, vKeyPoints, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
  mnPnPInliers = inliers.rows;
  cout << "pnp inliers = " << mnPnPInliers;
  mEstimatedTcw = SE3( SO3(rvec.at<double>(0,0), rvec.at<double>(0,1), rvec.at<double>(0,2)),
		       Vector3d(tvec.at<double>(0,0)), tvec.at<double>(0,1), tvec.at<double>(0,2)
  );
  cout << "EstimatedTcwByPnP = " << mEstimatedTcw.matrix() << endl;
  
  // g2o (StereoMatching, PnP, ICP or Bow matching methods were used to give a reliable initial value for BA)
  g2o::SparseOptimizer optimizer;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
  Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
  Block* blockSolver = new Block(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
  optimizer.setAlgorithm(solver);
  
  // vertex (pose of Tcw) this one-edge graph has only one vertex and mnInliners edges
  g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
  pose->setId(0);
  pose->setEstimate(g2o::SE3Quat(mEstimatedTcw.rotation_matrix(), mEstimatedTcw.translation()));
  optimizer.addVertex(pose);
  
  // edge of projection measurement
  vector<EdgeProjectXYZ2UVPoseOnly*> edges;
  for(size_t i=0; i<mnPnPInliers; i++)
  {
    int inlierId = inliers.at<int>(i,0);
    EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
    edge->setId(i);
    edge->setVertex(0,pose);
    edge->camera_ = mCurrentFrame->mpCamera.get();
    edge->point_ = Vector3d(vMapPoints[inlierId].x, vMapPoints[inlierId].y, vMapPoints[inlierId].z);
    edge->setMeasurement(Vector2d(vKeyPoints[inlierId].x, vKeyPoints[inlierId].y));
    edge->setInformation(Eigen::Matrix2d::Identity());
    edge->setRobustKernel(new g2o::RobustKernelHuber());
    optimizer.addEdge(edge);
    mvpMatchedMP[inlierId]->mnMatched++;
    edges.push_back(edge);
  }
  
  // begin optimize
  cout << "begin optimize" << endl;
  boost::timer timer;
  optimizer.setVerbose(true);
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  cout << "end optimize " << "--- elapsed time = " << timer.elapsed() << endl;
  mEstimatedTcw = SE3(pose->estimate().rotation(), pose->estimate().translation());
  cout << "EstimatedTcwByBA = " << mEstimatedTcw.matrix() << endl;
  
  // check inliers by chi2
  for(EdgeProjectXYZ2UVPoseOnly* e:edges)
  {
    e->computeError();
    if(e->chi2()>1)
    {
      cout << "error = " << e->chi2() << endl;
    }
    else
    {
      mnBAInliers++;
    }
    cout << "BA inliers = " << mnBAInliers << endl;
  }
}

bool VisualOdometry::CheckPose()
{
  if(mnPnPInliers < mnMinPnPInlier || mnBAInliers < mnMinBAInlier)
  {
    cout << "Skip this Frame and set to lost because inliers is too small." << endl
	 << "PnP inliers = " << mnPnPInliers << endl
	 << "BA inliers = " << mnBAInliers << endl;
    return false;
  }
  SE3 mTcr = mEstimatedTcw*mReferenceFrame->mTcw.inverse();
  Sophus::Vector6d se3 = mTcr.log();
  if(se3.norm() > mMaxMotion)
  {
    cout << "Skip this Frame and set to lost because motion is too large."  << endl;
    return false;
  }
}

void VisualOdometry::UpdateMapPoints()
{
  for(unordered_map::const_iterator iterMP = mMap->mmMapPoints.begin();iterMP != mMap->mmMapPoints.end(); iterMP++)
  {
    if(!mCurrentFrame->IsInFrame(iterMP->second->mWorldPos))
    {
      mMap->mmMapPoints.erase(iterMP);
      continue;
    }
    float matchRatio = float(iterMP->second->mnMatched)/iterMP->second->mnVisible;
    // MapPoint can not be matched by enough PnP 
    if( matchRatio < mMapPointTh)
    {
      mMap->mmMapPoints.erase(iterMP);
      continue;
    }
    // viewing angle too large > 60 degree
    if(GetNormal(mCurrentFrame, iterMP->second) < 0.5)
    {
      mMap->mmMapPoints.erase(iterMP);
      continue;
    }
    AddMapPoints();
    if(mMap->mmMapPoints.size()>1000)
    {
      mMapPointTh+=0.05;
    }
  }
}

void VisualOdometry::AddMapPoints()
{
//   vector<bool> matched(mvKeyPointsCur.size(), false);
//   for(int index:mvMatchedKPIndex)
//   {
//     
//   }
  for(size_t i=0; i<mvpMatchedMP.size(); i++)
  {
    cv::KeyPoint kp = mvKeyPointsCur[mvMatchedKPIndex[i]];
    double d = mCurrentFrame->FindDepth(kp);
    if(d<0)
      continue;
    Vector3d worldPos = mCurrentFrame->mpCamera->pixel2world( Vector2d(), 
							 mCurrentFrame->mTcw, 
							 d
						       );
    Vector3d normalVector = worldPos - mCurrentFrame->GetCameraCenter();
    normalVector.normalize();
    MapPoint::Ptr mapPoint = MapPoint::CreateMapPoint(worldPos, 
						      normalVector, 
						      mDescriptorCur.row(mvMatchedKPIndex[i]).clone(), 
						      mCurrentFrame.get());
    mMap->InsertMapPoint(mapPoint);
  }
}


double VisualOdometry::GetNormal(Frame::Ptr frame, MapPoint::Ptr mapPoint)
{
  Vector3d n = mapPoint->mWorldPos - frame->GetCameraCenter();
  n.normalize();
  return n.transpose()*mapPoint->mNormalVector;
}




}