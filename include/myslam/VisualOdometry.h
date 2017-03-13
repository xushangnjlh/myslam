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

#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/Common_include.h"
#include "myslam/Map.h"
#include "myslam/Config.h"

#include <opencv2/features2d/features2d.hpp>

namespace myslam
{
class VisualOdometry
{
public:
  typedef shared_ptr<VisualOdometry> Ptr;
  enum VOState{
    INITIALIZING=0,
    OK=1,
    LOST=-1
  };
  
  VOState mState;
  Map::Ptr mMap;
  Frame::Ptr mCurrentFrame;
  Frame::Ptr mReferenceFrame;
  
  cv::Ptr<cv::ORB> mpORB;
  vector<cv::Point3f> mvMapPointsRef;
  vector<cv::KeyPoint> mvKeyPointsCur;
  Mat mDescriptorRef;
  Mat mDescriptorCur;
  
  cv::FlannBasedMatcher mMatcher;
  vector<cv::DMatch> mvMatched;
  vector<int> mvMatchedKPIndex;
  vector<MapPoint::Ptr> mvpMatchedMP;
  
  SE3 mEstimatedTcw;
  int mnPnPInliers;
  int mnBAInliers;
  int mnLost;
  
  // parameters for feature extraction
  int mnFeatures;
  double mScaleFactor;
  int mnLevel;
  float mMatchRatio; // criteria for good match
  int mnMaxLost; // max frames for tracking lost
  int mnMinPnPInlier; // min inliers for matching
  int mnMinBAInlier;
  
  double mMaxMotion;
  double mKeyFrameMinR;
  double mKeyFrameMint;
  double mMapPointTh; // ratio to remove MapPoint
  
public:
  VisualOdometry();
  ~VisualOdometry();
  
  bool AddFrame(Frame::Ptr frame);

protected:
  void ExtractKeyPoints();
  void ComputeDescripter();
  void FeatureMatch();
  void PoseEstimateByPnP();
  double GetNormal(Frame::Ptr frame, MapPoint::Ptr mapPoint);
  
  void AddMapPoints();
  void AddKeyFrame();
  bool CheckPose();
  bool CheckKeyFrame();
  
  void UpdateMapPoints();
  
};
}

#endif // VISUALODOMETRY_H