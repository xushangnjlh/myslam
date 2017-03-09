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
#include "myslam/Frame.h"
#include "myslam/MapPoint.h"
#include "myslam/Config.h"

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
  vector<cv::Point3f> mvMapPointsInRef;
  vector<cv::KeyPoint> mvKeyPointsInCur;
  Mat mDescriptorRef;
  Mat mDescriptorCur;
  vector<cv::DMatch> mvMatched;
  
  SE3 mEstimatedTcr;
  int mnInliers;
  int mnLost;
  
  // parameters for feature extraction
  int mnFeatures;
  double mScaleFactor;
  int mnLevel;
  float mMatchedRatio;
  int mnMaxLost;
  int mnMinInlier;
  
  double mKeyFrameMinR;
  doublr mKeyFrameMint;
  
public:
  VisualOdometry();
  ~VisualOdometry();
  
  bool AddFrame(Frame::Ptr frame);

protected:
  void ExtractKeyPoints();
  void ComputeDescripter();
  void FeatureMatch();
  void PoseEstimateByPnP();
  void SetMapPointPosInRef();
  
  void AddKeyFrame();
  void CheckPose();
  void CheckKeyFrame();
};
}

#endif // VISUALODOMETRY_H