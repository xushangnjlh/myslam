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

#include "myslam/VisualOdometry.h"
namespace myslam
{

VisualOdometry::VisualOdometry():
mState(INITIALIZING), mMap(new Map), mCurrentFrame(nullptr), mReferenceFrame(nullptr), 
mnInliers(0), mnLost(0)
{
  mnFeatures = Config::get<int>("nFeatures");
  mScaleFactor = Cofig::get<double>("scaleFactor");
  mnLevel = Config::get<int>("nLevels");
  mMatchedRatio = Config::get<double>("matchedRatio");
}

bool VisualOdometry::AddFrame(Frame::Ptr frame)
{
  switch(mState)
  {
    case INITIALIZING:
    {
      mState = OK;
      mCurrentFrame = mReferenceFrame = frame;
      mMap->InsertKeyFrame(frame);
      ExtractKeyPoints();
      ComputeDescripter();
      SetMapPointPosInRef();
      break;
    }
    case OK:
    {
      mCurrentFrame = frame;
      ExtractKeyPoints();
      ComputeDescripter();
      FeatureMatch();
      PoseEstimateByPnP();
      if(CheckPose())
      {
	mCurrentFrame->mTcw = mEstimatedTcr*mReferenceFrame->mTcw;
	mReferenceFrame = mCurrentFrame;
	SetMapPointPosInRef();
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

}