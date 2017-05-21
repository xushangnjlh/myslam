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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "myslam/Common_include.h"
#include "myslam/Frame.h"
namespace myslam
{
class MapPoint
{
public:
  typedef std::shared_ptr<MapPoint> Ptr;
  unsigned long mnId;
  static unsigned long idx;
  bool mbBad;
  Vector3d mWorldPos;
  Vector3d mNormalVector;
  Mat mDescriptor;
  std::list<Frame*> mObservations;
  int mnVisible;
  int mnMatched;

// constructor and destructor  
  MapPoint();
  MapPoint(unsigned long id, 
	   const Vector3d& worldPos, 
	   const Vector3d& normalVector);
  MapPoint(unsigned long id, 
	   const Vector3d& worldPos, 
	   const Vector3d& normalVector, 
	   const Mat& descriptor, 
	   Frame* frame=nullptr
	  );
  ~MapPoint();
  
  static MapPoint::Ptr CreateMapPoint();
  static MapPoint::Ptr CreateMapPoint(const Vector3d& worldPos, 
					const Vector3d& normalVector, 
					const Mat& descriptor,
					Frame* frame
 				      );
  
  inline cv::Point3f GetPositionCV() const
  {
    return cv::Point3f(mWorldPos(0,0), mWorldPos(0,1), mWorldPos(0,2));
  }
};
}

#endif