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

#include "myslam/MapPoint.h"
namespace myslam {
MapPoint::MapPoint()
:mnId(-1), mbBad(false), mWorldPos(Vector3d(0,0,0)), mNormalVector(Vector3d(0,0,0)), mnVisible(0), mnMatched(0)
{
  
}
MapPoint::MapPoint(long unsigned int id, const Vector3d& worldPos, const Vector3d& normalVector, const Mat& descriptor, Frame* frame)
:mnId(id), mbBad(false), mWorldPos(worldPos), mNormalVector(normalVector), mDescriptor(descriptor), 
mnVisible(0), mnMatched(0)
{
  mObservations.push_back(frame);
}

MapPoint::~MapPoint()
{
  
}

unsigned long idx=0;

MapPoint::Ptr MapPoint::CreateMapPoint()
{
  return MapPoint::Ptr(new MapPoint(idx++, Vector3d(0,0,0), Vector3d(0,0,0)));
}

MapPoint::Ptr MapPoint::CreateMapPoint(const Vector3d& worldPos, const Vector3d& normalVector, const Mat& descriptor, Frame* frame)
{
  return MapPoint::Ptr(new MapPoint(idx++, worldPos, normalVector, descriptor, frame));
}

}
