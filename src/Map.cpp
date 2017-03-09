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

#include "myslam/Map.h"
namespace
{
  
void myslam::Map::InsertMapPoint(myslam::MapPoint::Ptr mapPoint)
{
  if(mmMapPoints.find(mapPoint->mnId) == mmMapPoints.end())
  {
    mmMapPoints.insert(make_pair(mapPoint->mnId, mapPoint));
  }
  else
  {
    mmMapPoints[mapPoint->mnId] = mapPoint;
  }
}

void myslam::Map::InsertKeyFrame(myslam::Frame::Ptr keyFrame)
{
  if(mmKeyFrames.find(keyFrame->mnId)==mmKeyFrames.end())
  {
    mmKeyFrames.insert(make_pair(keyFrame->mnId, keyFrame));
  }
  else
  {
    mmKeyFrames[keyFrame->mnId] = keyFrame;
  }
}

}
