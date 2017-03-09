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

#ifndef Frame_H
#define Frame_H
#include "myslam/Common_include.h"
#include "Camera.h"
namespace myslam{
class Frame{
public:
  typedef std::shared_ptr<Frame> Ptr;
  unsigned long mnId;
  static unsigned long idx;
  double mTimeStamp;
  SE3 mTcw;
  Camera::Ptr mpCamera;
  Mat mColor, mDepth;
  bool mbIsKeyFrame;
public:
  Frame();
  Frame(long id, double timeStamp=0, SE3 Tcw=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat());
  ~Frame();
  
  static Frame::Ptr CreateFrame();
  
  double FindDepth(const cv::KeyPoint& kp);
  void SetPose(const SE3& Tcw);
  Vector3d GetCameraCenter() const; // const member function
  bool IsInFrame(const Vector3d& pt_w);
};
}

#endif