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

#include "myslam/Frame.h"
namespace myslam
{
Frame::Frame()
:mnId(-1), mTimeStamp(-1), mpCamera(nullptr), mbIsKeyFrame(false)
{
  
}

Frame::Frame(long id, double timeStamp=0, SE3 Tcw=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat())
:mnId(id), mTimeStamp(timeStamp), mTcw(Tcw), mpCamera(camera), mColor(color), mDepth(depth), mbIsKeyFrame(false)
{
  
}

Frame::~Frame()
{
  
}

unsigned long Frame::idx=0;

Frame::Ptr Frame::CreateFrame()
{
  return Frame::Ptr(new Frame(idx++));
}

double Frame::FindDepth(const cv::KeyPoint& kp)
{
  int x = cvRound(kp.pt.x);
  int y = cvRound(kp.pt.y);
  ushort d = mDepth.ptr<ushort>(y)[x];
  if(d!=0)
  {
    return double(d)/mpCamera->mfDepthScale;
  }
  else
  {
    int dx[4] = { 0, 0, 1,-1 };
    int dy[4] = { 1,-1, 0, 0 };
    for(int i=0; i<4; i++)
    {
      d = mDepth.ptr<ushort>(y+dy[i])[x+dx[i]];
      if(d!=0)
      {
	return double(d)/mpCamera->mfDepthScale;
      }
    }
  }
  return -1;
}

void Frame::SetPose(const SE3& Tcw)
{
  mTcw = Tcw;
}

Vector3d Frame::GetCameraCenter() const
{
  return mTcw.inverse().translation();
}

bool Frame::IsInFrame(const Vector3d& pt_w)
{
  Vector3d pt_c = mpCamera->world2camera(pt_w, mTcw);
  if(pt_c(2,0)<0)
    return false;
  Vector2d pt_p = mpCamera->world2pixel(pt_w, mTcw);
  return pt_p(0,0)>0 && pt_p(1,0)>0 && pt_p(0,0)<mDepth.cols && pt_p(0,1)<mDepth.rows;
}
  
}