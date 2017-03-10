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

#include "myslam/Config.h"

namespace myslam{
  
shared_ptr<Config> Config::pConfig = nullptr;

void Config::setParameterFile(const string& filename)
{
  if(pConfig==nullptr)
    pConfig = shared_ptr<Config>(new Config);
  pConfig->mFile = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
  if(!pConfig->mFile.isOpened())
  {
    cerr<<"Parameter file "<< filename << " does not exist!" << endl;
    pConfig->mFile.release();
    return ;
  }
}

Config::~Config()
{
  if(mFile.isOpened())
    mFile.release();
}

}

