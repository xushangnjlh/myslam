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
#ifndef CONFIG_H
#define CONFIG_H

#include "myslam/Common_include.h"

namespace myslam{
class Config
{
private:
  static shared_ptr<Config> pConfig;
  cv::FileStorage mFile;
  Config(){}
public:
  ~Config();
  
  static void setParameterFile(const string& filename);
  
  template<typename T> // the parameter can be int, double or string
  static T get(const string& key)
  {
    return T(Config::pConfig->mFile[key]);
  }
};

}

#endif // CONFIG_H