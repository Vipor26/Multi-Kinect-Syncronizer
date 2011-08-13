/*
  Copyright (c) 2011, Board of Regents, NSHE, obo UNR 
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

  Wrapper for a camera object.

  @author Brian Hamilton
  @author Richard Kelley
*/

#ifndef CAMERA_H
#define CAMERA_H

// STD includes
#include <string>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_exception.h>

// Boost Includes
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/noncopyable.hpp>
#include <boost/signals2/mutex.hpp>

namespace unr_rgbd {
  namespace multikinect {

    class multiGrabberManager;

    class Camera
    {
    private:
      // Accessor functions
      Camera(boost::function<void (std::string&, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f );
      ~Camera();
      void initalize( std::string serialNumber );

      // This lets us start and stop the grabber from the manager - wrapper for pcl::Grabber functions. 
      void start();
      void stop();

      // signal ? register a callback
      // Handles the signal connections between grabber and this class and this class and manager
      boost::signals2::connection camSignalConnection_, managerSignalConnection_;

      // New Labaled callback handle
      boost::function<void (std::string&, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> managerfunction_;

      // Camera callback handle
      boost::function<void (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> camerafunction_;

      void cameraCallBack ( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud );

      // Information about camera
      std::string serialNumber_;
      pcl::Grabber* device_;
      friend class multiGrabberManager;
    };

  } // multikinect
} // unr_rgbd

#endif // CAMERA_H
