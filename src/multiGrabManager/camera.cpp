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
*/

#include <camera.h>

namespace unr_rgbd {
  namespace multikinect {
    
    Camera::Camera( boost::function<void (std::string&, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f ) :
      serialNumber_(NULL),
      device_(NULL)
    {
      managerfunction_ = f;
      
      //TODO set connection betwwen this and f                                                                                                                                                                                             
      //      managerSignalConnection_                                                                                                                                                                                                     
      
      camerafunction_ = boost::bind( &Camera::cameraCallBack, this, _1);
    }

    Camera::~Camera()
    {
      if (device_ != NULL) {
	delete device_;
      }
    }
    void Camera::initalize( std::string serialNumber )
    {
      device_ =  new(std::nothrow) pcl::OpenNIGrabber(serialNumber.c_str());
      //if( device_ == NULL )                                                                                                                                                                                                              
      //      throw                                                                                                                                                                                                                        
      
      serialNumber_ = serialNumber;

      camSignalConnection_ = device_->registerCallback( camerafunction_ );
    }

    void Camera::start() {
      device_->start();
    }
    
    void Camera::stop() {
      device_->stop();
    }

    void Camera::cameraCallBack ( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud )
    {
    }
  } // multikinect
} // unr_rgbd
