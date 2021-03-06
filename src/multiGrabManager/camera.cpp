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
    
    Camera::Camera() :
      serialNumber_("")
      //device_(NULL)
    { 
      camerafunction_ = boost::bind( &Camera::cameraCallBack, this, _1);
    }
    
    Camera::Camera( const Camera &rhs )
    {
      (*this) = rhs;
    }
    
    Camera::~Camera()
    {
      /*
      if (device_ != NULL) {
	delete device_;
	}*/
    }
    void Camera::initalize( boost::shared_ptr<pcl::Grabber> device, std::string serialNumber, boost::function<void 
                   (std::string&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&)> f  )
    {
      //device_.reset( new openni_wrapper::OpenNIDevice(*device));
      device_ = device;
      /*
      if (device_ != NULL) {
	delete device_;
      }
      try { //TODO REMOVE
	std::cout << "Connecting: " << serialNumber;
	device_ =  new(std::nothrow) pcl::OpenNIGrabber(serialNumber);
	if( device_ == NULL ) {
	  throw ConnectionFailedException();
	}
	std::cout << " ... done "  << std::endl;
      } catch ( pcl::PCLIOException E )
      {
        std::cout << std::endl << E.what() << std::endl;
	exit(3);
      }
      */
      serialNumber_ = serialNumber;

      std::cout << "reg" << std::endl;
      camSignalConnection_ = device_->registerCallback( camerafunction_ );
      
      std::cout << "reg" << std::endl;
      managerfunction_ = f;
      managerSignalConnection_.connect( managerfunction_ );
    }

    Camera& Camera::operator=(const Camera &rhs)
    {
      if (this != &rhs) {
        if( rhs.device_ != NULL ) {
          serialNumber_ = rhs.serialNumber_;
          device_.reset(new(std::nothrow) pcl::OpenNIGrabber( serialNumber_ ));
          if( device_ == NULL ) {
            throw ConnectionFailedException();
          }
          
          camerafunction_ = boost::bind( &Camera::cameraCallBack, this, _1);
          camSignalConnection_ = device_->registerCallback( camerafunction_ );
        
          managerfunction_ = rhs.managerfunction_;
          managerSignalConnection_.connect( managerfunction_ );
        }
        else  {
          serialNumber_ = "";
          //device_ = NULL;
        }
      }
      return *this;
    }

    void Camera::start() {
      device_->start();
    }
    
    void Camera::stop() {
      device_->stop();
    }
                                //const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >&
    void Camera::cameraCallBack ( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud )
    {
      managerSignalConnection_( serialNumber_, cloud );
    }

  } // multikinect
} // unr_rgbd
