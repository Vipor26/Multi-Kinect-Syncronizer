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

  @author Brian Hamilton
  @author Richard Kelley
*/

#ifndef MULTI_GRAB_MANAGER
#define MULTI_GRAB_MANAGER

// STD includes
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <new> // for peramiter nothrow

//#include <iostream> //REMOVE
    
// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h> // for devices
#include <pcl/io/openni_camera/openni_driver.h> // for driver
#include <pcl/io/openni_camera/openni_exception.h>
    
// Boost Includes
#include <boost/bimap.hpp>
#include <boost/thread.hpp>
#include <boost/date_time.hpp> // for thread sleep
#include <boost/noncopyable.hpp> // to make class not copiable
#include <boost/signals2.hpp>

// Project Includes
#include <synchronizer.h>
#include <camera.h>
#include <labeledCloud.h>


namespace unr_rgbd {
  namespace multikinect {
    
    
    // Manager is a single instance class that maintains a level abstraction between
    //   the user and the pcl::hardware
    class MultiGrabberManager : private boost::noncopyable
    {
     public:
     
      // Exceptions
      class CamerasNotFoundException{};
      
	    // Constructor Destructor
	    MultiGrabberManager();
	    ~MultiGrabberManager();
	
	    // available serial numbers
	    std::vector<std::string> getAvailableSerialNumbers();
	
	    // if nonSpecified will connect to all available
	    void connect( std::vector<std::string> = std::vector<std::string>() );
	
	    // functions to start stop the selected camera streams
	    void startSelected();
	    void stopSelected();
	
	    // register Callback
      void registerCallback( boost::function< void ( vector<LabeledCloud>& ) > f );
	    
	    // Accessors
      void setSyncBufferSize( unsigned bufSize ) {
        bufferSize_ = bufSize;
      }
      unsigned getSyncBufferSize() {
        return bufferSize_;
      }
	    
     private:
	
	    typedef boost::bimap< std::string, unsigned > StrIdxBm;
	    typedef StrIdxBm::value_type StrIdxPair;
	
	    Synchronizer sync_;
	    vector<Camera> Cameras_;

	    // Connected Cammera peramiters
	    StrIdxBm serialIndexBiMap_;

      unsigned bufferSize_;

      boost::signals2::signal<void ( vector<LabeledCloud>& )> userSignal_;
      boost::signals2::connection userSignalConnection_;
      
      // callback functions
	    void cameraCallback( std::string&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& );
	    void synchroCallback( vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& );

	    // Thread peramiters
	    bool updateThreadRunning;
	    boost::thread deviceUpdateThread;
	    std::vector< std::string > allSerialNumbers;
	    boost::signals2::mutex allSerialNumbersMutex;

      // Private Member functions
      std::vector< std::string > getConnectedDeviceSerialNumbers();
  
	    // Thread Private Member functions
	    void startUpdateThread();
	    void stopUpdateThread();
	    void updateThread();
    };
      
  } // multikinect
} // unr_rgbd

#endif // MULTI_GRAB_MANAGER
