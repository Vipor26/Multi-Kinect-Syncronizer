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
#include <boost/signals2/mutex.hpp>

// Project Includes
#include <camera.h>

namespace unr_rgbd {
  namespace multikinect {

    // wraps the point cloud with a serialNumber
    struct labeledCloud
    {
      //	labeledCloud();
      //	labeledCloud( const labeledCloud &rhs );
      //	~labeledCloud();
      std::string serialNumber;
    };
    
    class multiGrabberManager : private boost::noncopyable
      {
      public:
	// Constructor Destructor
	multiGrabberManager();
	~multiGrabberManager();
	
	// available serial numbers
	std::vector<std::string> getAvailableSerialNumbers();
	
	// if nonSpecified will connect to all available
	void connect( std::vector<std::string> = std::vector<std::string>() ); //NOTCOMPLETE
	
	// functions to start stop the selected camera streams
	//void startSelected();
	//void stopSelected();
	
	// Polled functions
	//bool newDataAvailable();
	//vector<labeledCloud> getData();
	
	// callback functions
	// unsigned addCallback( const ??? callback );
	//returns the number of callbacks, and return -1 is callback index 
	//void removeCallback( unsigned i );
      private:
	
	// Connected Cammera peramiters
	std::map< std::string, pcl::Grabber* > connectedDevices;
	boost::bimap< std::string, unsigned > serialIndexBiMap;


	// Thread peramiters
	bool updateThreadRunning;
	boost::thread deviceUpdateThread;
	std::vector< std::string > allSerialNumbers;
	boost::signals2::mutex allSerialNumbersMutex;


	// Private Member functions
	void startUpdateThread();
	void stopUpdateThread();
	void updateThread();
      };
    
  } // multikinect
} // unr_rgbd

#endif // MULTI_GRAB_MANAGER
