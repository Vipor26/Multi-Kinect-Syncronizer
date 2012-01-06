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

#include <multiGrabManager.h>

namespace unr_rgbd {
  namespace multikinect {

    // Constructor
    MultiGrabberManager::MultiGrabberManager()
      : updateThreadRunning(false)
      , bufferSize_(5)
      , driver_(openni_wrapper::OpenNIDriver::getInstance()) // get a refrence to the openni driver
    {
      using std::vector;
      using std::string;


      allSerialNumbers = getConnectedDeviceSerialNumbers();

      sync_.registerCallback( boost::bind( &MultiGrabberManager::synchroCallback, this, _1 ) );

      // Start update thread
      startUpdateThread();
    }

    // Destructor
    MultiGrabberManager::~MultiGrabberManager()
    {
      // Stop update thread
      stopUpdateThread();
    }


    std::vector<std::string> MultiGrabberManager::getAvailableSerialNumbers()
    {
      using std::vector;
      using std::string;
      vector< string > retVec;

      allSerialNumbersMutex.lock();
      retVec = allSerialNumbers;
      allSerialNumbersMutex.unlock();

      return retVec;
    }

    void MultiGrabberManager::connect( std::vector<std::string> serial_list )
    {
      using std::vector;
      using std::string;
      vector< string > connectedSerialList = getAvailableSerialNumbers();
      unsigned j, sj;
      unsigned numberStreams;

      // If user did not specify camera connect to all
      if( serial_list.size() == 0 ) { // empty so use all
        serial_list = connectedSerialList;
        numberStreams = serial_list.size();
      }
      else {
        // check to make sure that the list of cameras that the user
        //   specifed is a subset of connected cameras
        if( connectedSerialList.size() < serial_list.size() )
          {
            throw CamerasNotFoundException();
          }
        sort( serial_list.begin(), serial_list.end() );
        sort( connectedSerialList.begin(), connectedSerialList.end() );
        j=0;
        numberStreams = serial_list.size();
        sj = connectedSerialList.size();
        for( unsigned i=0; i<numberStreams; i++ )  {
          while(( serial_list[i] != connectedSerialList[j] ) && (j < sj) )  {
            j++;
          }
          if( j == sj ) {
            throw CamerasNotFoundException();
          }
        }
        // is a subset continue
      }

      // Clean up
      if( Cameras_.empty() == false ) {
        Cameras_.clear();
      }
      serialIndexBiMap_.clear();

      // Set up the rest
      Cameras_.resize( numberStreams );
      for( unsigned i=0; i<numberStreams; i++ )
        {
          //boost::shared_ptr<pcl::Grabber> deviceGrabber;
          boost::shared_ptr<openni_wrapper::OpenNIDevice> device;

          
          boost::function<void(std::string&, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >&)> func;
          std::string serialName;

          device = driver_.getDeviceBySerialNumber(serial_list[i]);
          
          //get the serial name of this device;
          serialName = serial_list[i];
          func = boost::bind( &MultiGrabberManager::cameraCallback, this, _1, _2 );

          //make sure the device was found
          if( device == NULL )
          {
            //device could not be found by that serial number
          }
          // TODO change camera initalize to take

          
          boost::shared_ptr<pcl::OpenNIGrabber> deviceGrabber( new pcl::OpenNIGrabber(serialName) );

          Cameras_[i].initalize( deviceGrabber, serialName, func);

          serialIndexBiMap_.insert( StrIdxPair( serial_list[i], i ) );
        }

      sync_.initalize( numberStreams, bufferSize_ );

    }

    void MultiGrabberManager::startSelected() {
      unsigned numberStreams = Cameras_.size();
      for( unsigned i=0; i<numberStreams; i++ ) {
        Cameras_[i].start();
      }
    }
    void MultiGrabberManager::stopSelected()  {
      unsigned numberStreams = Cameras_.size();
      for( unsigned i=0; i<numberStreams; i++ ) {
        Cameras_[i].stop();
      }
    }

    void MultiGrabberManager::registerCallback( boost::function< void ( vector<LabeledCloud>& ) > func )
    {
      if( userSignalConnection_.connected() == true ) {
        userSignalConnection_.disconnect();
      }
      userSignalConnection_ =  userSignal_.connect( func );
    }

    // <><><>    Private Member Functions    <><><>
    void MultiGrabberManager::cameraCallback(std::string &serialNum, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud )
    {
      unsigned streamIndex = ( *serialIndexBiMap_.left.find( serialNum ) ).second;
      sync_.add( streamIndex, cloud );
    }

    void MultiGrabberManager::synchroCallback( vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds )
    {

      using std::string;

      unsigned i, si;
      string cameraName;
      vector<LabeledCloud> labeledClouds;

      si = clouds.size();

      labeledClouds.resize( si );
      for( i=0; i<si; i++ ) {
        cameraName = ( *serialIndexBiMap_.right.find( i ) ).first;
        labeledClouds[i].serialNumber = cameraName;
        labeledClouds[i].cloud = clouds[i];
      }
      userSignal_( labeledClouds );
    }

    std::vector< std::string > MultiGrabberManager::getConnectedDeviceSerialNumbers()
    {
      using std::vector;
      using std::string;

      //PCL Drivers variables



      // all other variables
      unsigned devIter, curNumberDevices;
      vector< string > curSerialList;

      try       {
        curNumberDevices = driver_.updateDeviceList();
        for( devIter=0; devIter<curNumberDevices; devIter++ )
          {
            curSerialList.push_back( string( driver_.getSerialNumber(devIter) ) );
          }
      }
      catch( ... ) {
      }
      return curSerialList;
    }


    // Thread Functions
    void MultiGrabberManager::startUpdateThread()
    {
      if( !updateThreadRunning )
        {
          updateThreadRunning = true;
          deviceUpdateThread = boost::thread(&MultiGrabberManager::updateThread, this );
        }
    }
    void MultiGrabberManager::stopUpdateThread()
    {
      if( updateThreadRunning )
        {
          updateThreadRunning = false;
          deviceUpdateThread.join();
        }
    }

    void MultiGrabberManager::updateThread()
    {
      using std::vector;
      using std::string;

      //Boost sleep variable
      boost::posix_time::seconds sleepPeriod(1);  // wait for 1 sec

      // all other variables
      vector< string > curSerialList;

      while( updateThreadRunning )
        {
          // Command is blocking for x period of time
          curSerialList = getConnectedDeviceSerialNumbers();

          // because previous command is blocking
          if( updateThreadRunning )     {

            allSerialNumbersMutex.lock();
            allSerialNumbers = curSerialList;
            allSerialNumbersMutex.unlock();

            // sleep for (construct val) seconds
            boost::this_thread::sleep(sleepPeriod);
          }
        }
    }
  } // multikinect
} // unr_rgbd
