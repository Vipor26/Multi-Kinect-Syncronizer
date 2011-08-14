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
    multiGrabberManager::multiGrabberManager() : updateThreadRunning(false)
    {
      using std::vector;
      using std::string;
      
      allSerialNumbers = getConnectedDeviceSerialNumbers();

      // Start update thread
      startUpdateThread();
    }
    
    // Destructor
    multiGrabberManager::~multiGrabberManager()
    {
      // Stop update thread
      stopUpdateThread();
    }
    
    
    std::vector<std::string> multiGrabberManager::getAvailableSerialNumbers()
    {
      using std::vector;
      using std::string;
      vector< string > retVec;
      
      allSerialNumbersMutex.lock();
      retVec = allSerialNumbers;
      allSerialNumbersMutex.unlock();
      
      return retVec;
    }
    
    
    void multiGrabberManager::connect( std::vector<std::string> serial_list )
    {
      if( serial_list.size() == 0 ) // empty so use all
	serial_list = getAvailableSerialNumbers();
      else
	{ // check to make sure the requested are available
	  
	}
    }
    

// <><><>    Private Member Functions    <><><>
std::vector< std::string > multiGrabberManager::getConnectedDeviceSerialNumbers()
{
  using std::vector;
  using std::string;
  
  //PCL Drivers variables      
  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
      
  // all other variables
  unsigned devIter, curNumberDevices;
  vector< string > curSerialList;
  
  try	{
    curNumberDevices = driver.updateDeviceList();
    for( devIter=0; devIter<curNumberDevices; devIter++ )
    {
	    curSerialList.push_back( string( driver.getSerialNumber(devIter) ) );
    }
  }
  catch( ... ) {
  }
  return curSerialList;
}
    
    
// Thread Functions
void multiGrabberManager::startUpdateThread()
{
  if( !updateThreadRunning )
  {
    updateThreadRunning = true;
    deviceUpdateThread = boost::thread(&multiGrabberManager::updateThread, this );
  }
}
void multiGrabberManager::stopUpdateThread()
{
  if( updateThreadRunning )
  {
    updateThreadRunning = false;
    deviceUpdateThread.join();
  }
}

void multiGrabberManager::updateThread()
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
	  if( updateThreadRunning )	{
	  
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
