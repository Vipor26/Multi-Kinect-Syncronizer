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
*/

#include <iostream>
#include <vector>
#include <string>
#include <utility>

using std::iostream;
using std::vector;
using std::string;
using std::pair;

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h> // for devices
#include <pcl/io/openni_camera/openni_driver.h> // for driver
#include <pcl/visualization/pcl_visualizer.h> // for getting visalizers
#include <pcl/common/synchronizer.h> // for syncronizing streams

int main()
{
	unsigned numberDevices, di;
	vector< pair< string, pcl::Grabber*> > devices;
	pair< string, pcl::Grabber*> tempDevice;
	string serialNumber;

	openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();

	cout << endl;
	cout << "Welcome to the multiple kinect view test program" << endl;

	numberDevices = 0;
	try{
	  numberDevices = driver.updateDeviceList();

	  cout << "There are " << numberDevices << " connected to this computer" << endl;

	  if( numberDevices != 0 )
	  {
		  for( di = 0; di < numberDevices; di++ )
		  {
			  cout << "\tDevice " << di << ": info ( " << driver.getConnectionString(di) << " ) s#: ";
			  cout << driver.getSerialNumber(di) << endl;
		  }

		  cout << "Connecting to all devices" << endl;

		  for( di = 0; di < numberDevices; di++ )
		  {
			  serialNumber = string( driver.getSerialNumber(di) );

			  tempDevice.first = serialNumber;
			  tempDevice.second = new pcl::OpenNIGrabber(serialNumber);

			  devices.push_back( tempDevice );
		  }
    }
  }
  catch( ... )  {
  }
  if( numberDevices == 0 )  {
    cout << "Failed to see or connect to devices :( " << endl;
    return EXIT_SUCCESS;
  }


	// Clean up devices
	cout << "Deleting all connected devices" << endl;
	for( di = 0; di < numberDevices; di++ )
	{
		delete devices[di].second;
		devices[di].second = NULL;
	}
	devices.clear();

	cout << "Ending test, come back soon :)" << endl;
	cout << endl;
	return EXIT_SUCCESS;
}
