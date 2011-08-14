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
			  tempDevice.second = new pcl::OpenNIGrabber(serialNumber.c_str());

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
