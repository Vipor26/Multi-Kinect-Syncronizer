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

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
using namespace std;

#include <multiGrabManager.h>

using namespace unr_rgbd;
using namespace multikinect;

#include <pcl/visualization/pcl_visualizer.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer (std::vector<LabeledCloud> &clouds )
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new  
                                      pcl::visualization::PCLVisualizer ("Multi Cloud Viewer"));
  viewer->initCameraParameters ();
  
  int i, numC = (int)clouds.size();
  
  for( i=0; i< numC; i++ )
  {
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, i);
    viewer->setBackgroundColor (0, 0, 0, i);
    //viewer->addText(clouds[i].serialNumber, i);
    
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(clouds[i].cloud);
    //viewer->addPointCloud<pcl::PointXYZRGB> (clouds[i].cloud, rgb, clouds[i].serialNumber, i);
  }
}
/*
while (!viewer->wasStopped ())
{
  viewer->spinOnce (100);
  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}
*/
void multiKinectCB( vector<LabeledCloud>& ) {

  cout << "Ha !" << endl;
}

int main( int argc, char ** atgv )
{
	cout << endl;
	cout << "Welcome to the multiple kinect library tester" << endl;

  cout << "\tInitalizing (please wait)" << endl;
	MultiGrabberManager masterInterface;
	
	vector<string> connectedCameras = masterInterface.getAvailableSerialNumbers();

	cout << "The Connected Cameras Serial Numbers are ..." << endl;
	if( connectedCameras.size() == 0 )
	{
		cout << "\tError: no cameras connected :(" << endl;
    cout << "Tester is exiting" << endl;
		return EXIT_SUCCESS;
	}

	for(unsigned i=0; i<connectedCameras.size(); i++ )
	{
		cout << "\tCamera " << i << ": " << connectedCameras[i] << endl;
	}

  cout << " Binding masterInterface callback ..." << endl;
  masterInterface.registerCallback( boost::bind( &multiKinectCB, _1 ) );

  cout << " Connecting to cameras ..." << endl;
	masterInterface.connect(connectedCameras);


  cout << " Starting ..." << endl;
  masterInterface.startSelected();


	cout << "Tester is exiting\b" << endl;
	cout << endl;
	return EXIT_SUCCESS;
}
