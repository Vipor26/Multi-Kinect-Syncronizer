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

#include <vector>
#include <string>
using namespace std;

#include <multiGrabManager.h>

using namespace unr_rgbd;
using namespace multikinect;

int main( int argc, char ** atgv )
{
	cout << endl;
	cout << "Welcome to the multiple kinect class tester" << endl;

	multiGrabberManager masterInterface;
	cout << "Running update Test Press enter to cont. " << endl;	
	cin.get();
	
	vector<string> connectedCameras = masterInterface.getAvailableSerialNumbers();


	cout << "The Connected Cameras Serial Numbers are ..." << endl;
	if( connectedCameras.size() == 0 )
	{
		cout << "Error: no cameras connected :(" << endl;
		return EXIT_SUCCESS;
	}
	for(unsigned i=0; i<connectedCameras.size(); i++ )
	{
		cout << "\tCamera " << i << ": " << connectedCameras[i] << endl;
	}

	masterInterface.connect();

	cout << "Tester is exiting" << endl;
	cout << endl;
	return EXIT_SUCCESS;
}
