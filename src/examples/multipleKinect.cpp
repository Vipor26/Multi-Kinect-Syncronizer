#include <stdlib.h>
#include <iostream>

#include <vector>
#include <string>
using namespace std;



#include <multiGrabManager.h>

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
