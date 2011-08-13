// STD includes
#include <string.h>
#include <vector>
#include <map>

//#include <iostream> //REMOVE

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h> // for devices
#include <pcl/io/openni_camera/openni_driver.h> // for driver
#include <pcl/io/openni_camera/openni_exception.h>

// Boost Includes
#include <boost/thread.hpp>
#include <boost/date_time.hpp> // for thread sleep
#include <boost/noncopyable.hpp> // to make class not copiable
#include <boost/signals2/mutex.hpp>

// wraps the point cloud with a serialNumber
struct labeledCloud
{
//	labeledCloud();
//	labeledCloud( const labeledCloud &rhs );
//	~labeledCloud();
	std::string serailNumber;
};
/*
class cameraType
{
public:
	cameraType();
	~cameraType();
	bool initalize( string serialNumber );
	signal
private:
};*/

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
	//	map< string, ??? > clouds;


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
