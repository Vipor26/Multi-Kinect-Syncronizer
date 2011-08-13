#include <multiGrabManager.h>

//<><><> CAMERA CLASS <><><>
cameraType::cameraType( boost::function<void (std::string&, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f ) :
serialNumber_(NULL),
device_(NULL)
{
	managerfunction_ = f;

	//TODO set connection betwwen this and f
	//	managerSignalConnection_

	camerafunction_ = boost::bind( &cameraType::cameraCallBack, this, _1);
}
cameraType::~cameraType()
{
}
void cameraType::initalize( std::string serialNumber )
{
	device_ =  new(std::nothrow) pcl::OpenNIGrabber(serialNumber.c_str());
	//if( device_ == NULL )
	//	throw

	serialNumber_ = serialNumber;

	camSignalConnection_ = device_->registerCallback( camerafunction_ );
}



void cameraType::cameraCallBack ( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud )
{
}

//<><><>  MANAGER CLASS  <><><>

// Constructor
multiGrabberManager::multiGrabberManager()
{
	// Set up update thread variables
	updateThreadRunning = false;

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
	using namespace std;
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
	using namespace std;
	//PCL Drivers variables
	openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();

	//Boost sleep variable
	boost::posix_time::seconds sleepPeriod(1);  // 0.5 sec

	// all other variables
	unsigned devIter, curNumberDevices;
	vector< string > curSerialList;
	while( updateThreadRunning )
	{
		curSerialList.clear();
		try	{
			curNumberDevices = driver.updateDeviceList();
			for( devIter=0; devIter<curNumberDevices; devIter++ )
			{
				curSerialList.push_back( string( driver.getSerialNumber(devIter) ) );
			}
		}
		catch( ... ) { //openni_wrapper::OpenNIException E)	{
			// no cameras connected
			curNumberDevices = 0;
		}
		cout << curNumberDevices << endl; //REMOVE

		if( updateThreadRunning )	{
			allSerialNumbersMutex.lock();
			allSerialNumbers = curSerialList;
			allSerialNumbersMutex.unlock();

			// sleep for (construct val) seconds
			boost::this_thread::sleep(sleepPeriod);
		}
	}
}
