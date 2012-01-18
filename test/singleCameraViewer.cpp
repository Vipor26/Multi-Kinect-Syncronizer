#include <iostream>
using namespace std;

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
class SimpleOpenNIViewer
{
  public:
  SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

  void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
  {
    //cout << cloud->header << endl;
    //cout << *cloud << endl;
    if (!viewer.wasStopped())
      viewer.showCloud (cloud);
  }

  void run ()
  {
    pcl::Grabber* interface = new pcl::OpenNIGrabber();

    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
    boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

    interface->registerCallback (f);

    interface->start ();

    while (!viewer.wasStopped())
    {
      sleep (1);
    }

    interface->stop ();
  }

  pcl::visualization::CloudViewer viewer;
};

int main ()
{
  SimpleOpenNIViewer v;
  v.run ();
  return 0;
}
