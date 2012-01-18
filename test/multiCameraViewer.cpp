#include <iostream>
#include <string>
using namespace std;

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
class SimpleOpenNIViewer
{
  public:
  SimpleOpenNIViewer () {}

  void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
  {
    //cout << cloud->header << endl;
    //cout << *cloud << endl;
    if (!viewer->wasStopped())
      viewer->showCloud (cloud,cloudName);
  }

  void init (pcl::Grabber* Interface, string CloudName, pcl::visualization::CloudViewer *Viewer)
  {
    interface = Interface;
    cloudName = CloudName;
    viewer = Viewer;


    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
    boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

    interface->registerCallback (f);
  }

  void start()
  {
    interface->start ();
  }

  void stop()
  {
    interface->stop ();
  }

  string cloudName;
  pcl::Grabber* interface;
  pcl::visualization::CloudViewer *viewer;
};

int main ()
{
  pcl::visualization::CloudViewer *viewer = new pcl::visualization::CloudViewer("PCL OpenNI multi Viewer");

  SimpleOpenNIViewer v1, v2;
  v1.init (new pcl::OpenNIGrabber("#1"),"Cloud_1",viewer);
  v2.init (new pcl::OpenNIGrabber("#2"),"Cloud_2",viewer);

  v1.start();
  v2.start();

  while (!viewer->wasStopped())
  {
    sleep (1);
  }
  v1.stop();
  v2.stop();

  return 0;
}
