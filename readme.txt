This is the multikinect project.

This project was designed to be a abstraction for users so they would not have to worry about connecting and managing multiple point cloud streams.

The problem pcl::syncronization only allows for two streams to be syncronized, this means there is no efficent way to syncronize more then two streams. Fix, take ros::ApproximateTime policy algorithum and make it specific to point clouds.

More can be found for ros ApproximateTime policy here: http://www.ros.org/wiki/message_filters/ApproximateTime
