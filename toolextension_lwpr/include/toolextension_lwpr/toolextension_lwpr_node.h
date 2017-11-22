#ifndef TOOLEXTENSION_LWPR_TOOLEXTENSION_LWPR_NODE_H
#define TOOLEXTENSION_LWPR_TOOLEXTENSION_LWPR_NODE_H

#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <stddef.h>
#include <iostream>
#include <sstream>
#include <deque>
#include <vector>
#include <chrono>
#include <signal.h>

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

// For printing vectors
template<typename T>
std::ostream &operator <<(std::ostream &os, const std::vector<T> &v)
{
   using namespace std;
   os<<"[ ";
   copy(v.begin(), v.end(), ostream_iterator<T>(os, " "));
   os<<"]";
   return os;
}

// Absolute value
template<typename T> inline const T abs(T const & x)
{
    return ( x<0 ) ? -x : x;
}



#include "misc/config.h"


#define LWPRLEARNER_VERBOSE
#include "learner/lwpr_learner.h"


#include "lwpr/include/lwpr.hh"

#include "node.h"


#endif /* TOOLEXTENSION_LWPR_TOOLEXTENSION_LWPR_NODE_H */
