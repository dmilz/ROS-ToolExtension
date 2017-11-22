#include <visualization_msgs/Marker.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <ros/ros.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <geometry_msgs/PointStamped.h>
#include <boost/circular_buffer.hpp>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


// msg storage of BUFFER at RATE Hz
#define RATE   20.0
#define BUFFER 2 //(unsigned int)RATE*5
#define ID_MAX 100

typedef geometry_msgs::Point PT;
typedef boost::circular_buffer<PT> CircBuf;

int id { 0 };


void vis(const CircBuf &cb_r, const CircBuf &cb_p, const ros::Publisher &pub_Vis);

int main(int argc, char **argv)
{
        ros::init(argc, argv, "rviz_points_to_trajectory_node");
        ros::NodeHandle n;
        ROS_INFO("Starting rviz_points_to_trajectory_node...");
        ros::NodeHandle nh;
        ros::Publisher pub_Vis = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
        tf::TransformListener tf_listener, tf_listener_predic;
        tf::StampedTransform trans;

        CircBuf cb_real(BUFFER);
        CircBuf cb_plan(BUFFER);

        ros::Rate rate(RATE);
        while (nh.ok())
        {
            try
            {
                tf_listener.lookupTransform("/world", "/r_tool", ros::Time(0), trans);
                PT p;
                p.x = trans.getOrigin().x();
                p.y = trans.getOrigin().y();
                p.z = trans.getOrigin().z();
                cb_real.push_back((PT) p);

                tf_listener_predic.lookupTransform("/world", "/xte_predicted", ros::Time(0), trans);
                p.x = trans.getOrigin().x();
                p.y = trans.getOrigin().y();
                p.z = trans.getOrigin().z();
                cb_plan.push_back((PT) p);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(0.1).sleep();
                continue;
            }
            vis(cb_real, cb_plan, pub_Vis);
            ros::spinOnce();
            rate.sleep();
        }

        return 0;
}



void vis(const CircBuf &cb_r, const CircBuf &cb_p, const ros::Publisher &pub_Vis)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(5);
    marker.ns = "real";
    marker.id = id; id = (id + 1) % ID_MAX;
    marker.pose.orientation.w = 1;
    for(auto i : cb_r)
    {
        marker.points.push_back(i);
    }
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    pub_Vis.publish( marker );


    marker.header.frame_id = "world";
    marker.ns = "predicted";
    marker.points.clear();
    for(auto i : cb_p)
    {
        marker.points.push_back(i);
    }
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    pub_Vis.publish( marker );
}
