#include "ros/ros.h"
#include <cmath>
#include <string>
#include <vector>
#include <ctime>

#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "visualization_msgs/Marker.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "cordeliere/dist.h"

using namespace std;

double x, y, z;
double roll, pitch, yaw;

double detect_max;
double dist;

string name;

const double Pi = 3.14159265358979323846;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "magneto");
    ros::NodeHandle n;
    
        // Parametres initiaux
    n.param<string>("Name_auv", name, "unknown");
    n.param<double>("Dist_detect", detect_max, 5.0);
    
    ros::Rate loop_rate(25);
    
    tf::Quaternion q_mobile;
    
        // Magnetic field marker
    ros::Publisher magnet_mark = n.advertise<visualization_msgs::Marker>("/mark_magnet", 1000);
    visualization_msgs::Marker msgs_magnet;

        // Distance client
    ros::ServiceClient dist_client = n.serviceClient<cordeliere::dist>("/dist_magnet");
    cordeliere::dist dist_msg;

        // Tf Listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped tf_mobile_map;

        // Magnetic field points
    geometry_msgs::Point pt_magnet;
    vector<geometry_msgs::Point> magnet;

    while(ros::ok())
    {
        try{
            tf_mobile_map = tfBuffer.lookupTransform("map", name, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        x = tf_mobile_map.transform.translation.x;
        y = tf_mobile_map.transform.translation.y;
        z = tf_mobile_map.transform.translation.z;

        q_mobile.setX(tf_mobile_map.transform.rotation.x);
        q_mobile.setY(tf_mobile_map.transform.rotation.y);
        q_mobile.setZ(tf_mobile_map.transform.rotation.z);
        q_mobile.setW(tf_mobile_map.transform.rotation.w);

        tf::Matrix3x3(q_mobile).getRPY(roll, pitch, yaw);
        
        dist_msg.request.x = x;
        dist_msg.request.y = y;
        
        if (dist_client.call(dist_msg)) {
            dist = dist_msg.response.dist;
        } else {
            ROS_ERROR("Failed to call service");
        }

        //ROS_INFO("Pt: %lf %lf %lf | Dist: %lf", x, y, z, dist);
        if(1000*pow(dist*(2/detect_max), -3) >= 2.0)
        {
            pt_magnet.x = x;
            pt_magnet.y = y;
            pt_magnet.z = 5 + 1000*pow(dist*(2/detect_max), -3);
            
            //ROS_INFO("%lf", pt_magnet.z);

            magnet.push_back(pt_magnet);
        }

                // Topic pour RVIZ

            // Mark boat
        msgs_magnet.header.frame_id = "map";
        msgs_magnet.header.stamp = ros::Time::now();
        msgs_magnet.ns = ros::this_node::getNamespace();
        msgs_magnet.id = 0;
        msgs_magnet.type = visualization_msgs::Marker::POINTS;
        msgs_magnet.action = visualization_msgs::Marker::ADD;
        msgs_magnet.points = magnet;
        msgs_magnet.scale.x = 0.2;
        msgs_magnet.scale.y = 0.2;
        msgs_magnet.color.a = 1.0;
        msgs_magnet.color.r = 0.0;
        msgs_magnet.color.g = 0.0;
        msgs_magnet.color.b = 1.0;
        
        magnet_mark.publish(msgs_magnet);
        
        ros::spinOnce();
        
        loop_rate.sleep();
    }

    return 0;
}
