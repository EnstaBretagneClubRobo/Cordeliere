#include "ros/ros.h"
#include <string>
#include <cmath>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "visualization_msgs/Marker.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "std_srvs/Empty.h"

using namespace std;

const double Pi = 3.14159265358979323846;

double l_map;
double tar_x, tar_y, tar_x_B, tar_y_B;
int sizeList;

bool nextCoord;

string name;
string target_mode;

bool isNextGPSneeded(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    nextCoord = true;
    ROS_INFO("Message received");
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "target");
    ros::NodeHandle n;
    
        // Parametres initiaux
    n.param<string>("Name_target", name, "unknown");
    n.param<string>("Target_mode", target_mode, "unknown");

    n.param<double>("Square_map", l_map, 10.0);

    ros::Rate loop_rate(50);

    tf::Quaternion q;

        // Message pour représentation de la cible
    ros::Publisher mark_pub = n.advertise<visualization_msgs::Marker>("/mark_target", 1000);
    visualization_msgs::Marker msgs_target;
    
    msgs_target.pose.position.x = 0.0;
    msgs_target.pose.position.y = 0.0;
    msgs_target.pose.position.z = 0.0;
    
        // Trigger GPS
    ros::ServiceServer need_gps_srv = n.advertiseService("/new_gps_trigger", isNextGPSneeded);
    
        // Message tf
    
    tf2_ros::TransformBroadcaster br;
    
    geometry_msgs::TransformStamped tfStamp;
    
    tfStamp.header.frame_id    = "map";
    tfStamp.child_frame_id     = name;

    tfStamp.transform.translation.z  = 0.0;
    

        // Courbe Lissajous
    double t = 0.0;
    
        // Init
    int compteur = 0;

    nextCoord = true;
    int i, j;
    double var_x, var_y;

        // GPS Vector
    vector<geometry_msgs::Point> gps;
    geometry_msgs::Point pt_gps;

    int invert = 1;

        // Line vector
    vector<geometry_msgs::Point> line;

    if(target_mode.compare("GPS") || target_mode.compare("line"))
    {
        for(i=0.0; i <= l_map; i += 10)
        {
            var_x = i - (l_map/2);

            pt_gps.x = var_x;
            pt_gps.y = invert*l_map/2;
            pt_gps.z = 0.0;
            
            gps.push_back(pt_gps);
            
            invert *= -1;
            
            pt_gps.x = var_x;
            pt_gps.y = invert*(l_map/2 + 15);
            pt_gps.z = 0.0;
            
            gps.push_back(pt_gps);
        }
    }

    sizeList = gps.size();
    
    while(ros::ok())
    {
        if(target_mode.compare("lissajous") == 0)
        {
            // Courbe Lissajous
            tar_x = 50*cos(5*t);
            tar_y = 50*sin(t);
            t += 0.001;
        } else if (target_mode.compare("GPS") == 0) {
            if(nextCoord)
            {
                tar_x = gps.at(compteur).x;
                tar_y = gps.at(compteur).y;

                compteur = (compteur+1 >= sizeList) ? 0 : compteur+1;

                nextCoord = false;
            }
        } else if (target_mode.compare("line") == 0) {
            if(nextCoord)
            {
                tar_x_B = gps.at(compteur).x;
                tar_y_B = gps.at(compteur).y;

                compteur = (compteur+1 >= sizeList) ? 0 : compteur+1;

                tar_x = gps.at(compteur).x;
                tar_y = gps.at(compteur).y;

                compteur = (compteur+1 >= sizeList) ? 0 : compteur+1;

                nextCoord = false;

                q.setRPY(0, 0, atan2(tar_y-tar_y_B, tar_x-tar_x_B));

                tfStamp.transform.rotation.x     = q.getX();
                tfStamp.transform.rotation.y     = q.getY();
                tfStamp.transform.rotation.z     = q.getZ();
                tfStamp.transform.rotation.w     = q.getW();
            }
        } else {
            // Origin
            tar_x = 0.0;
            tar_y = 0.0;
        }

        //ROS_INFO("Coord: %lf %lf", tar_x, tar_y);

        tfStamp.header.stamp                = ros::Time::now();

        tfStamp.transform.translation.x     = tar_x;
        tfStamp.transform.translation.y     = tar_y;
        
        br.sendTransform(tfStamp);

            // Topic pour RVIZ
        
        msgs_target.header.stamp    = ros::Time::now();
        msgs_target.ns              = ros::this_node::getNamespace();
        msgs_target.id              = 0;
        msgs_target.action          = visualization_msgs::Marker::ADD;
        msgs_target.color.a         = 1.0;
        msgs_target.color.r         = 1.0;
        msgs_target.color.g         = 1.0;
        msgs_target.color.b         = 1.0;

        if(target_mode.compare("line") == 0)
        {
            msgs_target.header.frame_id = "map";
            msgs_target.type            = visualization_msgs::Marker::LINE_LIST;
            msgs_target.points          = gps;
            msgs_target.scale.x         = 0.2;
        } else {
            msgs_target.header.frame_id = name;
            msgs_target.type            = visualization_msgs::Marker::CYLINDER;
            msgs_target.scale.x         = 1.0;
            msgs_target.scale.y         = 1.0;
            msgs_target.scale.z         = 1.0;
        }
        
        mark_pub.publish(msgs_target);
        
        ros::spinOnce();
        
        loop_rate.sleep();
    }

    return 0;
}
