#include "ros/ros.h"
#include <cmath>
#include <string>
#include <vector>
#include <time.h>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "visualization_msgs/Marker.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std;
using namespace boost;

double x[3];
double theta, psi;

double debug;

double dt = 0.1;
double u_yaw, u_pitch;
double vitesse;

string name;

void chatCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    u_pitch = msg->angular.y;
    u_yaw   = msg->angular.z;
}

void integration_euler()
{
    x[0] = x[0] + vitesse*dt*cos(psi)*cos(theta);
    x[1] = x[1] + vitesse*dt*cos(psi)*sin(theta);
    x[2] = x[2] - vitesse*dt*sin(psi);
    
    theta   = theta + dt*u_yaw;
    psi     = psi   + dt*u_pitch;
}

int main(int argc, char** argv)
{
        // Iterateur
    int i = 0;

        // Node
    ros::init(argc, argv, "simu_auv");
    ros::NodeHandle n;
    
        // Parametres initiaux
    n.param<double>("Pos_x",  x[0],  0.0);
    n.param<double>("Pos_y",  x[1],  0.0);
    n.param<double>("Pos_z",  x[2], -5.0);
    
    n.param<double>("Theta", theta,  0.0);
    n.param<double>(  "Psi",   psi,  0.0);
    
    n.param<double>(  "Cons_u_yaw",   u_yaw, 0.0);
    n.param<double>("Cons_u_pitch", u_pitch, 0.0);
    n.param<double>( "Vitesse_auv", vitesse, 1.0);
    
    n.param<string>("Name_auv", name, "unknown");
    
    ros::Rate loop_rate(25);
    
    tf::Quaternion q;

        // Liste de points
    geometry_msgs::Point pt_auv;
    pt_auv.x = x[0];
    pt_auv.y = x[1];
    pt_auv.z = x[2];

    vector<geometry_msgs::Point> pos(4000, pt_auv);

        // AUV marker
    ros::Publisher mark_auv = n.advertise<visualization_msgs::Marker>("/mark_auv", 1000);
    visualization_msgs::Marker msgs_auv;
    
        // Magnet marker
    ros::Publisher mark_magnet = n.advertise<visualization_msgs::Marker>("/mark_magnet", 1000);
    visualization_msgs::Marker msgs_magnet;

        // Line marker
    ros::Publisher mark_line = n.advertise<visualization_msgs::Marker>("/line_auv", 1000);
    visualization_msgs::Marker msgs_line;
    
        // Cons message subscription
    ros::Subscriber cons_sub = n.subscribe("cons_auv", 1000, chatCallback);
    
        // TF sender
    tf2_ros::TransformBroadcaster br;
    
        // TF auv
    geometry_msgs::TransformStamped tf_auv;
    
    tf_auv.header.frame_id    = "map";
    tf_auv.child_frame_id     = name;
    
    
        // TF magneto
    geometry_msgs::TransformStamped tf_magneto;
    
    tf_magneto.header.frame_id    = name;
    tf_magneto.child_frame_id     = "magneto";
    
    tf_magneto.transform.translation.x  =  -4.0;
    tf_magneto.transform.translation.y  =   0.0;
    tf_magneto.transform.translation.z  =   0.0;
    tf_magneto.transform.rotation.x     =   0.0;
    tf_magneto.transform.rotation.y     =   0.0;
    tf_magneto.transform.rotation.z     =   0.0;
    tf_magneto.transform.rotation.w     =   1.0;
    


    while(ros::ok())
    {
            // Evolution etat

        integration_euler();
        q.setRPY(0.0, psi, theta);  
        

            // Mise a jour liste
        pt_auv.x = x[0];
        pt_auv.y = x[1];
        pt_auv.z = x[2];

        pos.erase(pos.begin());
        pos.push_back(pt_auv);

            // Topic tf auv
        tf_auv.header.stamp = ros::Time::now();
        
        tf_auv.transform.translation.x  = x[0];
        tf_auv.transform.translation.y  = x[1];
        tf_auv.transform.translation.z  = x[2];
        tf_auv.transform.rotation.x     = q.getX();
        tf_auv.transform.rotation.y     = q.getY();
        tf_auv.transform.rotation.z     = q.getZ();
        tf_auv.transform.rotation.w     = q.getW();
        
        br.sendTransform(tf_auv);
        
            // Topic tf magneto
        tf_magneto.header.stamp = ros::Time::now();
        
        br.sendTransform(tf_magneto);

                // Topic pour RVIZ

            // Mark auv
        msgs_auv.header.frame_id = name;
        msgs_auv.header.stamp = ros::Time::now();
        msgs_auv.ns = ros::this_node::getNamespace();
        msgs_auv.id = 0;
        msgs_auv.type = visualization_msgs::Marker::MESH_RESOURCE;
        msgs_auv.action = visualization_msgs::Marker::ADD;
        msgs_auv.scale.x = 1.0;
        msgs_auv.scale.y = 1.0;
        msgs_auv.scale.z = 1.0;
        msgs_auv.color.a = 1.0;
        msgs_auv.color.r = 1.0;
        msgs_auv.color.g = 1.0;
        msgs_auv.color.b = 1.0;
        msgs_auv.mesh_resource = "package://cordeliere/mesh/auv.dae";
        
        mark_auv.publish(msgs_auv);
        
            // Mark magneto
        msgs_magnet.header.frame_id = "magneto";
        msgs_magnet.header.stamp = ros::Time::now();
        msgs_magnet.ns = ros::this_node::getNamespace();
        msgs_magnet.id = 0;
        msgs_magnet.type = visualization_msgs::Marker::CUBE;
        msgs_magnet.action = visualization_msgs::Marker::ADD;
        msgs_magnet.scale.x = 1.0;
        msgs_magnet.scale.y = 1.0;
        msgs_magnet.scale.z = 1.0;
        msgs_magnet.color.a = 1.0;
        msgs_magnet.color.r = 1.0;
        msgs_magnet.color.g = 0.0;
        msgs_magnet.color.b = 0.0;
        
        mark_magnet.publish(msgs_magnet);

            // Mark Line
        msgs_line.header.frame_id = "map";
        msgs_line.header.stamp = ros::Time::now();
        msgs_line.ns = ros::this_node::getNamespace();
        msgs_line.id = 0;
        msgs_line.type = visualization_msgs::Marker::LINE_STRIP;
        msgs_line.action = visualization_msgs::Marker::ADD;
        msgs_line.points = pos;
        msgs_line.scale.x = 0.2;
        msgs_line.color.a = 0.5;
        msgs_line.color.r = 0.0;
        msgs_line.color.g = 1.0;
        msgs_line.color.b = 0.0;

        mark_line.publish(msgs_line);
        
        ros::spinOnce();
        
        loop_rate.sleep();
    }

    return 0;
}