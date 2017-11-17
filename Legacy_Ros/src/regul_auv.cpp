#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <cmath>

#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "visualization_msgs/Marker.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"

#include "cordeliere/depth.h"

#include "std_srvs/Empty.h"

using namespace std;
using namespace boost;
using namespace cv;

double gis;
double x_auv, y_auv, z_auv;
double target_x, target_y, target_z;
double dist_z;
double roll_auv, pitch_auv, yaw_auv;
double roll_target, pitch_target, yaw_target;
double pitch_cible;
double diff_z;

double pitch_hat;

double depth, deriv_sup;

double u_yaw, u_pitch;
double u_max;

double dif;

double e;

double depth_step, depth_step100;

string name;
string regul;

const double Pi = 3.14159265358979323846;

bool checkDistance()
{
    double dist = pow(pow(target_x - x_auv,2) + pow(target_y - y_auv,2), 0.5);
    
    return (dist<3.0) ? true : false;
}


int main(int argc, char** argv)
{
        // Iterateur
    int i = 0;

        // Noeud
    ros::init(argc, argv, "regul_auv");
    ros::NodeHandle n;

    ros::Rate loop_rate(50);

        // Parametres initiaux
    n.param<double>("Pos_x",  x_auv,  0.0);
    n.param<double>("Pos_y",  y_auv,  0.0);
    n.param<double>("Pos_z",  z_auv, -5.0);

    n.param<string>("Name_auv", name, "unknown");
    n.param<string>("Regul", regul, "unknown");

    n.param<double>("U_max", u_max, 1.0);

    tf::Quaternion q_auv;
    tf::Quaternion q_target;

        // Data AUV
    ros::Publisher data_auv = n.advertise<geometry_msgs::Pose>("/data_auv", 1000);
    geometry_msgs::Pose msgs_auv;

        // Box marker
    ros::Publisher mark_box = n.advertise<visualization_msgs::Marker>("/mark_box", 1000);
    visualization_msgs::Marker msgs_box;

        // Tf Listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped listenTarget;
    geometry_msgs::TransformStamped listenAUV;
    
        // New GPS client
    ros::ServiceClient new_gps_client = n.serviceClient<std_srvs::Empty>("/new_gps_trigger");
    std_srvs::Empty null_msg;

        // Depth client
    ros::ServiceClient depth_client = n.serviceClient<cordeliere::depth>("/right_depth");
    cordeliere::depth depth_msg;

        // Consigne
    ros::Publisher cons_pub = n.advertise<geometry_msgs::Twist>("cons_auv", 1000);
    geometry_msgs::Twist cons_msgs;


    double compt = ros::Time::now().toSec();

    while(ros::ok())
    { 
            // Coord AUV
        try{
            listenAUV = tfBuffer.lookupTransform("map", name, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        x_auv = listenAUV.transform.translation.x;
        y_auv = listenAUV.transform.translation.y;
        z_auv = listenAUV.transform.translation.z;

        q_auv.setX(listenAUV.transform.rotation.x);
        q_auv.setY(listenAUV.transform.rotation.y);
        q_auv.setZ(listenAUV.transform.rotation.z);
        q_auv.setW(listenAUV.transform.rotation.w);

        tf::Matrix3x3(q_auv).getRPY(roll_auv, pitch_auv, yaw_auv);

        msgs_auv.position.x = x_auv;
        msgs_auv.position.y = y_auv;
        msgs_auv.position.z = z_auv;

        msgs_auv.orientation.x = q_auv.getX();
        msgs_auv.orientation.y = q_auv.getY();
        msgs_auv.orientation.z = q_auv.getZ();
        msgs_auv.orientation.w = q_auv.getW();

        data_auv.publish(msgs_auv);


            // Coord cible
        try{
            listenTarget = tfBuffer.lookupTransform("map", "target", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        target_x = listenTarget.transform.translation.x;
        target_y = listenTarget.transform.translation.y;

        q_target.setX(listenTarget.transform.rotation.x);
        q_target.setY(listenTarget.transform.rotation.y);
        q_target.setZ(listenTarget.transform.rotation.z);
        q_target.setW(listenTarget.transform.rotation.w);
    
        tf::Matrix3x3(q_target).getRPY(roll_target, pitch_target, yaw_target);

        double det;

        if(regul.compare("yaw") == 0)
        {
            gis = atan2(target_y - y_auv, target_x - x_auv);
        } else if(regul.compare("line") == 0) {
            det     = (x_auv - target_x)*(-sin(yaw_target)) - (-cos(yaw_target))*(y_auv - target_y);
            gis     = (yaw_target - atan(det)) - yaw_auv;
        }

        e   = 2*atan(tan(gis/2));
        if(e < 0.0005 && e > -0.0005) {
            u_yaw = 0.0;
        } else if (e > 2.5 || e < -2.5) {
            u_yaw = 1.0;
        } else {
            u_yaw = tanh(e)/4.0;
        }

        		// Pitch regulation

        depth_msg.request.x             = x_auv;
        depth_msg.request.y             = y_auv;
        depth_msg.request.yaw_deriv     = yaw_auv;
        
        if (depth_client.call(depth_msg))
        {
            depth       = depth_msg.response.depth;
            deriv_sup   = depth_msg.response.deriv_sup;
        } else {
            ROS_ERROR("Failed to call service depth from regul_auv");
        }

        dist_z  = (depth+2) - z_auv;

        pitch_hat   = -1*tanh(dist_z) - atan(deriv_sup);
        u_pitch     = -1*sin(pitch_auv - pitch_hat);

            // Publication des consignes
        cons_msgs.angular.y = u_pitch;
        cons_msgs.angular.z = u_yaw;
        
        cons_pub.publish(cons_msgs);
        

            // Distance check
        if(checkDistance() && ros::Time::now().toSec()-compt > 2.0)
        {
            if (new_gps_client.call(null_msg))
            {
                ROS_INFO("Message from regul_boat delivered");
            } else{
                ROS_ERROR("Failed to call service");
            }
            compt = ros::Time::now().toSec();
        }        

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}