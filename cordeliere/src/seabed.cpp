#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"
#include <string>
#include <cmath>
#include <ctime>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <fstream>
#include <vector>

#include <ros/console.h>

#include "std_msgs/Float64.h"
#include "visualization_msgs/Marker.h"

#include "cordeliere/depth.h"
#include "cordeliere/dist.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std;
using namespace cv;

const double Pi = 3.14159265358979323846;

double depth;
double dist;

Mat heightXML;

double scaleXY, scaleZ, transZ;

double min_x, max_x, min_y, max_y;
double l_x, l_y;

int l_map;
int x_cannon, y_cannon, z_cannon;

string name;

void build_detect()
{
	Mat detect, final;
	detect = Mat::ones(l_map, l_map, CV_64F);

	int i, j;

    double var_x, var_y;

    double min = 100000000000;
    double detect_max = 2.0;
    double distMag;
    double val;

    for(i=0; i < detect.rows; i++)
    {
        var_x = i - (l_map/2);
        for(j = 0; j < detect.cols; j++)
        {
            var_y = j - (l_map/2);
            distMag = pow(pow(x_cannon - var_x, 2) + pow(y_cannon - var_y, 2), 0.5);
     		if(distMag == 0)
     			distMag = 0.1;
            val = 1000*pow(distMag*(2/detect_max), -3);
			detect.at<double>(i, j) = val;
			if(val<=min)
				min = val;
        }
    }

    detect 	-= min;
    detect.convertTo(detect, CV_8UC1);

	try {
		imwrite("/home/lallemfa/Stage/resultImg/test.png", detect);
	} catch (runtime_error& ex) {
	    ROS_INFO("Erreur");
	}
}

bool askDist(cordeliere::dist::Request &req, cordeliere::dist::Response &res)
{
    double x, y, z;

    x           = req.x;
    y           = req.y;
    z           = req.z;

    res.dist = pow(pow(x_cannon - x, 2) + pow(y_cannon - y, 2) + pow(z_cannon - z, 2), 0.5);

    return true;
}

bool askDepth(cordeliere::depth::Request &req, cordeliere::depth::Response &res)
{
    double x, y;
    double yaw_deriv;

    x           = req.x;
    y           = req.y;
    yaw_deriv   = req.yaw_deriv;

    int i, j;

    if(x < scaleXY*max_x && x > scaleXY*min_x && y < scaleXY*max_y && y > scaleXY*min_y)
    {
        i = ((x-scaleXY*min_x)/(scaleXY*max_x-scaleXY*min_x))*l_x;
        j = ((y-scaleXY*min_y)/(scaleXY*max_y-scaleXY*min_y))*l_y;
        res.depth       = transZ + scaleZ*heightXML.at<double>(i, j);
        res.deriv_sup   = scaleZ*(cos(yaw_deriv)*(heightXML.at<double>(i, j+cos(yaw_deriv)*1)-heightXML.at<double>(i, j)) + sin(yaw_deriv)*(heightXML.at<double>(i+sin(yaw_deriv)*1, j)-heightXML.at<double>(i, j)));
    } else {
        res.depth       = transZ;
        res.deriv_sup   = 0;
    }
    

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth");
    ros::NodeHandle n;
    
    n.param<string>("Name", name, "unknown");
    n.param<int>("Square_map", l_map, 10);
    n.param<double>("Depth", depth, -8);

        // Parametres initiaux
    
    ros::Rate loop_rate(50);
    
        // Service depth
    ros::ServiceServer depthSrv = n.advertiseService("/right_depth", askDepth);

        // Service dist
    ros::ServiceServer distSrv = n.advertiseService("/dist_magnet", askDist);

        // Heightmap
    
    scaleXY = 8.0;
    scaleZ  = 1.0;
    transZ  = -10.0;
    
    FileStorage fs;
    fs.open("/home/lallemfa/Stage/Img_Heightmap/world1/heightmap.xml", FileStorage::READ);
    fs["heightmap"] >> heightXML;
    fs["min_x"] >> min_x;
    fs["max_x"] >> max_x;
    fs["min_y"] >> min_y;
    fs["max_y"] >> max_y;
    fs.release();

    heightXML *= -1;

    l_x = heightXML.rows;
    l_y = heightXML.cols;

    	// Heightmap marker
    ros::Publisher heightmap_mark = n.advertise<visualization_msgs::Marker>("/mark_heightmap", 1000);
    visualization_msgs::Marker msgs_heightmap;

    msgs_heightmap.pose.position.z = transZ;
    
            // TF cannon

    x_cannon = -25;
    y_cannon = 15;

    z_cannon = depth + 5*sin((2*Pi/l_map)*x_cannon)*sin((2*Pi/l_map)*y_cannon) - 2;

    tf2_ros::TransformBroadcaster br;
    
    geometry_msgs::TransformStamped tfCannon;
    
    tfCannon.header.frame_id    = "map";
    tfCannon.child_frame_id     = "cannon";
    tfCannon.transform.translation.x      = x_cannon;
    tfCannon.transform.translation.y      = y_cannon;
    tfCannon.transform.translation.z      = z_cannon;
    tfCannon.transform.rotation.x     = 0.0;
    tfCannon.transform.rotation.y     = 0.0;
    tfCannon.transform.rotation.z     = 0.0;
    tfCannon.transform.rotation.w     = 1.0;

            // MArk cannon
    ros::Publisher cannon_mark = n.advertise<visualization_msgs::Marker>("/mark_cannon", 1000);
    visualization_msgs::Marker msgs_cannon;
    
    msgs_cannon.header.frame_id = "cannon";
    msgs_cannon.header.stamp = ros::Time::now();
    msgs_cannon.ns = ros::this_node::getNamespace();
    msgs_cannon.id = 0;
    msgs_cannon.type = visualization_msgs::Marker::MESH_RESOURCE;
    msgs_cannon.action = visualization_msgs::Marker::ADD;
    msgs_cannon.scale.x = 0.05;
    msgs_cannon.scale.y = 0.05;
    msgs_cannon.scale.z = 0.05;
    msgs_cannon.color.a = 1.0;
    msgs_cannon.color.r = 1.0;
    msgs_cannon.color.g = 1.0;
    msgs_cannon.color.b = 1.0;
    msgs_cannon.mesh_resource = "package://cordeliere/mesh/cannon.stl";

    build_detect();

    while(ros::ok())
    {      
            // Update TF cannon
        tfCannon.header.stamp = ros::Time::now();

        br.sendTransform(tfCannon);

        msgs_cannon.header.frame_id = "cannon";
        msgs_cannon.header.stamp = ros::Time::now();
        msgs_cannon.ns = ros::this_node::getNamespace();

        cannon_mark.publish(msgs_cannon);

                // Topic pour RVIZ

            // Heightmap marker
        msgs_heightmap.header.frame_id = "map";
        msgs_heightmap.header.stamp = ros::Time::now();
        msgs_heightmap.ns = ros::this_node::getNamespace();
        msgs_heightmap.id = 0;
        msgs_heightmap.type = visualization_msgs::Marker::MESH_RESOURCE;
        msgs_heightmap.action = visualization_msgs::Marker::ADD;
		msgs_heightmap.mesh_resource = "package://cordeliere/mesh/world1.dae";
        msgs_heightmap.scale.x = scaleXY;
        msgs_heightmap.scale.y = scaleXY;
        msgs_heightmap.scale.z = scaleZ;
        msgs_heightmap.color.a = 1.0;
        msgs_heightmap.color.r = 1.0;
        msgs_heightmap.color.g = 1.0;
        msgs_heightmap.color.b = 1.0;
        
        heightmap_mark.publish(msgs_heightmap);
        
        ros::spinOnce();
        
        loop_rate.sleep();
    }
    return 0;
}