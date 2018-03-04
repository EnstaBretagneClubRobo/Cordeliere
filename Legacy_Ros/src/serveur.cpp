#include <zmq.hpp>

#include "ros/ros.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/NavSatFix.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <errno.h>
#include <netinet/in.h>
#include <netdb.h>
#include <signal.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>


double x, y, z, yaw, pitch, roll;

pid_t child_pid, wpid;
int status;

	//  Initialisation du serveur

/* Fonction serveur:
Initialise le serveur
Return
	int 		-> ID du socket
*/
int serveur()
{
	char nom[30];
	int socket_RV;
	int socket_service;
	struct sockaddr_in adr;
	socklen_t lgadresse;
	
	int option = 1;

/*
	création du socket
*/

	if((socket_RV=socket(AF_INET, SOCK_STREAM, 0)) == -1) // Cree le socket
		{
		perror("Unable to create socket");
		exit(1);
		}

	setsockopt(socket_RV, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

	adr.sin_family 		= AF_INET;
	adr.sin_port 		= htons(29200);
	adr.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(socket_RV, (struct sockaddr *) &adr, sizeof(adr))==-1) // Bind du socket avec l'adresse
		{
		perror("No bind");
		exit(1);
		}

	if (listen(socket_RV,10)==-1) // Ecoute si quelqu'un se connecte
		{
		perror("Unable to listen");
		exit(1);
		}
	
	printf("Waiting for client...\n");
	socket_service = accept(socket_RV,(struct sockaddr *)&adr, &lgadresse); // Accepte la connection
	printf("New client\n");

    return socket_service;
}
	//  Fonction de dialogue

/* Fonction speak:
Envoie une chaine de caractere dans un socket
Arg:
	msg 		-> Chaine de caracteres a envoyer
	socket_RV 	-> ID du socket a utiliser
*/
void speak(char* msg, int socket_RV)
{
	char c;    
	int i = 0;
	while(msg[i] != 0)
	{
		c = msg[i];
		write(socket_RV, &c, 1); // On envoie caractere par caractere.
		i++;
	}
}

void dataCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	tf::Quaternion q_auv;

    x 		= msg->position.x;
    y 		= msg->position.y;
    z 		= msg->position.z;

    q_auv.setX(msg->orientation.x);
    q_auv.setY(msg->orientation.y);
    q_auv.setZ(msg->orientation.z);
    q_auv.setW(msg->orientation.z);

    tf::Matrix3x3(q_auv).getRPY(roll, pitch, yaw);
}

	// Fils

void filsProcess()
{
	
}

	// Main

int main(int argc, char *argv [])
{
	printf("Starting server\n");
	int socket_RV 	= serveur();

	ros::init(argc, argv, "serveur");
    ros::NodeHandle n;
    
        // Parametres initiaux    
    ros::Rate loop_rate(10);

	int compteur = 0;
	char buffer[100];

		// Subscribe msgs
    ros::Subscriber status_sub = n.subscribe("/data_auv", 1000, dataCallback);

	while(ros::ok())
	{
		sprintf(buffer,"$POS;%lf;%lf;%lf;%lf;%lf;%lf\n", x, y, z, roll, pitch, yaw);
		printf("%s", buffer);
		speak(buffer, socket_RV);

		ros::spinOnce();
        loop_rate.sleep();
	}

	close(socket_RV);

	exit(1);
}