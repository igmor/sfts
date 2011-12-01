
//------------------------------------------------------CLIENT CODE---------------------------------------------------------


//-------------------------------------------------Including The Header Files-----------------------------------------------

#include<stdio.h>						
#include<stdlib.h>
#include<unistd.h>
#include<errno.h>
#include<string.h>
#include<netdb.h>
#include<sys/types.h>
#include<netinet/in.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<sys/wait.h>
#include<math.h>
#include "time.h"
#include <ctime>


#define SERVERPORT 15000												// Static Port For The Server
#define MAXBUFLEN 100													// Maximum Length Constraint



//-------------------------------------------------------Arduino Unit-------------------------------------------------------


int main()
{

//-----------------------------------------------TCP Connection Begins------------------------------------------------------


		struct sockaddr_in Arduinounitinfo;								// Structure for TCP for the Arduino 
		struct sockaddr_in serverinfo;									// Structure for TCP for the server


		int HostnameError;												// To store the hostname retrieved
		char hostname[200];

		HostnameError=gethostname(hostname,sizeof(hostname));			// Obtain the hostname
		//printf("The Error Bit for gethostname Is : %d\n",HostnameError);
		//printf("The Hostname Is : %s\n",hostname);


//-----------------------------------------------Initialize The Structures-------------------------------------------------


		serverinfo.sin_family = AF_INET;								// To store the family as IPv4
		serverinfo.sin_port = htons(SERVERPORT);						// To connect to the received port number
		//printf("The Server's Port Number Is : %d\n",serverinfo.sin_port);

		struct hostent *ipaddr;
		ipaddr = gethostbyname(hostname);								// Obtaining the IP Address

		serverinfo.sin_addr = *((struct in_addr *)ipaddr->h_addr);  	// Loading the obtained address
		//printf("The Server's Host Address Is : %s\n",inet_ntoa(serverinfo.sin_addr));


		Arduinounitinfo.sin_family = AF_INET;							// To store the family as IPv4

		struct hostent *ipaddr1;
		ipaddr1 = gethostbyname(hostname);								// Obtaining the IP Address

		Arduinounitinfo.sin_addr = *((struct in_addr *)ipaddr1->h_addr);  // Loading the obtained address
		//printf("The Mobile Unit's Host Address Is : %s\n",inet_ntoa(Arduinounitinfo.sin_addr));

	

	int socketdescriptortcp;
	
	while(1)																// For repetitive moves of the Arduino unit
	{
		sleep(1);

//--------------------------------------------------Create The TCP Socket---------------------------------------------------
		
		
		int socketdescriptortcp;
		
		socketdescriptortcp = socket(Arduinounitinfo.sin_family, SOCK_STREAM, 0); // Getting a TCP Socket descriptor
		//printf("The TCP Socket Descriptor Is : %d\n",socketdescriptortcp);
		if (socketdescriptortcp == -1)										// Error Checking
		{
			perror("setsockopt");
			exit(1);
		}
		
		
		int yes=1;						
		
		if (setsockopt(socketdescriptortcp, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1)
		{
			perror("setsockopt");											// Error checking
			exit(1);
		}
		
		
//--------------------------------------------------Dynamic Port Obtained---------------------------------------------------
		
		
		int socketerrortcp;
		size_t addrlentcp;
		addrlentcp = sizeof Arduinounitinfo;								// Obtaining the dynamic Port Number Next
		socketerrortcp = getsockname(socketdescriptortcp,(struct sockaddr*)&Arduinounitinfo,(socklen_t*)&addrlentcp);
		if(socketerrortcp==-1)												// Error checking
		{
			perror("socketerror");
			exit(1);
		}
		
		//printf("The Mobile Unit's TCP Port Number Is : %d\n\n",Arduinounitinfo.sin_port);
		
		
//----------------------------------------------------Connect To Server-----------------------------------------------------
		
		
		int connecterror;
		
		connecterror=connect(socketdescriptortcp,(struct sockaddr*)&serverinfo, sizeof(struct sockaddr));
																			// Establishing The TCP Connection
		if(connecterror==-1)												// Error Checking
		{
			perror("connect");
			exit(1);
		}
		//printf("Mobile Is Now Trying To Send To The Server.......\n\n");
		//printf("The TCP Socket Descriptor Is : %d\n",connecterror);
		
//-------------------------------------------------------Message Formation--------------------------------------------------
		
		
		char messagetcp[MAXBUFLEN];										   // TCP Message to be sent
		char replytcp[MAXBUFLEN];
		
		unsigned int proto = 0xbeef;
		unsigned int sensorid = 1;
		unsigned int Whole = 59;
		unsigned int Fract = 75;
		
		sprintf(messagetcp,"%u %u %u %u",proto,sensorid,Whole,Fract);	   // Creating the message
		printf("The Message Sent Is : %s\n\n",messagetcp);
		
		
//--------------------------------------------------Send The Message To the Server------------------------------------------
		
		
		int senderror;
		senderror=send(socketdescriptortcp,messagetcp,strlen(messagetcp),0);  // Sending the TCP message
		if(senderror==-1)													  // Error checking
		{
			perror("send");
			exit(1);
		}
		//printf("The Send Error Variable Is %d\n",senderror);
		//printf("The TCP Message Sent Is %s\n\n",messagetcp);
	


		//exit(1);															  // Terminate the process
	}
		close(socketdescriptortcp);											  // Closing the TCP socket
}
