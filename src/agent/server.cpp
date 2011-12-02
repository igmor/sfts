
//------------------------------------------------------ARDUINO RECEIVER CODE--------------------------------------------------------


//-------------------------------------------------Including The Header Files----------------------------------------------

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

#define SERVERPORT 15000							// Static port for the server
#define MAXBUFLEN 100								// Maximum Length Constraint
#define BACKLOG 1000									// Number of connections allowed on the input queue															//         incoming queue


//--------------------------------------------Structures For Socket Information---------------------------------------------


struct sockaddr_in Arduinounitinfo;						// Structure for the Arduino unit
struct sockaddr_in serverinfo;							// Structure for the server


//----------------------------------------------------Server Program--------------------------------------------------------


int main()
{

//----------------------------------------------------Get the Hostname------------------------------------------------------


int HostnameError;
char hostname[200];														// To store the hostname retrieved

HostnameError=gethostname(hostname,sizeof(hostname));					// Get the host name
//printf("The Error Bit for gethostname Is : %d\n",HostnameError);
//printf("The Hostname Is : %s\n",hostname);


//-----------------------------------------------Initialize The Structure---------------------------------------------------


serverinfo.sin_family = AF_INET;										// To initialize to IPv4
serverinfo.sin_port = htons(SERVERPORT);								// To initialize the static port number 
//printf("The Server's Port Number Is : %d\n",serverinfo.sin_port);

struct hostent *ipaddr1;
ipaddr1 = gethostbyname(hostname);										// To get the IP Address

serverinfo.sin_addr = *((struct in_addr *)ipaddr1->h_addr);				// Load the IP Address to the structure
//printf("The Server's Host Address Is : %s\n",inet_ntoa(serverinfo.sin_addr));



//--------------------------------------------------Create The TCP Socket---------------------------------------------------


int socketdescriptortcp;

socketdescriptortcp = socket(serverinfo.sin_family, SOCK_STREAM, 0);	// Getting a TCP Socket descriptor
//printf("The Socket Descriptor Is : %d\n",socketdescriptortcp);
		if (socketdescriptortcp == -1)									// Error Checking
		{
			perror("socket");
			exit(1);
		}

char yes='1';															// Clear any hogging port if any

if(setsockopt(socketdescriptortcp,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(int)) == -1)
{
perror("setsockopt");													// Error Checking
exit(1);
}

//----------------------------------------------------Bind The Socket-------------------------------------------------------


int binderrortcp;
																		// Binding the socket
binderrortcp = bind(socketdescriptortcp, (struct sockaddr*)&serverinfo, sizeof(struct sockaddr));
//printf("The Bind Error Is : %d\n\n",binderrortcp);
if(binderrortcp==-1)
	{
		perror("Bind");													// Error Checking
		exit(1);
	}


printf("Server Is Listening On The TCP Port\n\n");



while(1)																// To loop back for next messages
{

//-----------------------------------------------Wait for Incoming Connections----------------------------------------------
	
	
int listenerror;
	
listenerror=listen(socketdescriptortcp, BACKLOG);						// Listen on incoming connections
if(listenerror==-1)
{
	perror("listen");													// Error checking
	exit(1);
}
	
	
//---------------------------------------------Get the Pending Connections--------------------------------------------------
	
int accepterror;
socklen_t addrlen;
addrlen = sizeof Arduinounitinfo;

accepterror=accept(socketdescriptortcp,(struct sockaddr*)&Arduinounitinfo,&addrlen); // Get the descriptor for Send & Receive
if(accepterror==-1)
	{
		perror("accept");												// Error checking
		exit(1);
	}

//printf("The TCP Socket Descriptor Is : %d\n",accepterror);
	
//-----------------------------------------------Receive The Message--------------------------------------------------------


int receiveerror;							
char replytcp[MAXBUFLEN];


receiveerror=recv(accepterror,replytcp,MAXBUFLEN-1,0);					// Receive the incoming message
if(receiveerror==-1)
	{
		perror("recv");													// Error checking
		exit(1);
	}
//printf("The Receive Error Variable Is %d\n",receiveerror);

replytcp[receiveerror]='\0';											// Appending String with a 'NULL' character
printf("The Message Received Is %s\n\n",replytcp);



//----------------------------------------------------Message Extraction-----------------------------------------------------


unsigned int proto;
unsigned int sensorid;
unsigned int Whole;
unsigned int Fract;
	
//sscanf(replytcp,"%u %u %u %u",proto,sensorid,Whole,Fract);				// Extract the message that is received


}
//---------------------------------------------------Terminate the Process--------------------------------------------------


close(socketdescriptortcp);												// Close the TCP socket
//exit(1);																// Terminate the ServerArduino Process


}
