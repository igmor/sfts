//------------------------------------------------------ARDUINO RECEIVER CODE--------------------------------------------------------


//-------------------------------------------------Including The Header Files----------------------------------------------

#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<errno.h>
#include<string.h>
#include<netdb.h>
#include<sys/types.h>
#include <netinet/tcp.h>
#include<netinet/in.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<sys/wait.h>
#include <sys/time.h>
#include<math.h>

#define SERVERPORT 15000							// Static port for the server
#define MAXBUFLEN 32								// Maximum Length Constraint
#define BACKLOG 1000									// Number of connections allowed on the input queue															//         incoming queue


//--------------------------------------------Structures For Socket Information---------------------------------------------


struct sockaddr_in Arduinounitinfo;						// Structure for the Arduino unit
struct sockaddr_in serverinfo;							// Structure for the server
unsigned prev_high = 0;
struct timeval tv;
struct timeval prev_tv;

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


  printf("Server Is Listening On The TCP Port %d\n\n", SERVERPORT);



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


    unsigned char reply[MAXBUFLEN];
    int offset = 0;
    int ret=recv(accepterror, reply, MAXBUFLEN, 0);					// Receive the incoming message
  
    //printf("The Receive Error Variable Is %d\n",receiveerror);

    while (ret > 0 )
    {
	reply[ret]='\0';	
	for (int i = 0; i < ret; i++)
	  {
	    printf("0x%x ",reply[offset + i]);
	  }
	offset += ret;
	ret = recv(accepterror, reply + offset, MAXBUFLEN - offset, 0);
    }
    printf("\n");

    //----------------------------------------------------Message Extraction-----------------------------------------------------


    unsigned int proto = 0;
    long unsigned int sensorid = 0;
    unsigned high = 0, low = 0;
	
    printf("%lu\n", sizeof(sensorid));
    for (int i = 0; i < offset; i++)
      {
	unsigned int ui = reply[i];
	if (i < 4)
	  proto += (ui << ((3 - i)*8));
	else if (i >= 4 && i <= 11)
	  {
	    unsigned long ul = ui;
	    sensorid += (ul << ((11-i)*8));
	  }
	else if (i == 12)
	  high = ui;
        else
	  low = ui;
      }

    short t = (high << 8) | low;
    printf("proto: 0x%.2x, sensor id: %#.16lx, high: 0x%.2x, low: 0x%.2x, temperature: %d.%d\n", proto, sensorid, high, low, t/100, t%100);
	  
	
	int time_error = gettimeofday(&tv, NULL);
	if(time_error)
	{
		perror("gettimeofday");
	}
	  
	
	if (((prev_high - high) > (unsigned)1.5 || (high - prev_high) > (unsigned)1.5) || (tv.tv_sec - prev_tv.tv_sec > 240))
	{
		prev_tv.tv_sec = tv.tv_sec;
		
		char cmd_line[80];
		sprintf(cmd_line, "%s %#.16lx %.2f", "./../web/insert_t_to_force.sh", sensorid, float(t/100) + float(t%100)/100);

		printf("calling %s, return code %d\n", cmd_line, system(cmd_line));
	}
	
	if (tv.tv_sec - prev_tv.tv_sec > 45)
	{
		prev_high = high;
	}

  }

  //---------------------------------------------------Terminate the Process--------------------------------------------------

  close(socketdescriptortcp);												// Close the TCP socket

  return 0;
}
