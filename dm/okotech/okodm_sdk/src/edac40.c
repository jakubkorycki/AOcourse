/****************************************************
 FileName: edac40.c
 Description: Low-level programming interface for
              EDAC40 40-channel Ethernet unit

 (c) 2010, 2013 Flexible Optical B.V.
          Seva Patlan
 2011-11-09 original release 
 2013-01-21 minor corrections for deal with devices on multiple network interfeces
 2013-12-03 adapted for unix, minor corrections
 *****************************************************/

#include "edac40.h"

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
#else
  #include <unistd.h>
  #include <sys/types.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <ifaddrs.h>
  #define INVALID_SOCKET (-1)
  #define SOCKET_ERROR (-1)
  #define WSAGetLastError() (1)
  typedef struct sockaddr SOCKADDR;
  typedef struct sockaddr_in SOCKARRD_IN;
  #define closesocket(s) close(s)
#endif

#ifdef _MSC_VER
    #include <msstdint.h>
#else
    #include <stdint.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <stdio.h>


#define EDAC40_DISCOVER_PORT 30303  // UDP port for discover protocol
#define EDAC40_DEVICE_NAME "EDAC40" // built-in device name
#define EDAC40_BUFLEN 256
#define EDAC40_MAXLEN 86

// logical (connector) to physical (DAC channel) mapping
const int EDAC40_CHANNEL_MAP[40]={6,7,4,5,2,3,0,1,22,23,20,21,18,19,16,17,14,15,12,13,
                              // Analog outputs 1-20 (right connector)
                              10,11,8,9,38,39,36,37,34,35,32,33,30,31,28,29,26,27,24,25};
                              // Analog outputs 21-40 (left connector)


// Functions are defined with DLL_EXPORT specifier, defined in the header file.
// This macro is not empty only in the case of building DLL.

/*******************************************************************
                   edac40_init
Initializes WinSock machinery.
Should be called only once before any other functions.
*******************************************************************/
int DLL_EXPORT edac40_init()
{
#ifdef _WIN32  
   WSADATA wsaData;
   WSAStartup(MAKEWORD(2,2), &wsaData);
#endif   
   return 0;
}

/*****************************************************************
                  edac40_finish
Terminates using of WinSock.
Should be called at the end of the program.
*****************************************************************/
int DLL_EXPORT edac40_finish()
{
#ifdef _WIN32  
   WSACleanup();
#endif   
   return 0;
}

/****************************************************************
                 edac40_set_timeout
Specify timeout for blocking TCP operations.
Operation (send) fails when timeout value exceeded
****************************************************************/
void DLL_EXPORT edac40_set_timeout(SOCKET edac40_socket, long milliseconds)
{
    struct timeval timeout;
    // It seems MS somewhat confused about s/millisecond/microseconds
    // Or I didn't get something... :((
    timeout.tv_sec=milliseconds; // (sic!)
    timeout.tv_usec=0;
    setsockopt(edac40_socket,SOL_SOCKET,SO_SNDTIMEO,(char *)&timeout,sizeof(timeout));
}

/****************************************************************
                     edac40_list_devices
 Get a list of available EDAC40 devives

 Up to max_device_num will be polled and their IP addresses
 and MAC addresses will be returned in the array (of structures)
 devices (it should be provided by caller). Additional parameters are
 timeout value (in milliseconds) and number of discovery attempts.

 Return value: number of devices found or negative value in the case of error
*****************************************************************/
int DLL_EXPORT edac40_list_devices(edac40_list_node *devices, int max_device_num, int discover_timeout, int discover_attempts)
{
  SOCKET mySocket;
  struct sockaddr_in sockAddr,otherAddr;
  char buffer[EDAC40_BUFLEN]="Discovery: Who is out there?",
       recvbuf[EDAC40_BUFLEN];
  int iResult;
  int optVal; 
  int otherAddrSize;
  fd_set fdvar;
  struct timeval timeout;
  int sel_result,AttemptN;
  char *p;
  int dev_num,i;
  char newip[20],newmac[20];
  
  //-- some variables used for device detection on multiple interfaces
#ifdef _WIN32
  INTERFACE_INFO InterfaceList[20];
  unsigned long nBytesReturned;
  int nNumInterfaces;
  int iface_idx;
#else
  struct ifaddrs *addrs, *this_addrs;
#endif  
  unsigned long if_unicast, if_mask, if_broadcast;
  //-------------------------
  mySocket=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
  if(mySocket==INVALID_SOCKET) 
  {
//    printf("Error creation socket\n");
    return (-WSAGetLastError());
  }  
  // Prepare for sending a broadcast request
  optVal=1;
  if(setsockopt (mySocket, SOL_SOCKET, SO_BROADCAST,(char*)&optVal, sizeof(int))==SOCKET_ERROR)
  {
//    printf("Error setting socket options");
    return (-WSAGetLastError());
  }  
  sockAddr.sin_family=AF_INET;
  sockAddr.sin_port=htons(EDAC40_DISCOVER_PORT);
  sockAddr.sin_addr.s_addr = INADDR_BROADCAST;
  otherAddrSize=sizeof(otherAddr);
  AttemptN=0;
  dev_num=0;
  //-------- list network interfaces
  // Is there a way to do this under Win and *nix in a more uniform fasion?
#ifdef _WIN32
  if (WSAIoctl(mySocket, SIO_GET_INTERFACE_LIST, 0, 0, &InterfaceList,
			sizeof(InterfaceList), &nBytesReturned, 0, 0) == SOCKET_ERROR) {
       return(-WSAGetLastError());
    }

  nNumInterfaces = nBytesReturned / sizeof(INTERFACE_INFO);
  //cout << "There are " << nNumInterfaces << " interfaces:" << endl;  
#else
  if(getifaddrs(&addrs)!=0)
  {
//    printf("Error obtaining interface list\n");
    return -1;
  }
#endif  
  do  // multiple detection attempts
     {
       // iterate all found network interfaces
#ifdef _WIN32       
        for(iface_idx=0; iface_idx<nNumInterfaces; iface_idx++) 
		{   // for each network interface prepare and send broadcast package
			if_unicast=((struct sockaddr_in *) & (InterfaceList[iface_idx].iiAddress))->sin_addr.s_addr;
			if_mask=((struct sockaddr_in *) & (InterfaceList[iface_idx].iiNetmask))->sin_addr.s_addr;
			if_broadcast=((struct sockaddr_in *) & (InterfaceList[iface_idx].iiBroadcastAddress))->sin_addr.s_addr;
#else
	for(this_addrs=addrs; this_addrs!=0; this_addrs=this_addrs->ifa_next)
	      {
			if(!(this_addrs->ifa_addr) || (this_addrs->ifa_addr->sa_family!=AF_INET)) continue; // avoid repetitions of interfaces for divverent protocol families
//			printf("%s\n",this_addrs->ifa_name);
			if_unicast=((struct sockaddr_in *) (this_addrs->ifa_addr))->sin_addr.s_addr;
			if_mask=((struct sockaddr_in *)  (this_addrs->ifa_netmask))->sin_addr.s_addr;
			if_broadcast=((struct sockaddr_in *) (this_addrs->ifa_ifu.ifu_broadaddr))->sin_addr.s_addr;
#endif			
		
			
			sockAddr.sin_addr.s_addr = if_unicast | ( if_broadcast & ~if_mask);
//			printf("Broadcasting to:%s\n",inet_ntoa(sockAddr.sin_addr));
			// Send broadcast request
			if(sendto(mySocket,buffer,strlen(buffer),MSG_DONTROUTE ,(SOCKADDR *)&sockAddr,sizeof(sockAddr))==SOCKET_ERROR)
			{
//			  printf("Error sending broadcast\n");
			  return (-WSAGetLastError());
			}
			timeout.tv_sec=0;//discover_timeout;
			timeout.tv_usec=discover_timeout*1000;
			FD_ZERO(&fdvar);
			FD_SET(mySocket,&fdvar);
			do
			  {
			   // wait for recv data or timeout
			   if((sel_result=select(mySocket+1,&fdvar,(fd_set *)0,(fd_set *)0,&timeout))<0)
			   {
//			     printf("Error after select\n");
			     return (-WSAGetLastError());
			   }
				else if(sel_result>0) // Some data available
					{
					   // Get the data and the sender address
					   iResult = recvfrom(mySocket, recvbuf, EDAC40_BUFLEN, 0,(SOCKADDR *)&otherAddr,&otherAddrSize);
					   if (iResult > 0)
						{
						  p=&(recvbuf[13]);
						  while(*p==' ') *(p--)='\0';
						  if(strcmp(recvbuf,EDAC40_DEVICE_NAME)) continue; // device signature does not match
						  strcpy(newip,(char *)inet_ntoa(otherAddr.sin_addr));
						  recvbuf[34]='\0';
						  strcpy(newmac,&(recvbuf[17]));
						  // check if the device is already listed
						  for(i=0; i<dev_num; i++)
							{
							   if(!strcmp(devices[i].MACAddress,newmac)) break;
							}
						  if(dev_num==0 || i==dev_num) // copy parameters for just found device into array
							{
							   strcpy(devices[dev_num].IPAddress,newip);
							   strcpy(devices[dev_num].MACAddress,newmac);
							   if(dev_num<max_device_num) dev_num++; // increment number of found devices unless it exeeds maximum
							}
						  continue;
						 }
						else if (iResult == 0) return(0); // connection closed?
					}
			  } while(sel_result>0); // someone responded and timeout is not reached
		   } // interfaces
       AttemptN++;
     } while (AttemptN<discover_attempts);
     // will send discover requset till some device found or max number of attempts reached
#ifndef _WIN32
   freeifaddrs(addrs);
#endif   
   return(dev_num);
}

/***********************************************************
          edac40_find_device

Returns IP address of the EDAC40 unit with MAC address
supplied as an argument (null-terminated string).
NULL string pointer or empty (zero length) string can be passed
to faciliate "auto mode" (i.e. getting first available device).

Returns NULL pointer as an indication of error (no device found).

This is a "simplified" version of edac40_list_devices function,
it has easier-to-use arguments and some constants hardcoded.

User should take care of disposing of returned string.

***********************************************************/
DLL_EXPORT char* edac40_find_device(const char *macaddress)
{
   int max_dev_num=50, dev_num,i;
   edac40_list_node *list;
   char *ipaddress;
   // prepare memory buffer for device list
   list=(edac40_list_node *)malloc(sizeof(edac40_list_node)*max_dev_num);
   // send discovery request and receive the list
   dev_num=edac40_list_devices(list,max_dev_num,200,1); // 200ms timeout, one attempt
   for(i=0; i<dev_num; i++)
       if(macaddress==NULL || (macaddress && strlen(macaddress)<1) || strncmp(macaddress,list[i].MACAddress,17)==0)
       // NULL pointer OR empty string for "auto mode" OR exact match
       {
         ipaddress=(char *)malloc(20);
         strcpy(ipaddress,list[i].IPAddress);
         free(list);
         return ipaddress;
       }
   free(list);
   return 0;
}


/*****************************************************
                 edac40_open
  Opens TCP or UDP socket and connects it to the
  port of the EDAC40 specified by its hostname (which
  is usually an IP address in the form of a string,
  first argument).

  Second argument allows to choose between datagram
  (UDP) use_tcp=0 and stream connection (TCP)
  use_tcp=1

  Return value: opened socket descriptor (>0) if
                 operation succeeded
                      or
                error code (<0) if failed.
*****************************************************/
SOCKET DLL_EXPORT edac40_open(const char *edac40_hostname, int use_tcp)
{
  SOCKET SendSocket;
  struct sockaddr_in RecvAddr;
  if(use_tcp)
        {
            // try to open TCP socket, return an error code if failed
            if((SendSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP))==INVALID_SOCKET) return (-WSAGetLastError());
            // set timeout of 10s for blocking TCP socket
            edac40_set_timeout(SendSocket,10000);
        }
       // open UDP socket
       else SendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  RecvAddr.sin_family = AF_INET;
  RecvAddr.sin_port = htons(EDAC40_PORT);
  RecvAddr.sin_addr.s_addr = inet_addr(edac40_hostname);
  if(connect(SendSocket,(SOCKADDR*)&RecvAddr,sizeof(RecvAddr))!=0) return (-WSAGetLastError());
  return SendSocket;
}

/********************************************
               edac40_close
Close a socket associated with the EDAC40 device
*********************************************/
void DLL_EXPORT edac40_close(SOCKET edac40_socket)
{
    closesocket(edac40_socket);
}

/********************************************************
             edac40_set
Set values, offset or gain (specified by value parameter)
for individual channel (specified by channel parameter)
One of the predefined constants
    EDAC40_SET_VALUE,
    EDAC40_SET_OFFSET,
    EDAC40_SET_GAIN,
    EDAC40_SET_GLOBAL_OFFSET
may be used as for command_code.

It is strongly recomended to use edac40_send_packet istead
to set all channels/parameters with one packet.
Network overhead is significantly decreased in this way.
**********************************************************/
int DLL_EXPORT edac40_set(SOCKET edac40_socket, int command_code, int channel, unsigned value)
{
   unsigned long long channel_mask;
   char *buffer;
   buffer=(char*)malloc(8);
   channel_mask=1ull<<EDAC40_CHANNEL_MAP[channel];
   memcpy(buffer,&channel_mask,5);
   buffer[5]=command_code;
   buffer[6]=value & 0xFF;
   buffer[7]=(value >>8) & 0xFF;
   send(edac40_socket,buffer,8,0);
   free(buffer);
   return 0;
}

/**************************************************************
               edac40_save_defaults
Sends command to store current parameters
(gain, offest, offset DACs code) into NVRAM.

These data are always read back at device reset.
**************************************************************/
int DLL_EXPORT edac40_save_defaults(SOCKET edac40_socket)
{
   char buf[8]; // smallest possible packet size
   buf[5]=0x04; // just set command code, all other data in the packet are ignored
   send(edac40_socket, buf, 8, 0);
   return 0;
}

/*************************************************************
                 edac40_restore_defaults
Helper function, set parameters as they were initially
programmed _and_ instruct EDAC40 module to save them into NVRAM
**************************************************************/
int DLL_EXPORT edac40_restore_defaults(SOCKET edac40_socket)
{
  char *edac40_buf; // this pointer is reused three times for GAIN, OFFSET, VALUE

  // prepare buffer to set GAIN, then send packet...
  edac40_prepare_packet_fill(0xFFFF,EDAC40_SET_GAIN,&edac40_buf);
  edac40_send_packet(edac40_socket,edac40_buf,86);
  free(edac40_buf);

  // prepare buffer to set OFFSET, then send packet...
  edac40_prepare_packet_fill(0x8000,EDAC40_SET_OFFSET,&edac40_buf);
  edac40_send_packet(edac40_socket,edac40_buf,86);
  free(edac40_buf);

  // prepare buffer to set VALUE, then send packet...
  edac40_prepare_packet_fill(0x0000,EDAC40_SET_VALUE,&edac40_buf);
  edac40_send_packet(edac40_socket,edac40_buf,86);
  free(edac40_buf);

  // OFFSET_DACS: one setting for all channels
  edac40_set(edac40_socket,EDAC40_SET_OFFSET_DACS,0,0x1FFF);

  edac40_save_defaults(edac40_socket);
  return 0;

}


int DLL_EXPORT edac40_prepare_packet(edac40_channel_value *channel_list, int channel_num, char **edac40_packet)
{
    int i,physical_channel,last_channel;
    unsigned long long channel_mask=0;
    char *buffer;
    buffer=(char *)malloc(channel_num*2+6);
    channel_mask=0;
    last_channel=0;
    // The values for individual channels should be ordered on physical channel number,
    // so looping and comparing...
    for(physical_channel=0; physical_channel<40; physical_channel++)
      {
         for(i=0;i<channel_num;i++)
           // data for this physical channel present in the list
           if(EDAC40_CHANNEL_MAP[channel_list[i].channel]==physical_channel)
             {
                // add 1 to proper bit in the channel mask
                channel_mask|=(1ull<<physical_channel);
                // store data in the next two bytes of the packet
                buffer[6+last_channel*2]=(channel_list[i].value) & 0xFF;
                buffer[7+last_channel*2]=((channel_list[i].value) >>8) & 0xFF;
                last_channel++;
             }
      }
    buffer[5]=0;
    // save channel mask to the packet buffer
    memcpy(buffer,&channel_mask,5);
    // place buffer address to the provided pointer variable
    *edac40_packet=buffer;
    // return the packet size in bytes
    return (channel_num*2+6);
}

/************************************************************************
                  edac40_prepare_packet_from_array
Prepare packet for sending to EDAC40 unit by filling data from array.
All 40 channels are programmed in one operation.
************************************************************************/
int DLL_EXPORT edac40_prepare_packet_from_array(unsigned value[40], int command_code, char **edac40_packet)
{
   int i;
   char *buffer;
   buffer=(char *)malloc(86); //
   buffer[0]=buffer[1]=buffer[2]=buffer[3]=buffer[4]=0xFF; // channel mask -- all channels
   buffer[5]=command_code;
   for(i=0; i<40; i++)
     {
       buffer[6+i*2]=value[i] & 0xFF;
       buffer[7+i*2]=(value[i]>>8) & 0xFF;
     }
   *edac40_packet=buffer;
   return 86;
}

/*************************************************************************
                    edac40_prepare_packet_fill
*************************************************************************/
int DLL_EXPORT edac40_prepare_packet_fill(unsigned value, int command_code, char **edac40_packet)
{
  int i;
  char *buffer;
  buffer=(char *)malloc(86); //
  buffer[0]=buffer[1]=buffer[2]=buffer[3]=buffer[4]=0xFF; // channel mask -- all channels
  buffer[5]=command_code;
  for(i=0; i<40; i++)
    {
      buffer[6+i*2]=value & 0xFF;
      buffer[7+i*2]=(value>>8) & 0xFF;
    }
  *edac40_packet=buffer;
  return 86;
}


/***********************************************************************
                     edac40_send_packet

************************************************************************/
int DLL_EXPORT edac40_send_packet(SOCKET edac40_socket, char *edac40_packet, int edac40_packet_size)
{
   int result;
   //send(edac40_socket,edac40_packet,edac40_packet_size,0);
   if((result=send(edac40_socket,edac40_packet,edac40_packet_size,0))<0) return -WSAGetLastError();
                                                                    else return result;
   // 0 if timeout reached on blocking socket,
   // negative indicates some error, positive -- number of bytes written
   // return 0;
}

// Helper function: fills the block of memory with 2-byte int
void DLL_EXPORT memset2(void *buf, uint16_t value, size_t n)
{
  uint16_t *p=(uint16_t *)buf;
  while((unsigned)(p-(uint16_t*)buf)<n) *(p++)=value;
}

// Wrap around standard free function. Defined on request of Python user.
void DLL_EXPORT edac40_free(void *p)
{
	free(p);
}
