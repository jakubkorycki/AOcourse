#ifndef EDAC40_H_
#define EDAC40_H_

#ifdef BUILD_DLL
    #define DLL_EXPORT __declspec(dllexport)
#else
//  #define DLL_EXPORT __declspec(dllimport)
//  suitable for both dll import header and the case of static linking
    #define DLL_EXPORT
#endif

#ifdef __cplusplus
   extern "C" {
#endif

#ifdef _MSC_VER
    #include <msstdint.h>
#else
    #include <stdint.h>
#endif

#include <stdlib.h>

#if defined(WIN32) || defined(_MSC_VER)     
  #include <winsock2.h>
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <unistd.h>
  typedef int SOCKET;
  #define Sleep(ms) usleep(1000*ms)
#endif

#define EDAC40_PORT              1234  // UDP and TCP ports for data

// command codes
#define EDAC40_SET_VALUE         0
#define EDAC40_SET_OFFSET        1
#define EDAC40_SET_GAIN          2
#define EDAC40_SET_OFFSET_DACS   3 // affects all channels

// logical (connector) to physical (DAC channel) mapping
extern const int EDAC40_CHANNEL_MAP[40];

// array of this kind of records is used to get a list of available units
// with Microchip "discover" protocol
typedef struct
{
   char IPAddress[16];
   char MACAddress[18];
} edac40_list_node;

// array of this kind of records is passed as an argument to
//edac40_prepare_packet function which fills EDAC40 data block
typedef struct
{
    int channel;
    uint16_t value;
} edac40_channel_value;

int DLL_EXPORT edac40_init();
int DLL_EXPORT edac40_finish();
int DLL_EXPORT edac40_list_devices(edac40_list_node *devices, int max_device_num, int discover_timeout, int discover_attempts);
DLL_EXPORT char*  edac40_find_device(const char *macaddress);
SOCKET DLL_EXPORT edac40_open(const char *edac40_host, int use_tcp);
void DLL_EXPORT edac40_set_timeout(SOCKET edac40_socket, long milliseconds);
void DLL_EXPORT edac40_close(SOCKET edac40_socket);
int DLL_EXPORT edac40_set(SOCKET edac40_socket, int command_code, int channel, unsigned value);
int DLL_EXPORT edac40_prepare_packet(edac40_channel_value *channel_list, int channel_num, char **edac40_packet);
int DLL_EXPORT edac40_prepare_packet_from_array(unsigned value[40], int command_code, char **edac40_packet);
int DLL_EXPORT edac40_prepare_packet_fill(unsigned value, int command_code, char **edac40_packet);
int DLL_EXPORT edac40_send_packet(SOCKET edac40_socket, char *edac40_packet, int edac40_packet_size);
int DLL_EXPORT edac40_save_defaults(SOCKET edac40_socket);
int DLL_EXPORT edac40_restore_defaults(SOCKET edac40_socket);
// Do we really need those two functions?
//int DLL_EXPORT edac40_channel_logical_to_physical(int channel);
//int DLL_EXPORT edac40_channel_physical_to_logical(int channel);
void DLL_EXPORT memset2(void *buf, uint16_t value, size_t n);
void DLL_EXPORT edac40_free(void *p);

#ifdef __cplusplus
   }
#endif

#endif // EDAC40_H_
