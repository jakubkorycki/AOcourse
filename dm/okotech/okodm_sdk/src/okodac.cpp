#include "okodac.h"
//#include <QDebug>
#include <cmath>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include "util.h"
#include "mirror_ffmat.h"


// Table of channels for USB40DAC
static uint8_t USBDAC_CHANNEL_TABLE[40]=
{
/*DAC-> 0   1   2   3   4    |
        ---------------------+  OUTPUT    */
        7, 15, 23, 31, 39, //|  A
        6, 14, 22, 30, 38, //|  B
        5, 13, 21, 29, 37, //|  C
        4, 12, 20, 28, 36, //|  D
        3, 11, 19, 27, 35, //|  E
        2, 10, 18, 26, 34, //|  F
        1,  9, 17, 25, 33, //|  G
        0,  8, 16, 24, 32  //|  H
    };

int EtherDAC::edac_instance_count=0;

// --------- generic DAC class member functions definitions

DAC::DAC(const char *id)
{
    dac_values=0;
    if(id) strcpy(dac_id,id);
    initInternals();
}

DAC::~DAC()
{
	//printf("in ~DAC()\n");
    if(dac_values) free(dac_values);
}

void DAC::initInternals()
{
    dac_immediate_update=true;
    if(dac_values) free(dac_values);
    dac_values=(unsigned*)malloc(sizeof(unsigned)*numberOfChannels());
    for(unsigned i=0; i<numberOfChannels(); i++) dac_values[i]=0;
}

void DAC::setOneChannel(unsigned channel, unsigned value)
{
//    int physical_channel=edac40_channel_logical_to_physical(channel);
    if(channel>=0 && channel<numberOfChannels()) dac_values[channel]=value;
    if(dac_immediate_update) updateAllChannels();
}

int DAC::getChannel(unsigned channel)
{
    //int physical_channel=edac40_channel_logical_to_physical(channel);
    if(channel>=0 && channel<numberOfChannels()) return  dac_values[channel];
                                            else return -1; // better throw exception
}

void DAC::setAllChannels(const vector<unsigned> values)
{
    unsigned channel_limit=values.size()<numberOfChannels()?values.size():numberOfChannels();
    for(unsigned i=0; i<channel_limit; i++) dac_values[i]=values.at(i);
    if(dac_immediate_update) updateAllChannels();
}

void DAC::setAllChannels(const unsigned *values, unsigned channel_number)
{
    memcpy(dac_values,values,(channel_number<numberOfChannels())?channel_number:numberOfChannels());
    if(dac_immediate_update) updateAllChannels();
}

// --------- EtherDAC class member functions definitions

EtherDAC::EtherDAC(const char *MAC_address)
{
    //char *hostname=0;
    initInternals();
    if(edac_instance_count<1) edac40_init();
    edac_instance_count++;
    // finding device is simple (even in auto mode) and is done with single function call,
    // but we wanted to get its MAC addreee so going to some troubles...
    int max_dev_num=50;
    edac40_list_node *list;
    list=(edac40_list_node *)malloc(sizeof(edac40_list_node)*max_dev_num);
    // send discovery request and receive the list
    int dev_num=edac40_list_devices(list,max_dev_num,200,1); // 200ms timeout, one attempt
    if(dev_num<1) throw mirror_fault("No EDAC40 device(s) found");
    for(int i=0; i<dev_num; i++)
        if(MAC_address==NULL || (MAC_address && strlen(MAC_address)<1) || strncmp(MAC_address,list[i].MACAddress,17)==0)
        // NULL pointer OR empty string for "auto mode" OR exact match
        {
          strcpy(dac_id,list[i].MACAddress);
          if(!(edac_host_name=edac40_find_device(MAC_address))) throw mirror_fault("There is no such EDAC40 device");
          if((edac_socket=edac40_open(edac_host_name,0))<0) throw mirror_fault("Can't open specified EDAC40 device");
          free(list);
        }
}

EtherDAC::~EtherDAC()
{
    edac40_close(edac_socket);
    edac_instance_count--;
    if(edac_instance_count<1) edac40_finish();
}

vector<char *> *EtherDAC::listDevices()
{
    vector<char*> *device_list=new vector<char*>;
    device_list->resize(0);
    int max_dev_num=50;
    edac40_list_node *list;
    // prepare memory buffer for device list
    list=(edac40_list_node *)malloc(sizeof(edac40_list_node)*max_dev_num);
    // send discovery request and receive the list
    edac40_init();
    int dev_num=edac40_list_devices(list,max_dev_num,200,1); // 200ms timeout, one attempt
    edac40_finish();
    for(int i=0; i<dev_num; i++) device_list->push_back(strdup(list[i].MACAddress));
    free(list);
    // sort the list to male some predictable order for multi-unit mirror configurations
    std::sort(device_list->begin(),device_list->end());
    return device_list;
}

void EtherDAC::updateAllChannels()
{
    char *edac_packet=0;
    int edac_packet_size;
    unsigned *buf=(unsigned*)malloc(numberOfChannels()*sizeof(unsigned));
    for(unsigned i=0; i<numberOfChannels(); i++) // map logical channel to physical
        buf[EDAC40_CHANNEL_MAP[i]]=dac_values[i];
    edac_packet_size=edac40_prepare_packet_from_array(buf,EDAC40_SET_VALUE,&edac_packet);
    if(edac40_send_packet(edac_socket,edac_packet,edac_packet_size)<0) throw mirror_fault("Problem during sending EDAC40 packet");
    if(edac_packet) free(edac_packet);
    free(buf);
}

// --------- generic USBDAC class member functions definitions
// Constructor  USBDAC::USBDAC
// Open device specifies by its model and serial number
//
// UPDATE: small fixes, it turns out that FD_ListDevices does not list devices that already opened.
//         is it correct behaviour?
USBDAC::USBDAC(const char *model, const char *serial_number)
{
    int ndevs=0;
    char sn[16];
    char descr[64];
    bool connected_ok=false;
    FT_STATUS fs=FT_OTHER_ERROR;
    // The "detection" mechanism below is used for both 4-ch and 40-ch devices as well as for "FAST" and HV DALSA variants with FT232H interface
    if(FT_ListDevices(&ndevs,0,FT_LIST_NUMBER_ONLY)!=FT_OK) throw mirror_fault("Can't list USB-DAC devices");
    if(ndevs<1) throw mirror_fault("No USB-DAC device(s) found");
    for(int i=0; i<ndevs; i++)
    {
       if(FT_ListDevices((void *)i,sn,FT_LIST_BY_INDEX|FT_OPEN_BY_SERIAL_NUMBER)!=FT_OK) continue;
       if(FT_ListDevices((void *)i,descr, FT_LIST_BY_INDEX|FT_OPEN_BY_DESCRIPTION)!=FT_OK) continue;
       if(strcmp(descr,model)!=0) continue; // this is a device of some other type, look for the next
       if(serial_number==NULL || (serial_number!=NULL && strlen(serial_number)<1) ||
             strcmp(serial_number,sn)==0) //   either the s/n is not specified (NULL pointer OR zero-length string) OR exact match
       {
           fs=FT_Open(i,&usbdac_handle);
           strcpy(dac_id,sn);
           if(fs!=FT_OK) throw mirror_fault("Can't connect to USB-DAC module");
           connected_ok=true;
           break;
       }
    }
    if(!connected_ok) throw mirror_fault("No such USB-DAC device");
 }

USBDAC::~USBDAC()
{
    //printf("in ~USBDAC()\n");
	if(FT_Close(usbdac_handle)!=FT_OK) throw mirror_fault("Problem with USB-DAC during close operation");
}

// List all USB DAC devices (either 4ch or 40ch) filtered accordingly to "description" in the mode string.
// It is also possible to pass NULL pointer or empty (zero-sized) string to get all devices regardless of their type (USB4DAC + USB4DAC).
//
// UPDATE: seems that it is does not list already opened devices! I didn't realize that fact before.
vector<char*> *USBDAC::listDevices(const char* model)
{
    int ndevs=0;
    char sn[16];
    char descr[64];
    vector<char*> *device_list=new vector<char*>;
    if(FT_ListDevices(&ndevs,0,FT_LIST_NUMBER_ONLY)!=FT_OK) throw mirror_fault("Can't list USB-DAC devices");
    for(int i=0; i<ndevs; i++)
    {
       if(FT_ListDevices((void *)i,sn,FT_LIST_BY_INDEX|FT_OPEN_BY_SERIAL_NUMBER)!=FT_OK) continue; // FIXME: it fails sometimes for unknown reason...
       if(FT_ListDevices((void *)i,descr, FT_LIST_BY_INDEX|FT_OPEN_BY_DESCRIPTION)!=FT_OK) continue;// ..probably after operations on the USB bus with other devices.
       if(model!=0 && strlen(model)>0 && strcmp(descr,model)!=0) continue; // this is a device of some other type, look for the next one
       device_list->push_back(strdup(sn));
    }
    std::sort(device_list->begin(),device_list->end());
    return device_list;
}

// --------- USB40DAC class member functions definitions

DAC40USB::DAC40USB(const char *serial_number):USBDAC(USB40DAC_MAGIC, serial_number)
{
    unsigned long nbytes;
    initInternals();
    memset(usb40dac_packet,0,130);
    if(FT_Write(usbdac_handle,usb40dac_packet,130,&nbytes)!=FT_OK) throw mirror_fault("Unable to write to DAC40USB module");
}

void DAC40USB::updateAllChannels()
{
    unsigned long nbytes;
    //-------------------- based on MakePacket (almost copy-pasted)
    uint8_t *p=usb40dac_packet+1;
    for(int i=0,s=0;i<8;i++,s+=5)
    {
        // Forming of the address parts of the control data words to be addressed to five DAC chips
    *(p++)=0;
    *(p++)=(i&4)?0x1f:0;
    *(p++)=(i&2)?0x1f:0;
    *(p++)=(i&1)?0x1f:0;

    // Forming of the voltage level codes from the buffer of DAC channels
    // using the table of DAC channels
    for(int j=0,mask=0x800;j<12;j++,mask>>=1)
        *(p++)=
            ((dac_values[USBDAC_CHANNEL_TABLE[s+0]]&mask)?0x01:0) |
            ((dac_values[USBDAC_CHANNEL_TABLE[s+1]]&mask)?0x02:0) |
            ((dac_values[USBDAC_CHANNEL_TABLE[s+2]]&mask)?0x04:0) |
            ((dac_values[USBDAC_CHANNEL_TABLE[s+3]]&mask)?0x08:0) |
            ((dac_values[USBDAC_CHANNEL_TABLE[s+4]]&mask)?0x10:0) ;
    }
    usb40dac_packet[0]   = 0xff; // non-zero start byte
    //-------------------------
    if(FT_Write(usbdac_handle,usb40dac_packet,130,&nbytes)!=FT_OK) throw mirror_fault("Can't wirite to USB40DAC module");
}

// --------- USB40DAC_FAST class member functions definitions

DAC40USB_FAST::DAC40USB_FAST(const char *model, const char *serial_number)
{
    initInternals();
    bytesToWrite=0;

    dwNumBytesSent = 0; // Count of actual bytes sent - used with FT_Write
    DWORD dwClockDivisor = 0;//0x05DB; // Value of clock divisor, SCL Frequency = 60/((1+0x05DB)*2) (MHz) = 1Mhz


  // Set up the Hi-Speed specific commands for the FTx232H
   buffer[bytesToWrite++]=0x8A;  // Use 60MHz master clock (disable divide by 5)
   buffer[bytesToWrite++]=0x97;  // Turn off adaptive clocking (may be needed for ARM)
   buffer[bytesToWrite++]=0x8D;  // disable 3-phase clocking
   if((ftStatus=FT_Write(ftHandle,buffer,bytesToWrite,&dwNumBytesSent))!=FT_OK) throw mirror_fault("Error writing");

   bytesToWrite=0;
   buffer[bytesToWrite++]=0x86; // command -- set SCK frequensy divider
   buffer[bytesToWrite++]=dwClockDivisor & 0xFF;
   buffer[bytesToWrite++]=(dwClockDivisor >> 8) & 0xFF;
   if((ftStatus=FT_Write(ftHandle,buffer,bytesToWrite,&dwNumBytesSent))!=FT_OK) throw mirror_fault("Error writing");



    bytesToWrite=0;

    gpio=0xFF;

    buffer[bytesToWrite++] = 0x82; // write to High GPIOs
    buffer[bytesToWrite++] = 0xF9; // data -- set 0 to MSB <- SYNC
    buffer[bytesToWrite++] = 0xFF; // direction mask -- out

    buffer[bytesToWrite++] = 0x82; // write to High GPIOs
    buffer[bytesToWrite++] = 0xFF; // data -- set 0 to MSB <- SYNC
    buffer[bytesToWrite++] = 0xFF; // direction mask -- out


    // -- dummy command for delay
    buffer[bytesToWrite++] = 0x82; // write to High GPIOs
    buffer[bytesToWrite++] = 0xFF; // data -- set 0 to MSB <- SYNC
    buffer[bytesToWrite++] = 0xFF; // direction mask -- out

    buffer[bytesToWrite++] = 0x82; // write to High GPIOs
    buffer[bytesToWrite++] = 0xFF; // data -- set 0 to MSB <- SYNC
    buffer[bytesToWrite++] = 0xFF; // direction mask -- out

    buffer[bytesToWrite++] = 0x82; // write to High GPIOs
    buffer[bytesToWrite++] = 0xFF; // data -- set 0 to MSB <- SYNC
    buffer[bytesToWrite++] = 0xFF; // direction mask -- out
    //---

    buffer[bytesToWrite++] = 0x80; // write to High GPIOs
    buffer[bytesToWrite++] = 0xFF; // data -- set 0 to MSB <- SYNC
    buffer[bytesToWrite++] = 0xFF; // direction mask -- out

   if((ftStatus=FT_Write(ftHandle,buffer,bytesToWrite,&dwNumBytesSent))!=FT_OK) throw mirror_fault("Error writing");

   //delay(1);

}

void DAC40USB_FAST::updateAllChannels()
{
        bytesToWrite=0;
        for(int channel=0; channel<numberOfChannels(); channel++)
               dac_set_value(buffer, &bytesToWrite, channel, dac_values[channel]);
        dac_cycle_ldac(buffer, &bytesToWrite);
        if((ftStatus=FT_Write(ftHandle,buffer,bytesToWrite,&dwNumBytesSent))!=FT_OK) throw mirror_fault("Error writing");
}

// -- utility functions specific for AD537X, they pre-fill the transfer buffer of MSwith some

void DAC40USB_FAST::dac_set_sync(uint8_t *buffer, DWORD *index, uint8_t value)
{
    gpio=(gpio & 0x7F)^((value<<7) & 0x80) ;
    buffer[(*index)++] = 0x82; // write to High GPIOs
    buffer[(*index)++] = gpio; // data -- set 0 to MSB <- SYNC
    buffer[(*index)++] = 0xFF; // direction mask -- out
}

void DAC40USB_FAST::dac_command(uint8_t *buffer, DWORD *index, uint8_t mode, uint8_t address, uint16_t data)
{

    dac_set_sync(buffer, index, 0);

    buffer[(*index)++]=0x10; // write to SPI
    buffer[(*index)++]=2; // 3 bytes; 0 means 1 byte!
    buffer[(*index)++]=0; //

    buffer[(*index)++]=(mode<<6)+address;
    buffer[(*index)++]=(data>>8) & 0xFF;
    buffer[(*index)++]=data & 0xFF;

    dac_set_sync(buffer, index, 1);
}

void DAC40USB_FAST::dac_set_value(uint8_t *buffer, DWORD *index, uint8_t channel, uint16_t data)
{
    dac_command(buffer, index, 3, channel+8, data);
}

void DAC40USB_FAST::dac_set_gain(uint8_t *buffer, DWORD *index, uint8_t channel, uint16_t data)
{
    dac_command(buffer, index, 1, channel+8, data);
}

void DAC40USB_FAST::dac_set_offset(uint8_t *buffer, DWORD *index, uint8_t channel, uint16_t data)
{
    dac_command(buffer, index, 2, channel+8, data);
}

void DAC40USB_FAST::dac_set_offset_dacs(uint8_t *buffer, DWORD *index, uint16_t data)
{
    dac_command(buffer, index, 0, 2, data);
    dac_command(buffer, index, 0, 3, data);
}

void DAC40USB_FAST::dac_cycle_ldac(uint8_t *buffer, DWORD *index)
{
    gpio=gpio & 0xFE;
    buffer[(*index)++] = 0x82; // write to High GPIOs
    buffer[(*index)++] = gpio; // data -- set 0 to MSB <- SYNC
    buffer[(*index)++] = 0xFF; // direction mask -- out

    // delay???

    gpio=gpio | 0x01;
    buffer[(*index)++] = 0x82; // write to High GPIOs
    buffer[(*index)++] = gpio; // data -- set 0 to MSB <- SYNC
    buffer[(*index)++] = 0xFF; // direction mask -- out

}

// ------------------- DAC_DALSA supports mirrors with built-in electronics based on
// Dalsa Teledyne DH9685A (high-voltage 96-channel DAC with S/H) and
// FTDI FT232H USB Hi-Speed bridge with MPSSE support.

DAC_DALSA::DAC_DALSA(const char *serial_number):USBDAC(USBDALSA_MAGIC, serial_number)
{

    FT_STATUS ftStatus=FT_OTHER_ERROR;

    DWORD dwNumBytesSent = 0; // Count of actual bytes sent - used with FT_Write
    DWORD dwClockDivisor = 0;//0x05DB; // Value of clock divisor, SCL Frequency = 60/((1+0x05DB)*2) (MHz) = 1Mhz

   initInternals();

      if((ftStatus=FT_SetBitMode(usbdac_handle,0x00,0x0))!=FT_OK)
//   {
//           printf("ftStatus=%ld\n",ftStatus);
           throw runtime_error("Failed to reset the device");
//   }

    if((ftStatus=FT_SetBitMode(usbdac_handle,0xFF,0x02))!=FT_OK)
//    {
//            printf("ftStatus=%ld\n",ftStatus);
            throw runtime_error("Unable to set MPSSE mode");
//     }

    //----------------------------------------------


   bytesToWrite=0;

   // Set up the Hi-Speed specific commands for the FTx232H
   buffer[bytesToWrite++]=0x8A; // Use 60MHz master clock (disable divide by 5)
   buffer[bytesToWrite++]=0x97; // Turn off adaptive clocking (may be needed for ARM)
   buffer[bytesToWrite++]=0x8D; // Disable 3-phase data clocking
   if((ftStatus=FT_Write(usbdac_handle,buffer,bytesToWrite,&dwNumBytesSent))!=FT_OK) throw runtime_error("Error writing");

   bytesToWrite=0;
   buffer[bytesToWrite++]=0x86; // command -- set SCK frequensy divider
   buffer[bytesToWrite++]=dwClockDivisor & 0xFF;
   buffer[bytesToWrite++]=(dwClockDivisor >> 8) & 0xFF;
   if((ftStatus=FT_Write(usbdac_handle,buffer,bytesToWrite,&dwNumBytesSent))!=FT_OK) throw runtime_error("Error writing");


   bytesToWrite=0;
   buffer[bytesToWrite++]=0x82; // Write High byte of GPIO
   buffer[bytesToWrite++]=0x40;    // data= !RST
   buffer[bytesToWrite++]=0xFF; // direction -- all bits configured for output

   buffer[bytesToWrite++] = 0x80; // write to Low GPIOs
   buffer[bytesToWrite++] = 0x01; // data -- set 0 to MSB <- SYNC, 1 for SCK line defauly value (!)
   buffer[bytesToWrite++] = 0xFF; // direction mask -- out

   if((ftStatus=FT_Write(usbdac_handle,buffer,bytesToWrite,&dwNumBytesSent))!=FT_OK) throw runtime_error("Error writing");
}

void DAC_DALSA::updateAllChannels()
{
    FT_STATUS ftStatus;
    DWORD dwNumBytesSent = 0; // Count of actual bytes sent - used with FT_Write
    bytesToWrite=0;
    for(unsigned quad_i=0; quad_i<24; quad_i++)
    {
        for(unsigned dac_i=0; dac_i<4; dac_i++)
        {

            buffer[bytesToWrite++] = 0x80; // write to Low GPIOs
            buffer[bytesToWrite++] = 0x01; // data -- set 0 to MSB <- SYNC, 1 for SCK line defauly value (!)
            buffer[bytesToWrite++] = 0xFF; // direction mask -- out

            buffer[bytesToWrite++]=0x10; // write to SPI
            buffer[bytesToWrite++]=3; // 4 bytes; 0 means 1 byte!
            buffer[bytesToWrite++]=0; //

            uint16_t value=dac_values[(quad_i<<2)+dac_i];
            //printf("%d %d %d\n",(quad_i<<2)+dac_i,actuator[(quad_i<<2)+dac_i],value);
            buffer[bytesToWrite++]=(dac_i<3)?0:2; // command -- just write for 0,1,2 then write+update for the last DAC
            buffer[bytesToWrite++]=(dac_i<<4)+((value>>12) & 0xFF);
            buffer[bytesToWrite++]=(value>>4) & 0xFF;
            buffer[bytesToWrite++]=(value<<4) & 0xF0;

            //if(quad_i==0 && dac_i==0) printf("%08X %02X %02X %02X %02X\n", value, buffer[bytesToWrite-4],buffer[bytesToWrite-3],buffer[bytesToWrite-2],buffer[bytesToWrite-1]);

            buffer[bytesToWrite++] = 0x80; // write to Low GPIOs
            buffer[bytesToWrite++] = 0x81; // data -- set 1 to MSB <- SYNC
            buffer[bytesToWrite++] = 0xFF; // direction mask -- out

        }

        for(int i=0; i<40; i++) // repeat this command to satisfy t_ds (DAC setup time) of 4us
        {
            buffer[bytesToWrite++]=0x82; // Write High byte of GPIO
            buffer[bytesToWrite++]=quad_i | 0x40;    // data
            buffer[bytesToWrite++]=0xFF; // direction -- all bits configured for output
        }

        for(int i=0; i<40; i++) // t_s=4us (20)
        {
            buffer[bytesToWrite++]=0x82; // Write High byte of GPIO
            buffer[bytesToWrite++]=quad_i | 0x60;    // data with SAMPLE line set
            buffer[bytesToWrite++]=0xFF; // direction -- all bits configured for output
        }

        buffer[bytesToWrite++]=0x82; // Write High byte of GPIO
        buffer[bytesToWrite++]=quad_i | 0x40;    // data
        buffer[bytesToWrite++]=0xFF; // direction -- all bits configured for output
   }
   if((ftStatus=FT_Write(usbdac_handle,buffer,bytesToWrite,&dwNumBytesSent))!=FT_OK) throw runtime_error("Error writing");
}



// --------- USB4DAC class member functions definitions

DAC4USB::DAC4USB(const char *serial_number):USBDAC(USB4DAC_MAGIC, serial_number)
{
    initInternals();
    updateAllChannels();
}

void DAC4USB::updateAllChannels()
{
    uint8_t *pBuffer=usb4dac_packet;
    for (uint8_t numChannel = 0; numChannel < 4; numChannel++)
    {
        uint8_t operation=numChannel;
        uint32_t data=dac_values[numChannel];
        uint8_t crcSum;

        crcSum   = operation & 0x0f;
        crcSum  += data & 0x000f;
        crcSum  += (data >> 4) & 0x000f;
        crcSum  += (data >> 8) & 0x000f;
        crcSum  += (data >> 12) & 0x000f;

        *(pBuffer++) = 0x80 | ((operation & 0x0f) << 3) | ((data & 0xe000) >> 13);
        *(pBuffer++) = (data & 0x1fc0) >> 6;
        *(pBuffer++) = ((data & 0x003f) << 1) | ((crcSum & 0x80) >> 7);
        *(pBuffer++) = crcSum & 0x7f;
    }
    unsigned long nbytes;
    if(FT_Write(usbdac_handle,usb4dac_packet,16,&nbytes)!=FT_OK) throw mirror_fault("Can't wirite to USB4DAC module");
}

//---------- Mirror class member functions definitions

Mirror::Mirror(DAC *dac0, DAC *dac1, DAC *dac2)
{
    // no actuator mapping defined yet
    mirror_actuator_map=0;
    mirror_unit_map=0;
    mirror_channels_n=0;
    // some reasonable defaults...
    mirror_quadratic_response=false;
    mirror_cancel_bias=false;
    mirror_compensate_hysteresis=false;
    mirror_immediate_update=true;
    // that one above seems reasonable default, so setOneChannel will has result immediately, while for
    // group updates it will be changed temporarily
    //
    mirror_flat_configuration=0;
    mirror_feedforward_matrix=0;
    mirror_zernike_modes=0;
    mirror_zernike_modes_n=0;
    mirror_voltages=0;


    //---------
//    int dac_n=0;
//    if(dac0!=0) dac_n++;
//    if(dac1!=0) dac_n++;
//    if(dac2!=0) dac_n++;
//    dacs.resize(dac_n);
//    if(dac0) dacs.at(0)=dac0;
//    if(dac1) dacs.at(1)=dac1;
//    if(dac2) dacs.at(2)=dac2;
     // FIXME: commented-out code replaced with three attaches. Untested, but should work.
     // We have mirrors which use up to three 40-channel DAC units
     attachDAC(dac0);
     attachDAC(dac1);
     attachDAC(dac2);
     if(dac0) dac0->setImmediateUpdate(mirror_immediate_update);
     if(dac1) dac1->setImmediateUpdate(mirror_immediate_update);
     if(dac2) dac2->setImmediateUpdate(mirror_immediate_update);
}

Mirror::~Mirror()
{
    //printf("in ~Mirror\n");
	dacs.resize(0);
}

void Mirror::attachDAC(DAC *dac)
{
    if(dac) dacs.push_back(dac); // add a DAC unit to the list, but only if it points to something
}

void Mirror::releaseDACs()
{
	// this function can be called explicitely just before destuctor to destroy attached DAC objects
    // useful when the [simple] application does not keep a list of dac devices
	for(int i=0; i<dacs.size(); i++)
    { 		
		delete dacs[i];
	}
    dacs.resize(0);	
}

void Mirror::setActuatorsMap(int channels_n, const int *actuator_map, const int *unit_map)
{

    if(mirror_actuator_map) free(mirror_actuator_map);
    if(mirror_unit_map) free(mirror_unit_map);

    mirror_channels_n=channels_n;

    safe_free(mirror_voltages);
    mirror_voltages=(double *)malloc(mirror_channels_n*sizeof(double));

    unsigned buffer_size=channels_n*sizeof(int);
   // mirror_actuator_map=(ACTUATOR_MAP)malloc(buffer_size);
    mirror_actuator_map=(unsigned*)malloc(buffer_size);
    mirror_unit_map=(unsigned*)calloc(buffer_size,1);
    if(actuator_map) memcpy(mirror_actuator_map, actuator_map,buffer_size );
                else memset(mirror_actuator_map,0,buffer_size);
    // check if NULL pointer passed for single-unit mirror configuration
    if(unit_map) memcpy(mirror_unit_map, unit_map, buffer_size);
          //  else memset(mirror_unit_map,0,buffer_size);
//    printf("--prepare--\n");
//    printf("channels_n=%d\n",channels_n);
//    for(int i=0; i<mirror_channels_n; i++) printf("%d: %d, %d\n",i,mirror_unit_map[i],mirror_actuator_map[i]);
}

void Mirror::setChannels(const double *values, int channels_n)
{
    double bias=0;
    int channels_to_set=MIN(channels_n,numberOfActuators());
    if(mirror_cancel_bias) for(int i=0; i<channels_to_set; i++) bias+=values[i];
    bias/=channels_to_set;
    bool tmp_mirror_immediate_update=isImmediateUpdate();
    setImmediateUpdate(false);
    for(int i=0; i<channels_to_set; i++) setOneChannel(i, values[i]-(mirror_cancel_bias?bias:0));
    updateAllChannels();
    setImmediateUpdate(tmp_mirror_immediate_update);
}

void Mirror::getChannels(double *values)
{
    for(unsigned i=0; i<mirror_channels_n; i++) values[i]=mirror_voltages[i];
}

void Mirror::setOneChannel(int channel, double value)
{
    // some simple sanity checks
    if(channel<0 || channel>numberOfActuators()) throw mirror_fault("Mirror actuator number is out of range");
    //if(value<-1 || value>1) throw mirror_fault("Mirror control value is out of range");
    // no, better just trim them...
    if(value>1) value=1;
    if(value<-1) value=-1;
    mirror_voltages[channel]=value;
    // find out the physical unit- and channel number
    unsigned dac_unit=*(mirror_unit_map+channel);
    int dac_channel=*(mirror_actuator_map+channel);
//    printf("%d:%d %d\n", dac_unit, dac_channel, dacs.size());
    if(dac_unit>dacs.size()-1) throw mirror_fault("Not enought DAC unit(s) assigned to the mirror");
    double dac_range=dacs.at(dac_unit)->maxValue();
    unsigned dac_code=(unsigned)(dac_range*(mirror_quadratic_response?sqrt((value+1)/2):(value+1)/2));
    dacs.at(dac_unit)->setOneChannel(dac_channel,dac_code);
}

// Set the same control value to all mirror actuators.
// It _does_not_ suppress bias, but corrects response in accord with present settings,
// setOneChannel() takes care of that.
void Mirror::setAllChannels(double value)
{
    bool tmp_immediate_update=isImmediateUpdate();
    setImmediateUpdate(false); // suppress updates temporary
    for(int channel=0; channel<numberOfActuators(); channel++) setOneChannel(channel,value);
    updateAllChannels();
    setImmediateUpdate(tmp_immediate_update); // restore initial value
}

void Mirror::updateAllChannels()
{

//    for(int i=0; i<numberOfActuators(); i++) printf("%lf ",mirror_voltages[i]);
//    printf("\n");

    // trigger update for every DAC unit the mirror connected to
    for(unsigned i=0; i<dacs.size(); i++) dacs.at(i)->updateAllChannels();
}

int Mirror::numberOfUnits()
{
    if(!mirror_actuator_map || !mirror_unit_map) return 0;
    unsigned n=0;
    for(int i=0; i<numberOfActuators(); i++) if((unsigned)(mirror_unit_map[i])>n) n=mirror_unit_map[i];
    return n+1;
}

void Mirror::setImmediateUpdate(bool immediate_update)
{
    mirror_immediate_update=immediate_update;
    for(unsigned i=0; i<dacs.size(); i++)
        dacs.at(i)->setImmediateUpdate(immediate_update);
}

char* Mirror::pinout()
{
    char chan_id1[256],chan_id2[256];
    unsigned nr_of_units=0;
    char *out_buf;
    if(!mirror_actuator_map) return 0;
    out_buf=(char*)malloc(2048);
    if(mirror_unit_map)
        for(unsigned i=0; i<mirror_channels_n; i++) if(mirror_unit_map[i]>nr_of_units) nr_of_units=mirror_unit_map[i];
    nr_of_units++;
    for(unsigned unit_idx=0; unit_idx<nr_of_units; unit_idx++)
      {
         sprintf(out_buf,"--- Unit %d ---\n",unit_idx+1);
         for(unsigned conn_idx=0; conn_idx<2; conn_idx++)
            {
              sprintf(out_buf,"Connector %d:\n",conn_idx+1);
              for(int pin_idx=0; pin_idx<10; pin_idx++)
                 {
                    strcpy(chan_id1," NC");
                    strcpy(chan_id2," NC");
                    for(unsigned i=0; i<mirror_channels_n; i++)
                      {
                         if(unit_idx==mirror_unit_map[i] && pin_idx*2+conn_idx*20==mirror_actuator_map[i]) sprintf(chan_id1,"%3d",i+1);
                         if(unit_idx==mirror_unit_map[i] && pin_idx*2+1+conn_idx*20==mirror_actuator_map[i]) sprintf(chan_id2,"%3d",i+1);
                      }
                    sprintf(out_buf,"%7s %2s %2s\n",(pin_idx==0)?"pin 1->":" ", chan_id1, chan_id2);
                 }
            }
      }
    return out_buf;
}


void Mirror::degauss(double delay_sec, bool serial, int steps)
{
    double decrement_per_step=1.0/steps;
    for(int channel=0;channel<numberOfActuators();channel++)
       {
          // gradually decresing range
          for(double range=1; range>0; range-=decrement_per_step)
            {
                if(range<0) range=0; // it is amplitude, it shouldn't be negative
                // cylcle for negative and positive state
                for(double value=-range; value<=1.1*range; value+=2*range)
                 {
                    //printf("channel=%d value=%lf\n",channel, value);
                    if(serial) setOneChannel(channel,value);
                          else setAllChannels(value);
                    delay(delay_sec);
                 }
            }
          if(!serial) break; // it was done for all channels at once, nothing more to do
    }
}

void Mirror::setFlatConfiguration(const double *values, int channels_n)
{
    if(mirror_channels_n<1) throw mirror_fault("Can't set flat configuration since no actuator map defined yet");
    if(!values) throw mirror_fault("Null pointer passed for flat configuration");
    safe_free(mirror_flat_configuration);
    mirror_flat_configuration=(double *)malloc(mirror_channels_n*sizeof(double));
    int channels_to_set=MIN(channels_n,mirror_channels_n);
    for(int i=0;i<channels_to_set;i++) mirror_flat_configuration[i]=values[i];
}

void Mirror::setFeedForwardMatrix(const double *matrix, int channels_n, int modes_n)
{
    if(mirror_channels_n<1) throw mirror_fault("Can't set feedforward matrix since no actuator map defined yet");
    if(!matrix) throw mirror_fault("Null pointer passed for feedforward matrix");
    mirror_zernike_modes_n=modes_n;
    safe_free(mirror_feedforward_matrix);
    mirror_feedforward_matrix=(double *)malloc(mirror_channels_n*modes_n*sizeof(double));
    int channels_to_set=MIN(channels_n,mirror_channels_n);
    for(int j=0;j<modes_n;j++)
       for(int i=0;i<channels_to_set;i++)
        {
            int idx_from=j*channels_n+i;
            int idx_to=j*mirror_channels_n+i;
            mirror_feedforward_matrix[idx_to]=matrix[idx_from];
        }
    safe_free(mirror_zernike_modes);
    mirror_zernike_modes=(double*)malloc(modes_n*sizeof(double));
}

void Mirror::setOneZernikeMode(int idx, double value)
{
    if(idx>mirror_zernike_modes_n) return;
    mirror_zernike_modes[idx]=value;
    calculateVoltagesByModes();
    if(isImmediateUpdate()) updateAllChannels();
}

void Mirror::setZernikeModes(double *values, int modes_n)
{
    if(!values) throw mirror_fault("NULL pointer passed to setZernikeModes");
    bool tmp_immediate_update=isImmediateUpdate();
    setImmediateUpdate(false);
    for(int i=0; i<modes_n; i++) setOneZernikeMode(i, values[i]);
    updateAllChannels();
    setImmediateUpdate(tmp_immediate_update);
}

// set the same value for all Zernike modes. Really useful for zeroing only (?)
void Mirror::setAllZernikeModes(double value)
{
    bool tmp_immediate_update=isImmediateUpdate();
    setImmediateUpdate(false);
    for(int i=0; i<mirror_zernike_modes_n; i++) setOneZernikeMode(i, value);
    updateAllChannels();
    setImmediateUpdate(tmp_immediate_update);
}

void Mirror::calculateVoltagesByModes()
{
//    printf("Z=");
//    for(int i=0; i<mirror_zernike_modes_n; i++)  printf("%lf ",mirror_zernike_modes[i]);
//    printf("\n");
    for(unsigned i=0; i<mirror_channels_n; i++)
    {
        double s=mirror_flat_configuration[i];
        for(int j=0; j<mirror_zernike_modes_n; j++)
            s+=mirror_zernike_modes[j]*mirror_feedforward_matrix[j*mirror_channels_n+i];
        if(s>1) s=1;
        if(s<-1) s=-1;
        mirror_voltages[i]=s;
        setOneChannel(i,s);
    }
//    printf("V=");
//    for(int i=0; i<mirror_channels_n; i++) printf("%lf ",mirror_voltages[i]);
//    printf("\n");
}

// Simple helper function, it loads actuator mapping for a specific mirror configuration
// and configures parameters in a way typical for MMDM/PDM mirror (those can be chanched later if desired).
void Mirror::prepare(bool is_pdm, int channels_n, const int *actuator_map, const int *unit_map)
{
    setActuatorsMap(channels_n,actuator_map,unit_map);
    setCancelBias(is_pdm);
    setQuadraticResponse(!is_pdm);
    setCompensateHysteresis(is_pdm);
}

//------------------- helper functions/classes for specific OKO mirrors

const char* OKOMirrors[]={"MMDM 37ch,15mm","MMDM 39ch,30mm", "MMDM 79ch,30mm","MMDM 79ch,40mm","MMDM 79ch,50mm",
           "PDM 19ch,30mm","PDM 37ch,30mm", "PDM 37ch,30mm,v.2005", "PDM 37ch,trihex grid", "PDM 37ch,rectangular grid",
           "PDM 37ch,50mm","PDM 37ch,50mm,v.2008", "PDM 79ch,50mm", "PDM 109ch,50mm",
           "MMDM linear 19ch","PDM linear 20ch",
           "PDM 18ch,50mm,low-order", "PDM 19ch,30mm,low-order", "PDM 18ch,30mm,low-order",
           "MMDM 17ch,with tip/tilt",
           "PDM 39ch,50mm,low-order",
           //"Generic MMDM mirror",
           //"Generic PDM mirror",
           "MMDM 96ch, embedded control",NULL};		   
		   
// Not very smart implementation... the good thing is that it is static stand-alone function
Mirror* newMirrorByName(const char *mirror_type, DAC *dac)
{
   		if(!strcmp(mirror_type,"MMDM 37ch,15mm")) return new Mirror_MMDM37(dac);
        if(!strcmp(mirror_type,"MMDM 39ch,30mm")) return new Mirror_MMDM39_30(dac);
        if(!strcmp(mirror_type,"MMDM 79ch,30mm")) return new Mirror_MMDM79_30(dac);
        if(!strcmp(mirror_type,"MMDM 79ch,40mm")) return new Mirror_MMDM79_40(dac);
        if(!strcmp(mirror_type,"MMDM 79ch,50mm")) return new Mirror_MMDM79_50(dac);
        if(!strcmp(mirror_type,"PDM 19ch,30mm")) return new Mirror_PDM19(dac);
        if(!strcmp(mirror_type,"PDM 37ch,30mm")) return new Mirror_PDM37(dac);
        if(!strcmp(mirror_type,"PDM 37ch,30mm,v.2005")) return new Mirror_PDM37_2005(dac);
        if(!strcmp(mirror_type,"PDM 37ch,rectangular grid")) return new Mirror_PDM37RECT(dac);
        if(!strcmp(mirror_type,"PDM 37ch,trihex grid")) return new Mirror_PDM37TRIHEX(dac);
        if(!strcmp(mirror_type,"PDM 37ch,50mm")) return new Mirror_PDM37_50(dac);
        if(!strcmp(mirror_type,"PDM 37ch,50mm,v.2008")) return new Mirror_PDM37_50_2008(dac);
        if(!strcmp(mirror_type,"PDM 79ch,50mm")) return new Mirror_PDM79_50(dac);
        if(!strcmp(mirror_type,"PDM 109ch,50mm")) return new Mirror_PDM109_50(dac);
        if(!strcmp(mirror_type,"MMDM linear 19ch")) return new Mirror_MMDM19LIN(dac);
        if(!strcmp(mirror_type,"PDM linear 20ch")) return new Mirror_PDM20LIN(dac);
        if(!strcmp(mirror_type,"PDM 18ch,50mm,low-order")) return new Mirror_PDM18LO_50(dac);
        if(!strcmp(mirror_type,"PDM 19ch,30mm,low-order")) return new Mirror_PDM19LO_30(dac);
        if(!strcmp(mirror_type,"PDM 18ch,30mm,low-order")) return new Mirror_PDM18LO_30(dac);
        if(!strcmp(mirror_type,"MMDM 17ch,with tip/tilt")) return new Mirror_MMDM17TT(dac);
        if(!strcmp(mirror_type,"PDM 39ch,50mm,low-order")) return new Mirror_PDM39_50_LO(dac);
        //if(!strcmp(mirror_type,"Generic MMDM mirror")) return new Mirror(dac); //***FIXME: probably need to call prepare here
        //if(!strcmp(mirror_type,"Generic PDM mirror")) return new Mirror(dac);
        if(!strcmp(mirror_type,"MMDM 96ch, embedded control")) return new Mirror_MMDM96_EMB(dac);
		return new Mirror(dac); // fall back to generic mirror type
}

const char* OKODACs[]={"USB DAC 40ch, 14bit","Ethernet DAC 40ch, 16bit","Fast USB DAC 40ch, 16bit","Embedded HV DAC","USB DAC 4ch",NULL};
DAC *newDACByName(const char *dac_type, const char *ID)
{
    if(!strcmp(dac_type,"USB DAC 40ch, 12bit")) return new DAC40USB(ID);
    if(!strcmp(dac_type,"Ethernet DAC 40ch, 16bit")) return new EtherDAC(ID);
    if(!strcmp(dac_type,"Fast USB DAC 40ch, 16bit")) return new DAC40USB_FAST(ID);
    if(!strcmp(dac_type,"Embedded HV DAC")) return new DAC_DALSA(ID);
    if(!strcmp(dac_type,"USB DAC 4ch")) return new DAC4USB(ID);
    return new DAC40USB(ID); // A kind of reasonable default
}

//-------------------
Mirror_MMDM37::Mirror_MMDM37(DAC *dac):Mirror(dac)
{
    int actuators[37]={10,18,13,9,5,6,14,19,17,12,11,1,3,2,4,7,8,15,16,
                       34,36,37,38,35,23,25,21,22,24,26,27,28,29,30,31,32,33};
    prepare(false,sizeof(actuators)/sizeof(int),actuators,0);

    setFlatConfiguration(MMDM37_FLAT, 37);
    setFeedForwardMatrix(MMDM37_FFMAT,37,21);
}

Mirror_PDM19::Mirror_PDM19(DAC *dac):Mirror(dac)
{
    int actuators[]={7,12,4,3,9,11,18,10,8,6,2,1,5,13,15,17,19,16,14};
    prepare(true,sizeof(actuators)/sizeof(int),actuators,0);

    setFlatConfiguration(PDM19_FLAT, 19);
    setFeedForwardMatrix(PDM19_FFMAT,19,14);
}

Mirror_PDM37::Mirror_PDM37(DAC *dac):Mirror(dac)
{
    int actuators[]={10,13,14,16,8,5,6,11,15,9,18,19,17,12,4,1,3,2,7,
                33,34,35,36,37,38,27,26,25,24,23,21,22,28,29,30,32,31};
    prepare(true,sizeof(actuators)/sizeof(int),actuators,0);
}

Mirror_PDM18LO_50::Mirror_PDM18LO_50(DAC *dac):Mirror(dac)
{
    int actuators[]={19,13,17,16,8,4,7,2,14,20,11,15,18,6,5,9,3,12};
    prepare(true,sizeof(actuators)/sizeof(int),actuators,0);

    setFlatConfiguration(PDM18LO_FLAT, 18);
    setFeedForwardMatrix(PDM18LO_FFMAT,18,10);
}

Mirror_MMDM19LIN::Mirror_MMDM19LIN(DAC *dac):Mirror(dac)
{
    int actuators[]={1,3,2,5,4,7,6,9,8,10,11,12,13,14,15,16,17,18,19};
    prepare(false,sizeof(actuators)/sizeof(int),actuators,0);
}

Mirror_MMDM17TT::Mirror_MMDM17TT(DAC *dac):Mirror(dac)
{
    int actuators[]={1,12,5,13,19,10,6,4,8,3,7,9,11,15,17,16,14,2,18};
    prepare(false,sizeof(actuators)/sizeof(int),actuators,0);

    setFlatConfiguration(MMDM17TT_FLAT, 19);
    setFeedForwardMatrix(MMDM17TT_FFMAT,19,14);
}

Mirror_MMDM39_30::Mirror_MMDM39_30(DAC *dac):Mirror(dac)
{
    int actuators[]={33, 3, 2, 6,27,26,31,37, 7,11,13,20,25,30,34,38, 5, 8,12,17,
                     21,23,29,35,1,4,9,10,15,14,16,19,18,22,24,28,32,36,39};
    prepare(false,sizeof(actuators)/sizeof(int),actuators,0);

    setFlatConfiguration(MMDM79_30_FLAT, 79);
    setFeedForwardMatrix(MMDM79_30_FFMAT,79,21);
}

Mirror_MMDM79_30::Mirror_MMDM79_30(DAC *dac0, DAC *dac1):Mirror(dac0,dac1)
{
    int actuators[]={38,17,39,5,24,18,14,10,31,33,36,3,6,9,29,25,20,19,13,9,
                    6,26,28,35,1,4,11,13,32,31,26,23,12,8,4,5,25,27,30,37,
                    2,8,12,14,37,34,30,27,21,16,11,7,2,3,21,22,24,29,32,0,
                    7,10,17,16,19,39,36,35,28,22,1,20,23,34,15,18,38,33,15};

    int units[]={1,0,1,1,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
                 0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,
                 1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,
                 1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,0,0,0};

    prepare(false,sizeof(actuators)/sizeof(int),actuators,units);
}

Mirror_MMDM79_40::Mirror_MMDM79_40(DAC *dac0, DAC *dac1):Mirror(dac0,dac1)
{
    int actuators[]={0,36,21,6,11,27,33,28,1,3,7,16,22,30,14,15,6,38,34,29,
                    25,2,8,13,17,21,29,32,37,18,10,4,35,30,26,20,5,12,15,19,
                    24,31,35,36,16,13,9,5,3,37,32,27,23,22,4,10,14,18,20,25,
                    26,33,34,39,19,12,11,7,2,1,39,31,24,9,23,28,38,17,8};

    int units[]={1,0,0,1,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
                0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,
                1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,
                1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0};

    prepare(false,sizeof(actuators)/sizeof(int),actuators,units);
}

Mirror_MMDM79_50::Mirror_MMDM79_50(DAC *dac0, DAC *dac1):Mirror(dac0,dac1)
{
    int actuators[]={7,9,24,32,31,2,3,10,15,26,33,36,19,15,23,28,37,5,36,17,
                    21,28,34,3,5,10,13,17,22,26,33,38,6,13,16,23,24,31,35,39,
                    1,9,7,12,16,20,27,34,39,4,8,14,18,20,25,27,29,30,37,0,
                    2,4,6,11,14,21,25,30,35,11,12,19,22,32,38,8,18,29,1};

    int units[]={0,0,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,0,1,0,
               0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,
               1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,
               1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,0};

    prepare(false,sizeof(actuators)/sizeof(int),actuators,units);
}

Mirror_PDM20LIN::Mirror_PDM20LIN(DAC *dac):Mirror(dac)
{
    int actuators[]={0,3,5,7, 9,11,13,15,17,19,1,2,4,6, 8,10,12,14,16,18};
//                   6,4,2,0,14,12,10, 8,22,20,7,5,3,1,15,13,11, 9,23,21};

    prepare(true,sizeof(actuators)/sizeof(int),actuators,0);
}

Mirror_PDM19LO_30::Mirror_PDM19LO_30(DAC *dac):Mirror(dac)
{
    int actuators[]={8,19,13,17,14,6,2,7,1,12,18,11,15,16,4,5,9,3,10};

    prepare(true,sizeof(actuators)/sizeof(int),actuators,0);

    setFlatConfiguration(PDM19LO_30_FLAT, 19);
    setFeedForwardMatrix(PDM19LO_30_FFMAT,19,14);
}

Mirror_PDM18LO_30::Mirror_PDM18LO_30(DAC *dac):Mirror(dac)
{
    int actuators[]={19,13,17,14,6,2,7,1,12,18,11,15,16,4,5,9,3,10};
    prepare(true,sizeof(actuators)/sizeof(int),actuators,0);
}


Mirror_PDM37_2005::Mirror_PDM37_2005(DAC *dac):Mirror(dac)
{
    int actuators[]={10,4,12,36,29,23,3,6,14,19,17,34,38,31,25,22,7,5,1,8,
                    16,18,15,13,11,30,32,37,33,27,21,24,26,28,9,35,2};

    prepare(true,sizeof(actuators)/sizeof(int),actuators,0);

    setFlatConfiguration(PDM37_2005_FLAT, 37);
    setFeedForwardMatrix(PDM37_2005_FFMAT,37,21);

}

Mirror_PDM37TRIHEX::Mirror_PDM37TRIHEX(DAC *dac):Mirror(dac)
{
    int actuators[]={14,16,38,23,26,1,8,10,13,37,18,36,25,28,22,7,5,2,6,
                                    19,17,32,15,11,35,34,39,33,31,21,27,29,24,9,3,4,12};

    prepare(true,sizeof(actuators)/sizeof(int),actuators,0);

    setFlatConfiguration(PDM37TRIHEX_FLAT, 37);
    setFeedForwardMatrix(PDM37TRIHEX_FFMAT,37, 20);
}

Mirror_PDM37RECT::Mirror_PDM37RECT(DAC *dac):Mirror(dac)
{
    int actuators[]={25,39,35,23,22,24,38,33,21,27,29,31,34,36,37,26,5,9,
                     1,15,13,32,28,7,2,19,14,17,30,3,8,6,16,18,4,10,12};
    prepare(true,sizeof(actuators)/sizeof(int),actuators,0);
}

Mirror_PDM37_50::Mirror_PDM37_50(DAC *dac):Mirror(dac)
{
    int actuators[]={1,4,7,6,3,2,5,17,16,19,18,9,8,11,10,13,12,15,14,
                    35,34,37,36,39,21,23,22,25,24,27,26,29,28,31,30,33,32};

    prepare(true,sizeof(actuators)/sizeof(int),actuators,0);
}

Mirror_PDM37_50_2008::Mirror_PDM37_50_2008(DAC *dac):Mirror(dac)
{
    int actuators[]={11,7,13,14,12,6,4,9,16,17,18,19,15,10,8,3,1,5,2,
                    28,30,33,31,35,36,37,38,34,32,21,23,27,25,22,29,24,26};

    prepare(true,sizeof(actuators)/sizeof(int),actuators,0);
}

Mirror_PDM79_50::Mirror_PDM79_50(DAC *dac0, DAC *dac1):Mirror(dac0, dac1)
{
    int actuators[]={19,12,24,23,27,15,17,10,6,28,31,20,25,29,31,9,10,17,19,13,
                    11,5,30,32,33,25,23,21,26,32,33,34,7,6,12,14,16,14,8,9,
                    2,1,38,37,34,29,26,22,20,24,30,35,38,36,0,1,5,8,11,16,
                    15,18,7,4,3,39,35,36,27,21,22,28,37,39,2,3,4,13,18};

    int units[]={1,0,0,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,0,0,
               0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,
               0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,
               0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1};

    prepare(true,sizeof(actuators)/sizeof(int),actuators,units);
}

Mirror_PDM109_50::Mirror_PDM109_50(DAC *dac0, DAC *dac1, DAC *dac2):Mirror(dac0,dac1,dac2)
{
    int actuators[]={8,15,34,26,24,7,10,16,36,1,5,32,31,22,1,39,38,6,9,18,
                    38,31,35,8,4,2,33,30,21,2,4,8,32,30,36,2,11,19,39,28,
                    24,22,13,15,10,6,39,35,28,25,3,9,13,11,28,27,26,33,37,4,
                    13,17,33,37,26,21,27,25,12,17,14,11,3,37,36,29,27,5,7,12,
                    15,14,16,25,21,23,31,35,3,5,14,34,32,30,23,18,16,9,7,38,
                    23,6,10,17,19,22,29,34,1};

    int units[]={2,2,0,0,0,2,2,2,1,1,1,0,0,0,0,2,2,2,2,2,
                1,1,1,1,1,1,0,0,0,0,0,0,2,2,2,2,2,2,1,1,
                1,1,1,1,1,1,0,0,0,0,0,0,0,0,2,2,2,2,2,2,
                2,2,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,
                0,0,0,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,0,
                0,0,0,0,0,2,2,2,2};

    prepare(true,sizeof(actuators)/sizeof(int),actuators,units);
}

Mirror_PDM39_50_LO::Mirror_PDM39_50_LO(DAC *dac):Mirror(dac)
{
   int actuators[]={20,13,17,14,15,33,34,32,35,30,28,24,25,27,26,5,6,8,9,10,12,16,19,18,39,38,37,36,31,29,22,23,21,1,2,3,4,7,11};
   prepare(true,sizeof(actuators)/sizeof(int),actuators,0);
}

Mirror_MMDM96_EMB::Mirror_MMDM96_EMB(DAC *dac): Mirror(dac)
{
   /*
     physical ch->actuator
   int actuators[]={25,11,45,68,26,88,27,46,69,12,70,89,71,47,13,4,28,90,29,73,72,48,91,49,74,30,92,75,50,14,51,76,31,15,93,32,55,95,
                   0,5,54,34,79,53,16,33,94,78,44,67,3,10,87,24,43,42,66,86,65,23,64,85,63,22,41,9,40,62,2,84,21,8,61,39,83,60,20,38,
                   59,19,1,82,7,37,81,58,36,18,57,6,56,80,17,35,77,52};
   */
   /*
     actuator no->physical channel
   */
   int actuators[]={38,80,68,50,15,39,89,82,71,65,51,1,9,14,29,33,44,92,87,79,76,70,63,59,53,0,4,6,16,18,25,32,35,45,41,93,86,83,77,
                    73,66,64,55,54,48,2,7,13,21,23,28,30,95,43,40,36,90,88,85,78,75,72,67,62,60,58,56,49,3,8,10,12,20,19,24,27,31,94,
                    47,42,91,84,81,74,69,61,57,52,5,11,17,22,26,34,46,37};

   prepare(false,sizeof(actuators)/sizeof(int),actuators,0);
   //setFlatConfiguration(MDM96_FLAT,96);
   //setFeedForwardMatrix(MDM96_FFMAT,96,21);
}

TipTilt::TipTilt(DAC *dac):Mirror(dac)
{
    int actuators[]={0,1,2,3};
    prepare(true,sizeof(actuators)/sizeof(int),actuators,0);
}


void TipTilt::setTip(double value)
{
    setOneChannel(0,value);
    setOneChannel(2,-value);
}

void TipTilt::setTilt(double value)
{
    setOneChannel(1,value);
    setOneChannel(3,-value);
}








