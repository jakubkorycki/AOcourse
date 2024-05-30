#ifndef DAC_H
#define DAC_H

#include "edac40.h"
#include "ftd2xx.h"
#include <vector>
#include <stdexcept>
//#include <string>

// Model descriptions below used to destinguish between several USB unit modules
// which share the same driver/support library and appeat quite similar
#define USB40DAC_MAGIC "DAC-40, v2.0"
#define USB4DAC_MAGIC "DAC-4, v1.0"
#define USB40FASTDAC_MAGIC "DAC-40, v.3"
#define USBDALSA_MAGIC "DAC-HV-96, v1.0"

// Serial numbers format examples:
// D40V2e12
// D96V1A01

using namespace std;

// An abstract class serves as a base class for hardware DAC units
class DAC
{
public:
    DAC(const char *id=0);
    virtual ~DAC();
    virtual void setOneChannel(unsigned channel, unsigned value);
    virtual int getChannel(unsigned channel);
    virtual void setAllChannels(const vector<unsigned> values);
    virtual void setAllChannels(const unsigned *values, unsigned channel_number);
    bool isImmediateUpdate() { return dac_immediate_update; }
    void setImmediateUpdate(bool immediate_update) { dac_immediate_update=immediate_update; }
    virtual void updateAllChannels()=0;
    virtual unsigned numberOfChannels() { return 0; }
    virtual int bitResolution() { return 1; }
    virtual uint32_t maxValue() { return 1; }
    char *getID() { return dac_id; }
protected:
    char dac_id[256];
    bool dac_immediate_update;
    unsigned *dac_values;
    virtual void initInternals();
};


class EtherDAC: public DAC
{
public:
    EtherDAC(const char *MAC_address=0);
    virtual ~EtherDAC();
    static vector<char *> *listDevices();
    virtual void updateAllChannels();
    virtual unsigned numberOfChannels() { return 40; }
    virtual int bitResolution() { return 16; }
    virtual uint32_t maxValue() { return 65535; }
    SOCKET getSocket() { return edac_socket; }
 private:
    char *edac_host_name;
    SOCKET edac_socket;
    static int edac_instance_count;
};


// This is an abstract class, implements (in its constructor) just "detection" and destructor
// functionality common for USB40DAC and USB4DAC
class USBDAC: public DAC
{
public:
    static vector<char *> *listDevices(const char *model=0);
protected:
    USBDAC(const char *model=0, const char *serial_number=0);
    virtual ~USBDAC();
    FT_HANDLE usbdac_handle;
};


class DAC40USB: public USBDAC
{
public:
    DAC40USB(const char *serial_number=0);
    static vector<char *> *listDevices(const char *model=0) { return USBDAC::listDevices(USB40DAC_MAGIC); }
    void updateAllChannels();
    virtual unsigned numberOfChannels() { return 40; }
    virtual int bitResolution() { return 12; }
    virtual uint32_t maxValue() { return 4095; }
private:
    BYTE usb40dac_packet[130];
};

class DAC4USB: public USBDAC
{
public:
    DAC4USB(const char *serial_number=0);
    static vector<char *> *listDevices(const char *model=0) { return USBDAC::listDevices(USB4DAC_MAGIC); }
    void updateAllChannels();
    virtual unsigned numberOfChannels() { return 4; }
    virtual int bitResolution() { return 16; }
    virtual uint32_t maxValue() { return 65535; }
private:
    BYTE usb4dac_packet[16];
};

class DAC40USB_FAST: public USBDAC
{
public:
    DAC40USB_FAST(const char *model=0, const char *serial_number=0);
    static vector<char *> *listDevices(const char *model=0) { return USBDAC::listDevices(USB40FASTDAC_MAGIC); }
    void updateAllChannels();
    virtual unsigned numberOfChannels() { return 40; }
    virtual int bitResolution() { return 16; }
    virtual uint32_t maxValue() { return 65535; }
private:
    uint8_t buffer[2048]; // Buffer to hold data read from the FT2232H
    DWORD dwNumBytesSent; // Count of actual bytes sent - used with FT_Write
    DWORD bytesToWrite;
    FT_HANDLE ftHandle;
    FT_STATUS ftStatus;
    uint8_t gpio;
    void dac_set_sync(uint8_t *buffer, DWORD *index, uint8_t value);
    void dac_command(uint8_t *buffer, DWORD *index, uint8_t mode, uint8_t address, uint16_t data);
    void dac_set_value(uint8_t *buffer, DWORD *index, uint8_t channel, uint16_t data);
    void dac_set_offset_dacs(uint8_t *buffer, DWORD *index, uint16_t data);
    void dac_cycle_ldac(uint8_t *buffer, DWORD *index);
    void dac_set_gain(uint8_t *buffer, DWORD *index, uint8_t channel, uint16_t data);
    void dac_set_offset(uint8_t *buffer, DWORD *index, uint8_t channel, uint16_t data);
};

class DAC_DALSA: public USBDAC
{
public:
    DAC_DALSA(const char *serial_number=0);
    static vector<char *> *listDevices(const char *model=0) { return USBDAC::listDevices(USBDALSA_MAGIC); }
    void updateAllChannels();
    virtual unsigned numberOfChannels() { return 96; }
    virtual int bitResolution() { return 16; }
    virtual uint32_t maxValue() { return 65535; }
private:
    //BYTE usb96dac_packet[2048];
    DWORD bytesToWrite;
    uint8_t buffer[16000];
};

// Mirror is NOT an abstract class, it can be instantiated for non-standard mirror configuration.
// But it is designed to be inherited by classes for several standard OKO mirrors
// with actuators mapping build-in.
class Mirror
{
public:
    Mirror(DAC *dac0=0, DAC *dac1=0, DAC *dac2=0);
    ~Mirror();
    void attachDAC(DAC *dac);
	void releaseDACs();
    void setActuatorsMap(int channels_n, const int *actuator_map, const int *unit_map=0);
    void setChannels(const double *values, int channels_n);
    void getChannels(double *values);
    void setOneChannel(int channel, double value);
    void setAllChannels(double value);
    void updateAllChannels();
    int numberOfActuators() { return mirror_channels_n; }
    int numberOfUnits();
    // setters-getters
    void setQuadraticResponse(bool quadratic_response) { mirror_quadratic_response=quadratic_response; }
    bool isQuadraticResponse() { return mirror_quadratic_response; }
    void setCancelBias(bool cancel_bias) { mirror_cancel_bias=cancel_bias; }
    bool isCancelBias() { return mirror_cancel_bias; }
    // hysteresis compensation is not implemented yet
    void setCompensateHysteresis(bool compensate) { mirror_compensate_hysteresis=compensate; }
    bool isCompensateHysteresis() { return mirror_compensate_hysteresis; }
    bool isImmediateUpdate() { return mirror_immediate_update; }
    void setImmediateUpdate(bool immediate_update);
    // connector pin diagram (just for fun?)
    char *pinout();
    void degauss(double delay_sec=0.001, bool serial=false, int steps=100);
    //-----------
    // feedforward related
    int numberOfZernikeModes() { return mirror_zernike_modes_n; }
    void setFlatConfiguration(const double *values, int channels_n);
    void setFeedForwardMatrix(const double *matrix, int channels_n, int modes_n);
    void setOneZernikeMode(int idx, double value);
    void setZernikeModes(double *values, int modes_n);
    void setAllZernikeModes(double value);
    void calculateVoltagesByModes();
    // void calculeteModesByVoltages(); // reverse, to be implemented

protected:
    unsigned mirror_channels_n;
    bool mirror_quadratic_response;
    bool mirror_cancel_bias;
    bool mirror_compensate_hysteresis; // not implemented yet
    bool mirror_immediate_update;
    vector<DAC*> dacs;       // dac unit list
    unsigned *mirror_actuator_map, *mirror_unit_map;
    double *mirror_voltages;
    //-------------------
    // feedforward related
    int mirror_zernike_modes_n;
    double *mirror_flat_configuration;
    double *mirror_feedforward_matrix;
    double *mirror_zernike_modes;
    //-------------------
    void prepare(bool is_pdm, int channels_n, const int *actuator_map, const int *unit_map=0);
};

// ----------- Utility classes for standard OKO mirrors ----
// basically just constructors with hardcoded actuator mapping and parameters

class Mirror_MMDM37: public Mirror { public: Mirror_MMDM37(DAC *dac=0);};
class Mirror_PDM19: public Mirror { public: Mirror_PDM19(DAC *dac=0);};
class Mirror_PDM37: public Mirror { public: Mirror_PDM37(DAC *dac=0); };
class Mirror_PDM18LO_50: public Mirror { public: Mirror_PDM18LO_50(DAC *dac=0); };
class Mirror_MMDM19LIN: public Mirror { public: Mirror_MMDM19LIN(DAC *dac=0); };
class Mirror_MMDM17TT: public Mirror { public: Mirror_MMDM17TT(DAC *dac=0); };
class Mirror_MMDM39_30: public Mirror { public: Mirror_MMDM39_30(DAC *dac=0); };
class Mirror_MMDM79_30: public Mirror { public: Mirror_MMDM79_30(DAC *dac0=0, DAC *dac1=0); };
class Mirror_MMDM79_40: public Mirror { public: Mirror_MMDM79_40(DAC *dac0=0, DAC *dac1=0); };
class Mirror_MMDM79_50: public Mirror { public: Mirror_MMDM79_50(DAC *dac0=0, DAC *dac1=0); };
class Mirror_PDM20LIN: public Mirror { public: Mirror_PDM20LIN(DAC *dac=0); };
class Mirror_PDM19LO_30: public Mirror { public: Mirror_PDM19LO_30(DAC *dac=0); };
class Mirror_PDM18LO_30: public Mirror { public: Mirror_PDM18LO_30(DAC *dac=0); };
class Mirror_PDM37_2005: public Mirror { public: Mirror_PDM37_2005(DAC *dac=0);};
class Mirror_PDM37TRIHEX: public Mirror { public: Mirror_PDM37TRIHEX(DAC *dac=0);};
class Mirror_PDM37_50: public Mirror { public: Mirror_PDM37_50(DAC *dac=0); };
class Mirror_PDM37RECT: public Mirror { public: Mirror_PDM37RECT(DAC *dac=0); };
class Mirror_PDM37_50_2008: public Mirror { public: Mirror_PDM37_50_2008(DAC *dac=0); };
class Mirror_PDM39_50_LO: public Mirror { public: Mirror_PDM39_50_LO(DAC *dac=0); };
class Mirror_PDM79_50: public Mirror { public: Mirror_PDM79_50(DAC *dac0=0, DAC *dac1=0); };
class Mirror_PDM109_50: public Mirror { public: Mirror_PDM109_50(DAC *dac0=0, DAC *dac1=0, DAC *dac2=0); };

class Mirror_MMDM96_EMB: public Mirror { public: Mirror_MMDM96_EMB(DAC *dac=0); };

// The list of standard device types and the following function are intended primary for use by GUI applications
// (e.g. fill a drop-out box with human-readable names, then create a right object according the
// users choice). It is a kind of poor guy "object factory" design pattern.

extern const char* OKOMirrors[]; // The last element is "guarding" NULL
Mirror *newMirrorByName(const char *miror_type, DAC *dac=0);

// same for DAC types, we have too many models now...
extern const char* OKODACs[];
DAC *newDACByName(const char *dac_type, const char *ID=0);

class TipTilt: public Mirror
{
public:
    TipTilt(DAC *dac=0);
    void setTip(double value);
    void setTilt(double value);
};

// exceptions to be thrown by dac/mirror classes
class mirror_fault:public runtime_error
{
public:
    explicit mirror_fault(const string &s):runtime_error(s){}
};


#endif // DAC_H
