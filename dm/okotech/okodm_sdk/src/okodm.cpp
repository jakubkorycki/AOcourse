#include "okodac.h"
#include "okodm.h"
#include <cstdio>
#include <stdexcept>

#define MIRROR_MAX_N 16

//Mirror *mirror[MIRROR_MAX_N];

Mirror *mirror=0; 
char last_error[256];
int last_error_code=0;

LIBRARY_API int okodm_open(const char* mirror_type, const char* dac_type, const char** dac_ids)
{
  //DAC *dacs[3];
  //char **dac_ids=0;
  last_error_code=0;
  try
  {
    mirror=newMirrorByName(mirror_type);
    int n_dacs=mirror->numberOfUnits();
    for(int i=0; i<n_dacs; i++)
    {
      const char *id=0;
      if(dac_ids && dac_ids[i]) id=dac_ids[i];
      DAC *dac=newDACByName(dac_type,id);
      mirror->attachDAC(dac);
    }
  }
  catch(mirror_fault &e)
  {
      printf("Exception condition: %s\n", e.what());
      last_error_code=1;
	  strncpy(last_error,e.what(),256);
      return 0;
  }
  return 1;
}

LIBRARY_API void okodm_close(int handle)
{
    if(mirror)
    {
	   mirror->releaseDACs();	
       delete mirror;
       mirror=0;
    }
}

LIBRARY_API int okodm_set(int handle, double *values, int size)
{
    //printf("okodm_set: handle=%d values=");
    //for(int i=0; i<size; i++) printf("%lf ",values[i]);
    //printf("\n");
	last_error_code=0;
	if(!mirror)
	{
		last_error_code=1;
		strcpy(last_error,"Mirror device is not open");
		return 0;
	}
    try
    { 
        mirror->setChannels(values,size);
    }
    catch(mirror_fault &e)
    {
		last_error_code=1;
        strncpy(last_error,e.what(),256);
        return 0;
    }
    return 1;
}

LIBRARY_API int okodm_chan_n(int handle)
{
    if(!mirror)
	{
		last_error_code=1;
		strcpy(last_error,"Mirror device is not open");
		return 0;
	}
	return mirror->numberOfActuators();
}

LIBRARY_API const char *okodm_lasterror()
{
   if(!last_error_code) strcpy(last_error,"No error"); 	
   return last_error;
}
