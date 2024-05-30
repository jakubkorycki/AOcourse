#include <cstdio>
#include <stdexcept>
#include "okodac.h"
int main()
{
	try
	{
		DAC *dac=new DAC_DALSA(); // built-in HV DAC with first available ID (can be specified explicitly)
		Mirror *mirror=new Mirror_MMDM96_EMB(dac); // attach to the mirror
	    int n=mirror->numberOfActuators(); // number of channels
		double *data=(double *)malloc(n*sizeof(double));  // array for storing contral values
		for(int i=0; i<n; i++) data[i]=0.5; // some fixed value to all channels
		mirror->setChannels(data,n);
		mirror->updateAllChannels(); // only necessary if synchronious update mode is switched off
	}
    catch(exception &e)
	{
		printf("Operation failed: %s",e.what());
	}    
	return 0;
}