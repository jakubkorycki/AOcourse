#include <cstdio>
#include <stdexcept>
#include "okodac.h"
#include "util.h"
#include "zernike.h"

int main(int argc, char *argv[])
{
	int n,m, idx;
	double amplitude;
    double *ffmat, *flat;
    long count=0; 	
	
	if(argc<3)
	{
		printf("Usage: %s <n> <m> <amplitude>\n",argv[0]);
		exit(1);
	}
	 
	n=atoi(argv[1]);
	m=atoi(argv[2]);
	amplitude=atof(argv[3]);
	
	try
	{
		count=0;
		ffmat=array_read("d96v1c02_ffmat.dat", &count); // Read Zernike calibration matrix from the file
		count=0;
		flat=array_read("d96v1c02_flat.dat", &count); // Read flat configuration
         
		
		DAC *dac=new DAC_DALSA(); // built-in HV DAC with first available ID (can be specified explicitly)
		Mirror *mirror=new Mirror_MMDM96_EMB(dac); // attach to the mirror
	    
		mirror->setFeedForwardMatrix(ffmat,96,21);
		mirror->setFlatConfiguration(flat,96);
		
	    idx=zernike_nm_to_malacara_index(n,m)-2; // Malacare indexing starts with 1 for piston, we do not have it	
		printf("setting Z[%d,%d]=%lf\n",n,m,amplitude);
		mirror->setAllZernikeModes(0);
		mirror->setOneZernikeMode(idx,amplitude);
		mirror->updateAllChannels(); // only necessary if synchronious update mode is switched off
	}
    catch(exception &e)
	{
		printf("Operation failed: %s. Aborting.",e.what());
	}    
	return 0;
}