#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "okodm.h"


void delay(double sec)
{
//#ifdef __GNUC__  // FIXME: use clock(), it is platform independent(?)
//	struct timeval t0,t1;
//    gettimeofday(&t0,NULL);
//    do gettimeofday(&t1,NULL); while(t1.tv_sec-t0.tv_sec+(t1.tv_usec-t0.tv_usec)/1e6<sec);
//#endif // need to find alternative aproach for MS VC
    clock_t t0=clock();
    while(((double)(clock()-t0))/CLOCKS_PER_SEC<sec);
}

int main()
{
  int mirror, chan_n, i,pass;
  double *value_up, *value_down;

  for(pass=0; pass<2; pass++)
  {	  
	printf("Pass %d\n",pass);
  
	if(!(mirror=okodm_open("MMDM 37ch,15mm","USB DAC 40ch, 12bit",0)))
	{
      printf("Error connecting to the mirror:%s\n",okodm_lasterror());
      return -1;
	}  
	chan_n=okodm_chan_n(mirror);
	value_up=(double*)malloc(chan_n*sizeof(double));
	value_down=(double*)malloc(chan_n*sizeof(double));
	for(i=0; i<chan_n; i++) value_up[i]=1;
	for(i=0; i<chan_n; i++) value_down[i]=-1;
	i=0;
	while(i<10)
	{ 	  
		if(!okodm_set(mirror,i%2?value_up:value_down,chan_n))
		{
			printf("Error setting value: %s\n",okodm_lasterror());
			return -1;
		}
		printf("."); fflush(stdout);
		delay(1);
		i++;
	} 	
	okodm_close(mirror);
	free(value_up);
	free(value_down);
	printf("\n");
  }	 
  return 0;
}
