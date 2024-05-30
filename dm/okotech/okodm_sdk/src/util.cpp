#ifndef UTIL_H
#define UTIL_H

#include <cstdlib>

//#ifdef __GNUC__
//    #include <sys/time.h>
//#endif
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cmath>
#include <stdexcept>
#include <ctime>
#include <cstring>

#include "util.h"

// nearest 2^n not exeeding given argument
int power2ceil(int x)
{
    return 1<<(int)ceil(log((double)x)/log(2.0));
    //return x;
}

#ifdef _MSC_VER
double round(double number)
{
    return ((number < 0.0) ? ceil(number - 0.5) : floor(number + 0.5));
}
#endif

void safe_free(void *obj)
{
    if(obj)
    {
        free(obj);
        obj=0;
    }
}

// Forms a delay specified in seconds (and fraction).
// Not very precise, on most system resolution of 1/100 sec should be expected
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


// trim whitespace characters in the begining and in the end of the given line
char* strtrim(char* s)
{
    while(isspace(*s)) s++;
    if(*s=='\0') return s;
    char *s_end=s+strlen(s)-1;
    while(s_end>s && isspace(*s_end)) *(s_end--)='\0';
    return s;
}

// Simple derivative of sprintf, which allocates some memory for a string buffer,
// if passed NULL pointer (that means that caller should take care of it later)
// and returns the pointer to the resulting string.
// No error checking so should be used with GREAT care.
char* ssprintf(char *buffer, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    if(!buffer) buffer=(char*)malloc(512);
    vsprintf(buffer,format,args); // we don't expect any error here
    va_end(args);
    return buffer;
}


// pseudo-random double in the range [from..to]
double frand(double from, double to) { return (from+((1.0*rand())/RAND_MAX)*(to-from)); }

int irand(int from, int to) { return round(from+(to-from)*(1.0*rand())/RAND_MAX); }

//  generates random deviates with normal (Gaussian) distribution using Box-Muller method
double nrand()
{ double v1,v2,r;
  do { v1=2.0*rand()/RAND_MAX-1.0;
       v2=2.0*rand()/RAND_MAX-1.0;
       r=v1*v1+v2*v2;
     } while (r>=1 || r<=1e-3);
  return v1*sqrt(-2*log(r)/r);
}


double* array_read(char *filename, long *count, double *buffer)
{
    FILE *f=fopen(filename, "rt");
    if(!f) throw std::runtime_error("Unable to open file for read");
    long idx=0;
    double tmp;
    long size_inc=100;
    long block_size=0;
    if(buffer && *count<1) throw std::runtime_error("Unspecified buffer size");
    if(!buffer)
    {
        buffer=(double*)malloc(size_inc*sizeof(double));
        if(!buffer) throw std::runtime_error("Memory allocation error");
        block_size=size_inc;
    }
    while(!feof(f) && (idx<*count || *count<1))
    {
        if(fscanf(f,"%lf",&tmp)!=1) break;
        //printf("%lf: %d %d\n",tmp, idx, block_size);
        buffer[idx++]=tmp;
        if(idx>=block_size)
        {
            block_size+=size_inc;
            buffer=(double*)realloc(buffer,block_size*sizeof(double));
            if(!buffer) throw std::runtime_error("Memory allocation error");
        }
    }
    *count=idx;
    buffer=(double *)realloc(buffer,idx*sizeof(double));
    fclose(f);
    return buffer;
}

void array_write(char *filename, long count, double *buffer)
{
    FILE *f=fopen(filename,"wt");
    if(!f) throw std::runtime_error("Unable to open file for write");
    for(int i=0; i<count; i++) //***FIXME! finish this simple code some day
        fprintf(f,"%lf\n", buffer[i]);
    fclose(f);
}

#endif //UTIL_H
