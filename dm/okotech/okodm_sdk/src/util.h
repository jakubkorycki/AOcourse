#ifndef UTIL_H
#define UTIL_H

#include <cstdlib>
#include <cstring>
#include <cmath>

#ifdef _MSC_VER
    #include <msstdint.h>
	double round(double number);
#else
    #include <stdint.h>
#endif

// some little things are used over-and-over again...

#define SQR(x) ((x)*(x))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))

// NAN is known to not compare equal to itself ***FIXME: there is a function isnan in math.h, standard, implementation?
#define IS_NAN(x) ((x)!=(x))

//#define FRAND(from,to) ()

#define SAFE_DELETE(x) {if(x) delete (x); (x)=NULL;}


// This template function doesn't work correctly, WHY?
template <class T> void safe_delete(T *obj)
{
    //qDebug("safe_delete: %p", obj);
    if(obj!=NULL)
     {
        delete obj;
        obj=NULL;
     }
}

void safe_free(void *obj);

void delay(double sec);

char* strtrim(char *s);

char* ssprintf(char *buffer, const char *format, ...);

// pseudo-random double in the range [from..to]
double frand(double from, double to);

int irand(int from, int to);

//  generates random deviates with normal (Gaussian) distribution using Box-Muller method
double nrand();

// ReaD the Time Stamp Counter
inline uint64_t rdtsc(void)
{
   unsigned a, d;
   #ifdef __GNUC__
   __asm__ volatile("rdtsc" : "=a" (a), "=d" (d));
   #elif defined(_MSC_VER)
//   __asm
//   {
//       rdtsc
//       mov a, eax
//       mov d, edx
//   }
   #endif
   return ((uint64_t)a) | (((uint64_t)d) << 32);
}

double *array_read(char *filename, long *count, double *buffer=0);

void array_write(char *filename, long count, double *buffer);


#endif //UTIL_H
