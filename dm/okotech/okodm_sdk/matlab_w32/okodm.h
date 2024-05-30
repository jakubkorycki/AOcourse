#ifndef OKODM_H
#define OKODM_H

#ifdef BUILD_DLL
    #define LIBRARY_API __declspec(dllexport)
	// __cdecl removed 
#else
    #define LIBRARY_API __declspec(dllimport)
////  suitable for both dll import header and the case of static linking
//    #define DLL_EXPORT
#endif

#ifdef __cplusplus
// to avoid C++ name decoration for exported functions
extern "C"
{
#endif

LIBRARY_API int okodm_open(const char* mirror_type, const char* dac_type, const char** dac_ids);
LIBRARY_API void okodm_close(int handle);
LIBRARY_API int okodm_set(int handle, double *values, int size);
LIBRARY_API int okodm_chan_n(int handle);
LIBRARY_API const char *okodm_lasterror();

#ifdef __cplusplus
} 
#endif // extern "C" 

#endif // OKODM_H
