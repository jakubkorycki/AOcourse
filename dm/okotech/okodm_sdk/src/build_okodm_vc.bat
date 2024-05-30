cl /LD /EHa /DBUILD_DLL /I. okodm.cpp okodac.cpp edac40.c util.cpp mirror_ffmat.cpp ws2_32.lib ftd2xx.lib 
cl /I. okodm_test.c okodm.lib
cl /EHsc /I. znm.cpp okodac.cpp edac40.c util.cpp mirror_ffmat.cpp zernike.cpp ws2_32.lib ftd2xx.lib
