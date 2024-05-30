cl /LD /EHa /DBUILD_DLL /I. okodm.cpp okodac.cpp edac40.c util.cpp mirror_ffmat.cpp ws2_32.lib ftd2xx64.lib /Feokodm64.dll
cl /I. okodm_test.c okodm64.lib

