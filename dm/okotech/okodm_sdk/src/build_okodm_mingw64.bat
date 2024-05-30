g++ -m64 -DBUILD_DLL -shared -o okodm.dll okodm.cpp okodac.cpp edac40.c util.cpp mirror_ffmat.cpp -L. -lftd2xx64 -lws2_32

g++ -m64 -o okodm_test okodm_test.c okodm.dll util.cpp
