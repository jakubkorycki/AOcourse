g++ -DBUILD_DLL -shared -o okodm.dll okodm.cpp okodac.cpp edac40.c util.cpp mirror_ffmat.cpp -L. -lftd2xx -lws2_32
g++ -o test test.cpp okodac.cpp edac40.c util.cpp mirror_ffmat.cpp -L. -lftd2xx -lws2_32
g++ -o okodm_test okodm_test.c okodm.dll util.cpp
g++ -o znm znm.cpp okodac.cpp edac40.c util.cpp mirror_ffmyat.cpp zernike.cpp -L. -lftd2xx -lws2_32