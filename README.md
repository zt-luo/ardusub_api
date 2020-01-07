[![HitCount](http://hits.dwyl.io/zt-luo/ardusub_api.svg)](http://hits.dwyl.io/zt-luo/ardusub_api)


## build requirement 

### Windows  

not supported yet.

### Linux  

For Arch Linux:  
``` shell
sudo pacman -S glib2 libserialport sqlite json-glib
```

## Build  

``` shell
mkdir build
cd build
cmake .. -GNinja
ninja
```

## LOC

```
github.com/AlDanial/cloc v 1.82  T=0.03 s (1010.7 files/s, 203287.3 lines/s)
-------------------------------------------------------------------------------
Language                     files          blank        comment           code
-------------------------------------------------------------------------------
C                               12            840            856           3260
C/C++ Header                     9            159            181            476
JSON                             3              0              0            200
CMake                            4             53             23            107
Markdown                         2             20              0             59
Bourne Shell                     1              0              0              1
-------------------------------------------------------------------------------
SUM:                            31           1072           1060           4103
-------------------------------------------------------------------------------
```

123 commits  15,307(264,666 - 249,359) ++  9,032 --

