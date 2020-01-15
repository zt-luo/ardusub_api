[![HitCount](http://hits.dwyl.io/zt-luo/ardusub_api.svg)](http://hits.dwyl.io/zt-luo/ardusub_api)


## build requirement 

only support gcc/g++.

### Windows (without serial port support)  

only support MinGW compilers of Msys2.  

``` shell
# x86_64
sudo pacman -S mingw-w64-x86_64-glib2 mingw-w64-x86_64-json-glib mingw-w64-x86_64-sqlite3 

# i686
sudo pacman -S mingw-w64-i686-glib2 mingw-w64-i686-json-glib mingw-w64-i686-sqlite3 
```

### Linux  

For Arch Linux:  
``` shell
sudo pacman -S glib2 json-glib sqlite libserialport
```

## Build  

``` shell
mkdir build
cd build

# use ninja
cmake .. -GNinja
ninja

# or use make
cmake ..
make
```
