[![HitCount](http://hits.dwyl.io/zt-luo/ardusub_api.svg)](http://hits.dwyl.io/zt-luo/ardusub_api)


## build requirement 

### Windows  

1. 需要安装 [Scoop](https://scoop.sh/)
2. Scoop 安装使用参见 [链接](https://luozongtong123.github.io/tags/scoop/)
3. 安装完成后添加我的 bucket: `scoop bucket add bucket-luo https://github.com/luozongtong123/bucket-luo.git`
4. 添加完 bucket 后即可安装 glib2: `scoop install bucket-luo/glib2`
5. [可选] 项目使用 [Visual Studio Code](https://code.visualstudio.com/) 作为编辑器，项目中已包含所需的启动调试和代码高亮的设置文件
6. 项目使用了 [CMake](https://cmake.org/) 可直接使用 Scoop 安装 `scoop install cmake`
7. 安装编译器 `scoop install gcc`

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

