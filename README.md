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
