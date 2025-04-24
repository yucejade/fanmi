#! /usr/bin/bash -f

clear
reset
# 
export FmDev=$(pwd)
export FmInstallPrefix="${FmDev}/build/package"
# 

rm build -fr

cd ${FmDev}/thirdparty/libgpiod-1.6.3
${FmDev}/thirdparty/libgpiod-1.6.3/autogen.sh --enable-tools=yes
${FmDev}/thirdparty/libgpiod-1.6.3/configure  --prefix=${FmInstallPrefix}
make -f ${FmDev}/thirdparty/libgpiod-1.6.3/Makefile
# sudo make -f ${FmDev}/thirdparty/libgpiod/Makefile install
cd -

#cmake -B build -S . -DCMAKE_BUILD_TYPE=debug ..
#cd build
#make install

cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=/home/kevin/Dev/tools/vcpkg/scripts/buildsystems/vcpkg.cmake -DCMAKE_BUILD_TYPE=Debug -Wnarrowing
cmake --build build
cmake --build install
 
