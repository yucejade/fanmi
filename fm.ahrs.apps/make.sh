clear
reset
# 
export FmDev=$(pwd)
export vcpkg=/home/kevin/Dev/tools/vcpkg
# 
rm -rf build
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=/home/kevin/Dev/tools/vcpkg/scripts/buildsystems/vcpkg.cmake -DCMAKE_BUILD_TYPE=Debug -Wnarrowing
cmake --build build -j10