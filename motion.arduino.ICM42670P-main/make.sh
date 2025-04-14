clear
reset
# 
export FmDev=$(pwd)
# 

rm -rf build
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=/home/kevin/Dev/tools/vcpkg/scripts/buildsystems/vcpkg.cmake -DCMAKE_BUILD_TYPE=Debug -Wnarrowing
cmake --build build
# 
