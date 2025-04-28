cmake_minimum_required(VERSION 3.15)

set(app_name "ahrs")

# Set project name
project(${app_name})

# 设置所有可执行文件的运行时输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $ENV{FmDev}/bin)

#
include_directories($ENV{FmDev}/libs/urho3d/include)
include_directories($ENV{FmDev}/libs/urho3d/include/Urho3D/ThirdParty)
include_directories($ENV{FmDev}/source)
include_directories($ENV{FmDev}/source/device)
include_directories($ENV{FmDev}/source/device/TDK40607P)
include_directories($ENV{FmDev}/libs/libgpiod/include)

#
# add_definitions(-DENABLE_LOGGING)
find_package(OpenGL REQUIRED)
find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")
find_package(nlohmann_json CONFIG REQUIRED)
find_package(unofficial-concurrentqueue CONFIG REQUIRED)
# 
# 查找源文件
file(GLOB app_src
    $ENV{FmDev}/source/*.cpp
    $ENV{FmDev}/source/*.inl
    $ENV{FmDev}/source/*/*.cpp
    $ENV{FmDev}/source/*/*.inl
    $ENV{FmDev}/source/*/*.c
    $ENV{FmDev}/source/*/*/*.cpp
    $ENV{FmDev}/source/*/*/*.inl
    $ENV{FmDev}/source/*/*/*.c
    $ENV{FmDev}/source/*/*/*/*.cpp
    $ENV{FmDev}/source/*/*/*/*.inl
    $ENV{FmDev}/source/*/*/*/*.c

)

# 添加可执行文件
add_executable(${app_name}
    ${app_src}
)

# 链接库到可执行文件
target_link_libraries(${app_name} PUBLIC

    #
    OpenGL::GL
    unofficial::concurrentqueue::concurrentqueue
    nlohmann_json::nlohmann_json
    #
    $ENV{FmDev}/libs/urho3d/libs/libAngelScript.a
    $ENV{FmDev}/libs/urho3d/libs/libBox2D.a
    $ENV{FmDev}/libs/urho3d/libs/libBullet.a
    $ENV{FmDev}/libs/urho3d/libs/libCivetweb.a
    $ENV{FmDev}/libs/urho3d/libs/libDetour.a
    $ENV{FmDev}/libs/urho3d/libs/libDetourCrowd.a
    $ENV{FmDev}/libs/urho3d/libs/libDetourTileCache.a
    $ENV{FmDev}/libs/urho3d/libs/libETCPACK.a
    $ENV{FmDev}/libs/urho3d/libs/libFreeType.a
    $ENV{FmDev}/libs/urho3d/libs/libik.a
    $ENV{FmDev}/libs/urho3d/libs/libLZ4.a
    $ENV{FmDev}/libs/urho3d/libs/libPugiXml.a
    $ENV{FmDev}/libs/urho3d/libs/libRecast.a
    $ENV{FmDev}/libs/urho3d/libs/libSDL.a
    $ENV{FmDev}/libs/urho3d/libs/libSLikeNet.a
    $ENV{FmDev}/libs/urho3d/libs/libStanHull.a
    $ENV{FmDev}/libs/urho3d/libs/libUrho3D.a
    $ENV{FmDev}/libs/urho3d/libs/libWebP.a
    # 
    $ENV{FmDev}/libs/libgpiod/lib/libgpiod.a
)