cmake_minimum_required(VERSION 3.22)
project(RadarResultViewer)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-O3 -Wall -Wextra)

find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Boost COMPONENTS date_time filesystem iostreams REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(mosquittopp REQUIRED libmosquittopp IMPORTED_TARGET)

file(COPY ground.png DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
configure_file(config.json ${CMAKE_CURRENT_BINARY_DIR}/config.json COPYONLY)

add_executable(RadarResultViewer)

target_sources(RadarResultViewer PRIVATE main.cpp)

# target_include_directories(PointCloudPredictor PRIVATE .)

target_link_libraries(RadarResultViewer PRIVATE
        nlohmann_json::nlohmann_json
        PkgConfig::mosquittopp
        Boost::boost
        Boost::date_time
        Boost::filesystem
        Boost::iostreams
        ${OpenCV_LIBS})

