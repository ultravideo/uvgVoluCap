message(DEBUG "./src/lib/camera/cmake")

# Define the library
add_library(cameraLibrary STATIC
    kinect_camera.cpp
    kinect_utilities.cpp
)

find_package(nlohmann_json 3.11.3 REQUIRED)
find_package(k4a CONFIG REQUIRED)
target_link_libraries(cameraLibrary PRIVATE k4a::k4a nlohmann_json::nlohmann_json)

# Include directories for headers
# target_include_directories(cameraLibrary PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# # Optionally, add additional configurations, dependencies, or options
target_compile_options(cameraLibrary PRIVATE -fPIC)
