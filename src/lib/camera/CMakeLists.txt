message(DEBUG "./src/lib/camera/cmake")

# Define the library
add_library(cameraLibrary STATIC
    based_camera.cpp
)

find_package(k4a CONFIG REQUIRED)
target_link_libraries(cameraLibrary PRIVATE k4a::k4a)

# Include directories for headers
# target_include_directories(cameraLibrary PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# # Optionally, add additional configurations, dependencies, or options
target_compile_options(cameraLibrary PRIVATE -fPIC)
