message(DEBUG "./src/lib/camera/cmake")

# Define the library
add_library(__geometryLibrary STATIC
    point_cloud.cpp
    grid.cpp
)


find_package(glm CONFIG REQUIRED)
target_link_libraries(__geometryLibrary PRIVATE glm::glm-header-only)

# Include directories for headers
# target_include_directories(cameraLibrary PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# # Optionally, add additional configurations, dependencies, or options
if(UNIX)
    target_compile_options(__geometryLibrary PRIVATE -fPIC)
endif()
