message(DEBUG "./src/lib/cmake")

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)

configure_file(version.cpp.in version.cpp)

add_subdirectory(utils)
add_subdirectory(camera)
add_subdirectory(geometry)

set(LIB_SOURCES
    version.cpp
    uvgvolucap.cpp
)

if(BUILD_SHARED_LIBS)
  add_library(uvgvolucap SHARED ${LIB_SOURCES})
else()
  add_library(uvgvolucap STATIC ${LIB_SOURCES})
endif()

find_package(nlohmann_json 3.11.3 REQUIRED)
find_package(cppzmq CONFIG REQUIRED)

target_link_libraries(uvgvolucap
    utilsLibrary
    logLibrary
    cameraLibrary
    geometryLibrary
    nlohmann_json::nlohmann_json
    cppzmq
    cppzmq-static
)

# Include directories for headers
target_include_directories(uvgvolucap PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Optionally, add additional configurations, dependencies, or options
# target_compile_options(BitstreamGenerationLibrary PRIVATE -std=c++11)

# Define installation destinations
install(TARGETS uvgvolucap DESTINATION lib)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include DESTINATION include)