message(DEBUG "./src/lib/cmake")

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)

configure_file(version.cpp.in version.cpp)

add_subdirectory(utils)
add_subdirectory(camera)

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

target_link_libraries(uvgvolucap
    utilsLibrary
    logLibrary
    cameraLibrary
    nlohmann_json::nlohmann_json
)

# Include directories for headers
target_include_directories(uvgvolucap PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(uvgvolucap PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/camera)

# Optionally, add additional configurations, dependencies, or options
# target_compile_options(BitstreamGenerationLibrary PRIVATE -std=c++11)

# Define installation destinations
install(TARGETS uvgvolucap DESTINATION lib)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include DESTINATION include)