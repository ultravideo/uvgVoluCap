

cmake_minimum_required(VERSION 3.27)

# Include CMake modules

set(uvgVoluCap_VER 0.1.0)
set(uvgVoluCap_DESCR "uvgVoluCap is an Real-Time V-PCC encoder library written in C++.")
set(uvgVoluCap_URL "https://github.com/ultravideo/uvgVoluCap")

option(BUILD_SHARED_LIBS "Build using shared uvgVoluCap library" ON)

project(uvgVoluCap VERSION ${uvgVoluCap_VER})

if(MSVC)
# add_compile_options(/W3 /WX)
add_compile_options(/W3)
else()
add_compile_options(-Wno-unused-but-set-variable -Wno-reorder)
# add_compile_options(-W -Wall -Werror -Wno-unused-variable)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

#Check if the platform is Windows
if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()

if($ENV{USE_CLANG_TIDY})
  set(CMAKE_CXX_CLANG_TIDY "clang-tidy;-checks=*,-llvmlibc-*,-llvm-*,-modernize-*,-fuchsia-*,-altera-*,-hicpp-use-auto,-bugprone-easily-swappable-parameters,-performance-avoid-endl,-cppcoreguidelines-pro-bounds-constant-array-index,-cppcoreguidelines-avoid-magic-numbers,-readability-magic-numbers,-readability-identifier-length")
endif()

# Usefull for debuging
if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer -fsanitize=undefined -fsanitize=leak")
endif()

# Run subdirectory make files
add_subdirectory(src/lib)
add_subdirectory(src/app)
# add_subdirectory(plyxporter)
# add_subdirectory(visualizer)
# add_subdirectory(examples)

# Custom target to apply formating to all source file
# todo : check if user has the package : sudo apt-get install clang-format
add_custom_target(mainformat
  find
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    -type f
    (
    -name "*.cc"
    -o
    -name "*.cpp"
    -o
    -name "*.h"
    -o
    -name "*.hpp"
    )
    -print0
  | xargs -0 clang-format -i
  COMMAND
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Formatting source code..."
  VERBATIM
)

# # Find Doxygen package
# find_package(Doxygen)

# # Check if Doxygen is found
# if(DOXYGEN_FOUND)
#     # Set input and output directories
#     set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
#     set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

#     # Configure Doxygen file
#     configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

#     # Add a target to generate the API documentation
#     add_custom_target(docs
#         COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
#         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#         COMMENT "Generating API documentation with Doxygen"
#         VERBATIM
#     )
# endif()

# Define installation destinations
set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/build/bin" CACHE PATH "Install path prefix" FORCE)

add_custom_target(MainPrintConfig ALL
    COMMAND ${CMAKE_COMMAND} -E echo "Build configuration: $<CONFIG>"
)

set(INSTALL_CONFIG_DIR "Release")

if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(INSTALL_CONFIG_DIR "Debug")
elseif(CMAKE_BUILD_TYPE MATCHES Release)
    set(INSTALL_CONFIG_DIR "Release")
elseif(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    set(INSTALL_CONFIG_DIR "RelWithDebInfo")
elseif(CMAKE_BUILD_TYPE MATCHES MinSizeRel)
    set(INSTALL_CONFIG_DIR "MinSizeRel")
endif()

message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "CMAKE_GENERATOR: ${CMAKE_GENERATOR}")
message(STATUS "INSTALL_CONFIG_DIR: ${INSTALL_CONFIG_DIR}")

if (CMAKE_GENERATOR MATCHES "Visual Studio")
    install(DIRECTORY ${CMAKE_BINARY_DIR}/src/app/${INSTALL_CONFIG_DIR}/ DESTINATION ${CMAKE_INSTALL_PREFIX})
    install(DIRECTORY ${CMAKE_BINARY_DIR}/src/lib/${INSTALL_CONFIG_DIR}/ DESTINATION ${CMAKE_INSTALL_PREFIX})
    # Install plyxporter module to bin
    install(DIRECTORY ${CMAKE_BINARY_DIR}/plyxporter/src/app/${INSTALL_CONFIG_DIR}/ DESTINATION ${CMAKE_INSTALL_PREFIX})
    install(DIRECTORY ${CMAKE_BINARY_DIR}/plyxporter/src/lib/${INSTALL_CONFIG_DIR}/ DESTINATION ${CMAKE_INSTALL_PREFIX})
    # Install visualizer module to bin
    install(DIRECTORY ${CMAKE_BINARY_DIR}/visualizer/${INSTALL_CONFIG_DIR}/ DESTINATION ${CMAKE_INSTALL_PREFIX})
    install(DIRECTORY ${CMAKE_BINARY_DIR}/visualizer/${INSTALL_CONFIG_DIR}/ DESTINATION ${CMAKE_INSTALL_PREFIX})
    file(COPY ${CMAKE_SOURCE_DIR}/visualizer/shaders DESTINATION ${CMAKE_INSTALL_PREFIX})
else()
    # Ninja or other generators: add your install logic here if needed
    install(DIRECTORY ${CMAKE_BINARY_DIR}/src/app/ DESTINATION ${CMAKE_INSTALL_PREFIX})
    install(DIRECTORY ${CMAKE_BINARY_DIR}/src/lib/ DESTINATION ${CMAKE_INSTALL_PREFIX})
endif()

# install(DIRECTORY ${CMAKE_SOURCE_DIR}/asset DESTINATION ${CMAKE_INSTALL_PREFIX})
