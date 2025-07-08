message(DEBUG "./src/lib/utils/cmake")

# Define the library
add_library(__utilsLibrary STATIC
    threadqueue.cpp
)

add_library(__logLibrary STATIC
    log.cpp
)

# # Optionally, add additional configurations, dependencies, or options
if(UNIX)
    target_compile_options(__utilsLibrary PRIVATE -fPIC)
    target_compile_options(__logLibrary PRIVATE -fPIC)
endif()